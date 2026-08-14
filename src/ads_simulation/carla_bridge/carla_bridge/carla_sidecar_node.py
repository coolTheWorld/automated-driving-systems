# Copyright 2026 孙帅
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""CARLA sidecar：补齐原生 ROS 2 接口缺失的那一半（SPEC §4.1 双通道）.

原生 ``--ros2`` 只是一根 sensor-out / control-in 的管子；本节点用 PythonAPI
补上：**完整 TF 树、/odom、/ego_pose_gt、/joint_states、控制反向通道**、
地图加载与自车 spawn。高频传感器流不经过这里 —— 那是原生通道的活。

## 为什么 import carla 是惰性的

PythonAPI wheel 只装在云机上。惰性 import 让本包在本机可构建、可 lint、
纯数学层可 L1 —— 「本地 dry-run 全过才上机」的前提（P8 风险表第一条）。

## ⚠️ 上机才能验证的四件事（P0b 已知 issue，S5 checklist）

1. 原生传感器话题的实际命名（能否直接落到 /carla/lidar/points_raw）；
2. 原生通道与 PythonAPI 的坐标系/时间戳一致性（issue #3）；
3. 内嵌 Fast DDS 与 Jazzy 自带版本的互操作；
4. vehicle_control 偶发失效（issue #1）—— 看门狗每 tick 重发兜底。
"""

import math
import threading
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu, JointState, NavSatFix, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ads_msgs.msg import VehicleCmd

from geometry_msgs.msg import Twist
import yaml

from carla_bridge.control_mapping import ControlMapping
from carla_bridge.npc_kinematics import scenario_actor_names, step_pose
from carla_bridge.transforms import (
    position_to_carla, position_to_ros, relative_pose_in_frame, yaw_rate_to_ros,
    yaw_to_carla, yaw_to_quaternion, yaw_to_ros)


class CarlaSidecarNode(Node):
    """PythonAPI 会话的 ROS 包装。carla 句柄由 main() 注入，本类不 import carla."""

    def __init__(self, carla_module, client):
        """构造：加载世界、spawn 自车、建立话题与定时器.

        :param carla_module: 已 import 的 carla 模块（惰性注入，测试可传桩）
        :param client: 已连接的 carla.Client
        """
        super().__init__('carla_sidecar')
        self._carla = carla_module
        self._client = client

        # use_sim_time 由参数文件带入（rclpy 自动声明），这里不重复声明。
        host_desc = '（上机时由 launch 覆盖，本地 dry-run 不连接）'
        self.declare_parameter('map_xodr_path', '')
        # spawn 位姿：ROS ENU。⚠️ 必须与 campus_loop.sdf 的自车 spawn 一致 ——
        # map→odom 就是它（CLAUDE.md §5：发单位变换 = 宣称地图原点=出生点，P0a 踩过）。
        self.declare_parameter('spawn.x_m', 30.0)
        self.declare_parameter('spawn.y_m', -51.75)
        self.declare_parameter('spawn.yaw_rad', 0.0)
        # 蓝图：citroen.c3（轴距 2.684 vs 2.700，差 0.6%）。
        # ⚠️ 不要换 tesla.model3（3.005，差 11.3%）—— 轴距 apply_physics_control
        #    改不了，只能换蓝图，而它直接进 Stanley 的前轴换算（P0b 实测结论）。
        self.declare_parameter('ego.blueprint', 'vehicle.citroen.c3')
        self.declare_parameter('control.watchdog_timeout_s', 0.5)
        # throttle/brake 标定常数：⚠️ 上机重标（P0b：τ、稳态达成率都与 Gazebo
        # 不同）。初值取「1.5 m/s² 满油门的 40%、3.0 m/s² 满刹车」量级的保守猜测。
        self.declare_parameter('control.throttle_per_mps2', 0.4)
        self.declare_parameter('control.brake_per_mps2', 0.33)
        self.declare_parameter('limits.max_steer_angle_rad', 0.6)
        self.declare_parameter('limits.max_accel_mps2', 1.5)
        self.declare_parameter('limits.max_decel_mps2', 3.0)
        self.declare_parameter('tick_hz', 50.0)
        self.get_logger().info(f'sidecar 参数就绪 {host_desc}')

        self._mapping = ControlMapping(
            self.get_parameter('limits.max_steer_angle_rad').value,
            self.get_parameter('limits.max_accel_mps2').value,
            self.get_parameter('limits.max_decel_mps2').value,
            self.get_parameter('control.throttle_per_mps2').value,
            self.get_parameter('control.brake_per_mps2').value)

        # 传感器外参的单一来源（SPEC §4.1）—— sidecar 直接读 vehicle_params，
        # 不在 carla_bridge_params 里抄一份。
        self.declare_parameter('vehicle_params_yaml', '')

        self._world = self._load_world()
        self._ego = self._spawn_ego()
        self._sensors = self._spawn_sensors()

        spawn_x = self.get_parameter('spawn.x_m').value
        spawn_y = self.get_parameter('spawn.y_m').value
        spawn_yaw = self.get_parameter('spawn.yaw_rad').value
        self._spawn_pose = (spawn_x, spawn_y, spawn_yaw)

        # ---- 话题（QoS 与 gazebo_bridge 逐条对齐，SPEC §4.1 契约）----
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # 传感器中继出口。⚠️ 点云走中间名（/carla 前缀 —— bridge_topics.yaml
        # 当年留的位置），由 lidar_preprocessor 做自车裁剪与 frame 换算后出
        # 规范名 /lidar/points；imu/gnss 无需预处理，sidecar 直接出规范名。
        # QoS 一律 reliable（陷阱表：best-effort 静默丢帧只剩 35%）。
        self._lidar_pub = self.create_publisher(PointCloud2, '/carla/lidar/points_raw', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu', 10)
        self._gnss_pub = self.create_publisher(NavSatFix, '/gnss', 10)
        self._gt_pub = self.create_publisher(Odometry, '/ego_pose_gt', 10)
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._publish_map_to_odom()

        # 控制反向通道 + 看门狗。**校验在先、喂狗在后**（陷阱表那条，别改顺序）。
        self._last_cmd = None
        self._last_cmd_time = None
        self.create_subscription(VehicleCmd, '/vehicle_cmd', self._on_cmd, 10)

        # ---- NPC 道具（P8-S4b，用户拍板「sidecar 复刻航点驱动」）----------
        # 控制器复用 gazebo_bridge 的 npc_controller（仿真器无关：吃
        # /model/<n>/pose_gt 出 /model/<n>/cmd_vel）。sidecar 只补两头：
        # 发 pose_gt（CARLA 位姿）、收 cmd_vel（体系 twist → step_pose 积分
        # → set_transform）。行为与 gz VelocityControl 逐语义对齐
        # （npc_kinematics.py 的对照表），场景判据时间窗才对得上。
        self.declare_parameter('dynamic_actors_yaml', '')
        self.declare_parameter('scenario', '')
        self._npcs = {}
        actors_yaml = self.get_parameter('dynamic_actors_yaml').value
        scenario = self.get_parameter('scenario').value
        if actors_yaml and scenario:
            self._spawn_npcs(actors_yaml, scenario)

        period_s = 1.0 / self.get_parameter('tick_hz').value
        self.create_timer(period_s, self._on_tick)

        # 墙钟节拍器：同步模式下 world.tick() 的唯一来源。必须用**墙钟**——
        # 本节点 use_sim_time=true，ROS 定时器等仿真钟，而仿真钟等 tick：
        # 用 ROS 定时器推 tick 就是死锁。这是仿真配速不是算法时序
        # （SPEC §5 豁免类比：bridge 看门狗的墙钟健康检查）。
        self._ticking = True
        self._tick_thread = threading.Thread(target=self._pace_world, daemon=True)
        self._tick_thread.start()

    # ------------------------------------------------------------------
    #  世界与自车
    # ------------------------------------------------------------------
    def _load_world(self):
        """用 generate_opendrive_world 加载 campus.xodr —— 两环境共用同一份地图."""
        xodr_path = self.get_parameter('map_xodr_path').value
        if not xodr_path:
            raise RuntimeError(
                'map_xodr_path 为空 —— launch 必须传 campus.xodr 的绝对路径。'
                '不给默认值是有意的：静默加载错地图比启动失败更难查。')
        with open(xodr_path, encoding='utf-8') as f:
            xodr = f.read()
        # 参数含义：顶点距离/最大道路长度影响网格质量；额外宽度给路肩。
        # 取 CARLA 文档的常用值；画质无关紧要（我们只要几何与语义一致）。
        params = self._carla.OpendriveGenerationParameters(
            vertex_distance=2.0, max_road_length=50.0, wall_height=0.0,
            additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=True)
        world = self._client.generate_opendrive_world(xodr, params)
        # ---- 同步模式（S5 实测后加）--------------------------------------
        # 异步模式下世界自由狂奔（实测 ~300 FPS）：仿真钟三倍速于墙钟、
        # lidar 每 tick 只出部分弧段、物理步长不定 —— issue #3「原生与
        # PythonAPI 不一致」的温床。同步模式 + 固定步长 0.05 s（20 FPS），
        # 由本节点的墙钟节拍线程推 tick ⟹ RTF ≈ 1。
        # ⚠️ 20 FPS 的含义：IMU 上限 20 Hz（Gazebo 侧 100 Hz）——S5 的六项
        #    验收不含 IMU 频率；S6 做定位时要重估步长（见 p8_carla_bringup §4）。
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        return world

    def _spawn_ego(self):
        """spawn 自车并对齐物理参数（apply_physics_control）."""
        blueprint_name = self.get_parameter('ego.blueprint').value
        blueprint = self._world.get_blueprint_library().find(blueprint_name)
        blueprint.set_attribute('role_name', 'ego_vehicle')
        # ros_name 进原生话题路径的父段：没有它路径是 /carla//<sensor>/...，
        # 而 rclcpp **拒绝**含重复斜杠的话题名 —— 原生话题在 DDS 层存在、
        # ROS 节点却订不了（S5 实测，issue #2 的真实杀伤方式）。
        if blueprint.has_attribute('ros_name'):
            blueprint.set_attribute('ros_name', 'ego_vehicle')
        x, y, yaw = (self.get_parameter('spawn.x_m').value,
                     self.get_parameter('spawn.y_m').value,
                     self.get_parameter('spawn.yaw_rad').value)
        cx, cy, _ = position_to_carla(x, y, 0.0)
        transform = self._carla.Transform(
            self._carla.Location(x=cx, y=cy, z=0.3),
            self._carla.Rotation(yaw=yaw_to_carla(yaw)))
        ego = self._world.spawn_actor(blueprint, transform)
        # 物理对齐的完整映射在 scripts/carla_align_vehicle.py（P0b 验证过一次）；
        # S5 上机时按它重跑一遍再把结果固化到这里 —— 现在不抄一份过来，
        # 抄了就是两处漂移（单一来源，SPEC §4.1）。
        self.get_logger().info(f'自车已 spawn：{blueprint_name} @ ROS({x:.2f}, {y:.2f})')
        return ego

    def _spawn_sensors(self):
        """按 vehicle_params 外参 spawn 原生通道传感器（S5 实测三条落地）.

        2026-08-14 云机实测（手册 §2 核对项 1）：
          · 话题名 = /carla//<ros_name>[/point_cloud] —— **双斜杠**是 CARLA
            issue #2（父级车辆名没进路径），照实配下游，不猜官方哪天修；
          · **listen(no-op) 是开流条件**：原生通道没有消费者不发布；
            客户端断开后流不断（"sensor still alive" 警告即此语义）；
          · ros_publish_tf 必须显式关：默认开，会与 robot_state_publisher
            抢传感器 TF —— 一段两个发布者的老坑（CLAUDE.md §5 同类）。

        ⚠️ 安装位姿相对 **CARLA 车辆原点**（包围盒中心系），与 base_link
        （后轴地面）差一个纵向偏移 —— S6 一致性表要量的项，先按
        车长/2 − 后悬近似换算。
        """
        params_path = self.get_parameter('vehicle_params_yaml').value
        if not params_path:
            self.get_logger().warn('vehicle_params_yaml 为空 —— 不 spawn 传感器（裸联调模式）')
            return []
        with open(params_path, encoding='utf-8') as f:
            vehicle = yaml.safe_load(f)
        geo = vehicle['geometry']
        # base_link（后轴）→ CARLA 原点（近似包围盒中心）的纵向偏移
        axle_to_center_m = 0.5 * geo['length_m'] - geo['rear_overhang_m']
        library = self._world.get_blueprint_library()
        sensors = []
        specs = [
            ('sensor.lidar.ray_cast', 'lidar', 'lidar_link', {
                'channels': str(vehicle['sensors']['lidar']['channels']),
                'rotation_frequency': '10',
                'points_per_second': str(
                    vehicle['sensors']['lidar']['channels'] *
                    vehicle['sensors']['lidar']['horizontal_samples'] * 10),
                'range': '30.0',
                'upper_fov': str(math.degrees(
                    vehicle['sensors']['lidar']['vertical_fov_max_rad'])),
                'lower_fov': str(math.degrees(
                    vehicle['sensors']['lidar']['vertical_fov_min_rad'])),
            }),
            ('sensor.other.imu', 'imu', 'imu_link', {
                'sensor_tick': str(1.0 / vehicle['sensors']['imu']['update_rate_hz']),
            }),
            ('sensor.other.gnss', 'gnss', 'gnss_link', {
                'sensor_tick': str(1.0 / vehicle['sensors']['gnss']['update_rate_hz']),
            }),
        ]
        for bp_name, ros_name, frame_id, attrs in specs:
            bp = library.find(bp_name)
            bp.set_attribute('ros_name', ros_name)
            if bp.has_attribute('ros_frame_id'):
                bp.set_attribute('ros_frame_id', frame_id)
            if bp.has_attribute('ros_publish_tf'):
                bp.set_attribute('ros_publish_tf', 'false')
            for key, value in attrs.items():
                if bp.has_attribute(key):
                    bp.set_attribute(key, value)
            mount = vehicle['sensors'][ros_name]
            transform = self._carla.Transform(self._carla.Location(
                x=mount['mount_x_m'] - axle_to_center_m,
                y=-mount['mount_y_m'],
                z=mount['mount_z_m']))
            actor = self._world.spawn_actor(bp, transform, attach_to=self._ego)
            # ---- 中继回调（S5 实测后的方案变更）------------------------------
            # 原生通道的话题名恒为 /carla//<name>（父段为空，issue #2），而
            # rclcpp **拒绝**重复斜杠 —— 原生流对 ROS 节点不可达。改由 sidecar
            # 在 listen 回调里转 ROS 消息发**合法名**。带宽账：32×1800×10 Hz
            # ≈ 9.2 MB/s，numpy 翻 y 毫秒级；RTF 判据（verify [6/6]）守着上限。
            if ros_name == 'lidar':
                actor.listen(self._on_lidar)
            elif ros_name == 'imu':
                actor.listen(self._on_imu)
            else:
                actor.listen(self._on_gnss)
            sensors.append(actor)
            self.get_logger().info(f'传感器已 spawn+中继：{ros_name}（{bp_name}）')
        return sensors

    def _pace_world(self):
        """按固定步长的墙钟节拍推 world.tick()（RTF ≈ 1 的机制本体）."""
        step_s = 0.05
        next_t = time.monotonic()
        while self._ticking:
            try:
                self._world.tick()
            except RuntimeError as error:
                self.get_logger().warn(f'world.tick 失败：{error}', throttle_duration_sec=5.0)
            next_t += step_s
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()  # 跟不上就不追帧（RTF < 1 而不是抖动）

    # ------------------------------------------------------------------
    #  传感器中继（CARLA 左手系 → ENU：线量 y 反号；角量 x/z 反号）
    # ------------------------------------------------------------------
    def _sensor_stamp(self, data):
        """CARLA 测量时间戳 → ROS 时间。与原生 /clock 同一个仿真钟源."""
        from rclpy.time import Time
        return Time(seconds=data.timestamp).to_msg()

    def _on_lidar(self, data):
        """LidarMeasurement → PointCloud2（xyzi float32，y 反号；按回卷聚合整圈）.

        同步 20 FPS × 旋转 10 Hz ⟹ 每圈 2 个 tick、每 tick 半圈弧段。
        按 horizontal_angle **回卷**聚合（不是数 tick —— 数 tick 在服务器
        偶尔丢帧时会错半圈，回卷判据对任何步长/转速组合都成立）。
        """
        chunk = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
        # ---- 按 tick 计数切帧（S5 实测终版）-------------------------------
        # horizontal_angle 实测**不回卷**（模 2π 也救不了 —— 前两版按它切帧
        # 一个点都发不出，兜底冲刷则攒出 5 圈半的巨包被 DDS UDP 分片静默
        # 丢光，小消息 IMU/GNSS 全过正是旁证）。同步模式下每圈 tick 数是
        # **确定的**：20 FPS / 旋转 10 Hz = 2 tick/圈 —— 计数切帧，确定且
        # 与角度语义无关。留档：首几拍打印角度原值，供后续版本对表。
        if not hasattr(self, '_lidar_chunks'):
            self._lidar_chunks = []
            self._lidar_angle_logged = 0
        if self._lidar_angle_logged < 4:
            self._lidar_angle_logged += 1
            self.get_logger().info(
                f'lidar tick 角度留档 #{self._lidar_angle_logged}: '
                f'horizontal_angle={data.horizontal_angle:.4f}, 点数={chunk.shape[0]}')
        self._lidar_chunks.append(chunk)
        if len(self._lidar_chunks) < 2:  # = round(1/(rotation_hz·fixed_delta))
            return
        points = np.concatenate(self._lidar_chunks).copy()
        self._lidar_chunks = []
        points[:, 1] = -points[:, 1]
        msg = PointCloud2()
        msg.header.stamp = self._sensor_stamp(data)
        msg.header.frame_id = 'lidar_link'
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * points.shape[0]
        msg.data = points.tobytes()
        msg.is_dense = True
        self._lidar_pub.publish(msg)

    def _on_imu(self, data):
        """IMUMeasurement → sensor_msgs/Imu.

        左手系 → 右手系：加速度（极矢量）y 反号；角速度（赝矢量）x、z 反号。
        CARLA 的加速度计与真实 IMU 一样含重力反作用（比力），语义与
        Gazebo 侧一致，ESKF 不需要区分来源。
        """
        msg = Imu()
        msg.header.stamp = self._sensor_stamp(data)
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x = data.accelerometer.x
        msg.linear_acceleration.y = -data.accelerometer.y
        msg.linear_acceleration.z = data.accelerometer.z
        msg.angular_velocity.x = -data.gyroscope.x
        msg.angular_velocity.y = data.gyroscope.y
        msg.angular_velocity.z = -data.gyroscope.z
        msg.orientation_covariance[0] = -1.0  # 不提供朝向（与 Gazebo IMU 同约定）
        self._imu_pub.publish(msg)

    def _on_gnss(self, data):
        """GnssMeasurement → NavSatFix（大地坐标无手性问题，直通）."""
        msg = NavSatFix()
        msg.header.stamp = self._sensor_stamp(data)
        msg.header.frame_id = 'gnss_link'
        msg.latitude = data.latitude
        msg.longitude = data.longitude
        msg.altitude = data.altitude
        self._gnss_pub.publish(msg)

    def _publish_map_to_odom(self):
        """静态 map→odom = spawn 位姿（与 gazebo 的 map_to_odom_static 同一约定）."""
        x, y, yaw = self._spawn_pose
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._static_tf.sendTransform(tf)

    # ------------------------------------------------------------------
    #  NPC 道具
    # ------------------------------------------------------------------
    def _spawn_npcs(self, actors_yaml_path, scenario):
        """按场景 spawn NPC 道具并接上 npc_controller 的话题对."""
        cfg = yaml.safe_load(open(actors_yaml_path, encoding='utf-8'))
        library = self._world.get_blueprint_library()
        for name in scenario_actor_names(cfg, scenario):
            actor_cfg = cfg['actors'][name]
            first_wp = actor_cfg['waypoints'][0]
            x0, y0 = float(first_wp[0]), float(first_wp[1])
            # 车用与 ego 不同的蓝图（道具不需要对齐动力学 —— 位姿由脚本管）；
            # 行人用 walker。⚠️ 生成器的机械校验（外廓/航点在界内）依旧由
            # gen_dynamic_actors --check 守着 —— 这里不重复校验，单一来源。
            if actor_cfg.get('classification') == 'pedestrian':
                blueprint = library.filter('walker.pedestrian.*')[0]
            else:
                blueprint = library.find('vehicle.nissan.micra')
            cx, cy, _ = position_to_carla(x0, y0, 0.0)
            transform = self._carla.Transform(self._carla.Location(x=cx, y=cy, z=0.2))
            actor = self._world.spawn_actor(blueprint, transform)
            # 位姿全归脚本：物理开着会与 set_transform 抢位姿（道具抖动）。
            actor.set_simulate_physics(False)
            npc = {
                'actor': actor, 'pose': (x0, y0, 0.0), 'cmd': (0.0, 0.0, 0.0),
                'pub': self.create_publisher(Odometry, f'/model/{name}/pose_gt', 10),
            }
            self._npcs[name] = npc
            self.create_subscription(
                Twist, f'/model/{name}/cmd_vel',
                lambda msg, key=name: self._npcs[key].__setitem__(
                    'cmd', (msg.linear.x, msg.linear.y, msg.angular.z)), 10)
            self.get_logger().info(f'NPC 已 spawn：{name} @ ({x0:.1f}, {y0:.1f})')

    def _tick_npcs(self, dt_s, stamp):
        """积分 cmd_vel → set_transform，并发 pose_gt（npc_controller 的输入）."""
        for name, npc in self._npcs.items():
            x, y, yaw = step_pose(*npc['pose'], *npc['cmd'], dt_s)
            npc['pose'] = (x, y, yaw)
            cx, cy, _ = position_to_carla(x, y, 0.0)
            npc['actor'].set_transform(self._carla.Transform(
                self._carla.Location(x=cx, y=cy, z=0.2),
                self._carla.Rotation(yaw=yaw_to_carla(yaw))))
            gt = Odometry()
            gt.header.stamp = stamp
            gt.header.frame_id = 'map'
            gt.child_frame_id = f'{name}_base'
            gt.pose.pose.position.x = x
            gt.pose.pose.position.y = y
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            gt.pose.pose.orientation.x = qx
            gt.pose.pose.orientation.y = qy
            gt.pose.pose.orientation.z = qz
            gt.pose.pose.orientation.w = qw
            npc['pub'].publish(gt)

    # ------------------------------------------------------------------
    #  控制反向通道
    # ------------------------------------------------------------------
    def _on_cmd(self, msg: VehicleCmd):
        """收指令：**先校验后喂狗**。坏指令不喂狗 —— 持续 NaN 流等价于失联."""
        if not self._mapping.is_valid(msg.steer_angle_rad, msg.accel_mps2):
            self.get_logger().warn(
                '丢弃含非有限值的 /vehicle_cmd（不喂看门狗）', throttle_duration_sec=3.0)
            return
        self._last_cmd = (msg.steer_angle_rad, msg.accel_mps2)
        self._last_cmd_time = self.get_clock().now()

    def _apply_control(self):
        """每 tick 重发控制（兜 CARLA issue #1「控制偶发失效」的底）."""
        timeout_s = self.get_parameter('control.watchdog_timeout_s').value
        expired = (
            self._last_cmd_time is None or
            (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9 > timeout_s)
        fields = (self._mapping.full_brake() if expired
                  else self._mapping.to_carla(*self._last_cmd))
        control = self._carla.VehicleControl(
            throttle=fields['throttle'], brake=fields['brake'], steer=fields['steer'])
        self._ego.apply_control(control)

    # ------------------------------------------------------------------
    #  周期发布
    # ------------------------------------------------------------------
    def _on_tick(self):
        """读自车真值 → /ego_pose_gt、/odom、TF odom→base_link、/joint_states."""
        transform = self._ego.get_transform()
        velocity = self._ego.get_velocity()
        angular = self._ego.get_angular_velocity()

        x, y, z = position_to_ros(
            transform.location.x, transform.location.y, transform.location.z)
        yaw = yaw_to_ros(transform.rotation.yaw)
        vx, vy, _ = (velocity.x, -velocity.y, velocity.z)
        yaw_rate = yaw_rate_to_ros(angular.z)
        stamp = self.get_clock().now().to_msg()

        # ---- /ego_pose_gt：map 系真值（仅评测可订阅，SPEC §4.1）----
        gt = Odometry()
        gt.header.stamp = stamp
        gt.header.frame_id = 'map'
        gt.child_frame_id = 'base_link'
        gt.pose.pose.position.x = x
        gt.pose.pose.position.y = y
        gt.pose.pose.position.z = z
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        gt.pose.pose.orientation.x = qx
        gt.pose.pose.orientation.y = qy
        gt.pose.pose.orientation.z = qz
        gt.pose.pose.orientation.w = qw
        gt.twist.twist.linear.x = vx
        gt.twist.twist.linear.y = vy
        gt.twist.twist.angular.z = yaw_rate
        self._gt_pub.publish(gt)

        # ---- /odom + TF odom→base_link：相对 spawn ----
        # ⚠️ 与 Gazebo 的轮速推算不同，这里没有里程计漂移 —— 保真度差异
        #    记进两环境一致性表（S6），localization:=true 时不受影响
        #    （P4 的 map→odom 由 localization_node 动态发，这条静态链关掉）。
        lx, ly, lyaw = relative_pose_in_frame(x, y, yaw, *self._spawn_pose)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = lx
        odom.pose.pose.position.y = ly
        oqx, oqy, oqz, oqw = yaw_to_quaternion(lyaw)
        odom.pose.pose.orientation.x = oqx
        odom.pose.pose.orientation.y = oqy
        odom.pose.pose.orientation.z = oqz
        odom.pose.pose.orientation.w = oqw
        # twist 在 child_frame（base_link）系：投影到车体
        speed = math.hypot(vx, vy)
        heading_err = math.atan2(vy, vx) - yaw
        odom.twist.twist.linear.x = speed * math.cos(heading_err)
        odom.twist.twist.linear.y = speed * math.sin(heading_err)
        odom.twist.twist.angular.z = yaw_rate
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = lx
        tf.transform.translation.y = ly
        tf.transform.rotation.x = oqx
        tf.transform.rotation.y = oqy
        tf.transform.rotation.z = oqz
        tf.transform.rotation.w = oqw
        self._tf_broadcaster.sendTransform(tf)

        # ---- /joint_states：robot_state_publisher 靠它画轮子/出传感器 TF ----
        # 轮角从 CARLA 拿不到便宜的真值，置零 —— 消费者只有 RViz 可视化，
        # 传感器 TF 由 URDF 的固定关节给出，不受影响（与 verify 脚本核对项一致）。
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [
            'front_left_steer_joint', 'front_right_steer_joint',
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_left_wheel_joint', 'rear_right_wheel_joint']
        joints.position = [0.0] * 6
        self._joint_pub.publish(joints)

        self._tick_npcs(1.0 / self.get_parameter('tick_hz').value, stamp)
        self._apply_control()


def main(args=None):
    """入口：惰性 import carla —— 本机没有 wheel 时给出指名的错误."""
    try:
        import carla
    except ImportError as error:
        raise SystemExit(
            'carla PythonAPI 不可用：本节点只在云机上跑（SPEC §4.1 环境 B）。'
            '本地开发用 dry-run（test_pure_math.py + launch 加载测试）。'
            f'原始错误：{error}') from error

    rclpy.init(args=args)
    # 连接参数走环境变量而不是 ROS 参数：连接失败发生在节点构造之前，
    # 那时还没有参数服务器可查 —— 报错要能不依赖 ROS 图就看懂。
    import os
    host = os.environ.get('CARLA_HOST', '127.0.0.1')
    port = int(os.environ.get('CARLA_PORT', '2000'))
    client = carla.Client(host, port)
    client.set_timeout(30.0)

    node = CarlaSidecarNode(carla, client)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
