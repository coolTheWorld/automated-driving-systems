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

from rosgraph_msgs.msg import Clock

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
        self.declare_parameter('control.throttle_per_mps2', 0.113)
        self.declare_parameter('control.brake_per_mps2', 0.185)
        self.declare_parameter('control.throttle_bias', 0.54)
        self.declare_parameter('control.idle_below_mps2', 0.05)
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
            self.get_parameter('control.brake_per_mps2').value,
            self.get_parameter('control.throttle_bias').value,
            self.get_parameter('control.idle_below_mps2').value)

        # 传感器外参的单一来源（SPEC §4.1）—— sidecar 直接读 vehicle_params，
        # 不在 carla_bridge_params 里抄一份。
        self.declare_parameter('vehicle_params_yaml', '')

        self._world = self._load_world()
        self._carla_map = self._world.get_map()  # 瞬移走廊夹取用（见 _tick_npcs）
        self._ego = self._spawn_ego()
        self._sensors = self._spawn_sensors()

        spawn_x = self.get_parameter('spawn.x_m').value
        spawn_y = self.get_parameter('spawn.y_m').value
        spawn_yaw = self.get_parameter('spawn.yaw_rad').value
        self._spawn_pose = (spawn_x, spawn_y, spawn_yaw)

        # ---- 话题（QoS 与 gazebo_bridge 逐条对齐，SPEC §4.1 契约）----
        # /clock 由 sidecar 发（S6 实测：原生 --ros2 的 clock 在世界重建后
        # **静默死亡** —— 话题名还在发现缓存里、一条消息都不发，而 use_sim_time
        # 的所有 ROS 定时器随之冻住，TF 树断成两棵。节拍线程每 tick 后发
        # 快照时间，桥从此不依赖任何原生通道。⚠️ 若原生 clock 复活会出现
        # 双发布者（时间抖动）——上机 checklist 有核对项。
        self._clock_pub = self.create_publisher(Clock, '/clock', 10)
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
        # P3/S04 静态障碍场景（avoid/block）：与 Gazebo 同一份 obstacles.yaml
        self.declare_parameter('obstacles_yaml', '')
        self.declare_parameter('obstacles_scenario', '')
        self._npcs = {}
        self._last_npc_tick_ns = None  # NPC 积分用实际仿真 dt（见 _on_tick 注释）
        self._last_throttle = 0.0      # 油门上升速率限幅的记忆（见 _apply_control）
        obstacles_yaml = self.get_parameter('obstacles_yaml').value
        obstacles_scenario = self.get_parameter('obstacles_scenario').value
        # 'none' = 什么都不生成（与 Gazebo 的「没有 none 场景」语义一致：
        # stack.launch 默认透传 obstacles:=none）。
        if obstacles_yaml and obstacles_scenario not in ('', 'none'):
            self._spawn_obstacles(obstacles_yaml, obstacles_scenario)
        actors_yaml = self.get_parameter('dynamic_actors_yaml').value
        scenario = self.get_parameter('scenario').value
        # 'none' 与空同义（stack.launch 默认透传 dynamic:=none，与 obstacles 同规矩）
        if actors_yaml and scenario not in ('', 'none'):
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
        # ---- 开锁器（S6 实测两次 wedge 后加）------------------------------
        # 上一个 sidecar 被 SIGTERM 杀掉时清理钩子不执行（rclpy 默认信号
        # 处理直接终止进程），世界留在同步模式无节拍器 —— 服务器停在 tick
        # 栅栏上，后续重载/tick 全超时。启动时无条件先掰回异步：
        # apply_settings 在 wedge 状态下仍能执行，这一步是唯一的解锁钥匙。
        try:
            world = self._client.get_world()
            settings = world.get_settings()
            if settings.synchronous_mode:
                self.get_logger().warn('检测到遗留的同步模式（上任死得不干净）——先解锁')
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
        except RuntimeError as error:
            self.get_logger().warn(f'解锁尝试失败（继续硬闯 generate）：{error}')
        # 参数含义：顶点距离/最大道路长度影响网格质量；额外宽度给路肩。
        # 取 CARLA 文档的常用值；画质无关紧要（我们只要几何与语义一致）。
        # smooth_junctions=False（S6 实测第 2 次 Segfault 后改）：两次崩溃都在
        # 车驶向路口区时（纯 Signal 11、无 OOM、与 --ros2 无关）——生成式路口
        # 网格平滑是社区已知的崩溃源。牺牲路口网格观感换稳定；车道语义来自
        # xodr 解析，不受网格影响。
        # vertex_distance=0.5（1.0 仍崩，再压一档）：短目标（纯直道）轮服务器
        # 存活、车稳停 goal —— 崩溃锚定在**弯道/路口区网格**。R=8–13.75 的
        # 弯上 2.0 m 顶点距会产生退化三角形（物理网格上车轮碾过 = Segfault
        # 的经典配方）。细分一档换稳定；内存富余（实测 20 GB 可用）。
        # wall_height=1.0（S6 富遥测定案后改）：生成式世界**路外即虚空** ——
        # 车一旦出路面就坠入虚空翻滚（实测航向冻结、伪水平速度 33 m/s），
        # 引擎随后 Segfault。护栏墙把「硬崩溃」变成「诚实的红判据」：
        # 车出界撞墙停下，判据照常打分，服务器活着跑下一场。
        # ⚠️ 感知会看见墙（雷达回波）——感知层场景的影响到时候评，
        #    真值层判据不受影响。
        # additional_width 0.6 → 2.0（P8-S6 junction 实测）：弯道油门超调
        # 0.7 m/s（粗粒度映射，已在差异台账）× 轮胎滑移把 R=13.75 拉宽 ~13%
        # ⟹ 90° 弯累计外漂 >0.6 m，车**亲上贴边的护栏墙**被物理逮捕
        # （控制器命令 +1.5 时实测 −4.6，规划器全程「正常」—— 症状是
        # 「车在弯里自己停了」，五轮排查全在查规划/控制）。Gazebo 同弯是
        # **草肩**，出线无代价 —— 这是世界生成保真度差异，不是栈的病。
        # 2.0 m 硬化路肩恢复同一容错语义；wall_height 的虚空保护不动。
        # additional_width 2.0 → 6.0（P9-S2）：墙是**虚空脚手架不是世界内容**
        # （Gazebo 同位置是草地）。2.0 时墙簇在弯角被 L-Shape OBB 斜撑成
        # 29×3.75 的空腹大框侵入车道（虚警 + 抢真值配对 = 近边误差恒 11.6），
        # 且行人路肩路径压在墙线上。6.0：墙退出感知主战场、平坦裙板变宽
        # 反哺 RANSAC 稳定性、行人路径落回 mesh 上。路肩容错语义（弯道
        # 冲宽回弹）只增不减。
        params = self._carla.OpendriveGenerationParameters(
            vertex_distance=0.5, max_road_length=50.0, wall_height=1.0,
            additional_width=6.0, smooth_junctions=False, enable_mesh_visibility=True)
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
        # ---- 转向曲线拉平（S6 弯道欠转定案后固化）------------------------
        # CARLA 的 steering_curve **随速度衰减最大转角**（同一 steer 归一值，
        # 车速越高实际轮角越小）——Gazebo 的 AckermannSteering 没有这条曲线，
        # 两环境的「转向指令 → 实际轮角」增益因此不一致（P0b 量到的稳态
        # 达成率 86.3% 的机制本体）。按对齐哲学拉平成 1.0：让 CARLA 对齐
        # Gazebo 的执行语义，而不是给控制器开环境特例。轮胎侧偏仍在
        # （那是本质差异，保留）。
        physics = ego.get_physics_control()
        physics.steering_curve = [
            self._carla.Vector2D(x=0.0, y=1.0),
            self._carla.Vector2D(x=50.0, y=1.0)]
        ego.apply_physics_control(physics)
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
                # ---- 丢点模型拉平（P9-S2，对齐哲学 —— steering_curve 同法理）--
                # CARLA 默认 dropoff_general_rate=0.45（随机丢 45%！）+
                # 零强度丢弃 0.4：3.6 m 的车实测只剩 16-40 点/帧（应有数百），
                # 检测率近场 17% < 远场 73% 的倒挂由此而来。Gazebo 雷达无
                # 丢点模型 —— 双环境行为漂移的头号来源之一，全部关掉；
                # 噪声对齐 Gazebo 的 σ=1 cm（check_sensor_noise 的那把尺子）。
                'dropoff_general_rate': '0.0',
                'dropoff_intensity_limit': '0.0',
                'dropoff_zero_intensity': '0.0',
                'atmosphere_attenuation_rate': '0.0',
                'noise_stddev': '0.01',
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
                # 短超时：默认 120 s 是给一次性重载用的，对 50 ms 一拍的
                # tick 来说钝得离谱 —— 服务器真死时要 10 s 内暴露不是 2 分钟。
                self._world.tick(10.0)
                snapshot = self._world.get_snapshot()
                clock = Clock()
                clock.clock.sec = int(snapshot.timestamp.elapsed_seconds)
                clock.clock.nanosec = int(
                    (snapshot.timestamp.elapsed_seconds % 1.0) * 1e9)
                self._clock_pub.publish(clock)
            except Exception as error:  # noqa: B902 —— 服务器崩溃时异常类型不可枚举
                # ⚠️ 只 catch RuntimeError 的教训（S6 实测）：服务器 Segfault 时
                #    抛的不是 RuntimeError，节拍线程**无声死亡**，全系统 sim-time
                #    冻结而无任何日志 —— 比崩溃更难查。catch-all + 大声报。
                self.get_logger().error(f'world.tick 异常（服务器崩了？）：{error!r}')
                time.sleep(2.0)
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
        # ⚠️ **不翻 y**（P9-S1 镜像探针铁案，2026-08-15）：按「CARLA 左手系」
        #    的教科书推理这里该 y 反号 —— 实测反了之后点云与真值成镜像
        #    （npc 车 60 帧正窗 0 点、镜像窗 58054 点），即 0.9.16 的
        #    LidarMeasurement 原始数据手性已与 ROS 约定一致，再翻一次 =
        #    镜像世界。路与墙左右对称 + S04/行为判据全走真值，这个符号错
        #    潜伏了三个租机窗口，直到 P5（点云第一个消费者）上线才炸 ——
        #    「接第一个消费者 = 对上游再验收」的第 N 次执行。
        #    回归守卫：scripts/p9_mirror_probe.py（非对称地标是唯一能抓
        #    符号错误的证据 —— 对称世界里镜像与正确不可区分）。
        #    ⚠️ IMU 中继的反号（_on_imu）与此同源存疑，P4-CARLA 上线前
        #    必须用同类非对称实验重审，不要照抄本结论。
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
    def _spawn_obstacles(self, obstacles_yaml_path, scenario):
        """S04 静态障碍（锥桶）：读与 Gazebo 同一份 obstacles.yaml（单一来源）.

        可行性校验不在这里重复 —— gen_obstacles --check 守着那条不等式。
        CARLA 蓝图用 constructioncone（几何近似 0.5×0.5 锥桶；判据量的是
        自车与障碍**位置**的间距，道具形状差异进两环境一致性表）。
        """
        cfg = yaml.safe_load(open(obstacles_yaml_path, encoding='utf-8'))
        scen = cfg.get('scenarios', {}).get(scenario)
        if scen is None:
            raise RuntimeError(
                f'obstacles.yaml 里没有场景 {scenario!r} —— 拼错要炸在启动，不要静默无障碍')
        library = self._world.get_blueprint_library()
        bp = library.find('static.prop.constructioncone')
        for i, obstacle in enumerate(scen['obstacles']):
            # 坐标是**路线相对**的（along_x + lateral_offset）——车道中心取
            # yaml 自带的 lane.center_y_m（与 gazebo_sim.launch 同一来源同一换算）。
            x = float(obstacle['along_x_m'])
            y = float(cfg['lane']['center_y_m']) + float(obstacle['lateral_offset_m'])
            cx, cy, _ = position_to_carla(x, y, 0.0)
            actor = self._world.spawn_actor(
                bp, self._carla.Transform(self._carla.Location(x=cx, y=cy, z=0.05)))
            # ⚠️ 物理必须开着（P9-S1 实锤）：set_simulate_physics(False) 把
            #    actor 从物理场景摘除，gpu-lidar 的 raycast **打不到它** ——
            #    迎面车 6 m 处三帧 0 回波，而同距离路面回波正常。
            #    关重力替代关物理：不下坠、不施力，碰撞体留在场景里。
            actor.set_enable_gravity(False)
            self.get_logger().info(f'障碍物已 spawn：#{i} @ ENU({x:.2f}, {y:.2f})')

    def _spawn_npcs(self, actors_yaml_path, scenario):
        """按场景 spawn NPC 道具并接上 npc_controller 的话题对."""
        cfg = yaml.safe_load(open(actors_yaml_path, encoding='utf-8'))
        library = self._world.get_blueprint_library()
        for name in scenario_actor_names(cfg, scenario):
            actor_cfg = cfg['actors'][name]
            route = actor_cfg.get('carla_waypoints', actor_cfg['waypoints'])
            first_wp = route[0]
            x0, y0 = float(first_wp[0]), float(first_wp[1])
            # 车头朝第二个航点（gazebo launch 的既有约定，sidecar 曾漏抄）：
            # kinematic 道具能原地掉头，真车面错方向出生 = 三米后怼墙卡死
            # （P9-S2 实测：环线副本首段向南、车默认面东，整场停在 66 m 外）。
            yaw0 = 0.0
            if len(route) > 1:
                yaw0 = math.atan2(
                    float(route[1][1]) - y0, float(route[1][0]) - x0)
            # 车用与 ego 不同的蓝图（道具不需要对齐动力学 —— 位姿由脚本管）；
            # 行人用 walker。⚠️ 生成器的机械校验（外廓/航点在界内）依旧由
            # gen_dynamic_actors --check 守着 —— 这里不重复校验，单一来源。
            if actor_cfg.get('classification') == 'pedestrian':
                blueprint = library.filter('walker.pedestrian.*')[0]
            else:
                blueprint = library.find('vehicle.nissan.micra')
            cx, cy, _ = position_to_carla(x0, y0, 0.0)
            # ⚠️ 高空 spawn（P8-S6 实测：junction 三辆 NPC 在路面 spawn 直接
            #    「Spawn failed because of collision」）：Gazebo 道具无碰撞、
            #    草地航点物理上没事，而 CARLA 的 spawn_actor **做碰撞检查**
            #    （路缘墙 wall_height=1.0 / 彼此 / 自车都算）。位姿本就全归
            #    脚本管 —— 首拍 set_transform（瞬移，无碰撞检查）即归位；
            #    逐台抬 3 m 错开，免得道具之间在空中互撞。
            is_walker = actor_cfg.get('classification') == 'pedestrian'
            if is_walker:
                # walker：高空 spawn（避碰撞检查）+ 首拍瞬移落地，与既往一致。
                spawn_z = 30.0 + 3.0 * len(self._npcs)
            else:
                # 车辆（P9-S2 apply_control 闭环）：轮胎要接地才有牵引力 ——
                # 直接路面 spawn（P5/P7 的车辆首航点都在路上；撞车/撞墙时
                # 才退回高空——那说明场景摆错了，宁可它掉下来砸出红判据）。
                spawn_z = 0.5
            transform = self._carla.Transform(
                self._carla.Location(x=cx, y=cy, z=spawn_z),
                self._carla.Rotation(yaw=yaw_to_carla(yaw0)))
            try:
                actor = self._world.spawn_actor(blueprint, transform)
            except RuntimeError:
                transform.location.z = 30.0 + 3.0 * len(self._npcs)
                actor = self._world.spawn_actor(blueprint, transform)
                self.get_logger().warn(
                    f'NPC {name} 路面 spawn 碰撞，退高空 —— 查场景首航点摆放')
            # 物理必须开（关物理 = 从物理场景摘除 = lidar 打不到，P9-S1 实锤）。
            # walker 关重力（瞬移驱动防下坠抢位姿）；车辆重力**开**——
            # apply_control 的牵引力来自轮胎接地（P9-S2 闭环）。
            if is_walker:
                actor.set_enable_gravity(False)
            npc = {
                'actor': actor, 'pose': (x0, y0, yaw0), 'cmd': (0.0, 0.0, 0.0),
                'pub': self.create_publisher(Odometry, f'/model/{name}/pose_gt', 10),
                'yaw0': yaw0,
                # 车辆与行人走不同的物理驱动（见 _tick_npcs 的实测注释）
                'is_walker': is_walker,
                # ---- P9-S2 黄金线索的仪器（actor 运行中消失）----------------
                # 服务器侧存活（get_actors 普查，10 s 一次）+ z 轨迹环形缓冲
                # （每拍一样，留最近 60 拍 = 3 s）。死亡瞬间一次性倒出：消失
                # 时刻、最后位姿、最近 z 序列 —— 「z 一路下坠 = 掉出世界被
                # 引擎回收（UE4 KillZ）」与「z 正常却没了 = 别的客户端销毁」
                # 在同一份日志里就分得开，不必再猜。
                'server_alive': True,
                'z_trace': [],
                'spawn_z': float(transform.location.z),
            }
            self._npcs[name] = npc
            self.create_subscription(
                Twist, f'/model/{name}/cmd_vel',
                lambda msg, key=name: self._npcs[key].__setitem__(
                    'cmd', (msg.linear.x, msg.linear.y, msg.angular.z)), 10)
            self.get_logger().info(f'NPC 已 spawn：{name} @ ({x0:.1f}, {y0:.1f})')

    def _npc_census(self, stamp):
        """10 s 一次的服务器侧普查：谁还在世界里（get_actors 是 RPC，问的是服务器）.

        ⚠️ 为什么不用 actor.is_alive：那是**客户端本地**标志，只有经本客户端
        destroy() 才翻假；服务器把 actor 收走（KillZ / 别的客户端销毁）时它
        照样 True，而 get_transform() 对不存在的 actor **不抛错、返回全零**
        （libcarla CopyActorSnapshotIfPresent 的语义）—— 于是真值悄悄变成原点、
        感知看到 0 点、没有任何一层报错。这就是「0 点轮」的机制候选之一。
        """
        try:
            ids = {actor.id for actor in self._world.get_actors()}
        except Exception as error:  # noqa: B902
            self.get_logger().warn(f'NPC 普查 get_actors 失败：{error!r}')
            return
        sim_t = stamp.sec + stamp.nanosec * 1e-9
        for name, npc in self._npcs.items():
            present = npc['actor'].id in ids
            x, y, _ = npc['pose']
            zs = ' '.join(f'{z:.1f}' for _, z in npc['z_trace'][-12:])
            if npc['server_alive'] and not present:
                npc['server_alive'] = False
                self.get_logger().error(
                    f'NPC {name}（id={npc["actor"].id}）**已从服务器消失** t={sim_t:.1f} s：'
                    f'最后位姿 ({x:.1f}, {y:.1f})，spawn_z={npc["spawn_z"]:.1f}，'
                    f'最近 z 序列（每 5 拍一样）[{zs}] —— z 一路下坠 = 掉出世界'
                    f'（KillZ 回收，查航点/墙）；z 正常 = 被别的客户端 destroy')
            else:
                self.get_logger().info(
                    f'NPC 普查 t={sim_t:.1f} s：{name} 在服务器={present} '
                    f'pose=({x:.1f}, {y:.1f}) z 近况 [{zs}]（世界 actor 总数 {len(ids)}）')

    def _tick_npcs(self, dt_s, stamp):
        """积分 cmd_vel → set_transform，并发 pose_gt（npc_controller 的输入）."""
        self._npc_tick_index = getattr(self, '_npc_tick_index', 0) + 1
        for name, npc in self._npcs.items():
            # z 轨迹留样（每 5 拍一样，环形 60 样 = 15 s）——普查/死亡报告用
            if self._npc_tick_index % 5 == 0:
                try:
                    z_now = float(npc['actor'].get_transform().location.z)
                except Exception:  # noqa: B902 —— 仪器不许把主环带崩
                    z_now = float('nan')
                npc['z_trace'].append((self._npc_tick_index, z_now))
                del npc['z_trace'][:-60]
            if not npc['server_alive']:
                continue  # 已死的 actor 不再驱动/不再发真值（发原点会把判据带偏）
            x, y, yaw = step_pose(*npc['pose'], *npc['cmd'], dt_s)
            npc['pose'] = (x, y, yaw)
            cx, cy, _ = position_to_carla(x, y, 0.0)
            location = self._carla.Location(x=cx, y=cy, z=0.2)
            # ⚠️ 瞬移走廊夹取（P9-S1 实锤）：航点约定允许「草地上空掉头」
            #    （P5 冻结基线），Gazebo 无碰撞道具没事；CARLA 物理开着
            #    （雷达可见性所需）时瞬移出路 = 穿墙 = PhysX 穿透解算把车
            #    弹到 4.8 km 高空（cast_ray 实测抓到的飞车）。判「在不在路上」
            #    问地图本身（project_to_road=False 路外返回 None），路外冻住
            #    **物理体**；pose_gt（真值/判据/npc_controller 的世界）照常走完
            #    剧本 —— 与 Gazebo 的语义差只剩「路外那截物理车不动」，
            #    而所有判据窗口都在路内。
            # 「在不在路上」→「离路近不近」（P9-S2 放宽）：行人的路肩路径
            # （P5 冻结基线）在车道外 —— 严格 on-lane 夹取把行人物理体冻在
            # 高空 spawn 点（实测行人 0% 的一半原因）。放宽为投影距离 ≤5 m：
            # 路肩/裙板可走，路尽头的草地端点（x=105 等）依旧冻住。
            projected = self._carla_map.get_waypoint(
                location, project_to_road=True,
                lane_type=self._carla.LaneType.Any)
            near_road = (
                projected is not None and
                location.distance(projected.transform.location) <= 5.0)
            # 车辆不受走廊夹取（物理车有墙兜底；冻控制会让真车失控滑行）。
            if near_road or not npc['is_walker']:
                # ⚠️ 统一理论（P9-S1/S2 九轮判别的终点）：PhysX **轮式**车辆
                #    「近期被 set_transform 过」的若干 tick 内对 gpu-lidar 隐形 ——
                #    dwell 静止（无瞬移）5-11k 点可见、移动（速度驱动跟不上 →
                #    每 2-3 拍 drift 复位 = 高频瞬移流）0 点隐形、裸测瞬移一次后
                #    静置可见；walker（运动学胶囊）免疫，每拍瞬移照常可见。
                #    且 set_target_velocity 实测不产生位移（裸测末位置不变）。
                #    ⟹ 车辆走 **apply_control 物理闭环**（ego 本身就是「移动 +
                #    可见」的存在性证明）：cmd_vel 经 ControlMapping 转油门刹车，
                #    转向按轴距反解；真值改发**物理实际**（_tick_npcs 末尾），
                #    判据/感知/npc_controller 对物理闭环，积分器只剩走廊夹取用。
                if npc['is_walker']:
                    npc['actor'].set_transform(self._carla.Transform(
                        location, self._carla.Rotation(yaw=yaw_to_carla(yaw))))
                else:
                    velocity = npc['actor'].get_velocity()
                    speed_mps = math.hypot(velocity.x, velocity.y)
                    accel_cmd = max(-3.0, min(1.5, 1.5 * (npc['cmd'][0] - speed_mps)))
                    # 转向：cmd 的 wz 与当前车速按自行车模型反解前轮角
                    #（δ = atan(wz·L/v)），低速夹到 0.5 m/s 防除零。
                    steer_rad = math.atan2(
                        npc['cmd'][2] * 2.7, max(speed_mps, 0.5))
                    fields = self._mapping.to_carla(
                        steer_rad, accel_cmd, speed_mps=speed_mps)
                    npc['actor'].apply_control(self._carla.VehicleControl(
                        throttle=fields['throttle'], brake=fields['brake'],
                        steer=fields['steer']))
            if not npc['is_walker']:
                # 真值 = 物理实际（apply_control 闭环后积分器不再是事实来源；
                # 报积分值会让判据在「感知看的车」与「真值说的车」之间量出
                # 纯粹的虚构误差 —— P5 检测率的隐形杀手之一）。
                actual = npc['actor'].get_transform()
                x, y, _ = position_to_ros(actual.location.x, actual.location.y, 0.0)
                yaw = yaw_to_ros(actual.rotation.yaw)
                npc['pose'] = (x, y, yaw)
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
            # twist 必须填（P8-S6 实测：漏填时判据读到 lead_v 恒 0，
            # 「前车驶离」永远判不出来）。语义与 gz 桥的 Odometry 一致：
            # child_frame（体系）下的速度 —— cmd 本来就是体系 twist，直传。
            gt.twist.twist.linear.x = npc['cmd'][0]
            gt.twist.twist.linear.y = npc['cmd'][1]
            gt.twist.twist.angular.z = npc['cmd'][2]
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
        if expired:
            fields = self._mapping.full_brake()
        else:
            # 车速给驻车闩锁用（0.9.x 的 get_velocity 读客户端每 tick 快照，无额外 RPC）
            v = self._ego.get_velocity()
            fields = self._mapping.to_carla(
                *self._last_cmd, speed_mps=math.hypot(v.x, v.y))
            # 油门**上升**速率限幅（P8-S6 实测）：bias 0.54 的断崖 × 20 FPS
            # ⟹ 小加速指令一拍打满 0.6+ 开度，弯道目标 3.9 实测冲到 4.6，
            # 轮胎侧滑外漂 —— S01 弯道 a_lat 3.07（判据 2.0）的机理本体。
            # 只限上升（2.0/s ⟹ 每 tick 0.1，断崖爬 ~5 拍）；收油与刹车
            # 不限 —— 减速是安全方向，必须立即。纯执行器域，不碰控制律。
            rise_cap = self._last_throttle + 2.0 * 0.05
            if fields['throttle'] > rise_cap:
                fields = dict(fields, throttle=rise_cap)
        self._last_throttle = fields['throttle']
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
        # ---- 位姿跳变滤波（S6 block 场景定案）-----------------------------
        # get_transform 在同步模式+负载下**偶发返回零/陈旧位姿**（PythonAPI
        # 一致性 issue #3 家族）。一个坏样本进 TF ⟹ 规划起点瞬移 10-30 m ⟹
        # 投影越过障碍物、「绕不过去」翻成「正常」一拍，车顶进锥桶（实测
        # 119 碰撞拍）；S01 的横向 rms 劣化同源。单拍位移超过物理上限
        # （8 m/s × 0.05 s × 5 倍余量 = 2 m）即丢样保持上一拍，计数进日志。
        if not hasattr(self, '_last_good_xy'):
            self._last_good_xy = (x, y)
            self._pose_rejects = 0
        jump = math.hypot(x - self._last_good_xy[0], y - self._last_good_xy[1])
        if jump > 2.0:
            self._pose_rejects += 1
            self.get_logger().warn(
                f'丢弃跳变位姿样本（单拍位移 {jump:.1f} m，累计 {self._pose_rejects}）',
                throttle_duration_sec=5.0)
            return
        self._last_good_xy = (x, y)
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

        # ⚠️ dt 用**实际流逝的仿真时间**，不能用 1/tick_hz（P8-S6 实测）：
        #    仿真钟粒度 = 同步步长 0.05 s（20 更新/秒），50 Hz 的 ROS 定时器
        #    每次钟更新至多触发一次 → 实际 20 Hz，dt 填 1/50 ⟹ NPC 全员
        #    慢 2.5 倍（lead 3.0 → 实测 1.2 m/s），行人在道内滞留超窗、
        #    「驶离」滑出记录窗，三个行为场景的相位账全部错位。
        now_ns = self.get_clock().now().nanoseconds
        if self._last_npc_tick_ns is not None:
            npc_dt_s = min(max((now_ns - self._last_npc_tick_ns) * 1e-9, 0.0), 0.2)
        else:
            npc_dt_s = 0.0
        self._last_npc_tick_ns = now_ns
        self._tick_npcs(npc_dt_s, stamp)
        # 每 200 拍（20 Hz 下 10 s）一次服务器侧普查 —— get_actors 是 RPC，
        # 0.1 Hz 的代价可忽略；首拍也查一次，拿到「出生即在」的基线。
        if self._npcs and (self._npc_tick_index == 1 or self._npc_tick_index % 200 == 0):
            self._npc_census(stamp)
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
    # 120 s：generate_opendrive_world 从重量级 Town 切园区实测可超 30 s
    # （S6 首跑在这儿超时崩过）。加载是一次性的，宽限时不掩盖任何持续性故障。
    client.set_timeout(120.0)

    node = CarlaSidecarNode(carla, client)

    # SIGTERM 也要走清理（pkill/launch 收进程都是 TERM）——不装这个钩子，
    # finally 只在 SIGINT 路径执行，TERM 直接终止、世界留在同步模式 = 埋雷。
    import signal

    def _on_term(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_term)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ⚠️ 必须把世界还原成异步再走 —— 同步模式的世界没了节拍器，
        #    CARLA 会**永远停在 tick 栅栏上**，之后所有 RPC（包括重载世界）
        #    一律超时，看起来像服务器挂了。S6 首跑实测踩过：唯一解法是
        #    重启服务器。清理钩子把这个坑焊死。
        node._ticking = False
        try:
            settings = node._world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            node._world.apply_settings(settings)
        except RuntimeError:
            pass  # 服务器已经没了就算了 —— 别让清理挡住退出
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
