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

"""
Closed-loop wiring test with no GPU: fake vehicle + map_node + control_node.

L3-G（SPEC §8）：**不需要 Gazebo、不需要 GPU、能进 CI** 的端到端闭环。

它验的是**节点接线**，不是控制律，也不是真物理：
    话题名对不对、QoS 兼不兼容、TF 链通不通、参数装没装上、时序对不对。

    L1 单元测试（S2/S3）  验控制律：毫秒级，不起 ROS
    **本测试（S5）**       验接线：不要 GPU，十几秒，进 CI
    CP-P2-B（S4）         验真物理：要 Gazebo + GPU + 人

⚠️ **不要用它替代 CP-P2-B。** 假车没有轮胎、没有质量惯量、转向瞬时生效 ——
   CP-P2-B 发现的那个「转向执行机构 1.2 s 滞后」在这里**结构上不可能出现**
   （见 docs/modules/control.md §3.9）。这一层全绿只说明线接对了。

**为什么值得有这一层**：`/route/path` 的 QoS 从 P1 一直错到 P2-S4 都没人发现 ——
`transient_local` 漏了，而 P1 唯一的订阅者是 RViz，人总是先起 RViz 再点目标点，
所以永远碰不到。控制节点晚起就永远收不到路径，症状是「车不动」。
本测试**先起全部节点、再发目标点**，正是那个顺序；QoS 一旦改回 volatile 就红。
"""

import os
from pathlib import Path
import time
import unittest

from ads_msgs.msg import VehicleCmd
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

# 自车起点：与 worlds/campus_loop.sdf 的 spawn 一致（南侧道路东行车道）。
# 用同一个起点是有意的 —— 这样本测试和 CP-P2-B 跑的是同一条路的同一段，
# 两者的数出现分歧时能直接对比。
START_X_M = 30.0
START_Y_M = -51.75
START_YAW_RAD = 0.0

# 目标点：东侧道路上，绕过东南角。约 95 m，包含一个 R≈13.75 的环线弯角。
# **刻意不选一条纯直路** —— 直路上横向误差恒为 0，投影、曲率、限速全都测不到。
GOAL_X_M = 91.75
GOAL_Y_M = -20.0

# 假车相对墙钟的倍率。3 倍下 ~20 s 仿真 ≈ 7 s 墙钟。
REAL_TIME_FACTOR = 3.0


def _control_params() -> dict:
    """
    Collect control_node parameters from the two YAML files.

    与 `stack.launch.py` 的 `control_node_params()` 同一套。
    这里**重新读一遍**而不是 import stack.launch.py：本包不依赖 ads_bringup，
    反过来依赖会形成环（ads_bringup 已经 exec_depend 了 ads_control）。
    代价是这段搬运逻辑有两份 —— 但它没有算法，只是键名映射，
    而且一旦漂移，control_node 的构造函数会指名报错（参数默认值全是 0）。

    :return: 传给 control_node 的参数字典
    """
    share = Path(get_package_share_directory('ads_control')) / 'config'
    vehicle = yaml.safe_load((share / 'vehicle_params.yaml').read_text(encoding='utf-8'))
    control = yaml.safe_load((share / 'control_params.yaml').read_text(encoding='utf-8'))
    lim = vehicle['limits']
    return {
        'geometry.wheelbase_m': vehicle['geometry']['wheelbase_m'],
        'limits.max_steer_angle_rad': lim['max_steer_angle_rad'],
        'limits.max_steer_rate_rad_s': lim['max_steer_rate_rad_s'],
        'limits.max_accel_mps2': lim['max_accel_mps2'],
        'limits.max_decel_mps2': lim['max_decel_mps2'],
        'lateral.gain': control['lateral']['gain'],
        'lateral.soft_speed_mps': control['lateral']['soft_speed_mps'],
        'lateral.search_window': control['lateral']['search_window'],
        'longitudinal.kp': control['longitudinal']['kp'],
        'longitudinal.ki': control['longitudinal']['ki'],
        'goal.stop_distance_m': control['goal']['stop_distance_m'],
        'safety.max_lateral_error_m': control['safety']['max_lateral_error_m'],
        'safety.odom_timeout_s': control['safety']['odom_timeout_s'],
        'control_rate_hz': control['control_rate_hz'],
        'use_sim_time': True,
    }


def _planning_params() -> dict:
    """
    Collect planning_node parameters from the two YAML files.

    ⚠️ **P3-S4 起 L3-G 的链路里多了 planning_node。**
    control_node 不再直接订阅 /route/path，而是吃 /planning/trajectory ——
    不把规划器拉进来的话，控制器永远收不到轨迹，闭环测试会以
    「车一直不动」的形式失败，而根因看起来像控制器坏了。

    与 `stack.launch.py` 的 `planning_node_params()` 同一套，理由见 `_control_params`。

    :return: 传给 planning_node 的参数字典
    """
    share = Path(get_package_share_directory('ads_planning')) / 'config'
    vehicle = yaml.safe_load((share / 'vehicle_params.yaml').read_text(encoding='utf-8'))
    planning = yaml.safe_load((share / 'planning_params.yaml').read_text(encoding='utf-8'))
    geo = vehicle['geometry']
    lim = vehicle['limits']
    return {
        'lateral.max_offset_m': planning['lateral']['max_offset_m'],
        'lateral.offset_step_m': planning['lateral']['offset_step_m'],
        'longitudinal.min_horizon_m': planning['longitudinal']['min_horizon_m'],
        'longitudinal.max_horizon_m': planning['longitudinal']['max_horizon_m'],
        'longitudinal.horizon_step_m': planning['longitudinal']['horizon_step_m'],
        'trajectory.resample_step_m': planning['trajectory']['resample_step_m'],
        'safety.margin_m': planning['safety']['margin_m'],
        'safety.stop_margin_m': planning['safety']['stop_margin_m'],
        'cost.weight_offset': planning['cost']['weight_offset'],
        'cost.weight_curvature': planning['cost']['weight_curvature'],
        'cost.weight_clearance': planning['cost']['weight_clearance'],
        'cost.weight_consistency': planning['cost']['weight_consistency'],
        'vehicle.length_m': geo['length_m'],
        'vehicle.width_m': geo['width_m'],
        'vehicle.rear_overhang_m': geo['rear_overhang_m'],
        'speed.cruise_speed_mps': lim['cruise_speed_mps'],
        'speed.max_lateral_accel_mps2': planning['speed']['max_lateral_accel_mps2'],
        'speed.max_accel_mps2': lim['max_accel_mps2'],
        'speed.max_decel_mps2': lim['max_decel_mps2'],
        'use_sim_time': True,
    }


def _vehicle_params() -> dict:
    share = Path(get_package_share_directory('ads_control')) / 'config'
    vehicle = yaml.safe_load((share / 'vehicle_params.yaml').read_text(encoding='utf-8'))
    return {
        'geometry.wheelbase_m': vehicle['geometry']['wheelbase_m'],
        'limits.max_steer_angle_rad': vehicle['limits']['max_steer_angle_rad'],
        'limits.max_speed_mps': vehicle['limits']['max_speed_mps'],
        'initial.x_m': START_X_M,
        'initial.y_m': START_Y_M,
        'initial.heading_rad': START_YAW_RAD,
        'real_time_factor': REAL_TIME_FACTOR,
    }


@pytest.mark.launch_test
def generate_test_description():
    """起假车 + map→odom 静态 TF + map_node + control_node."""
    fake_vehicle = os.environ['FAKE_VEHICLE_EXECUTABLE']

    return launch.LaunchDescription([
        # 假车。**它是 /clock 的来源**，所以自己 use_sim_time=false
        # （由节点的 parameter_overrides 强制，launch 改不动）。
        launch.actions.ExecuteProcess(
            cmd=[fake_vehicle, '--ros-args'] + [
                arg
                for key, value in _vehicle_params().items()
                for arg in ('-p', f'{key}:={value}')
            ],
            output='screen'),

        # map → odom 单位变换。
        #
        # ⚠️ 真实栈里这一段**不是单位变换** —— Gazebo 把 odom 原点放在自车 spawn 处，
        #    所以 gazebo_sim.launch.py 要从世界文件读出 spawn 位姿再发（P1-S4 实测）。
        #    这里是单位变换，因为假车直接在 **map 坐标**里积分，odom 原点就是 map 原点。
        #    **别把这里的"单位变换没问题"当成真实栈也可以** —— 那个坑踩过一次：
        #    车在 RViz 里画到园区正中央的草地上，而 Gazebo 里它好端端停在路上。
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': True}], output='screen'),

        Node(package='ads_map', executable='map_node', name='map_node',
             parameters=[{'use_sim_time': True}], output='screen'),

        Node(package='ads_planning', executable='planning_node', name='planning_node',
             parameters=[_planning_params()], output='screen'),

        Node(package='ads_control', executable='control_node', name='control_node',
             parameters=[_control_params()], output='screen'),

        launch_testing.actions.ReadyToTest(),
    ])


class TestClosedLoop(unittest.TestCase):
    """Drive one A-to-B run and check that the wiring holds up."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('closed_loop_tester')
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.samples = []
        self.commands = 0

        # ⚠️ 订阅端 QoS 必须与 map_node 的发布端一致（transient_local）。
        #    这也正是本测试要守的那条 bug —— 见文件头。
        self.goal_pub = self.node.create_publisher(
            PoseStamped, '/goal_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(
            DiagnosticArray, '/control/diagnostics', self._on_diag, 200)
        self.node.create_subscription(
            VehicleCmd, '/vehicle_cmd', lambda _msg: setattr(self, 'commands', self.commands + 1),
            10)

    def tearDown(self):
        self.node.destroy_node()

    def _on_diag(self, msg):
        values = {kv.key: kv.value for kv in msg.status[0].values}
        self.samples.append(values)

    def _spin(self, wall_seconds):
        deadline = time.monotonic() + wall_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_drives_to_the_goal(self):
        # 等节点都起来、时钟开始走。**目标点在这之后才发** ——
        # 这正是 P1 那个 QoS bug 的触发顺序（先起订阅者，后发布）。
        self._spin(4.0)
        self.assertGreater(len(self.samples), 0, '没收到 /control/diagnostics —— control_node 起来了吗？')

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = GOAL_X_M
        goal.pose.position.y = GOAL_Y_M
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        # 约 95 m / 5 m/s ≈ 20 s 仿真；3 倍速 → 约 7 s 墙钟。给 40 s 余量。
        deadline = time.monotonic() + 40.0
        reached = False
        while time.monotonic() < deadline and not reached:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.samples and self.samples[-1].get('state') == 'GOAL_REACHED':
                if abs(float(self.samples[-1]['measured_speed_mps'])) < 0.05:
                    reached = True

        tracking = [s for s in self.samples if s.get('state') in ('TRACKING', 'GOAL_REACHED')]
        states = {s.get('state') for s in self.samples}
        self.assertTrue(
            reached,
            f'没能到达终点并停住。经历过的状态：{states}，'
            f'共 {len(self.samples)} 拍、其中跟踪 {len(tracking)} 拍')

        # ---- 接线层面的断言 ----
        # 只查"线接对了"，**不查控制精度** —— 精度是 CP-P2-B 的事，
        # 而假车没有轮胎，在这里定一个紧判据等于给自己一个虚假的安全感。
        self.assertGreater(self.commands, 100, '/vehicle_cmd 发得太少，控制回调是不是没在跑')

        errors = [abs(float(s['lateral_error_m'])) for s in tracking]
        max_error = max(errors)
        # 1.5 m = safety.max_lateral_error_m。超过它控制器自己就会转 OFF_PATH，
        # 所以这条断言的意思是「全程没有触发偏离保护」，而不是精度考核。
        self.assertLess(max_error, 1.5, f'横向误差 {max_error:.3f} m 触到了偏离保护阈值')

        goal_distance = float(tracking[-1]['goal_distance_m'])
        self.assertLess(goal_distance, 2.0, f'停在离终点 {goal_distance:.2f} m 的地方')

        print(
            f'\n  [L3-G] {len(self.samples)} 拍，指令 {self.commands} 条，'
            f'最大横向误差 {max_error:.4f} m，终点距离 {goal_distance:.3f} m')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """
    Require every launched process to exit cleanly.

    ⚠️ 崩溃会被 launch 报成 "process has finished cleanly"（本仓库踩过，见
    CLAUDE.md 陷阱表）—— 唯一可信的证据是**退出码**，所以这条断言不可省。
    """

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
