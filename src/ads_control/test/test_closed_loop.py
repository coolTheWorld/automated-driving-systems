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
import sys
import time
import unittest

from ads_msgs.msg import VehicleCmd
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

# ⚠️ **同目录的模块必须先把这个目录加进 sys.path。** 实测过：去掉这两行，
#    launch 测试直接 error（launch_test 按路径执行本文件，不把它所在目录入栈）。
#    因此下面那行 import 必然在代码之后 —— E402/I100 是冲着这种写法来的，
#    这里是**必要的例外**，所以显式 noqa 而不是去改 lint 配置
#    （改配置会让全仓库都豁免，那就真的会有人乱放 import 了）。
#
#    用 sys.path 而不是 importlib 按路径加载：后者对含 @dataclass 的模块
#    要先 sys.modules[spec.name] = module 才行（见 CLAUDE.md 陷阱表），
#    而报错完全不提根因。能不走那条路就别走。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from closed_loop_common import (  # noqa: E402,I100
    control_params, GOAL_X_M, GOAL_Y_M, planning_params, vehicle_params)


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
                for key, value in vehicle_params().items()
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
             parameters=[planning_params()], output='screen'),

        Node(package='ads_control', executable='control_node', name='control_node',
             parameters=[control_params()], output='screen'),

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
