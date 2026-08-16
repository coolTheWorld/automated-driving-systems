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
Watchdog behavior test for vehicle_cmd_bridge — no Gazebo needed.

守着一个已经确认过的缺陷（2026-08-12 复检发现）：喂狗曾在 isfinite 校验
**之前**，于是上游持续发 NaN 时（本仓库已实际发生过两次的故障形态），
每条被丢弃的坏指令都在刷新看门狗，0.5 s 超时永不触发，
车以**闩存的旧指令**一直开 —— 恰是看门狗要防止的后果。

⚠️ check_vehicle_cmd.py 的 NaN 阶段抓不到它：那里只断言「输出无 NaN」，
   而 NaN 流期间 bridge 发的是闩存的**有限**值，断言恒过。
   要抓它必须断言**语义**：持续 NaN 流 = 没有任何有效指令 = 失联 = 刹停。

节点用墙钟（不设 use_sim_time），所以本测试不需要 /clock —— 真实时间即可。

## 故障注入实测（2026-08-12，写完立刻做的）

| 注入 | 结果 |
|---|---|
| 把喂狗挪回 on_cmd 第一行（复原缺陷） | **红**：NaN 流 1.4 s 后 linear.x 仍为 1.00 |
"""

import math
import time
import unittest

from ads_msgs.msg import VehicleCmd
from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node as RclpyNode


@pytest.mark.launch_test
def generate_test_description():
    """Launch only vehicle_cmd_bridge with realistic limits."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_bridge',
            executable='vehicle_cmd_bridge',
            name='vehicle_cmd_bridge',
            parameters=[{
                # 取值与 config/vehicle_params.yaml 同量级即可 —— 本测试验的是
                # 看门狗语义，不是限幅数值（那归 check_vehicle_cmd.py）。
                'geometry.wheelbase_m': 2.7,
                'limits.max_steer_angle_rad': 0.6,
                'limits.max_speed_mps': 8.333,
                'limits.max_accel_mps2': 1.5,
                'limits.max_decel_mps2': 3.0,
                'limits.emergency_decel_mps2': 5.0,
                'cmd_timeout_s': 0.5,
            }],
            output='screen',
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestWatchdogVsNanStream(unittest.TestCase):
    """A continuous NaN stream must be treated as loss of contact, not as contact."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('watchdog_tester')
        self.cmd_pub = self.node.create_publisher(VehicleCmd, '/vehicle_cmd', 10)
        self.outputs = []
        self.node.create_subscription(Twist, '/gazebo/cmd_vel', self.outputs.append, 50)

    def tearDown(self):
        self.node.destroy_node()

    def _drive(self, duration_s, accel, steer=0.0):
        """Publish commands at 50 Hz for duration_s while spinning."""
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            cmd = VehicleCmd()
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd.steer_angle_rad = steer
            cmd.accel_mps2 = accel
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_nan_stream_triggers_watchdog_brake(self):
        """Valid drive → continuous NaN stream → the bridge must brake to zero."""
        # ---- 阶段一：有效指令，把速度设定值抬起来 --------------------------
        # 没有 /odom ⟹ measured=0 ⟹ 设定值被 lead cap 钳在 1.0 m/s —— 正好，
        # 一个确定的非零值便于断言。
        self._drive(1.2, accel=1.5)
        self.assertTrue(self.outputs, '桥没有任何输出 —— 节点起来了吗？')
        speed_before = self.outputs[-1].linear.x
        self.assertGreater(speed_before, 0.5, '有效指令没把速度设定值抬起来，测试前提不成立')

        # ---- 阶段二：持续 NaN 流 ------------------------------------------
        # 语义：没有任何**有效**指令到达 = 失联。看门狗 0.5 s 超时 +
        # max_decel 3.0 刹掉 1.0 m/s 约 0.33 s ⟹ 0.9 s 内应当归零；
        # 给 1.4 s 是为了在慢 CI 机器上不 flake。
        self.outputs.clear()
        self._drive(1.4, accel=math.nan, steer=math.nan)

        self.assertTrue(self.outputs, 'NaN 流期间桥停止了输出 —— 应当继续发（刹车）指令')
        # 全程无 NaN（这是 check_vehicle_cmd.py 已有的性质，这里顺带守着）
        for twist in self.outputs:
            self.assertTrue(
                math.isfinite(twist.linear.x) and math.isfinite(twist.angular.z),
                'NaN 泄漏到了输出')
        final_speed = self.outputs[-1].linear.x
        print(f'[test] 阶段一末速 {speed_before:.2f} → NaN 流 1.4 s 后 {final_speed:.2f} m/s')
        self.assertLess(
            final_speed, 0.05,
            f'持续 NaN 流 1.4 s 后速度设定值仍为 {final_speed:.2f} —— '
            '被丢弃的指令在喂狗，看门狗永不触发（正是 2026-08-12 确认的缺陷）')

    def test_command_silence_triggers_watchdog_brake(self):
        """Valid drive → no commands at all → the bridge must brake to zero (清单 #9)."""
        # 与 NaN 流那条是同一只看门狗的两张脸：那条守「坏指令不算指令」，
        # 这条守「没有指令」本身 —— teleop 崩了 / ssh 断了 / 控制节点死了的形态。
        # verify_teleop.sh 在真 Gazebo 上量过（6.3 s 后 8.33→0），这里让它进 CI。
        self._drive(1.2, accel=1.5)
        self.assertTrue(self.outputs, '桥没有任何输出 —— 节点起来了吗？')
        speed_before = self.outputs[-1].linear.x
        self.assertGreater(speed_before, 0.5, '有效指令没把速度设定值抬起来，测试前提不成立')

        # 静默：只 spin，不发任何指令
        self.outputs.clear()
        deadline = time.monotonic() + 1.4
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(self.outputs, '静默期间桥停止了输出 —— 应当继续发（刹车）指令')
        final_speed = self.outputs[-1].linear.x
        print(f'[test] 阶段一末速 {speed_before:.2f} → 指令静默 1.4 s 后 {final_speed:.2f} m/s')
        self.assertLess(
            final_speed, 0.05,
            f'指令静默 1.4 s 后速度设定值仍为 {final_speed:.2f} —— 看门狗没有刹车')


@launch_testing.post_shutdown_test()
class TestBridgeExitsCleanly(unittest.TestCase):
    """The node must not crash on shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
