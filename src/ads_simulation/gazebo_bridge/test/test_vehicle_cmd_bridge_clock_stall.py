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
Sim-clock stall guard: when /clock stops, vehicle_cmd_bridge must command zero speed.

守着异常注入清单（docs/fault_injection.md #14）的一条洞：桥的看门狗与积分定时器都跑在
仿真钟上，/clock 一停两者一起冻住 —— 既判不出失联也发不出刹车，Gazebo 插件带着最后
一条速度指令一直开（陷阱表「仿真钟停走时控制器冻住而不降级」，2026-08 实测过）。
P9-S5b 给桥加了一只**墙钟**守卫：仿真钟连续 clock_stall_s（墙钟）没走 ⟹ 发零速。

本测试自己当 /clock 的来源（桥 use_sim_time=true）：先按 100 Hz 发钟 + 有效指令把
速度设定值抬起来，然后**停发 /clock**（指令照发 —— 只坏钟，不坏指令，两个变量分开），
断言桥在 clock_stall_s + 余量（墙钟）内输出零速并保持；钟恢复后能继续跟指令。

## 故障注入实测（2026-08-16，写完立刻做的）

| 注入 | 结果 |
|---|---|
| 桥里不建 stall_timer（复原缺陷） | **红**：停钟后桥再没有任何输出（积分定时器冻住），
  最后一条 /gazebo/cmd_vel 仍是 1.0 m/s —— 正是被守的「带着最后指令一直开」 |
"""

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
from rosgraph_msgs.msg import Clock

CLOCK_STALL_S = 1.0   # 与桥的默认参数同步；断言里与余量相加，写死便于评审


@pytest.mark.launch_test
def generate_test_description():
    """Launch only vehicle_cmd_bridge on sim time; the test publishes /clock."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gazebo_bridge',
            executable='vehicle_cmd_bridge',
            name='vehicle_cmd_bridge',
            parameters=[{
                'geometry.wheelbase_m': 2.7,
                'limits.max_steer_angle_rad': 0.6,
                'limits.max_speed_mps': 8.333,
                'limits.max_accel_mps2': 1.5,
                'limits.max_decel_mps2': 3.0,
                'limits.emergency_decel_mps2': 5.0,
                'cmd_timeout_s': 0.5,
                'clock_stall_s': CLOCK_STALL_S,
                'use_sim_time': True,
            }],
            output='screen',
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestClockStallStopsTheCar(unittest.TestCase):
    """Drive on a healthy clock, freeze the clock, demand zero speed; unfreeze, demand recovery."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('clock_stall_tester')   # 墙钟节点：它就是钟的来源
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.cmd_pub = self.node.create_publisher(VehicleCmd, '/vehicle_cmd', 10)
        self.outputs = []   # (墙钟单调秒, linear.x)
        self.node.create_subscription(
            Twist, '/gazebo/cmd_vel',
            lambda m: self.outputs.append((time.monotonic(), m.linear.x)), 50)
        self.sim_t = 0.0

    def tearDown(self):
        self.node.destroy_node()

    def _run(self, duration_s, tick_clock, accel=1.5, until_speed_above=None):
        """
        Spin for duration_s (wall): cmd at 50 Hz, and /clock at 100 Hz if tick_clock.

        until_speed_above：先跑到输出速度超过它（前提建立，轮询而不是固定窗口 —— DDS 发现
        期不算在窗口里，见 watchdog 用例同一条复审记录），再多跑 duration_s。
        """
        if until_speed_above is not None:
            premise_deadline = time.monotonic() + 8.0
            while time.monotonic() < premise_deadline:
                self._run(0.1, tick_clock, accel)
                if self.outputs and self.outputs[-1][1] > until_speed_above:
                    break
        deadline = time.monotonic() + duration_s
        next_clock = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if tick_clock and now >= next_clock:
                self.sim_t += 0.01
                msg = Clock()
                msg.clock.sec = int(self.sim_t)
                msg.clock.nanosec = int((self.sim_t % 1.0) * 1e9)
                self.clock_pub.publish(msg)
                next_clock = now + 0.01
            cmd = VehicleCmd()
            cmd.steer_angle_rad = 0.0
            cmd.accel_mps2 = accel
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def test_clock_stall_forces_zero_speed_then_recovers(self):
        # ---- 阶段一：钟正常，指令有效，速度设定值抬起来（无 /odom ⟹ 钳在 1.0） ----
        self._run(0.5, tick_clock=True, until_speed_above=0.5)
        self.assertTrue(self.outputs, '桥没有任何输出 —— 节点起来了吗？/clock 有没有到？')
        speed_before = self.outputs[-1][1]
        self.assertGreater(speed_before, 0.5, '钟正常时速度设定值没抬起来，测试前提不成立')

        # ---- 阶段二：只停钟，指令照发 ------------------------------------------
        t_stall_wall = time.monotonic()
        self.outputs.clear()
        self._run(CLOCK_STALL_S + 1.5, tick_clock=False)
        self.assertTrue(
            self.outputs,
            '停钟后桥再没有任何输出 —— 积分定时器冻住而没有墙钟守卫接管，'
            '车会带着最后一条速度指令一直开（正是被守的缺陷）')
        zeros = [o for o in self.outputs if abs(o[1]) < 1e-6]
        self.assertTrue(
            zeros,
            f'停钟 {CLOCK_STALL_S + 1.5:.1f} s（墙钟）内桥从未发出零速；末拍 {self.outputs[-1][1]:.2f}')
        latency_s = zeros[0][0] - t_stall_wall
        # 余量 1.0：守卫定时器粒度 0.2 + 单次调度抖动，实测 1.38 s（0.5 余量只剩 0.12，
        # 全量并行时会越线 —— 复审）。被守的缺陷形态是「停钟后**完全没有**输出/永不归零」，
        # 放到 1.0 不失区分力。
        self.assertLess(
            latency_s, CLOCK_STALL_S + 1.0,
            f'零速来得太慢：停钟后 {latency_s:.2f} s（阈值 {CLOCK_STALL_S} + 余量 1.0）')
        self.assertLess(abs(self.outputs[-1][1]), 1e-6, '零速没有保持到停钟阶段末尾')

        # ---- 阶段三：钟恢复，车应当重新跟指令 -------------------------------------
        self.outputs.clear()
        self._run(2.0, tick_clock=True)
        self.assertTrue(self.outputs and self.outputs[-1][1] > 0.5,
                        f'钟恢复后速度设定值没有回来：{self.outputs[-1][1] if self.outputs else None}')
        print(f'[test] 停钟前 {speed_before:.2f} m/s → 停钟 {latency_s:.2f} s 后零速 → '
              f'钟恢复后 {self.outputs[-1][1]:.2f} m/s')


@launch_testing.post_shutdown_test()
class TestBridgeExitsCleanly(unittest.TestCase):
    """The node must not crash on shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
