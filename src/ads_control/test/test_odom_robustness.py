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
Bad-input robustness: a NaN /odom must not kill control_node.

守着一个已确认的缺陷（2026-08-12 复检）：on_odom 原来不做 isfinite 校验，
一条含 NaN 车速的消息在下一拍进到 stanley_->update()，lib 的 RequireFinite
抛异常，穿出定时器回调被 main 接住 → FATAL + exit(1) —— **整个控制节点死掉**，
而不是丢弃坏样本、靠里程计超时降级刹停。

lib 抛异常是**设计**（test_stanley 用 HUGE_VAL 钉死了这个行为）——
它的前提是调用方接住。本测试守的就是"调用方确实接住了"这半边。

⚠️ 只起 control_node 一个节点、用墙钟（覆盖掉 params 里的 use_sim_time）：
   这里验的是「坏输入不死」，不是控制质量 —— 那归 L3-G 闭环与 CP-P2-B。

## 故障注入实测（2026-08-12，写完立刻做的）

| 注入 | 结果 |
|---|---|
| 去掉 on_odom 的 isfinite 校验（复原缺陷） | **红**：NaN 之后诊断停发，节点已死 |
"""

import math
import os
import sys
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.node import Node as RclpyNode

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from closed_loop_common import control_params  # noqa: E402,I100


@pytest.mark.launch_test
def generate_test_description():
    """Launch only control_node, on the wall clock."""
    return launch.LaunchDescription([
        Node(
            package='ads_control', executable='control_node', name='control_node',
            # 后一个字典覆盖前一个：沿用闭环测试的全套参数，只把时钟换成墙钟
            # （本测试不发 /clock，仿真钟下定时器永远不走）。
            parameters=[control_params(), {'use_sim_time': False}],
            output='screen'),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestNanOdomDoesNotKillTheNode(unittest.TestCase):
    """Feed NaN odometry and assert the node keeps ticking."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('odom_robustness_tester')
        self.odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        self.diag_stamps = []
        self.node.create_subscription(
            DiagnosticArray, '/control/diagnostics',
            lambda m: self.diag_stamps.append(time.monotonic()), 50)

    def tearDown(self):
        self.node.destroy_node()

    def _spin_s(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_nan_odom_is_dropped_not_fatal(self):
        """Precondition: diagnostics flowing. Then NaN storm. Then: still flowing."""
        # ---- 前提：节点活着（诊断在流）。没有这一步，后面的断言毫无意义 ——
        #      「NaN 后没诊断」与「节点根本没起来」在表上长得一模一样。
        self._spin_s(3.0)
        self.assertGreater(len(self.diag_stamps), 5, '前提不成立：诊断没在流，节点起来了吗？')

        # ---- NaN 风暴：车速与横摆角速度轮流各种非有限值 ----------------------
        for bad in (math.nan, math.inf, -math.inf):
            for _ in range(10):
                msg = Odometry()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.twist.twist.linear.x = bad
                msg.twist.twist.angular.z = bad
                self.odom_pub.publish(msg)
                rclpy.spin_once(self.node, timeout_sec=0.01)

        # ---- 断言：节点还在跳（坏样本被丢弃，而不是抛异常把节点炸掉）--------
        self.diag_stamps.clear()
        self._spin_s(2.0)
        print(f'[test] NaN 风暴后 2 s 内诊断 {len(self.diag_stamps)} 拍')
        self.assertGreater(
            len(self.diag_stamps), 5,
            'NaN /odom 之后诊断停发 —— 节点被 lib 的异常炸死了（正是 2026-08-12 确认的缺陷）')


@launch_testing.post_shutdown_test()
class TestControlNodeExitsCleanly(unittest.TestCase):
    """The node must not have crashed."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
