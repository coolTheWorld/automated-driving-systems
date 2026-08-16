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
Sensor dropouts must degrade the localization state machine on time — and never freeze it.

异常注入清单（docs/fault_injection.md）#11 的「超时分支」：
  雷达在仿真 12.0 s 断流 ⟹ NDT 1.0 s 无成功帧 ⟹ 降出 NDT_AIDED（进 GNSS_ONLY）；
  GNSS 在仿真 17.0 s 也断 ⟹ 2.0 s 超时 ⟹ DEAD_RECKONING；
  全程 IMU/轮速照发，map→odom 必须**继续发布**（真车上传感器坏了车还在动，
  滤波器要在航位推算上继续出位姿，不能冻住）。

这条守着一个已经发生过的缺陷（2026-08-12 复检）：`ndt_ok_` 原来是**无超时的锁存量**，
只在下一帧 NDT 跑过之后才可能翻 false —— 雷达断流时**根本没有下一帧**，状态机永远停在
NDT_AIDED、诊断谎报正常，实际早已在纯航位推算上漂。修复加了 1.0 s 超时，但一直没有用例
守着它，本文件补上。

判据带的时间余量：诊断 1 Hz 发布 ⟹ 观测滞后 ≤1 s；断流后允许 timeout + 1.5 s 才要求
状态已翻，之后**每一拍**都必须是降级态（不许翻回去 —— 没有新帧就没有理由翻回去）。

## 故障注入实测（2026-08-16，写完立刻做的）

| 注入 | 结果 |
|---|---|
| localization_node 的 NDT 超时 1.0 → 1e9（复原 2026-08-12 之前的锁存缺陷） | 红：雷达断流后 15/16/17 s 三拍
  仍报 NDT_AIDED，判据 ② 抓住（基线：12 s 断流 → 14.0 s 首个 GNSS_ONLY；17 s GNSS 断 → 19.0 s DEAD_RECKONING；
  降级期间位姿 400 + 500 条） |
"""

import os
import sys
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node as RclpyNode

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from localization_fixtures import fake_sensors_params, localization_params  # noqa: E402,I100

# 注入时刻（仿真秒）。雷达先断、GNSS 后断，两段降级各自看得见。
LIDAR_STOP_S = 12.0
GNSS_STOP_S = 17.0
# 节点里的两个超时（localization_node.cpp UpdateState：NDT 1.0 s 硬编码；GNSS gnss_timeout_s 2.0）
NDT_TIMEOUT_S = 1.0
GNSS_TIMEOUT_S = 2.0
# 诊断 1 Hz ⟹ 观测滞后 ≤1 s，再留 0.5 s 调度余量。
OBSERVE_SLACK_S = 1.5
RUN_UNTIL_S = 23.0


@pytest.mark.launch_test
def generate_test_description():
    """Launch fake sensors (with dropout switches) plus localization_node."""
    fake_sensors = os.environ['FAKE_SENSORS_EXECUTABLE']
    params = fake_sensors_params()
    params['fault.lidar_stop_after_s'] = LIDAR_STOP_S
    params['fault.gnss_stop_after_s'] = GNSS_STOP_S
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[fake_sensors, '--ros-args'] + [
                arg for key, value in params.items() for arg in ('-p', f'{key}:={value}')],
            output='screen'),
        Node(package='ads_localization', executable='localization_node',
             name='localization_node', parameters=[localization_params()], output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSensorTimeoutDegrades(unittest.TestCase):
    """Lidar dropout leaves NDT_AIDED in time; GNSS dropout reaches DEAD_RECKONING; poses go on."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('localization_timeout_tester')
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.diagnostics = []   # (sim_t, state)
        self.pose_stamps = []   # sim_t of every /localization/pose
        self.node.create_subscription(
            DiagnosticArray, '/localization/diagnostics', self._on_diag, 50)
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/localization/pose', self._on_pose, 50)

    def tearDown(self):
        self.node.destroy_node()

    def _on_diag(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.diagnostics.append((t, msg.status[0].message))

    def _on_pose(self, msg):
        self.pose_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def test_dropouts_degrade_but_do_not_freeze(self):
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.diagnostics and self.diagnostics[-1][0] >= RUN_UNTIL_S:
                break
        self.assertTrue(self.diagnostics, '一条 /localization/diagnostics 都没收到')
        self.assertGreaterEqual(
            self.diagnostics[-1][0], RUN_UNTIL_S,
            f'90 s 墙钟内仿真钟只走到 {self.diagnostics[-1][0]:.1f} s —— 假传感器太慢？')

        # ---- ① 断流之前必须真的锁上过 NDT，否则后面的「降出」无从谈起 ----
        before = [s for t, s in self.diagnostics if t < LIDAR_STOP_S]
        self.assertIn('NDT_AIDED', before, f'雷达断流前从未进入 NDT_AIDED：{before}')

        # ---- ② 雷达断流 ⟹ timeout+slack 之后每一拍都不再是 NDT_AIDED ----
        after_lidar = [(t, s) for t, s in self.diagnostics
                       if LIDAR_STOP_S + NDT_TIMEOUT_S + OBSERVE_SLACK_S <= t < GNSS_STOP_S]
        self.assertTrue(after_lidar, '雷达断流后到 GNSS 断流前没有诊断样本 —— 窗口太窄？')
        self.assertTrue(
            all(s == 'GNSS_ONLY' for _, s in after_lidar),
            f'雷达断流 {NDT_TIMEOUT_S + OBSERVE_SLACK_S:.1f} s 后状态应恒为 GNSS_ONLY，'
            f'实际：{after_lidar} —— NDT 超时是不是又成了无超时锁存？')

        # ---- ③ GNSS 也断 ⟹ DEAD_RECKONING 且保持 ----
        after_gnss = [(t, s) for t, s in self.diagnostics
                      if t >= GNSS_STOP_S + GNSS_TIMEOUT_S + OBSERVE_SLACK_S]
        self.assertTrue(after_gnss, 'GNSS 断流后没有诊断样本')
        self.assertTrue(
            all(s == 'DEAD_RECKONING' for _, s in after_gnss),
            f'GNSS 断流 {GNSS_TIMEOUT_S + OBSERVE_SLACK_S:.1f} s 后状态应恒为 DEAD_RECKONING，'
            f'实际：{after_gnss}')

        # ---- ④ 降级不是冻结：位姿流在两段降级期间都必须继续 ----
        # 假传感器 odom 50 Hz 驱动滤波器出位姿；按 ≥ 20 Hz 的下限要求（调度余量）。
        n_gnss_only = sum(1 for t in self.pose_stamps if LIDAR_STOP_S + 1.0 <= t < GNSS_STOP_S)
        n_dead = sum(1 for t in self.pose_stamps if GNSS_STOP_S + 1.0 <= t < RUN_UNTIL_S)
        self.assertGreater(
            n_gnss_only, 20 * (GNSS_STOP_S - LIDAR_STOP_S - 1.0),
            f'GNSS_ONLY 段位姿只发了 {n_gnss_only} 条 —— 滤波器在雷达断流后冻住了？')
        self.assertGreater(
            n_dead, 20 * (RUN_UNTIL_S - GNSS_STOP_S - 1.0),
            f'DEAD_RECKONING 段位姿只发了 {n_dead} 条 —— 滤波器在 GNSS 断流后冻住了？')

        first_leave = next(
            (t for t, s in self.diagnostics if t > LIDAR_STOP_S and s != 'NDT_AIDED'), None)
        first_dead = next((t for t, s in self.diagnostics if s == 'DEAD_RECKONING'), None)
        print(f'\n  [L3-G 定位超时] 雷达 {LIDAR_STOP_S:.0f} s 断流 → 首个非 NDT_AIDED 诊断 @ '
              f'{first_leave} s；GNSS {GNSS_STOP_S:.0f} s 断流 → 首个 DEAD_RECKONING @ {first_dead} s；'
              f'降级期间位姿 {n_gnss_only} + {n_dead} 条')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Crashes hide behind "finished cleanly" — exit codes are the only evidence."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
