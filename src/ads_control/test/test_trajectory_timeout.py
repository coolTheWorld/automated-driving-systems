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
Trajectory-silence watchdog: a dead planner must brake the car, not coast it.

P7-S0 补的安全洞：此前规划器死掉/卡死后，control_node 会把**最后一条轨迹跟完**
（滚动窗口最长 30 m）。planning_node 头上那句「不发轨迹 ⟹ 下游刹停」的假设
从来不成立 —— 本测试就是让那句话变成真的之后，钉住它。

结构上模仿 test_odom_robustness.py 而不是 test_closed_loop.py：
**规划器缺席正是被测场景**，所以链路里刻意没有 planning_node / map_node，
轨迹由测试自己发 —— 先按 10 Hz 发一条直线轨迹让车跑起来，然后**戛然而止**，
断言控制器在 trajectory_timeout_s 内转入 TRAJECTORY_STALE 并把车刹停。

⚠️ 用假车 + 仿真钟（它是 /clock 的来源）：超时判的是仿真时间，
   墙钟下这个测试就跑不到被测代码。延迟也一律用**诊断消息的仿真时间戳**量，
   不用墙钟 —— 3 倍速下墙钟量出来的数没有意义。

## 故障注入实测（2026-08-13，写完立刻做的）

| 注入 | 结果 |
|---|---|
| 跳过 on_timer 的静默检查（复原缺陷） | **红**：静默 5 s（墙钟，≈15 s 仿真）后从未进入
  TRAJECTORY_STALE，状态 TRACKING → **GOAL_REACHED** —— 车把 60 m 的旧轨迹
  **一路跟到了头**，正是被守的缺陷本身 |

修好后的实测（同日）：静默 0.520 s 后 TRAJECTORY_STALE（阈值 0.5 + 一拍离散化），
3.0 m/s 起 1.54 s 刹停（= v/max_decel = 3.0/2.0，物理自洽）。
"""

import os
import sys
import time
import unittest

from ads_msgs.msg import Trajectory, TrajectoryPoint
from diagnostic_msgs.msg import DiagnosticArray
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node as RclpyNode

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from closed_loop_common import (  # noqa: E402,I100
    control_params, START_X_M, START_Y_M, vehicle_params)

# 直线轨迹的常值目标速度。取 3.0 而不是巡航 5.556：车加速到它只要 ~3 s 仿真，
# 测试更短；而对被测逻辑来说速度是多少无关紧要（超时判的是消息年龄）。
CRUISE_MPS = 3.0

# 与 control_params.yaml 的 safety.trajectory_timeout_s 同步。
# 这里**不读 YAML 重新拼**是有意的：断言里要它和一个余量相加，
# 写死能让判据在代码评审里一眼看清；若与 YAML 漂移，下面的延迟断言会红。
TIMEOUT_S = 0.5


@pytest.mark.launch_test
def generate_test_description():
    """Launch fake vehicle + static TF + control_node; the planner is absent by design."""
    fake_vehicle = os.environ['FAKE_VEHICLE_EXECUTABLE']

    return launch.LaunchDescription([
        # 假车。**它是 /clock 的来源**（自己 use_sim_time=false）。
        launch.actions.ExecuteProcess(
            cmd=[fake_vehicle, '--ros-args'] + [
                arg
                for key, value in vehicle_params().items()
                for arg in ('-p', f'{key}:={value}')
            ],
            output='screen'),

        # map → odom 单位变换（假车直接在 map 坐标里积分，见 test_closed_loop.py）。
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': True}], output='screen'),

        Node(package='ads_control', executable='control_node', name='control_node',
             parameters=[control_params()], output='screen'),

        launch_testing.actions.ReadyToTest(),
    ])


def straight_trajectory() -> Trajectory:
    """
    Build a straight constant-speed trajectory starting at the spawn pose.

    60 m @ 0.5 m 步长，全程 3.0 m/s、加速度 0。静默发生在车走出 ~10 m 时，
    剩余 ~50 m ≫ goal.stop_distance_m（0.5），所以 goal_reached 分支
    绝不会先把超时检查短路掉 —— 判据判的确实是超时，不是到达。
    """
    msg = Trajectory()
    msg.header.frame_id = 'map'
    msg.status = Trajectory.STATUS_OK
    step_m = 0.5
    for i in range(121):
        pt = TrajectoryPoint()
        pt.x_m = START_X_M + step_m * i
        pt.y_m = START_Y_M
        pt.heading_rad = 0.0
        pt.curvature_1pm = 0.0
        pt.s_m = step_m * i
        pt.speed_mps = CRUISE_MPS
        pt.accel_mps2 = 0.0
        msg.points.append(pt)
    return msg


class TestTrajectorySilenceBrakesTheCar(unittest.TestCase):
    """Feed a trajectory, go silent, and demand a stop within the timeout."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('trajectory_timeout_tester')
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.traj_pub = self.node.create_publisher(Trajectory, '/planning/trajectory', 10)
        # 每拍存 (仿真时间戳 s, state, measured_speed)。延迟一律用它量，不用墙钟。
        self.samples = []
        self.node.create_subscription(
            DiagnosticArray, '/control/diagnostics', self._on_diag, 200)

    def tearDown(self):
        self.node.destroy_node()

    def _on_diag(self, msg):
        values = {kv.key: kv.value for kv in msg.status[0].values}
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append(
            (stamp_s, values.get('state'), float(values.get('measured_speed_mps', 'nan'))))

    def _spin_publishing(self, wall_seconds, publish):
        """Spin for wall_seconds; if publish, keep the 10 Hz trajectory stream up."""
        deadline = time.monotonic() + wall_seconds
        next_pub = 0.0
        while time.monotonic() < deadline:
            if publish and time.monotonic() >= next_pub:
                msg = straight_trajectory()
                self.traj_pub.publish(msg)
                # 3 倍速下墙钟 33 ms ≈ 仿真 100 ms，正好是规划器的 10 Hz。
                next_pub = time.monotonic() + 0.033
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def test_silence_degrades_and_stops(self):
        # ---- 前提 1：诊断在流（节点活着）。没有它，后面的断言毫无意义。----
        self._spin_publishing(4.0, publish=False)
        self.assertGreater(len(self.samples), 0, '没收到 /control/diagnostics —— control_node 起来了吗？')

        # ---- 前提 2：喂轨迹，车真的跑起来（TRACKING 且有速度）。 ----------
        # 「静默后车停了」只有在「静默前车在跑」时才是被测行为 ——
        # 否则一辆从没动过的车也能让断言变绿。
        self._spin_publishing(4.0, publish=True)
        moving = [s for s in self.samples if s[1] == 'TRACKING' and s[2] > 1.0]
        self.assertGreater(
            len(moving), 10,
            f'车没跑起来（TRACKING 且 v>1.0 的拍数不足）。'
            f'末拍：{self.samples[-1] if self.samples else None}')

        # ---- 静默：规划器"死了"。 -----------------------------------------
        t_silence_s = self.samples[-1][0]  # 静默起点，仿真时间
        v_at_silence = self.samples[-1][2]
        self._spin_publishing(5.0, publish=False)

        # ---- 断言 1：在 timeout + 余量内转入 TRAJECTORY_STALE。 -----------
        # 余量 0.5 s：静默起点取的是"最后一拍诊断"而非"最后一条轨迹到达"
        # （两者差最多一个控制周期 + 一个发布周期 ≈ 0.12 s 仿真），
        # 再留出诊断发布离散化的一拍。
        stale = [s for s in self.samples if s[1] == 'TRAJECTORY_STALE' and s[0] > t_silence_s]
        self.assertGreater(
            len(stale), 0,
            f'静默 5 s（墙钟）后从未进入 TRAJECTORY_STALE —— 超时降级没生效。'
            f'静默后见过的状态：{sorted({s[1] for s in self.samples if s[0] > t_silence_s})}')
        latency_s = stale[0][0] - t_silence_s
        self.assertLess(
            latency_s, TIMEOUT_S + 0.5,
            f'降级太慢：静默后 {latency_s:.3f} s 才进入 TRAJECTORY_STALE'
            f'（阈值 {TIMEOUT_S} + 余量 0.5）')

        # ---- 断言 2：车停住，且**保持**停住（末拍仍为静止）。 -------------
        # 3.0 m/s ÷ 2.0 m/s²（max_decel）= 1.5 s 刹停；5 s 墙钟 ≈ 15 s 仿真，绰绰有余。
        self.assertLess(
            abs(self.samples[-1][2]), 0.05,
            f'静默后车没有停住：末拍速度 {self.samples[-1][2]:.3f} m/s（静默时 {v_at_silence:.3f}）')

        stop_s = next((s[0] for s in stale if abs(s[2]) < 0.05), None)
        stop_text = f'刹停于 t={stop_s:.2f}' if stop_s is not None else '（未记录到停止拍）'
        print(
            f'\n  [P7-S0] 静默于 t={t_silence_s:.2f}（v={v_at_silence:.2f} m/s），'
            f'{latency_s:.3f} s 后 TRAJECTORY_STALE，{stop_text}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Crashes hide behind "finished cleanly" — exit codes are the only evidence."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
