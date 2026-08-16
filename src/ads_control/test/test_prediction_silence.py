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
Prediction-silence chain: a dead predictor must stop the car, through planning (清单 #6).

test_perception_silence.py 的孪生：那条守障碍物流断了之后的
`safety.obstacle_timeout_s`，这条守预测流断了之后的 `behavior.prediction_timeout_s`
（1.0 s，P7-S3 与障碍物同一段逻辑）。同一段代码两个分支各守一条 —— 一条绿证明不了
另一条（分支各自的 dt 与话题都不同）。

链路：假车（/clock 来源）+ map→odom + map_node + planning_node（expect_prediction=true，
expect_perception=false 且不发障碍物 —— 链路隔离，只坏预测这一条）+ control_node；
预测流（空的 PredictedTrajectoryArray）由测试自己发。判据用诊断消息的仿真时间戳。

## 故障注入实测（2026-08-16，写完立刻做的）

| 注入 | 结果 |
|---|---|
| planning `behavior.prediction_timeout_s` 改成 1e9（复原"没有过期检查"） | **红**：静默后规划器
  拿着最后一份预测继续规划，车一路 TRACKING 到 GOAL_REACHED，从未 STALE |
"""

import os
import sys
import time
import unittest

from ads_msgs.msg import PredictedTrajectoryArray
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from closed_loop_common import (  # noqa: E402,I100
    control_params, GOAL_X_M, GOAL_Y_M, planning_params, vehicle_params)

# 与 planning_params.yaml 的 behavior.prediction_timeout_s、control_params.yaml 的
# safety.trajectory_timeout_s 同步。写死而不读 YAML，理由同 test_trajectory_timeout。
PREDICTION_TIMEOUT_S = 1.0
TRAJECTORY_TIMEOUT_S = 0.5


@pytest.mark.launch_test
def generate_test_description():
    """Launch fake vehicle + TF + map + planning(expect_prediction) + control."""
    fake_vehicle = os.environ['FAKE_VEHICLE_EXECUTABLE']
    planning = planning_params()
    planning['expect_prediction'] = True

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[fake_vehicle, '--ros-args'] + [
                arg
                for key, value in vehicle_params().items()
                for arg in ('-p', f'{key}:={value}')
            ],
            output='screen'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='ads_map', executable='map_node', name='map_node',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='ads_planning', executable='planning_node', name='planning_node',
             parameters=[planning], output='screen'),
        Node(package='ads_control', executable='control_node', name='control_node',
             parameters=[control_params()], output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestPredictionSilenceStopsTheCar(unittest.TestCase):
    """Feed an (empty) prediction stream, go silent, demand planning ERROR + control stop."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('prediction_silence_tester')
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.prediction_pub = self.node.create_publisher(
            PredictedTrajectoryArray, '/prediction/trajectories', 10)
        self.goal_pub = self.node.create_publisher(
            PoseStamped, '/goal_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.control_samples = []    # (仿真时间戳, state, measured_speed)
        self.planning_samples = []   # (仿真时间戳, level, message)
        self.node.create_subscription(
            DiagnosticArray, '/control/diagnostics', self._on_control_diag, 200)
        self.node.create_subscription(
            DiagnosticArray, '/planning/diagnostics', self._on_planning_diag, 200)

    def tearDown(self):
        self.node.destroy_node()

    def _on_control_diag(self, msg):
        values = {kv.key: kv.value for kv in msg.status[0].values}
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.control_samples.append(
            (stamp_s, values.get('state'), float(values.get('measured_speed_mps', 'nan'))))

    def _on_planning_diag(self, msg):
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.planning_samples.append((stamp_s, msg.status[0].level, msg.status[0].message))

    def _spin(self, wall_seconds, publish_predictions):
        """Spin for wall_seconds; if publish_predictions, keep an empty 10 Hz stream up."""
        deadline = time.monotonic() + wall_seconds
        next_pub = 0.0
        while time.monotonic() < deadline:
            if publish_predictions and time.monotonic() >= next_pub:
                msg = PredictedTrajectoryArray()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.node.get_clock().now().to_msg()
                self.prediction_pub.publish(msg)
                next_pub = time.monotonic() + 0.033   # 3 倍速下 ≈ 仿真 10 Hz
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def test_silence_propagates_to_a_stop(self):
        # ---- 前提：两个诊断流都在（节点活着），且链路"存在"（预测流已到达）----
        self._spin(4.0, publish_predictions=True)
        self.assertGreater(len(self.control_samples), 0, '没收到 /control/diagnostics')
        self.assertGreater(len(self.planning_samples), 0, '没收到 /planning/diagnostics')

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = GOAL_X_M
        goal.pose.position.y = GOAL_Y_M
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        # ---- 车真的跑起来（TRACKING 且有速度）——否则"停住"不是被测行为 ----
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            self._spin(0.5, publish_predictions=True)
            moving = [s for s in self.control_samples if s[1] == 'TRACKING' and s[2] > 1.0]
            if len(moving) > 10:
                break
        moving = [s for s in self.control_samples if s[1] == 'TRACKING' and s[2] > 1.0]
        self.assertGreater(
            len(moving), 10,
            f'车没跑起来。末拍：{self.control_samples[-1] if self.control_samples else None}；'
            f'规划最后一条：{self.planning_samples[-1] if self.planning_samples else None}')

        # ---- 静默：预测"死了"（预测流戛然而止） --------------------------------
        t_silence_s = self.control_samples[-1][0]
        v_at_silence = self.control_samples[-1][2]
        self._spin(6.0, publish_predictions=False)

        # ---- 断言 1：规划器在 prediction_timeout + 余量内报 ERROR「预测列表…没有更新」 --
        errors = [p for p in self.planning_samples
                  if p[0] > t_silence_s and '预测列表' in p[2] and '没有更新' in p[2]]
        self.assertGreater(
            len(errors), 0,
            '静默后规划器从未报「预测列表没有更新」—— 过期检查没生效。'
            f'静默后的规划消息：{sorted({p[2][:30] for p in self.planning_samples if p[0] > t_silence_s})}')
        planning_latency_s = errors[0][0] - t_silence_s
        self.assertLess(
            planning_latency_s, PREDICTION_TIMEOUT_S + 0.5,
            f'规划器过期判定太慢：{planning_latency_s:.3f} s（阈值 {PREDICTION_TIMEOUT_S} + 余量 0.5）')

        # ---- 断言 2：控制器接着转入 TRAJECTORY_STALE 并刹停、保持停住 ----------
        stale = [s for s in self.control_samples
                 if s[1] == 'TRAJECTORY_STALE' and s[0] > t_silence_s]
        self.assertGreater(
            len(stale), 0,
            '静默后控制器从未进入 TRAJECTORY_STALE —— 规划停发没有传导到控制。'
            f'静默后见过的状态：{sorted({s[1] for s in self.control_samples if s[0] > t_silence_s})}')
        control_latency_s = stale[0][0] - t_silence_s
        self.assertLess(
            control_latency_s, PREDICTION_TIMEOUT_S + TRAJECTORY_TIMEOUT_S + 0.7,
            f'整条链路降级太慢：静默后 {control_latency_s:.3f} s 才 TRAJECTORY_STALE'
            f'（两段阈值 {PREDICTION_TIMEOUT_S}+{TRAJECTORY_TIMEOUT_S} + 余量 0.7）')
        self.assertLess(
            abs(self.control_samples[-1][2]), 0.05,
            f'静默后车没有停住：末拍速度 {self.control_samples[-1][2]:.3f} m/s（静默时 {v_at_silence:.3f}）')
        print(
            f'\n  [P9-S5b] 预测静默于 t={t_silence_s:.2f}（v={v_at_silence:.2f} m/s）：'
            f'规划 ERROR 延迟 {planning_latency_s:.3f} s，控制 STALE 延迟 {control_latency_s:.3f} s，'
            f'末拍 v={self.control_samples[-1][2]:.3f}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Crashes hide behind "finished cleanly" — exit codes are the only evidence."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
