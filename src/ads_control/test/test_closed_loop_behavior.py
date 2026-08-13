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
Behavior closed loop: follow-stop behind an in-lane target, release, resume.

P7-S3 的 L3-G：**不需要 GPU** 的行为决策闭环（假车 + map_node + planning_node +
control_node），障碍物由测试自己发 —— obstacle_truth 都不用起。

一条链走完 CP-P7-B 的机理原型（真值判据在 S4 的真仿真里量，这里只验接线）：
    车道内静止目标 → 行为层 FOLLOW → 按 stand_off 停住 →
    目标消失 → 释放 → 继续开到终点。

它守三件接线事：
  ① planning_node 的行为链路真的在工作（behavior_state 从 CRUISE 进 FOLLOW）；
  ② 停车点的推导量没有错位（真值间距落在 stand_off 的解析带里 ——
     错一个 front_offset 或错一个坐标系，这个数立刻差 3.55 m）；
  ③ 释放后能恢复（判据 ③⑥ 的机理半边：约束消失 → 车重新走到 GOAL_REACHED）。

⚠️ 与 test_closed_loop.py 的关系：那一条跑的是**无目标**基线 —— P7 之后它就是
   「行为层默认不改变既有行为」的回归（CP-P7-B ⑩ 的 L3-G 半边），不需要复制。

## 故障注入实测（2026-08-13，写完立刻做的）

| 注入 | 结果 |
|---|---|
| planning_node 不把约束传给 plan()（传 nullptr，复原 P6 行为）
  | **红**：真值间距 **1.81 m**（预写表猜 2.24，又不准，照实测改）——
  静态障碍物准入把车停在了 safety_margin 上，比 stand_off 近 2.4 m
  （间距带断言 [3.8, 6.0] 抓住）；behavior_state 照样报 FOLLOW
  （标签与约束分离的结构使然 —— 只断言标签抓不住这条，
  **间距断言才是主判据**） |
"""

import os
import sys
import time
import unittest

from ads_msgs.msg import Obstacle, ObstacleArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from closed_loop_common import (  # noqa: E402,I100
    control_params, GOAL_X_M, GOAL_Y_M, planning_params, vehicle_params)

# 车道内静止目标：与自车同源尺寸的"车"，停在直道 x=60 的车道中心。
# 解析带：stop_at = (60−2.2) − 3.55 − 4.0 = 50.25（后轴），
# 真值间距 = 57.8 − (x_ego + 3.55) ⟹ 理想 4.0，采样/到达容差后 [3.8, 6.0]。
TARGET_X_M = 60.0
TARGET_Y_M = -51.75
TARGET_LEN_M = 4.4
TARGET_WID_M = 1.8
FRONT_OFFSET_M = 3.55


@pytest.mark.launch_test
def generate_test_description():
    """Launch fake vehicle + static TF + map/planning/control; obstacles come from the test."""
    fake_vehicle = os.environ['FAKE_VEHICLE_EXECUTABLE']

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
             parameters=[planning_params()], output='screen'),
        Node(package='ads_control', executable='control_node', name='control_node',
             parameters=[control_params()], output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestFollowStopAndResume(unittest.TestCase):
    """Drive at a stopped in-lane target, assert the stand-off stop, then release."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('behavior_loop_tester')
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.goal_pub = self.node.create_publisher(
            PoseStamped, '/goal_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.obstacle_pub = self.node.create_publisher(ObstacleArray, '/perception/obstacles', 10)
        self.control_diag = []
        self.behavior_states = []
        self.ego_x = None
        self.node.create_subscription(
            DiagnosticArray, '/control/diagnostics', self._on_control_diag, 200)
        self.node.create_subscription(
            DiagnosticArray, '/planning/diagnostics', self._on_planning_diag, 200)
        self.node.create_subscription(Odometry, '/odom', self._on_odom, 20)

    def tearDown(self):
        self.node.destroy_node()

    def _on_control_diag(self, msg):
        self.control_diag.append({kv.key: kv.value for kv in msg.status[0].values})

    def _on_planning_diag(self, msg):
        values = {kv.key: kv.value for kv in msg.status[0].values}
        state = values.get('behavior_state')
        if state and (not self.behavior_states or self.behavior_states[-1] != state):
            self.behavior_states.append(state)

    def _on_odom(self, msg):
        # 假车直接在 map 坐标里积分（map→odom 是单位变换），x 就是世界 x。
        self.ego_x = msg.pose.pose.position.x

    def _publish_obstacles(self, present):
        msg = ObstacleArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        if present:
            obstacle = Obstacle()
            obstacle.id = 77
            obstacle.classification = Obstacle.CLASSIFICATION_VEHICLE
            obstacle.pose.position.x = TARGET_X_M
            obstacle.pose.position.y = TARGET_Y_M
            obstacle.pose.orientation.w = 1.0
            obstacle.size_m.x = TARGET_LEN_M
            obstacle.size_m.y = TARGET_WID_M
            obstacle.size_m.z = 1.5
            msg.obstacles.append(obstacle)
        # present=False 时发**空数组**：感知说"没有目标"，与"感知死了"是两回事
        # （后者是停发，会触发 planning 的过期检查 → 不发轨迹 → 刹停）。
        self.obstacle_pub.publish(msg)

    def _spin(self, wall_seconds, obstacles_present):
        deadline = time.monotonic() + wall_seconds
        next_pub = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_pub:
                self._publish_obstacles(obstacles_present)
                next_pub = time.monotonic() + 0.05  # 20 Hz 墙钟 ≈ 60 Hz 仿真，远快于 1 s 过期
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _latest_speed(self):
        if not self.control_diag:
            return float('nan')
        return abs(float(self.control_diag[-1].get('measured_speed_mps', 'nan')))

    def test_follow_stop_then_release(self):
        # ---- 前提：链路活着 ----
        self._spin(4.0, obstacles_present=True)
        self.assertGreater(len(self.control_diag), 0, '没收到 /control/diagnostics')

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = GOAL_X_M
        goal.pose.position.y = GOAL_Y_M
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        # ---- 阶段一：开向静止目标，必须在 stand_off 带内停住 ----
        deadline = time.monotonic() + 40.0
        stopped = False
        while time.monotonic() < deadline and not stopped:
            self._spin(0.2, obstacles_present=True)
            moving_started = any(
                float(d.get('measured_speed_mps', '0')) > 1.0 for d in self.control_diag[-50:])
            if moving_started and self._latest_speed() < 0.05 and self.ego_x is not None:
                stopped = self.ego_x > 40.0  # 排除起步前的静止
        self.assertTrue(stopped, f'40 s 内没有在目标前停住（ego_x={self.ego_x}）')

        gap_m = (TARGET_X_M - TARGET_LEN_M / 2.0) - (self.ego_x + FRONT_OFFSET_M)
        self.assertIn('FOLLOW', self.behavior_states, f'行为状态从未进 FOLLOW：{self.behavior_states}')
        # 解析带 [3.8, 6.0]：stand_off 4.0 + 采样格 0.5 + 控制到达容差 0.5，
        # 下界收 0.2 容浮点。错一个 front_offset（3.55）或坐标系，立刻出带。
        self.assertGreater(gap_m, 3.8, f'停得太近：真值间距 {gap_m:.2f} m —— 撞进 stand_off')
        self.assertLess(gap_m, 6.0, f'停得太远：真值间距 {gap_m:.2f} m —— 怂到不跟')
        print(f'\n  [P7-S3] 跟停：ego_x={self.ego_x:.2f}，真值间距 {gap_m:.2f} m'
              f'（解析 4.0），状态序列 {self.behavior_states}')

        # ---- 保持 2 s（墙钟）：停住是稳态不是路过 ----
        self._spin(2.0, obstacles_present=True)
        self.assertLess(self._latest_speed(), 0.05, '跟停不是稳态，车又动了')

        # ---- 阶段二：目标消失（空数组），必须恢复并到达终点 ----
        deadline = time.monotonic() + 40.0
        reached = False
        while time.monotonic() < deadline and not reached:
            self._spin(0.2, obstacles_present=False)
            latest = self.control_diag[-1] if self.control_diag else {}
            if latest.get('state') == 'GOAL_REACHED' and self._latest_speed() < 0.05:
                reached = True
        last_state = self.control_diag[-1].get('state') if self.control_diag else None
        self.assertTrue(
            reached,
            f'目标消失后 40 s 没到终点。末拍 {last_state}，行为序列 {self.behavior_states}')
        self.assertEqual(self.behavior_states[-1], 'CRUISE', '收尾时行为标签没有释放回 CRUISE')
        print(f'  [P7-S3] 释放后到达终点，行为序列 {self.behavior_states}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Exit codes are the only trustworthy evidence of a clean run."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
