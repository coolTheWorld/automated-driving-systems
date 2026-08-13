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
P6 预测的 L3-G 闭环：假障碍物流 + prediction_node，不需要 GPU，进 CI.

验的是**节点接线**（话题/参数/时序/逐 id 历史），不是预测算法 ——
算法判据在 L1（CP-P6-A 八条，对闭式解）。它抓得住的典型接线错误：
订错话题、参数没传进去、位移历史没接（④ 那条直接红）、id 没透传。

⚠️ 本文件自己发 /clock（use_sim_time=false，理由见 setUp），
   ROS_DOMAIN_ID=47（41-46 已被占，见各包 CMakeLists 的分配表）。

## 故障注入实测（2026-08-12，跑完回填）

| 注入 | 结果 |
|---|---|
| 节点订阅话题改错（/perception/obstacle） | **红**：判据 ① 收不到任何预测 |
| 位移历史绕过（节点恒填 999，不调 NetDisplacement） | **红**：判据 ③（第一帧就给出运动预测）—— 这正是"历史没接进选择器"的接线失效 |

⚠️ 「min_displacement_ratio=0」这种**参数级**注入在本测试抓不到（诚实输入下
比例门限不是决定路径），门限本身由 L1 的选择器用例守着 —— 分层各守各的。
"""

import math
import os
import unittest

from ads_msgs.msg import Obstacle, ObstacleArray, PredictedTrajectory, PredictedTrajectoryArray
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node as RclpyNode
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import MarkerArray

# 车辆目标：南直道对向车道西行（真园区地图的车道上 —— 节点加载的就是
# campus.xodr，车道跟随在这条线上应当成立）。
CAR_START_X_M = 60.0
CAR_Y_M = -48.25
CAR_SPEED_MPS = 4.0   # 朝 −x
# 静止目标：路肩上一个箱子（不在车道上）。
BOX_X_M = 40.0
BOX_Y_M = -46.0

FRAME_DT_S = 0.1      # 10 Hz，与感知一致
TOTAL_FRAMES = 30     # 3 s：位移窗 1 s，前 ~11 帧无位移证据、之后有


@pytest.mark.launch_test
def generate_test_description():
    """Launch only prediction_node — the obstacle stream comes from the test."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ads_prediction',
            executable='prediction_node',
            name='prediction_node',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {}


def make_obstacle(oid, x, y, vx, vy, length, width, heading_resolved):
    """构造一个 map 系的 Obstacle."""
    obstacle = Obstacle()
    obstacle.id = oid
    obstacle.pose.position.x = float(x)
    obstacle.pose.position.y = float(y)
    obstacle.pose.position.z = 0.75
    yaw = math.atan2(vy, vx) if (vx, vy) != (0.0, 0.0) else 0.0
    obstacle.pose.orientation.z = math.sin(0.5 * yaw)
    obstacle.pose.orientation.w = math.cos(0.5 * yaw)
    obstacle.heading_resolved = bool(heading_resolved)
    obstacle.velocity_mps.x = float(vx)
    obstacle.velocity_mps.y = float(vy)
    obstacle.size_m.x = float(length)
    obstacle.size_m.y = float(width)
    obstacle.size_m.z = 1.5
    obstacle.existence_probability = 1.0
    return obstacle


class TestPredictionClosedLoop(unittest.TestCase):
    """Feed synthetic obstacles and check what comes out of /prediction/*."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # ⚠️ 本节点是 **/clock 的来源**，自己 use_sim_time=false —— 设 true
        #    会等一个自己还没发出来的时钟，死锁（与感知闭环同一条注释）。
        self.node = RclpyNode('prediction_closed_loop_tester')
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.obstacle_pub = self.node.create_publisher(
            ObstacleArray, '/perception/obstacles', 10)
        self.predictions = []
        self.markers = []
        self.node.create_subscription(
            PredictedTrajectoryArray, '/prediction/trajectories',
            lambda m: self.predictions.append(m), 20)
        self.node.create_subscription(
            MarkerArray, '/prediction/markers', lambda m: self.markers.append(m), 5)

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds):
        deadline = self.node.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_prediction_pipeline(self):
        """①-⑦：链路、id 透传、位移门限、车道跟随、静态、时间参数化、markers."""
        # 等订阅配对（prediction_node 起动 + DDS 发现）。
        for _ in range(100):
            if (self.obstacle_pub.get_subscription_count() > 0 and
                    self.node.count_publishers('/prediction/trajectories') > 0):
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # ---- 喂 3 s 的障碍物流：车辆匀速西行 + 静止箱子 --------------------
        sim_t = 100.0   # 起始仿真时刻任意，非零免得跟默认零混淆
        for frame in range(TOTAL_FRAMES):
            sim_t += FRAME_DT_S
            clock = Clock()
            clock.clock.sec = int(sim_t)
            clock.clock.nanosec = int((sim_t % 1.0) * 1e9)
            self.clock_pub.publish(clock)

            array = ObstacleArray()
            array.header.frame_id = 'map'
            array.header.stamp = clock.clock
            car_x = CAR_START_X_M - CAR_SPEED_MPS * frame * FRAME_DT_S
            array.obstacles.append(make_obstacle(
                1, car_x, CAR_Y_M, -CAR_SPEED_MPS, 0.0, 4.4, 1.8, True))
            array.obstacles.append(make_obstacle(
                2, BOX_X_M, BOX_Y_M, 0.0, 0.0, 0.5, 0.4, False))
            self.obstacle_pub.publish(array)
            self._spin_for(0.05)
        self._spin_for(0.5)

        # ① 链路通，且 /prediction/trajectories 只有一个发布者（机械查）。
        print(f'① 收到预测帧 {len(self.predictions)}（喂入 {TOTAL_FRAMES}）')
        self.assertGreaterEqual(
            len(self.predictions), TOTAL_FRAMES // 2,
            '预测帧太少 —— 节点没起来 / 订错话题 / QoS 不配')
        self.assertEqual(
            self.node.count_publishers('/prediction/trajectories'), 1,
            '发布者数不是 1 —— 有别人也在发这个话题')

        # ② 每条轨迹的 obstacle_id 都在输入里（id 透传）。
        for message in self.predictions:
            for trajectory in message.trajectories:
                self.assertIn(
                    trajectory.obstacle_id, (1, 2),
                    f'冒出未知 id {trajectory.obstacle_id} —— id 没透传')
        print('② id 透传 ✓')

        def car_paths(message):
            return [t for t in message.trajectories if t.obstacle_id == 1]

        # ③ 位移证据攒够之前（头 ~1 s），车辆必须是 STATIC ——
        #    位移一致性门限的**接线**判据（门限本身 L1 已判）。
        early = self.predictions[0]
        for trajectory in car_paths(early):
            self.assertEqual(
                trajectory.model, PredictedTrajectory.MODEL_STATIC,
                '第一帧就给出运动预测 —— 位移历史没接进选择器')
        print('③ 无位移证据 → STATIC ✓')

        # ④ 位移证据攒够之后，车辆变成 LANE_FOLLOW（它就在车道上顺行）。
        late = self.predictions[-1]
        late_models = {t.model for t in car_paths(late)}
        print(f'④ 末帧车辆模型集合 {late_models}（应含 LANE_FOLLOW）')
        self.assertIn(
            PredictedTrajectory.MODEL_LANE_FOLLOW, late_models,
            '车道上的顺行车没有得到车道跟随预测')

        # ⑤ 车道跟随的 3 s 末点仍在这条直道车道附近（y ≈ −48.25）。
        for trajectory in car_paths(late):
            if trajectory.model != PredictedTrajectory.MODEL_LANE_FOLLOW:
                continue
            end = trajectory.points[-1]
            print(f'⑤ 末点 ({end.x_m:.2f}, {end.y_m:.2f})，t={end.t_s:.1f}')
            self.assertLess(abs(end.y_m - CAR_Y_M), 0.5, '预测跑出了车道')
            self.assertLess(end.x_m, CAR_START_X_M - 12.0 + 1.0, '预测没朝行驶方向走满视界')

        # ⑥ 静止箱子恒为 STATIC 且原地不动。
        for message in self.predictions:
            for trajectory in message.trajectories:
                if trajectory.obstacle_id != 2:
                    continue
                self.assertEqual(trajectory.model, PredictedTrajectory.MODEL_STATIC)
                for point in trajectory.points:
                    self.assertLess(
                        math.hypot(point.x_m - BOX_X_M, point.y_m - BOX_Y_M), 1e-6,
                        '静态预测动了')
        print('⑥ 静止目标恒 STATIC 原地 ✓')

        # ⑦ 时间参数化单调 + markers 在发。
        for trajectory in late.trajectories:
            times = [p.t_s for p in trajectory.points]
            self.assertEqual(times, sorted(times), 't_s 不单调')
            self.assertAlmostEqual(times[0], 0.0, places=9)
        self.assertGreater(len(self.markers), 0, 'markers 没发 —— RViz 交付物缺失')
        print(f'⑦ t_s 单调、markers {len(self.markers)} 帧 ✓')


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    """节点必须干净退出."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)


# 机械拦截漏配 domain：忘了 ENV 的话所有 launch 测试共享 domain 0，
# 两个包的闭环会互相污染（CLAUDE.md 陷阱表那条跨包坑）。
assert os.environ.get('ROS_DOMAIN_ID'), '必须经 CMake 的 ENV ROS_DOMAIN_ID=47 运行'
