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
Route-layer faults must leave the car parked, and a good goal afterwards must still work.

异常注入清单（docs/fault_injection.md）#15「目标点无路由」与 #13「TF 一开始就不发」：
两条都是「上游给不出路 ⟹ 下游**不许动**」的形态，且都要求故障排除后链路还能恢复
（否则守卫本身就是一个新的故障：把车永远锁死）。

  #15：目标点离任何车道都远（500, 500）⟹ map_node WARN + 发空 Path 清屏 ⟹ planning 忽略
       （<2 点）、诊断仍是「还没收到 /route/path」⟹ control NO_PATH，车不动；
       随后发一个合法目标 ⟹ 车动起来（恢复）。
  #13：launch 里**没有** map→odom 的静态 TF ⟹ map_node「拿不到 TF … 没有起点就无法规划」
       ⟹ 同上不动；测试再把静态 TF 发出来 + 重发目标 ⟹ 车动起来。
  ⚠️ 静态 TF 一旦发出就永不过期（tf2 对 /tf_static 的语义，陷阱表），所以「拿不到 TF」
     只能在**一开始就不发**时测 —— #13 必须排在 #15 前面，同一个 launch 里按方法名字母序跑
     （unittest 默认顺序），方法名前缀 a_/b_ 就是为这个。

链路：假车（/clock 来源）+ map_node + planning_node + control_node；**不含**静态 TF。
判「不动」用控制诊断：状态从未 TRACKING 且 measured_speed 恒 < 0.05。

## 故障注入实测（2026-08-16，写完立刻做的）

| 注入 | 结果 |
|---|---|
| #13：launch 里偷偷加一个 `static_transform_publisher map→odom`（= 守卫失效、TF 一开始就在） | 红：
  「没有 map→odom TF、发了合法目标，车却开始跟踪了」（状态集 {NO_PATH, TRACKING}） |
| #15：把 (500,500) 换成路上的合法目标 (60,−51.75)（= 无路由这一支根本没被走到） | 红：
  「目标点 (500,500) 无路由，车却开始跟踪了」（状态集 {TRACKING, GOAL_REACHED}） |
| ⚠️ 第一次给 #15 注入用的是 (GOAL_X−40, GOAL_Y)=(51.75,20) —— 那点在园区中央草地上，
  **本身也无路由**，注入后照样绿。「注入没红」先查注入本身是不是有效刺激（同一条纪律：判据要量刺激物） | 复盘 |

基线（2026-08-16）：#13 没 TF 时 4 s 内状态恒 NO_PATH、v=0.000，TF 到位重发目标后 TRACKING；
#15 无路由目标 4 s 内状态恒 GOAL_REACHED（上一条路线的末态）、v=0.000。
"""

import os
import sys
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TransformStamped
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from closed_loop_common import (  # noqa: E402,I100
    control_params, GOAL_X_M, GOAL_Y_M, planning_params, vehicle_params)


@pytest.mark.launch_test
def generate_test_description():
    """Launch fake vehicle + map + planning + control — deliberately without map→odom TF."""
    fake_vehicle = os.environ['FAKE_VEHICLE_EXECUTABLE']
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[fake_vehicle, '--ros-args'] + [
                arg
                for key, value in vehicle_params().items()
                for arg in ('-p', f'{key}:={value}')
            ],
            output='screen'),
        Node(package='ads_map', executable='map_node', name='map_node',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='ads_planning', executable='planning_node', name='planning_node',
             parameters=[planning_params()], output='screen'),
        Node(package='ads_control', executable='control_node', name='control_node',
             parameters=[control_params()], output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestRouteFaultsKeepTheCarParked(unittest.TestCase):
    """No TF / no route → parked; fix it → drives."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('route_faults_tester')
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.goal_pub = self.node.create_publisher(
            PoseStamped, '/goal_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.control_samples = []    # (state, speed)
        self.planning_messages = []
        self.node.create_subscription(
            DiagnosticArray, '/control/diagnostics', self._on_control_diag, 200)
        self.node.create_subscription(
            DiagnosticArray, '/planning/diagnostics',
            lambda m: self.planning_messages.append(m.status[0].message), 200)
        self.static_tf = tf2_ros.StaticTransformBroadcaster(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def _on_control_diag(self, msg):
        values = {kv.key: kv.value for kv in msg.status[0].values}
        self.control_samples.append(
            (values.get('state'), float(values.get('measured_speed_mps', 'nan'))))

    def _spin(self, wall_seconds):
        deadline = time.monotonic() + wall_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def _assert_parked(self, wall_seconds, why):
        n0 = len(self.control_samples)
        self._spin(wall_seconds)
        recent = self.control_samples[n0:]
        self.assertGreater(len(recent), 0, '没收到 /control/diagnostics —— control_node 起来了吗？')
        states = {s[0] for s in recent}
        vmax = max(abs(s[1]) for s in recent)
        print(f'[test] {why}：{wall_seconds:.0f} s（墙钟）内控制状态 {sorted(states)}，最大车速 {vmax:.3f}')
        self.assertNotIn('TRACKING', states, f'{why}，车却开始跟踪了')
        self.assertLess(vmax, 0.05, f'{why}，车却动了（v={vmax:.3f}）')

    def _assert_drives(self, wall_seconds, why):
        deadline = time.monotonic() + wall_seconds
        while time.monotonic() < deadline:
            self._spin(0.5)
            moving = [s for s in self.control_samples if s[0] == 'TRACKING' and s[1] > 1.0]
            if len(moving) > 10:
                print(f'[test] {why}：车动起来了（TRACKING 且 v>1.0 的拍数 {len(moving)}）')
                return
        self.fail(f'{why}，{wall_seconds:.0f} s 内车却没动 —— 守卫把链路锁死了？'
                  f'末拍 {self.control_samples[-1] if self.control_samples else None}；'
                  f'规划最后一条：{self.planning_messages[-1] if self.planning_messages else None}')

    def test_a_no_tf_then_tf(self):
        """No map->odom: goal rejected, car parked; TF appears + goal resent: car drives (#13)."""
        self._spin(3.0)
        self._send_goal(GOAL_X_M, GOAL_Y_M)
        self._assert_parked(4.0, '没有 map→odom TF、发了合法目标')
        self.assertTrue(
            any('还没收到 /route/path' in m for m in self.planning_messages),
            '规划器没有报「还没收到 /route/path」—— 没有 TF 时路由不该成功')

        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)
        self._spin(1.0)
        self._send_goal(GOAL_X_M, GOAL_Y_M)
        self._assert_drives(30.0, 'TF 到位后重发合法目标')

    def test_b_unroutable_goal_keeps_last_route_out(self):
        """Goal far from any lane: empty Path ignored, no new route, car parked (#15)."""
        # ⚠️ 前一个方法已经让车开向合法目标；这里等它到达/停稳后再注入，
        #    否则「不动」量的是上一条路线。GOAL_REACHED 或速度归零即算稳。
        deadline = time.monotonic() + 40.0
        while time.monotonic() < deadline:
            self._spin(0.5)
            if self.control_samples and self.control_samples[-1][0] == 'GOAL_REACHED' \
                    and abs(self.control_samples[-1][1]) < 0.05:
                break
        self._send_goal(500.0, 500.0)
        self._assert_parked(4.0, '目标点 (500,500) 无路由')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Crashes hide behind "finished cleanly" — exit codes are the only evidence."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
