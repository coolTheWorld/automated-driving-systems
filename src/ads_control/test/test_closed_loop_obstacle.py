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
L3-G with a static obstacle: check the stack actually drives around it.

**这是 P3 唯一能进 CI 的绕障验收。** CP-P3-B 要真 Gazebo、要 GPU、要人看；
而绕障这件事最容易坏在**接线**上（障碍物话题没接、坐标系搞错、真值发布器
没起来），那一类故障不需要物理引擎就能抓住。

链路：假车 + map→odom + map_node + **obstacle_truth** + planning_node + control_node。
`obstacle_truth` 只从参数发布，**不需要 Gazebo**，所以整条链路能在 CI 里跑。

## 判据是 SPEC §8 场景 S04 的那一条，不是"看着绕过去了"

    侧向间距 > 0.5 m

测的是**车体外廓**到障碍物外廓的间距，不是轨迹点到障碍物 ——
车头比后轴前伸 3.55 m，拿轨迹点算的话车会一头顶上去而判据还显示"很安全"。

## 与 CP-P3-B 的分工

| | 本测试（L3-G） | CP-P3-B（Gazebo） |
|---|---|---|
| 验什么 | **接线**：障碍物进得来、规划器绕得出去、控制器跟得上 | 真物理下的**精度与安全余量** |
| 车辆模型 | 运动学自行车，无轮胎、无质量 | Gazebo 的 AckermannSteering |
| 进 CI | ✅ | ❌（要 GPU、跑不确定） |

**不要在这里定紧判据。** 假车没有轮胎，紧判据在这儿只会给出虚假的安全感 ——
真正的精度考核在 CP-P3-B。
"""

import math
import os
from pathlib import Path
import sys
import time
import unittest

from ads_msgs.msg import ObstacleArray
from ament_index_python.packages import get_package_share_directory
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
import yaml

# ⚠️ 同目录的模块必须先把这个目录加进 sys.path —— 实测过：去掉这两行，
#    launch 测试直接 error。理由与 noqa 的取舍见 test_closed_loop.py 里的同一段。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from closed_loop_common import (  # noqa: E402,I100
    control_params, GOAL_X_M, GOAL_Y_M, planning_params, vehicle_params)

# 用哪个场景。avoid = 贴右边的锥桶，侧向留得下车。
# **scripts/gen_obstacles.py 已经机械校验过它确实可绕**（按规划器的采样网格判），
# 所以这里不再重算 —— 重算一遍就是第二份判据，而两份判据迟早分家。
SCENARIO = 'avoid'


def _scenario():
    """
    Load the obstacle scenario from the single source of truth.

    读的是 gazebo_bridge share 里的 config/obstacles.yaml —— **与真值发布器、
    与 Gazebo 模型生成器同一个文件**。在测试里另写一份障碍物坐标就是第三份数据，
    而它漂移的症状是「测试里绕的那个东西和实际放的不是一个」。

    :return: (车道配置, 该场景的配置)
    """
    path = (Path(get_package_share_directory('gazebo_bridge')) / 'config' / 'obstacles.yaml')
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    return config['lane'], config['scenarios'][SCENARIO]


def _obstacle_world_pose(lane, obstacle):
    """
    Convert one obstacle from lane coordinates to world coordinates.

    与 gazebo_sim.launch.py / gen_obstacles.py 里的换算相同，且同样只支持
    heading = 0（南侧直道）。三处一致不是靠纪律 —— 生成器对非零 heading
    显式拒绝，所以这个假设不可能悄悄失效。

    :param lane: 车道配置
    :param obstacle: 障碍物配置
    :return: (x, y)
    """
    return (float(obstacle['along_x_m']),
            float(lane['center_y_m']) + float(obstacle['lateral_offset_m']))


@pytest.mark.launch_test
def generate_test_description():
    """起假车 + 静态 TF + map_node + obstacle_truth + planning_node + control_node."""
    fake_vehicle = os.environ['FAKE_VEHICLE_EXECUTABLE']
    lane, spec = _scenario()

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

        # 障碍物真值。**不需要 Gazebo** —— 它只从参数发布，这正是本测试
        # 能进 CI 的原因。参数与 gazebo_sim.launch.py 用同一个 YAML 搬运。
        Node(
            package='gazebo_bridge', executable='obstacle_truth', name='obstacle_truth',
            parameters=[{
                'obstacles.center_x_m': [_obstacle_world_pose(lane, o)[0]
                                         for o in spec['obstacles']],
                'obstacles.center_y_m': [_obstacle_world_pose(lane, o)[1]
                                         for o in spec['obstacles']],
                'obstacles.yaw_rad': [float(lane['heading_rad']) for _ in spec['obstacles']],
                'obstacles.length_m': [float(o['length_m']) for o in spec['obstacles']],
                'obstacles.width_m': [float(o['width_m']) for o in spec['obstacles']],
                'obstacles.height_m': [float(o['height_m']) for o in spec['obstacles']],
                'frame_id': 'map',
                'use_sim_time': True,
            }],
            output='screen'),

        Node(package='ads_planning', executable='planning_node', name='planning_node',
             parameters=[planning_params()], output='screen'),

        Node(package='ads_control', executable='control_node', name='control_node',
             parameters=[control_params()], output='screen'),

        launch_testing.actions.ReadyToTest(),
    ])


class TestClosedLoopObstacle(unittest.TestCase):
    """Drive past a static obstacle and check the clearance actually held."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('closed_loop_obstacle_tester')
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.control_samples = []
        self.planning_samples = []
        self.obstacle_messages = 0
        # 全程车体外廓到障碍物外廓的最小间距。**边跑边算**而不是事后回放：
        # 事后回放要存下每一拍的位姿，而漏存一拍就可能恰好漏掉最近的那一拍。
        self.min_clearance_m = float('inf')
        # 最近点**发生在哪儿**。只记一个标量的话，"规划得太贴"和"控制跟丢了"
        # 分不开 —— 而两者的修法完全不同。
        self.min_clearance_pose = None

        params = planning_params()
        self.vehicle_length_m = params['vehicle.length_m']
        self.vehicle_width_m = params['vehicle.width_m']
        self.rear_axle_to_center_m = (
            0.5 * self.vehicle_length_m - params['vehicle.rear_overhang_m'])
        self.safety_margin_m = params['safety.margin_m']

        lane, spec = _scenario()
        self.obstacles = [
            (*_obstacle_world_pose(lane, o), float(o['length_m']), float(o['width_m']))
            for o in spec['obstacles']
        ]

        self.goal_pub = self.node.create_publisher(
            PoseStamped, '/goal_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(
            DiagnosticArray, '/control/diagnostics', self._on_control_diag, 200)
        self.node.create_subscription(
            DiagnosticArray, '/planning/diagnostics', self._on_planning_diag, 200)
        self.node.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.node.create_subscription(
            ObstacleArray, '/perception/obstacles',
            lambda _m: setattr(self, 'obstacle_messages', self.obstacle_messages + 1), 10)

    def tearDown(self):
        self.node.destroy_node()

    def _on_control_diag(self, msg):
        self.control_samples.append({kv.key: kv.value for kv in msg.status[0].values})

    def _on_planning_diag(self, msg):
        self.planning_samples.append({
            'message': msg.status[0].message,
            **{kv.key: kv.value for kv in msg.status[0].values},
        })

    def _on_odom(self, msg):
        # 假车直接在 **map 坐标**里积分，且 map→odom 是单位变换（见 test_closed_loop.py
        # 里那段说明），所以 /odom 的位姿就是 map 位姿，不用再过 TF。
        # ⚠️ **真实栈里不成立** —— 那边 odom 原点在自车 spawn 处。
        # 直接从四元数取 yaw，不引入 tf_transformations 这个新依赖
        # （CLAUDE.md：引入第三方依赖是先问后做项，而这里只要一个两行的公式）。
        # 平面运动下 roll = pitch = 0，所以 yaw = atan2(2(wz+xy), 1−2(y²+z²)) 是精确的。
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        clearance_m = self._clearance_at(
            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        if clearance_m < self.min_clearance_m:
            self.min_clearance_m = clearance_m
            self.min_clearance_pose = (
                msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    @staticmethod
    def _corners(center_x_m, center_y_m, yaw_rad, length_m, width_m):
        """Return the four corners in winding order (adjacent pairs are real edges)."""
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        half_l = 0.5 * length_m
        half_w = 0.5 * width_m
        return [
            (center_x_m + dx * cos_yaw - dy * sin_yaw,
             center_y_m + dx * sin_yaw + dy * cos_yaw)
            for dx, dy in ((half_l, half_w), (half_l, -half_w),
                           (-half_l, -half_w), (-half_l, half_w))
        ]

    @staticmethod
    def _point_to_segment(point, from_point, to_point):
        """Return the minimum distance from a point to a segment."""
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        length_squared = dx * dx + dy * dy
        ratio = 0.0
        if length_squared > 0.0:
            ratio = ((point[0] - from_point[0]) * dx + (point[1] - from_point[1]) * dy)
            ratio = max(0.0, min(1.0, ratio / length_squared))
        return math.hypot(point[0] - (from_point[0] + ratio * dx),
                          point[1] - (from_point[1] + ratio * dy))

    @classmethod
    def _polygon_distance(cls, poly_a, poly_b):
        """
        Exact minimum distance between two disjoint convex polygons.

        遍历所有（顶点, 边）对取最小 —— 对**不相交**的凸多边形是精确的，
        因为最近点对中至少有一个必在顶点上。两个方向都要算：
        只算单向会漏掉"A 的边贴着 B 的顶点"，给出偏大的距离。

        ⚠️ **不要退回用分离轴的最大间隙当距离。** 那只是下界，
           在**顶点对顶点**的位形下会严重低估 —— 实测过：本用例第一版就用了
           那个下界，在车刚越过障碍物的位形上把真实的 0.656 m 报成 0.457 m，
           于是判据红了，而系统其实是对的。**保守估计不会造成假通过，
           但它会造成假失败** —— 而假失败同样让人去查一个不存在的问题。
           C++ 侧 `distance_m()` 早就是精确算法，`test_collision.cpp` 里还有
           一条专门的用例（VertexToVertexDistanceIsExactNotTheSeparatingAxisLowerBound）
           说明这件事 —— 结果我转头在这里用了下界。
        """
        best = float('inf')
        for first, second in ((poly_a, poly_b), (poly_b, poly_a)):
            for point in first:
                for i in range(len(second)):
                    best = min(best, cls._point_to_segment(
                        point, second[i], second[(i + 1) % len(second)]))
        return best

    @staticmethod
    def _overlaps(poly_a, poly_b, yaw_a, yaw_b):
        """Separating axis theorem: disjoint on any axis means no overlap."""
        for angle in (yaw_a, yaw_a + math.pi / 2, yaw_b, yaw_b + math.pi / 2):
            axis = (math.cos(angle), math.sin(angle))
            proj_a = [p[0] * axis[0] + p[1] * axis[1] for p in poly_a]
            proj_b = [p[0] * axis[0] + p[1] * axis[1] for p in poly_b]
            if min(proj_a) > max(proj_b) or min(proj_b) > max(proj_a):
                return False
        return True

    def _clearance_at(self, rear_x_m, rear_y_m, yaw_rad):
        """
        Exact vehicle-body-to-obstacle distance at one pose. 0 if overlapping.

        ⚠️ 车体中心不是 base_link：后轴要沿车头方向前移
           length/2 − rear_overhang（本项目 1.35 m）。漏掉这一步，
           整个间距会算成"很安全"而车实际上一头顶了上去。

        :param rear_x_m: 后轴 x（map 系）
        :param rear_y_m: 后轴 y
        :param yaw_rad: 车身朝向
        :return: 间距，米；0 表示重叠
        """
        body_x = rear_x_m + self.rear_axle_to_center_m * math.cos(yaw_rad)
        body_y = rear_y_m + self.rear_axle_to_center_m * math.sin(yaw_rad)
        body = self._corners(
            body_x, body_y, yaw_rad, self.vehicle_length_m, self.vehicle_width_m)

        best = float('inf')
        for obs_x, obs_y, obs_length, obs_width in self.obstacles:
            obstacle = self._corners(obs_x, obs_y, 0.0, obs_length, obs_width)
            if self._overlaps(body, obstacle, yaw_rad, 0.0):
                return 0.0
            best = min(best, self._polygon_distance(body, obstacle))
        return best

    def test_drives_around_the_obstacle(self):
        self._spin(4.0)
        self.assertGreater(
            self.obstacle_messages, 0,
            '一条 /perception/obstacles 都没收到 —— obstacle_truth 起来了吗？')
        self.assertGreater(
            len(self.planning_samples), 0,
            '没收到 /planning/diagnostics —— planning_node 起来了吗？')

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = GOAL_X_M
        goal.pose.position.y = GOAL_Y_M
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        deadline = time.monotonic() + 40.0
        reached = False
        while time.monotonic() < deadline and not reached:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.control_samples and self.control_samples[-1].get('state') == 'GOAL_REACHED':
                if abs(float(self.control_samples[-1]['measured_speed_mps'])) < 0.05:
                    reached = True

        states = {s.get('state') for s in self.control_samples}
        self.assertTrue(
            reached, f'有障碍物时没能到达终点。经历过的状态：{states}')

        # ---- ① 规划器确实决定了绕行 ----
        offsets = [abs(float(s['lateral_offset_m'])) for s in self.planning_samples
                   if 'lateral_offset_m' in s]
        self.assertTrue(offsets, '/planning/diagnostics 里没有 lateral_offset_m')
        max_offset_m = max(offsets)
        self.assertGreater(
            max_offset_m, 0.1,
            f'规划器全程横向偏移最大只有 {max_offset_m:.3f} m —— 它没打算绕，'
            '障碍物是不是没进到规划器里？')

        # 断言**之前**先把数都打出来。只看红/绿分不清「规划得太贴」和「控制跟丢了」，
        # 而两者的修法完全不同：前者要加规划裕度，后者要查控制器。
        tracking = [c for c in self.control_samples
                    if c.get('state') in ('TRACKING', 'GOAL_REACHED')]
        max_lateral_error_m = max(abs(float(c['lateral_error_m'])) for c in tracking)
        print(
            f'\n  [L3-G/障碍物] 规划器最大横向偏移 {max_offset_m:.3f} m，'
            f'控制器最大横向误差 {max_lateral_error_m:.3f} m\n'
            f'                实测最小间距 {self.min_clearance_m:.3f} m '
            f'@ 后轴 ({self.min_clearance_pose[0]:.2f}, {self.min_clearance_pose[1]:.2f})，'
            f'该处车道偏移 d = {self.min_clearance_pose[1] + 51.75:+.3f} m\n'
            f'                **高出判据 {self.min_clearance_m - self.safety_margin_m:.3f} m** '
            f'—— 这个数要盯：safety.margin_m 是**规划**约束，'
            f'而 SPEC §8 S04 的 0.5 m 是对**实际车**的判据，两者之间只隔着执行误差')

        # ---- ② SPEC §8 场景 S04 的判据：侧向间距 > 0.5 m ----
        self.assertGreater(
            self.min_clearance_m, 0.0,
            f'车体与障碍物重叠了（间距 {self.min_clearance_m:.3f} m）')
        self.assertGreaterEqual(
            self.min_clearance_m, self.safety_margin_m,
            f'最小间距 {self.min_clearance_m:.3f} m < 判据 {self.safety_margin_m} m')

        # ---- ③ 绕完要回到中心线 ----
        # 不回中心线的症状很隐蔽：不报错、不压线、轨迹平滑，只是从此不走车道中心。
        # 守它的硬约束是 w_consistency < w_offset（planning.md §4.5），
        # 那条有 L1 用例；这里验的是它在**整条链路上**也确实成立。
        final_offset_m = abs(float(self.planning_samples[-1]['lateral_offset_m']))
        self.assertLess(
            final_offset_m, 0.1,
            f'过了障碍物之后仍偏在 {final_offset_m:.3f} m 上，没有回到车道中心')
        print(f'                绕完回到中心线：终点偏移 {final_offset_m:.3f} m')

    def _spin(self, wall_seconds):
        deadline = time.monotonic() + wall_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)


@launch_testing.post_shutdown_test()
class TestObstacleExitCodes(unittest.TestCase):
    """
    Every process must exit cleanly.

    崩溃的节点会让上面的断言变成"没数据"而不是"红" —— 而"没数据"很容易
    被读成"这条链路没接上"，于是人去查接线，而实际是某个节点挂了。
    """

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
