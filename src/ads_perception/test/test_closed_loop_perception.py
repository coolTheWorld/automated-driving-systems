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
Perception closed-loop wiring test with no GPU: synthetic cloud + perception_node.

L3-G（SPEC §8）：**不需要 Gazebo、不需要 GPU、能进 CI** 的感知闭环。
与 ads_control 的 test_closed_loop.py、ads_localization 的
test_closed_loop_localization.py 是同一层的三个用例，各管一条链路。

它验的是**接线**，不是感知质量：
    话题名对不对、QoS 兼不兼容、参数装没装上、
    **base_link → map 那次变换做没做**、四个阶段有没有真的串起来。

⚠️ **不要用它替代 CP-P5-B。** 这里的点云是**解析合成**的：
   没有遮挡、没有噪声起伏、没有运动、目标只有一个、地面是理想平面。
   CP-P5-B 那些真正难的东西（可见面伪影、轴向翻转、重复航迹）在这里
   一个都不会发生。收紧判据得到的不是更强的保证，而是一个假的安心。

**为什么值得有这一层**：CP-P5-B 那次调试挖出来的四个根因里，**没有一个**
在最初怀疑的地方，而其中「所有车被判 STATIC」「位置差半个车长」两类
一旦回归，现场表现都是「感知不准」—— 于是人第一反应是去调聚类参数。
有了这一层，同类回归一次推送就红。

## 合成点云为什么必须只画**可见面**

⚠️ 这一条是 CP-P5-B 用例里踩过三次的坑的预防：**合成场景必须物理自洽**。
   把盒子六个面都画上，L-Shape 会拟出一个"从背面也看得见"的完美矩形，
   而真实雷达永远看不到背面 —— 于是用例会给出一个真实系统达不到的判据，
   或者反过来，掩盖掉一个真实存在的偏差。
   这里只画**朝向传感器的两个面**（前脸 + 右侧），与真实雷达一致。

## 故障注入实测（2026-08-12，写完立刻做的）

| 注入 | 结果 |
|---|---|
| 去掉 `base_link → map` 的变换（直接发 base_link 系坐标） | 红 ②：`59.82 not less than 0.5` |
| 地面分割阈值 0.15 → 2.0 m（把目标也当成地面） | 红 ①：`0 not greater than 5` |
| 聚类容差 0.5 → 0.02 m（目标碎成一地，都不够 min_cluster_size） | 红 ①：`0 not greater than 5` |
| `static_max_height_m` 1.0 → 3.0（一切都是 STATIC） | 红 ④ |

⚠️ 这张表**先写后跑会写错**：我预写的版本里位移写成"30 m"（实测 59.82，
   因为误差是自车位姿的模长而不是某一个分量），容差那条写成"断言 ①/③"
   （实际只红 ①，因为一个簇都没成，根本走不到尺寸判据）。
   **注入表必须是跑出来的**，这是本仓库第三次因为预写而写错。
⚠️ 还有一次注入**静默地没生效**（内联 `python3 -c` 的引号吃掉了替换串），
   当时误判成"判据太弱"。现在每个注入脚本都打印替换计数。
"""

import math
import os
import unittest

from ads_msgs.msg import ObstacleArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros

# 自车在 map 里的位姿。**故意不放原点** —— 放原点的话，
# 「忘了做 base_link → map 变换」这个错误会完全看不出来。
EGO_X_M = 30.0
EGO_Y_M = -51.75

# 目标在 base_link 里的位置与尺寸（对向车道上的一辆车）。
TARGET_X_M = 12.0
TARGET_Y_M = 3.5
TARGET_LENGTH_M = 4.4
TARGET_WIDTH_M = 1.8
TARGET_HEIGHT_M = 1.5
# 浮空碎片（车顶远端环）：目标远端之外 3 m、车顶高度、竖向延展 0。
FLOATING_X_M = TARGET_X_M + 0.5 * TARGET_LENGTH_M + 3.0
FLOATING_Z_M = TARGET_HEIGHT_M
# 车身底面离地 0.16 m（与 models/npc_car 的车身盒一致）。
TARGET_BOTTOM_M = 0.16


@pytest.mark.launch_test
def generate_test_description():
    """Launch only perception_node — the cloud and TF come from the test itself."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ads_perception',
            executable='perception_node',
            name='perception_node',
            parameters=[{
                'use_sim_time': True,
                # 每帧都发诊断，用例要读它。
                'diagnostics_period_s': 0.0,
            }],
            output='screen',
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {}


def build_cloud_points():
    """
    Synthesize one frame: a ground plane plus the **two visible faces** of a box.

    :return: [(x, y, z), ...]，base_link 系
    """
    points = []
    # ---- 地面 ----------------------------------------------------------
    # 从 3 m 起：更近的地面被自车挡住，真实雷达也看不到（range_min = 2.2 m）。
    x = 3.0
    while x <= 30.0:
        y = -8.0
        while y <= 8.0:
            points.append((x, y, 0.0))
            y += 0.5
        x += 0.5

    # ---- 目标：只画朝向传感器的两个面 ----------------------------------
    near_x = TARGET_X_M - 0.5 * TARGET_LENGTH_M   # 前脸
    near_y = TARGET_Y_M - 0.5 * TARGET_WIDTH_M    # 右侧面
    far_x = TARGET_X_M + 0.5 * TARGET_LENGTH_M
    far_y = TARGET_Y_M + 0.5 * TARGET_WIDTH_M
    z = TARGET_BOTTOM_M
    while z <= TARGET_HEIGHT_M:
        y = near_y
        while y <= far_y:
            points.append((near_x, y, z))
            y += 0.06
        x = near_x
        while x <= far_x:
            points.append((x, near_y, z))
            x += 0.06
        z += 0.08

    # ---- 浮空碎片：目标前方 3 m 处一片车顶高度的水平点片（P9-S5c）--------------
    # 复刻 Gazebo 实测：雷达 2.2 m 打车顶时相邻两线在车顶上相距 ~2 m > 聚类容差，
    # 车顶远端那一环自成一簇（0.9×0.5×0.03，底 1.48）。它不是目标，浮空碎片门
    # 必须把它剃掉；否则它会成为第二个（STATIC）障碍物 —— 断言 ⑧ 守着。
    x = FLOATING_X_M
    while x <= FLOATING_X_M + 0.5:
        y = TARGET_Y_M - 0.45
        while y <= TARGET_Y_M + 0.45:
            points.append((x, y, FLOATING_Z_M))
            y += 0.06
        x += 0.06
    return points


class TestPerceptionClosedLoop(unittest.TestCase):
    """Feed a synthetic cloud and check what comes out of /perception/obstacles."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # ⚠️ 本节点是 **/clock 的来源**，所以它自己 use_sim_time = false。
        #    设成 true 的话它会等一个自己还没发出来的时钟，直接死锁。
        self.node = RclpyNode('perception_closed_loop_tester')
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, '/lidar/points',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.static_tf = tf2_ros.StaticTransformBroadcaster(self.node)
        self.obstacles = []
        self.diagnostics = []
        self.node.create_subscription(
            ObstacleArray, '/perception/obstacles', self.obstacles.append, 10)
        self.node.create_subscription(
            DiagnosticArray, '/perception/diagnostics', self.diagnostics.append, 10)

        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = EGO_X_M
        transform.transform.translation.y = EGO_Y_M
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)

        self.points = build_cloud_points()

    def tearDown(self):
        self.node.destroy_node()

    def _publish_frame(self, sim_time_s, cloud_lag_s=0.0):
        clock = Clock()
        clock.clock.sec = int(sim_time_s)
        clock.clock.nanosec = int((sim_time_s - int(sim_time_s)) * 1e9)
        self.clock_pub.publish(clock)

        header = Header()
        stamp_s = sim_time_s - cloud_lag_s   # cloud_lag_s > 0：注入陈旧点云（清单 #7）
        header.stamp.sec = int(stamp_s)
        header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
        header.frame_id = 'base_link'
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, self.points))

    def test_perception_pipeline_is_wired(self):
        """Publish 40 frames at 10 Hz (sim time) and check the whole chain."""
        sim_time_s = 1.0
        for _ in range(40):
            self._publish_frame(sim_time_s)
            sim_time_s += 0.1
            deadline = self.node.get_clock().now().nanoseconds + int(0.15e9)
            while self.node.get_clock().now().nanoseconds < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)

        # ---- ① 链路通：确认航迹发出来了 --------------------------------
        with_obstacles = [msg for msg in self.obstacles if msg.obstacles]
        print(f'[test] 收到 {len(self.obstacles)} 条 ObstacleArray，'
              f'其中 {len(with_obstacles)} 条非空')
        self.assertGreater(
            len(with_obstacles), 5,
            '流水线没有输出确认航迹 —— 地面分割/聚类/拟合/跟踪里有一环断了')

        latest = with_obstacles[-1]

        # ---- ② 位置在 **map** 系 ---------------------------------------
        # ⚠️ 这一条抓的是「忘了做 base_link → map 变换」：忘了的话位置会是
        #    (12.0, 3.5) 而不是 (42.0, −48.25)，差 60 m，
        #    但**每一个数看起来都合理**，RViz 里也只是"目标画在别处"。
        self.assertEqual(latest.header.frame_id, 'map')
        expected_x = EGO_X_M + TARGET_X_M
        expected_y = EGO_Y_M + TARGET_Y_M
        best = min(
            latest.obstacles,
            key=lambda o: math.hypot(o.pose.position.x - expected_x,
                                     o.pose.position.y - expected_y))
        error_m = math.hypot(best.pose.position.x - expected_x,
                             best.pose.position.y - expected_y)
        print(f'[test] 目标真值 map=({expected_x:.2f}, {expected_y:.2f})，'
              f'感知 ({best.pose.position.x:.2f}, {best.pose.position.y:.2f})，'
              f'误差 {error_m:.3f} m')
        self.assertLess(error_m, 0.5, 'base_link → map 的变换没做或做错了')

        # ---- ③ 尺寸：L-Shape 接上了 -------------------------------------
        print(f'[test] 感知尺寸 {best.size_m.x:.2f} × {best.size_m.y:.2f} '
              f'× {best.size_m.z:.2f}（真值 {TARGET_LENGTH_M} × {TARGET_WIDTH_M} '
              f'× {TARGET_HEIGHT_M}）')
        self.assertAlmostEqual(best.size_m.x, TARGET_LENGTH_M, delta=0.4)
        self.assertAlmostEqual(best.size_m.y, TARGET_WIDTH_M, delta=0.4)

        # ---- ④ 分类：高度没被地面分割吃掉 -------------------------------
        # ⚠️ 这一条守的是 CP-P5-B 挖出来的那个根因：雷达少了 5 根线时
        #    车高量成 0.4 m，**146/146 辆车被判 STATIC**，而分类器本身没问题。
        #    这里高度不该被吃掉，所以必须是 VEHICLE（= 3）。
        print(f'[test] 分类 = {best.classification}（VEHICLE = 3）')
        self.assertEqual(best.classification, 3, '高度被吃掉了 → 车被判成 STATIC')

        # ---- ⑤ 地面没被当成障碍物 --------------------------------------
        # 地面 27 × 16 m，一旦漏进非地面点就会聚成一个巨大的簇。
        largest = max(max(o.size_m.x, o.size_m.y) for o in latest.obstacles)
        print(f'[test] 最大障碍物尺寸 {largest:.2f} m（地面漏出来的话会是十几米）')
        self.assertLess(largest, 8.0, '地面被当成障碍物了 —— 地面分割没接上')

        # ---- ⑥ ID 稳定：跟踪接上了 -------------------------------------
        # 目标静止、场景不变，ID 不该变。变了说明关联或生命周期断了。
        ids = {msg.obstacles[0].id for msg in with_obstacles[-10:]}
        print(f'[test] 最后 10 帧的 ID 集合：{sorted(ids)}')
        self.assertEqual(len(ids), 1, 'ID 在静止场景里都在跳 —— 关联或生命周期断了')

        # ---- ⑧ 浮空碎片不成目标（P9-S5c）----------------------------------
        # 合成云里有一片车顶高度的水平点片（见 build_cloud_points）。它离目标 3 m、
        # 0.9×0.5、底离地 1.5、延展 0 —— 是车顶远端环的复刻。浮空碎片门该把它剃掉：
        # 最后 10 帧里除目标外不许有别的确认航迹，诊断 floating_dropped 必须 ≥ 1。
        extras = [
            o for msg in with_obstacles[-10:] for o in msg.obstacles
            if math.hypot(o.pose.position.x - expected_x, o.pose.position.y - expected_y) > 1.0]
        floating_dropped = [
            float(value.value) for msg in self.diagnostics for status in msg.status
            for value in status.values if value.key == 'floating_dropped']
        print(f'[test] 目标之外的确认航迹 {len(extras)} 个（最后 10 帧），'
              f'floating_dropped 最近一拍 {floating_dropped[-1] if floating_dropped else None}')
        extras_desc = [
            (round(o.pose.position.x, 1), round(o.pose.position.y, 1),
             round(o.size_m.x, 2), round(o.size_m.y, 2), round(o.size_m.z, 2))
            for o in extras[:3]]
        self.assertEqual(len(extras), 0, f'浮空碎片成了目标：{extras_desc}')
        self.assertTrue(floating_dropped and floating_dropped[-1] >= 1,
                        '浮空碎片门一个都没剃 —— 门没接上或合成碎片没进流水线')

        # ---- ⑦ 单帧耗时（CP-P5-B 第 8 条的 CI 版）----------------------
        # ⚠️ 这里的点数（约 3.5k）远小于真实的 57.6k，所以**只能抓数量级回归**，
        #    不能替代 CP-P5-B 那次 5.02 ms 的实测。判据放到 100 ms 是有意的。
        totals = [
            value.value for msg in self.diagnostics for status in msg.status
            for value in status.values if value.key == 'total_ms']
        self.assertTrue(totals, '诊断话题没数据')
        worst = max(float(v) for v in totals)
        print(f'[test] 单帧耗时最大 {worst:.2f} ms（判据 100，点数只有真实的 6%）')
        self.assertLess(worst, 100.0)

        # ---- 异常注入清单 #7：陈旧点云必须丢弃并计数（P9-S5b） ----------------
        # 钟照走、点云的 stamp 落后 1.0 s（> max_cloud_age_s 0.15）：这段里不许有
        # 任何 ObstacleArray 出来（用旧帧算出来的位置对应的是过去的时刻，下游会当成
        # 现在的），dropped_stale_clouds 要涨够 10；随后正常帧一到立刻恢复输出。
        n_before = len(self.obstacles)
        for _ in range(10):
            self._publish_frame(sim_time_s, cloud_lag_s=1.0)
            sim_time_s += 0.1
            deadline = self.node.get_clock().now().nanoseconds + int(0.15e9)
            while self.node.get_clock().now().nanoseconds < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
        n_during_stale = len(self.obstacles) - n_before
        for _ in range(5):
            self._publish_frame(sim_time_s)
            sim_time_s += 0.1
            deadline = self.node.get_clock().now().nanoseconds + int(0.15e9)
            while self.node.get_clock().now().nanoseconds < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
        dropped = [
            float(value.value) for msg in self.diagnostics for status in msg.status
            for value in status.values if value.key == 'dropped_stale_clouds']
        print(f'[test] 陈旧点云 10 帧：期间输出 {n_during_stale} 条，'
              f'dropped_stale_clouds 计数 {max(dropped) if dropped else None}，'
              f'恢复后又收到 {len(self.obstacles) - n_before - n_during_stale} 条')
        self.assertEqual(n_during_stale, 0, '陈旧点云被当成现在的算了 —— max_cloud_age_s 没生效')
        self.assertTrue(dropped and max(dropped) >= 10.0, '丢弃计数没涨 —— 诊断没记录陈旧帧')
        self.assertGreater(len(self.obstacles) - n_before - n_during_stale, 0,
                           '正常帧恢复后没有输出 —— 陈旧守卫把流水线卡死了')


@launch_testing.post_shutdown_test()
class TestPerceptionNodeExitsCleanly(unittest.TestCase):
    """The node must not crash on shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])


assert os.environ.get('ROS_DOMAIN_ID'), (
    'ROS_DOMAIN_ID 必须由 CMakeLists 设定。colcon 按**包**并行跑测试，'
    '默认 domain 0 下本测试会与 ads_control / ads_localization 的闭环共享话题与 TF。')
