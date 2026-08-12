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
Localization closed-loop wiring test with no GPU: fake sensors + localization_node.

L3-G（SPEC §8）：**不需要 Gazebo、不需要 GPU、能进 CI** 的定位闭环。
与 ads_control 的 test_closed_loop.py 是同一层的两个用例，各管一条链路。

它验的是**接线**，不是定位精度：
    话题名对不对、QoS 兼不兼容、参数装没装上、大地原点与杆臂搬对没有、
    **map→odom 这一段发的是不是它**、状态机进不进得了 NDT_AIDED。

⚠️ **不要用它替代 CP-P4-B。** 扫描是从先验地图里抠出来的（世界与地图同源）、
   没有遮挡、没有运动畸变 —— 三条理由见 fake_sensors_node.cpp 的文件头。
   这里的判据只到「链路通、误差没发散」，收紧到 0.30 m 得到的不是更强的
   保证，而是一个在 CI 上随机变红的测试。

**为什么值得有这一层**：CP-P4-B 那次调试挖出来的五个根因里，有三个是纯粹的
接线/时序问题（点云比 NDT 快、杆臂没减、初值航向没给），而它们的现场表现
全都是「定位飘」—— 于是人第一反应是去查 NDT。有了这一层，同类回归一次推送就红。

## 故障注入实测（2026-08-10，写完立刻做的）

「全绿」本身不是证据。逐条往里塞已知的错，看它红不红：

| 注入 | 结果 | 说明 |
|---|---|---|
| 冷启动航向先验错 90° | **红** | 断言 ⑤（map→odom 偏起点 5.358 m）。NDT 靠粗网格恢复了一部分，但没回到位 |
| NDT 全帧判退化（等于关掉 NDT） | **红** | 断言 ①，状态序列全是 GNSS_ONLY |
| 多一个静态 map→odom 发布者 | **红** | 断言 ④。⚠️ 加这条断言**之前**它是绿的，见下 |
| 新息门限阈值调到 0.5 mm | **红** | 断言 ①。证明门限确实接上了：每帧都被拒 → 全程 GNSS_ONLY。（它先撞上 ① 而不是 ③，因为 ① 排在前面） |
| 节点侧杆臂清零 | **绿 —— 抓不到** | 见下 |

两条「抓不到」的实测结论，比抓到的更值得记：

1. **多一个 map→odom 发布者，数值上看不出来。** 末段位置误差只从 0.012 m
   变成 0.043 m，全部数值判据照样绿。所以断言 ③ **机械地查 `/tf_static`**，
   而不是指望从误差里看出端倪。
2. **这一层看不见杆臂补偿。** 节点侧杆臂清零后误差 0.012 → 0.018 m。
   NDT 一锁上，GNSS（σ=2 m）在融合里就几乎没有权重，0.5 m 的杆臂偏差
   被压进噪声里。杆臂归 CP-P4-B 验 —— 那里 GNSS 是唯一的绝对基准。
"""

import math
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
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

# 同目录的模块必须先把这个目录加进 sys.path —— 理由与 ads_control 的
# test_closed_loop.py 完全一样（launch_test 按路径执行本文件，不入栈其所在目录），
# 所以下面那行 import 必然在代码之后，显式 noqa 而不是去改 lint 配置。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from localization_fixtures import (  # noqa: E402,I100
    fake_sensors_params, localization_params, START_X_M, START_Y_M, START_YAW_RAD)


@pytest.mark.launch_test
def generate_test_description():
    """Launch fake sensors plus localization_node — deliberately no static map to odom TF."""
    fake_sensors = os.environ['FAKE_SENSORS_EXECUTABLE']

    return launch.LaunchDescription([
        # 假传感器。**它是 /clock 的来源**，所以自己 use_sim_time=false
        # （由节点的 parameter_overrides 强制，launch 改不动）。
        launch.actions.ExecuteProcess(
            cmd=[fake_sensors, '--ros-args'] + [
                arg
                for key, value in fake_sensors_params().items()
                for arg in ('-p', f'{key}:={value}')
            ],
            output='screen'),

        # ⚠️ **这里没有 static_transform_publisher，那是有意的。**
        #    map→odom 归定位发（SPEC §3.3：每一段有且只有一个发布者）。
        #    真栈里靠 stack.launch.py 的 localization 参数二选一，这里靠"压根不写"。
        #    多一个发布者**不会报错、也不一定看得出来** —— 见下面断言 ③ 的说明，
        #    那里记着实测：加一个静态的之后所有数值判据照样全绿。
        Node(
            package='ads_localization', executable='localization_node',
            name='localization_node',
            parameters=[localization_params()], output='screen'),

        launch_testing.actions.ReadyToTest(),
    ])


class TestLocalizationClosedLoop(unittest.TestCase):
    """Run one synthetic drive and check that the localization wiring holds up."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('localization_closed_loop_tester')
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.diagnostics = []
        self.poses = 0
        self.truth = None
        self.samples = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # /tf_static 上出现 map→odom = 有第二个发布者。见 test 里那条断言的说明。
        self.static_edges = set()
        self.node.create_subscription(
            TFMessage, '/tf_static', self._on_tf_static,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.node.create_subscription(
            DiagnosticArray, '/localization/diagnostics', self._on_diag, 50)
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/localization/pose',
            lambda _msg: setattr(self, 'poses', self.poses + 1), 50)
        # /ego_pose_gt 是**评测基准**，只有测试能订阅（SPEC §4.1）。
        # 假传感器用 reliable 发，这里也用 reliable，QoS 才匹配。
        self.node.create_subscription(
            Odometry, '/ego_pose_gt', self._on_truth,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def tearDown(self):
        self.node.destroy_node()

    def _on_tf_static(self, msg):
        for tf in msg.transforms:
            self.static_edges.add((tf.header.frame_id.lstrip('/'), tf.child_frame_id.lstrip('/')))

    def _on_diag(self, msg):
        self.diagnostics.append({
            'state': msg.status[0].message,
            **{kv.key: kv.value for kv in msg.status[0].values},
        })

    def _on_truth(self, msg):
        self.truth = msg
        # 每收到一帧真值就查一次 TF。**必须走 map→base_link 整条链**，
        # 而不是直接读 /localization/pose —— 后者只证明节点算出了一个位姿，
        # 前者才证明 map→odom 这一段真的发出去了、且能与 odom→base_link 接上。
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        est_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tq = msg.pose.pose.orientation
        truth_yaw = math.atan2(
            2.0 * (tq.w * tq.z + tq.x * tq.y), 1.0 - 2.0 * (tq.y * tq.y + tq.z * tq.z))
        heading_error_rad = math.atan2(
            math.sin(est_yaw - truth_yaw), math.cos(est_yaw - truth_yaw))
        self.samples.append({
            'truth_x': msg.pose.pose.position.x,
            'truth_y': msg.pose.pose.position.y,
            'est_x': t.x,
            'est_y': t.y,
            'position_error_m': math.hypot(
                t.x - msg.pose.pose.position.x, t.y - msg.pose.pose.position.y),
            'heading_error_deg': math.degrees(abs(heading_error_rad)),
        })

    def _spin(self, wall_seconds):
        deadline = time.monotonic() + wall_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_localization_wiring(self):
        # 假传感器 1.0 倍速：4 s 静止（GNSS 攒 30 帧初始化）+ 约 16 s 行驶。
        # 给 45 s 墙钟余量 —— CI 机器起节点可能慢几秒。
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if len(self.samples) > 150 and self.diagnostics:
                if self.diagnostics[-1]['state'] == 'NDT_AIDED':
                    break

        states = [d['state'] for d in self.diagnostics]
        self.assertGreater(
            len(self.diagnostics), 0,
            '一条 /localization/diagnostics 都没收到 —— localization_node 起来了吗？')
        self.assertGreater(self.poses, 0, '/localization/pose 一条都没发')
        self.assertGreater(
            len(self.samples), 100,
            f'map→base_link 只查通了 {len(self.samples)} 次 —— '
            f'map→odom 这一段发出去了吗？经历过的状态：{set(states)}')

        # ---- ① 状态机必须进得了 NDT_AIDED ----
        # 进不去说明点云链路（QoS / frame / 时间戳 / 先验地图路径）有一环断了。
        # ⚠️ 判"最后是不是 NDT_AIDED"而不是"出现过" —— 出现过一次然后掉回
        #    GNSS_ONLY 正是 CP-P4-B 那次失锁的形态。
        self.assertIn('NDT_AIDED', states, f'从未进入 NDT_AIDED，状态序列：{states}')
        self.assertEqual(
            states[-1], 'NDT_AIDED',
            f'末态不是 NDT_AIDED 而是 {states[-1]}，状态序列：{states}')

        # ---- ② 陈旧点云一帧都不该丢 ----
        # 丢帧说明 NDT 追不上雷达周期。CP-P4-B 的五个根因里最隐蔽的就是这个：
        # 它不报错，只让配准结果对应两三帧之前的位姿。
        dropped = int(float(self.diagnostics[-1]['dropped_stale_clouds']))
        self.assertEqual(dropped, 0, f'丢了 {dropped} 帧陈旧点云 —— NDT 没跟上雷达周期')

        # ---- ③ 新息门限不许误杀好帧 ----
        # 这一条不是「门限有没有生效」，而是「阈值有没有定得太紧」。
        # 太紧的代价是好帧被丢 → 掉进 GNSS_ONLY（σ=2 m）→ 定位变差，
        # 而日志里只有一条 3 s 一次的节流告警，很容易被当成噪声忽略。
        rejected = int(float(self.diagnostics[-1]['ndt_rejected_innovation']))
        self.assertEqual(
            rejected, 0,
            f'新息门限拒了 {rejected} 帧 —— 要么真锁错了，要么阈值太紧在误杀好帧')

        # ---- ③b 开机自举必须真的执行过 ----
        # ⚠️ 这条守着一个已经发生过的缺陷（2026-08-12 复检发现）：粗网格的
        #    参数推导、声明、消费代码全都在，唯独**构造那一行缺失** ——
        #    ndt_coarse_map_ 恒为空指针，bootstrap 与失锁恢复整条是死代码，
        #    而 L1 用例全绿（它们自己构造粗网格）、本测试当时也全绿。
        #    「参数声明得再讲究，也证明不了消费它的对象存在」——
        #    所以这里机械地断言自举计数，而不是相信代码结构。
        recovery = int(float(self.diagnostics[-1]['ndt_recovery_attempts']))
        self.assertGreaterEqual(
            recovery, 1,
            '开机自举一次都没执行 —— ndt_coarse_map_ 是不是又没被构造？')

        # ---- ④ map→odom 只许有**一个**发布者 ----
        # SPEC §3.3。这条**必须机械地查**，不能指望从数值上看出来 ——
        # 实测（本文件的故障注入 C）：再挂一个静态 map→odom，末段位置误差
        # 只从 0.012 m 变成 0.043 m，所有数值判据照样全绿。
        #
        # 原因是 tf2 的行为**取决于两个发布者的启动顺序**：某个 frame 第一次
        # 被写入时决定它用静态缓存还是时间缓存，静态先到时后来的动态变换会
        # 把它覆盖掉，于是"看起来正常"。反过来先到的是动态就不是这个结果。
        # **一个结果取决于启动顺序的系统，不能靠观察它这一次的输出来验收。**
        self.assertNotIn(
            ('map', 'odom'), self.static_edges,
            '/tf_static 上出现了 map→odom —— 这一段归 localization_node 动态发，'
            '多一个静态发布者不会报错，但哪一份生效取决于启动顺序')

        # ---- ⑤ map→odom 必须约等于自车起点 ----
        # 假传感器的 odom 原点钉在起点且不漂，所以这一段的真值是常量。
        # **这是"发的是不是正确那一段"的直接检验** —— 错发成 map→base_link
        # 的话这里会读到一个随车移动的值（几十米）。
        tf = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        offset_m = math.hypot(
            tf.transform.translation.x - START_X_M, tf.transform.translation.y - START_Y_M)
        self.assertLess(
            offset_m, 1.0,
            f'map→odom 偏离自车起点 {offset_m:.3f} m —— 发的是 map→odom 还是 map→base_link？')

        # ---- ⑥ 误差不能发散 ----
        # 判据取**末段 30 个样本**（约 3 s）。开头那几秒一定是差的：
        # 初值来自 30 帧 GNSS 的平均（σ 2 m / √30 ≈ 0.37 m），NDT 要几帧才收进去。
        # 拿全程平均去判等于把初始化的瞬态算进稳态指标里。
        tail = self.samples[-30:]
        mean_position_error_m = sum(s['position_error_m'] for s in tail) / len(tail)
        max_heading_error_deg = max(s['heading_error_deg'] for s in tail)

        # 判据的量级刻意比 CP-P4-B（0.30 m / 2.0°）**松**，理由见文件头：
        # 这一层没有模型失配、没有遮挡、没有运动畸变，理论上应当更准；
        # 但它跑在 CI 机器上，调度抖动会让 NDT 偶尔多丢一拍。
        # 定在 0.60 m / 3.0° 是"没发散"的判据，不是精度考核。
        self.assertLess(
            mean_position_error_m, 0.60,
            f'末段平均位置误差 {mean_position_error_m:.3f} m —— 定位发散了')
        self.assertLess(
            max_heading_error_deg, 3.0,
            f'末段最大航向误差 {max_heading_error_deg:.3f}° —— 航向发散了')

        # ⚠️ **这一层看不见杆臂补偿写没写对。** 实测（故障注入 A）：把节点侧的
        #    杆臂清零、假传感器照常加，末段误差 0.012 → 0.018 m，全绿。
        #    因为 NDT 一旦锁上，GNSS（σ=2 m）在融合里的权重就微乎其微，
        #    0.5 m 的杆臂偏差被压进了噪声里。它只在初始化那一瞬间可见，
        #    而那时误差本来就被 GNSS 均值的 σ≈0.37 m 主导，分不开。
        #    ——**杆臂归 CP-P4-B 验**（那里 GNSS 是唯一的绝对基准）。
        print(
            f'\n  [L3-G 定位] {len(self.samples)} 个样本，诊断 {len(self.diagnostics)} 拍，'
            f'末态 {states[-1]}，末段平均位置误差 {mean_position_error_m:.4f} m，'
            f'最大航向误差 {max_heading_error_deg:.3f}°，'
            f'map→odom 偏离起点 {offset_m:.3f} m，'
            f'NDT 单帧 {float(self.diagnostics[-1]["ndt_time_ms"]):.1f} ms，'
            f'新息峰值 {float(self.diagnostics[-1]["ndt_innovation_max_m"]):.4f} m，'
            f'扫描 {int(float(self.diagnostics[-1]["scan_points"]))} 点，'
            f'起点朝向 {START_YAW_RAD:.2f} rad')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """
    Require every launched process to exit cleanly.

    ⚠️ 崩溃会被 launch 报成 "process has finished cleanly"（本仓库踩过，见
    CLAUDE.md 陷阱表）—— 唯一可信的证据是**退出码**，所以这条断言不可省。
    """

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
