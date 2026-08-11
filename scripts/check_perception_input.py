#!/usr/bin/env python3
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

"""Measure how many lidar points each ground-truth target actually gets, by distance.

P5-S1 (1.6) 的**感知输入体检** —— 在写任何感知算法**之前**回答一个问题：

    「雷达到底能看到什么？各个目标在各个距离上有几个点？」

⚠️ **这不是可有可无的一步。** 欧式聚类的 `min_cluster_size` 直接由它决定，
而那个参数是**安全关键**的：调到 20，锥桶在 18 m 外就检测不到，
症状是「车快撞上了才开始绕」—— 看起来像规划器反应慢。

⚠️ **必须实测，不能拿理论值填参数。** plan.md 的 P5-0 有一张按雷达角分辨率
算出来的理论表（锥桶 20 m 处约 15 点）。理论表用来**判断量级**，
实测用来**定参数** —— 两者对不上时先查为什么，那个差异本身就是信息
（遮挡？自车裁剪？点云 QoS 丢帧？）。

用法（需要一个正在跑的仿真，且 dynamic 非 none）：
    ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false dynamic:=both
    python3 scripts/check_perception_input.py --duration-s 40
"""

import argparse
import math
from collections import defaultdict

from ads_msgs.msg import ObstacleArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

# 距离分桶。10 m 一档，最远 40 m —— 再远锥桶只剩个位数点，没有讨论价值。
BUCKETS = [(0, 10), (10, 15), (15, 20), (20, 25), (25, 30), (30, 40)]


def quaternion_to_yaw(q) -> float:
    """Extract yaw from a quaternion.

    :param q: geometry_msgs/Quaternion
    :return: 偏航角，rad
    """
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PerceptionInputProbe(Node):
    """Count, per target and per distance bucket, how many lidar points land inside it."""

    def __init__(self, margin_m: float):
        super().__init__('perception_input_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.margin_m = margin_m
        self.truth = None
        self.frames = 0
        # {(目标id, 分类, 桶下标): [点数, ...]}
        self.samples = defaultdict(list)
        self.classification_of = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ⚠️ 真值只有评测脚本能订阅（SPEC §4.1）。本脚本正是评测脚本。
        self.create_subscription(ObstacleArray, '/perception/obstacles_gt', self._on_truth, 10)
        # ⚠️ 点云用 reliable —— 与 lidar_preprocessor 的发布端一致。
        #    best-effort 会静默丢帧（实测只剩标称的 35%），而丢帧会让点数统计
        #    偏低，于是 min_cluster_size 被定得过小，虚警变多。
        self.create_subscription(
            PointCloud2, '/lidar/points', self._on_cloud,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

    def _on_truth(self, msg):
        self.truth = msg
        for obstacle in msg.obstacles:
            self.classification_of[obstacle.id] = obstacle.classification

    def _on_cloud(self, msg):
        if self.truth is None:
            return
        # 点云在 base_link 系，真值在 map 系。**必须走 TF** —— 手写变换是 SPEC §3.3 禁的。
        try:
            tf = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())
        except Exception:
            return
        t = tf.transform.translation
        yaw = quaternion_to_yaw(tf.transform.rotation)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        points = list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not points:
            return
        self.frames += 1

        for obstacle in self.truth.obstacles:
            # 目标中心（map）→ base_link
            cx_map = obstacle.pose.position.x
            cy_map = obstacle.pose.position.y
            cz_map = obstacle.pose.position.z
            cx = cos_y * cx_map - sin_y * cy_map + t.x
            cy = sin_y * cx_map + cos_y * cy_map + t.y
            cz = cz_map + t.z

            distance_m = math.hypot(cx, cy)
            bucket = next(
                (i for i, (lo, hi) in enumerate(BUCKETS) if lo <= distance_m < hi), None)
            if bucket is None:
                continue

            # 目标自身的朝向（在 base_link 系里）
            obstacle_yaw = quaternion_to_yaw(obstacle.pose.orientation) + yaw
            cos_o, sin_o = math.cos(-obstacle_yaw), math.sin(-obstacle_yaw)
            half_l = 0.5 * obstacle.size_m.x + self.margin_m
            half_w = 0.5 * obstacle.size_m.y + self.margin_m
            half_h = 0.5 * obstacle.size_m.z + self.margin_m

            count = 0
            for px, py, pz in points:
                dx, dy, dz = px - cx, py - cy, pz - cz
                # 转到目标自身的轴系再判包围盒 —— 用轴对齐盒近似会在车斜着时
                # 多算进一大片空气，把点数统计做得偏乐观。
                lx = cos_o * dx - sin_o * dy
                ly = sin_o * dx + cos_o * dy
                if abs(lx) <= half_l and abs(ly) <= half_w and abs(dz) <= half_h:
                    count += 1
            self.samples[(obstacle.id, bucket)].append(count)

    def report(self) -> None:
        """Print the measured point counts, and say plainly what they imply."""
        names = {0: 'UNKNOWN', 1: 'PEDESTRIAN', 2: 'BICYCLE', 3: 'VEHICLE', 4: 'STATIC'}
        print(f'\n共处理 {self.frames} 帧点云\n')
        if not self.samples:
            print('✗ 一个样本都没有 —— 真值发出来了吗？TF map→base_link 通吗？'
                  'dynamic 参数是不是 none？')
            return

        ids = sorted({key[0] for key in self.samples})
        header = '目标'.ljust(22) + ''.join(f'{lo}-{hi}m'.rjust(14) for lo, hi in BUCKETS)
        print(header)
        print('-' * len(header))
        for obstacle_id in ids:
            label = f'id={obstacle_id} {names.get(self.classification_of.get(obstacle_id), "?")}'
            row = label.ljust(22)
            for bucket in range(len(BUCKETS)):
                values = self.samples.get((obstacle_id, bucket))
                if not values:
                    row += '—'.rjust(14)
                    continue
                hits = sorted(v for v in values if v > 0)
                rate = len(hits) / len(values)
                # 「命中率 / 命中帧的中位点数」。**两个数缺一不可**，理由见下。
                median = hits[len(hits) // 2] if hits else 0
                row += f'{rate * 100:>5.0f}%/{median:<3d}({len(values):>2})'
            print(row)

        print('\n每格是 **命中率 / 命中帧的中位点数 (总样本帧数)**。')
        print('''
⚠️ **为什么必须给「命中率」，只看点数会得出错误结论**（2026-08-11 实测）：

   锥桶（0.8 m 高）在 20 m 处，**只有一条扫描线**能打到它 —— 相邻两线在
   那个距离相隔 20 × tan(1.09°) = 0.38 m，而锥桶总共才 0.8 m 高。
   打不打得中，取决于锥桶顶部相对于扫描线的**相位**；而车在动，相位在变。
   实测同一个锥桶连续三帧：8 点 → 8 点 → **0 点**。

   于是「中位点数」这个统计量会给出 0（超过一半的帧确实是 0），
   而「平均点数」会给出 4 左右 —— **两个都是误导**：
   前者说"完全看不见"，后者说"稳定有 4 个点"，而真相是**时有时无**。

   这对 P5 是根本性的：**目标会在连续帧之间闪烁**，所以
     · 聚类的 min_cluster_size 再小也救不了「这一帧一个点都没有」
     · **跟踪的航迹生命周期（N 帧确认 / M 帧删除）才是那个救命的东西**
     · CP-P5-B 的「检测率」判据必须按**命中率**定，而不是按点数

⚠️ 拿这张表去定 min_cluster_size：它必须小于「最远要检测到的距离」那一档
   **命中帧的**点数。调大 → 最远检测距离缩一截；调小 → 虚警变多。''')


def main() -> int:
    """Entry point.

    :return: 进程退出码
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=40.0, help='采样时长（墙钟秒）')
    parser.add_argument(
        '--margin-m', type=float, default=0.15,
        help=('判包围盒时的外扩量。取 0.15：雷达测距噪声 σ=1 cm，但**真值是包围盒**'
              '而模型有轮子等突出物，不留余量会把边缘点漏掉'))
    args = parser.parse_args()

    rclpy.init()
    node = PerceptionInputProbe(args.margin_m)
    import time
    deadline = time.monotonic() + args.duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.report()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
