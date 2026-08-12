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

"""Score a perception run against ground truth — CP-P5-B.

判据来自 tasks/plan.md 的 CP-P5-B 表，**这里不重新发明**。

⚠️ **检测率按「命中率」定，不是按点数** —— S1 体检实测目标在连续帧之间
**闪烁**（同一锥桶连续三帧 8 点→8 点→0 点），命中率只有 33–74%。
拿"平均点数"或"中位点数"去定判据都会误导（前者说稳定有 4 个点、
后者说完全看不见，而真相是时有时无）。

⚠️ **路侧杆件不是虚警。** 感知会把路灯杆检测成静态障碍物 —— 那是**对的**，
它们真实存在，只是不在真值列表里（真值只列 obstacles.yaml 与 dynamic_actors.yaml
里的东西）。所以虚警只统计**自车车道内**的多余目标：车道外的东西
不影响行车，而把它们算成虚警会让判据永远红。

用法（需要一个正在跑的仿真，perception:=true）：
    ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false \\
        perception:=true dynamic:=both
    python3 scripts/record_perception_run.py --duration-s 45 --out /tmp/p5.csv
"""

import argparse
import csv
import math
import time
from collections import defaultdict

from ads_msgs.msg import ObstacleArray
import rclpy
from rclpy.node import Node
import tf2_ros

# 与真值配对的最大距离，m。
#
# ⚠️ 取 2.5 而不是 0.5：感知给的是**可见面**为主的包围盒中心，而真值是
#    几何中心 —— 雷达打不到背面，两者天然差"进深的一半"（4.4 m 的车
#    正对时就是 2.2 m，见 S2 的实测）。用 0.5 去配对的话，一辆完全
#    正确检测到的车会被判成"没检测到 + 一个虚警"。
#    **配对半径与位置误差判据是两回事**：前者问"是不是同一个东西"，
#    后者问"位置差多少"。
MATCH_RADIUS_M = 2.5

# 自车车道半宽，m（用于判虚警）。与 config/dynamic_actors.yaml 一致。
EGO_LANE_HALF_WIDTH_M = 1.75
EGO_LANE_CENTER_Y_M = -51.75

BUCKETS = [(0, 10), (10, 15), (15, 20), (20, 25), (25, 30)]

CLASS_NAMES = {0: 'UNKNOWN', 1: 'PEDESTRIAN', 2: 'BICYCLE', 3: 'VEHICLE', 4: 'STATIC'}


def stamp_to_seconds(stamp) -> float:
    """Convert a ROS time message to float seconds.

    :param stamp: builtin_interfaces/Time
    :return: 秒
    """
    return stamp.sec + stamp.nanosec * 1e-9


class PerceptionScorer(Node):
    """Pair each ground-truth object with the nearest perceived one, frame by frame."""

    def __init__(self, match_radius_m: float):
        super().__init__('perception_scorer')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.match_radius_m = match_radius_m
        self.truth_frames = []
        self.rows = []
        # {真值 id: [感知 id, ...]}，用来数 ID 切换
        self.assigned_ids = defaultdict(list)
        self.false_positives_in_lane = 0
        self.perceived_frames = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # ⚠️ 真值只有评测脚本能订阅（SPEC §4.1）。本脚本正是评测脚本。
        self.create_subscription(ObstacleArray, '/perception/obstacles_gt', self._on_truth, 20)
        self.create_subscription(ObstacleArray, '/perception/obstacles', self._on_perceived, 20)

    def _on_truth(self, msg):
        self.truth_frames.append((stamp_to_seconds(msg.header.stamp), msg))
        if len(self.truth_frames) > 200:
            del self.truth_frames[:100]

    def _ego_position(self):
        """Ego position in map, or None when TF is not available yet."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _on_perceived(self, msg):
        if not self.truth_frames:
            return
        ego = self._ego_position()
        if ego is None:
            return
        t = stamp_to_seconds(msg.header.stamp)
        truth = min(self.truth_frames, key=lambda item: abs(item[0] - t))
        if abs(truth[0] - t) > 0.15:
            return
        self.perceived_frames += 1

        matched_perceived = set()
        for gt in truth[1].obstacles:
            gx, gy = gt.pose.position.x, gt.pose.position.y
            distance_m = math.hypot(gx - ego[0], gy - ego[1])

            best = None
            best_distance = self.match_radius_m
            for index, obstacle in enumerate(msg.obstacles):
                if index in matched_perceived:
                    continue
                d = math.hypot(obstacle.pose.position.x - gx, obstacle.pose.position.y - gy)
                if d < best_distance:
                    best, best_distance = index, d

            row = {
                't_s': f'{t:.3f}',
                'gt_id': gt.id,
                'gt_class': CLASS_NAMES.get(gt.classification, '?'),
                'range_m': f'{distance_m:.3f}',
                'detected': 0,
                'position_error_m': '',
                'velocity_error_mps': '',
                'velocity_sign_ok': '',
                'perceived_id': '',
                'perceived_class': '',
            }
            if best is not None:
                matched_perceived.add(best)
                obstacle = msg.obstacles[best]
                row['detected'] = 1
                row['position_error_m'] = f'{best_distance:.3f}'
                row['perceived_id'] = obstacle.id
                row['perceived_class'] = CLASS_NAMES.get(obstacle.classification, '?')
                gvx, gvy = gt.velocity_mps.x, gt.velocity_mps.y
                pvx, pvy = obstacle.velocity_mps.x, obstacle.velocity_mps.y
                row['velocity_error_mps'] = f'{math.hypot(pvx - gvx, pvy - gvy):.3f}'
                # ⚠️ 符号**单独判**：符号反了大小照样对，而 P7 会因此
                #    认为"对方要开走"而不让行。只对确实在动的目标判。
                if math.hypot(gvx, gvy) > 0.5:
                    row['velocity_sign_ok'] = int(pvx * gvx + pvy * gvy > 0.0)
                self.assigned_ids[gt.id].append(obstacle.id)
            self.rows.append(row)

        # ---- 虚警：没配上真值、**且在自车车道内**的感知目标 ----------------
        # ⚠️ 车道外的多余目标不算虚警 —— 路侧杆件是**真实存在**的障碍物，
        #    只是不在真值列表里。把它们算成虚警会让判据永远红。
        for index, obstacle in enumerate(msg.obstacles):
            if index in matched_perceived:
                continue
            if abs(obstacle.pose.position.y - EGO_LANE_CENTER_Y_M) <= EGO_LANE_HALF_WIDTH_M:
                self.false_positives_in_lane += 1

    def write(self, path: str) -> None:
        """Dump the per-frame rows so the numbers can be re-checked later."""
        if not self.rows:
            return
        with open(path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

    def score(self) -> bool:
        """Print the CP-P5-B table.

        :return: 全部通过为 True
        """
        print(f'\n===== CP-P5-B 感知实测（{self.perceived_frames} 帧配对）=====')
        if not self.rows:
            print('✗ 一条样本都没有 —— perception:=true 了吗？TF 通吗？')
            return False

        # ---- 按类别与距离分桶的检测率 ----
        print('\n检测率（按真值类别与距离；括号内是样本帧数）')
        header = '类别'.ljust(14) + ''.join(f'{lo}-{hi}m'.rjust(13) for lo, hi in BUCKETS)
        print(header)
        print('-' * len(header))
        rates = {}
        for class_name in sorted({row['gt_class'] for row in self.rows}):
            line = class_name.ljust(14)
            for bucket, (lo, hi) in enumerate(BUCKETS):
                subset = [
                    row for row in self.rows
                    if row['gt_class'] == class_name and lo <= float(row['range_m']) < hi]
                if not subset:
                    line += '—'.rjust(13)
                    continue
                rate = sum(row['detected'] for row in subset) / len(subset)
                rates[(class_name, bucket)] = rate
                line += f'{rate * 100:>7.1f}%({len(subset):>3})'
            print(line)

        detected = [row for row in self.rows if row['detected']]
        ok = True

        def check(label, value, limit, unit='', greater_is_better=False):
            nonlocal ok
            passed = value > limit if greater_is_better else value < limit
            ok = ok and passed
            arrow = '>' if greater_is_better else '<'
            print(f'{label:<34}{value:>10.4f}{arrow:>4} {limit:<8} '
                  f'{"PASS" if passed else "FAIL"}  {unit}')

        print('\n项                                     实测    判据  结果')
        # ① NPC 车检测率（20 m 内）
        vehicle_close = [
            row for row in self.rows
            if row['gt_class'] == 'VEHICLE' and float(row['range_m']) < 20.0]
        if vehicle_close:
            check('NPC 车检测率 (20 m 内)',
                  sum(r['detected'] for r in vehicle_close) / len(vehicle_close),
                  0.95, f'n={len(vehicle_close)}', greater_is_better=True)
        # ② 行人检测率（15 m 内）
        ped_close = [
            row for row in self.rows
            if row['gt_class'] == 'PEDESTRIAN' and float(row['range_m']) < 15.0]
        if ped_close:
            check('行人检测率 (15 m 内)',
                  sum(r['detected'] for r in ped_close) / len(ped_close),
                  0.90, f'n={len(ped_close)}', greater_is_better=True)
        # ③ 位置误差
        if detected:
            errors = sorted(float(r['position_error_m']) for r in detected)
            check('位置误差 (95 分位, m)', errors[int(len(errors) * 0.95)], 0.5, 'm')
        # ④ 速度误差 + 符号
        moving = [r for r in detected if r['velocity_error_mps']]
        if moving:
            v_errors = sorted(float(r['velocity_error_mps']) for r in moving)
            check('速度误差 (95 分位, m/s)', v_errors[int(len(v_errors) * 0.95)], 1.0, 'm/s')
        signed = [r for r in detected if r['velocity_sign_ok'] != '']
        if signed:
            wrong = sum(1 for r in signed if int(r['velocity_sign_ok']) == 0)
            print(f'{"速度符号错的样本":<34}{wrong:>10}   == 0        '
                  f'{"PASS" if wrong == 0 else "FAIL"}  n={len(signed)}')
            ok = ok and wrong == 0
        # ⑤ ID 切换
        switches = sum(len(set(ids)) - 1 for ids in self.assigned_ids.values() if ids)
        print(f'{"ID 切换次数":<34}{switches:>10}   <= 2        '
              f'{"PASS" if switches <= 2 else "FAIL"}')
        ok = ok and switches <= 2
        # ⑦ 车道内虚警
        print(f'{"车道内虚警帧次":<34}{self.false_positives_in_lane:>10}   == 0        '
              f'{"PASS" if self.false_positives_in_lane == 0 else "FAIL"}')
        ok = ok and self.false_positives_in_lane == 0

        print('\n全部通过' if ok else '\n有判据未通过')
        return ok


def main() -> int:
    """Entry point.

    :return: 0 表示全部判据通过
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=45.0)
    parser.add_argument('--out', default='/tmp/p5_perception.csv')
    parser.add_argument(
        '--match-radius-m', type=float, default=MATCH_RADIUS_M,
        help='与真值配对的最大距离。⚠️ 它与「位置误差」判据是两回事，见文件头')
    args = parser.parse_args()

    rclpy.init()
    node = PerceptionScorer(args.match_radius_m)
    deadline = time.monotonic() + args.duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.write(args.out)
    print(f'CSV: {args.out}')
    passed = node.score()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
