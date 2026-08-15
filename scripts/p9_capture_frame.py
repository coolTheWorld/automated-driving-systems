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

"""P9-S1 感知域诊断采样：一轮拿全三嫌疑的数据.

在 CARLA 栈（perception:=true dynamic:=both）旁边跑，采 N 帧后输出：

  ① 挂高嫌疑   —— /lidar/points（base_link 系）的 z 直方图 + 地面诊断的
     ground_height_m（系统性 ≠ 0 = 传感器挂载基准差的直接读数）；
  ② mesh 嫌疑  —— ground_ratio / slope_rejected / ground_slope_deg
     （对照 Gazebo 基线：占比 ~0.5、拒绝低位、坡度 <1°）；
  ③ walker 嫌疑 —— 行人/NPC 车真值位置 3 m 邻域内的非地面点数
     （0 = 雷达根本没打到 / 全被吞成地面，分不同的病）。

点云同时落盘成 xyz csv（一帧），供本地离线复算。
判据脚本不在这里 —— 这是**测量仪**，裁决写回 plan.md P9-S1。
"""

import argparse
import math
import struct

import rclpy
from rclpy.node import Node

from ads_msgs.msg import ObstacleArray
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def cloud_points(msg):
    """PointCloud2 → [(x, y, z)]，只认 float32 xyz 在前的布局（本项目两处都是）."""
    step = msg.point_step
    out = []
    data = msg.data
    for i in range(0, len(data) - step + 1, step):
        x, y, z = struct.unpack_from('<fff', data, i)
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            out.append((x, y, z))
    return out


class CaptureNode(Node):
    """采样器：攒 N 帧，聚合后一次性打印报告."""

    def __init__(self, frames_wanted, out_prefix):
        super().__init__('p9_capture')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.frames_wanted = frames_wanted
        self.out_prefix = out_prefix
        self.frames = []          # 每帧：base_link 点列表
        self.diag_rows = []       # 每次诊断：dict
        self.truth = {}           # name → (x, y) map 系
        self.ego = None           # (x, y, yaw) map 系
        self.saved_raw = False

        self.create_subscription(PointCloud2, '/lidar/points', self.on_cloud, 10)
        self.obstacles = []   # 最近一帧感知输出：[(x, y, vx, vy)] map 系
        self.create_subscription(
            ObstacleArray, '/perception/obstacles',
            lambda m: setattr(self, 'obstacles', [
                (o.pose.position.x, o.pose.position.y,
                 o.velocity_mps.x, o.velocity_mps.y)
                for o in m.obstacles]), 10)
        self.create_subscription(PointCloud2, '/carla/lidar/points_raw', self.on_raw, 10)
        self.create_subscription(DiagnosticArray, '/perception/diagnostics', self.on_diag, 10)
        for name in ('npc_car', 'pedestrian'):
            self.create_subscription(
                Odometry, f'/model/{name}/pose_gt',
                lambda m, n=name: self.truth.__setitem__(
                    n, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.create_subscription(Odometry, '/ego_pose_gt', self.on_ego, 10)

    def on_ego(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.ego = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_raw(self, msg):
        if self.saved_raw:
            return
        self.saved_raw = True
        points = cloud_points(msg)
        with open(f'{self.out_prefix}_raw.csv', 'w', encoding='utf-8') as f:
            f.write('x,y,z\n')
            for x, y, z in points:
                f.write(f'{x:.3f},{y:.3f},{z:.3f}\n')
        self.get_logger().info(f'原始帧已落盘：{len(points)} 点')

    def on_cloud(self, msg):
        if len(self.frames) >= self.frames_wanted:
            return
        # ⚠️ 真值随帧**同拍**快照（首版在报告时刻读真值 —— 迎面车 4 m/s，
        #    帧与报告差几秒 = 位移十几米，邻域比对全是垃圾）。
        self.frames.append({
            'points': cloud_points(msg),
            'truth': dict(self.truth),
            'ego': self.ego,
            'obstacles': list(self.obstacles),
        })

    def on_diag(self, msg):
        if not msg.status:
            return
        row = {}
        for kv in msg.status[0].values:
            try:
                row[kv.key] = float(kv.value)
            except ValueError:
                pass
        if 'ground_height_m' in row:
            self.diag_rows.append(row)

    def done(self):
        return len(self.frames) >= self.frames_wanted and len(self.diag_rows) >= 3

    def report(self):
        lines = ['===== P9-S1 感知域诊断报告 =====']
        # ① 挂高：z 直方图（全部帧合并）+ 诊断的地面高度
        zs = sorted(z for frame in self.frames for _, _, z in frame['points'])
        if zs:
            lines.append(f'/lidar/points 合并 {len(self.frames)} 帧共 {len(zs)} 点；'
                         f'z 分位数：p5={zs[int(0.05 * len(zs))]:.3f} '
                         f'p25={zs[int(0.25 * len(zs))]:.3f} '
                         f'p50={zs[len(zs) // 2]:.3f} '
                         f'p75={zs[int(0.75 * len(zs))]:.3f} '
                         f'p95={zs[int(0.95 * len(zs))]:.3f}')
            lines.append('  ↳ 嫌疑①判读：路面点应聚在 z≈0。p25/p50 整体偏移 = 挂载基准差')
            histogram = {}
            for z in zs:
                histogram[round(z * 2) / 2] = histogram.get(round(z * 2) / 2, 0) + 1
            top = sorted(histogram.items(), key=lambda kv: -kv[1])[:6]
            lines.append('  z 众数桶(0.5m)：' +
                         '  '.join(f'{k:+.1f}m×{v}' for k, v in sorted(top)))
        if self.diag_rows:
            heights = sorted(r['ground_height_m'] for r in self.diag_rows)
            ratios = sorted(r.get('ground_ratio', 0.0) for r in self.diag_rows)
            rejected = sorted(r.get('ground_slope_rejected', 0.0) for r in self.diag_rows)
            slopes = sorted(r.get('ground_slope_deg', 0.0) for r in self.diag_rows)
            found = [r.get('ground_found', 0.0) for r in self.diag_rows]
            lines.append(
                f'地面诊断 {len(self.diag_rows)} 拍：found={sum(found):.0f}/{len(found)}  '
                f'height 中位 {heights[len(heights) // 2]:+.3f} m  '
                f'ratio 中位 {ratios[len(ratios) // 2]:.2f}（Gazebo 基线≈0.5）  '
                f'slope 中位 {slopes[len(slopes) // 2]:.1f}°  '
                f'slope_rejected 中位 {rejected[len(rejected) // 2]:.0f}')
        # ③ walker：真值邻域点数 —— **逐帧配同拍真值**（异步教训见 on_cloud）
        names = sorted({n for f in self.frames for n in f['truth']})
        for name in names:
            per_frame = []
            for frame in self.frames:
                if name not in frame['truth'] or frame['ego'] is None:
                    continue
                ex, ey, eyaw = frame['ego']
                tx, ty = frame['truth'][name]
                dx, dy = tx - ex, ty - ey
                cos_yaw, sin_yaw = math.cos(eyaw), math.sin(eyaw)
                bx = dx * cos_yaw + dy * sin_yaw
                by = -dx * sin_yaw + dy * cos_yaw
                near = sum(1 for x, y, _ in frame['points']
                           if math.hypot(x - bx, y - by) < 3.0)
                per_frame.append((math.hypot(bx, by), near))
            if per_frame:
                text = '  '.join(f'{d:.1f}m→{n}点' for d, n in per_frame)
                lines.append(f'真值 {name} 逐帧（距离→3m 邻域点数）：{text}')
            # 三分法第二层：感知输出里离真值最近的障碍物（map 系直接比）
            trichotomy = []
            for frame in self.frames:
                if name not in frame['truth'] or not frame['obstacles']:
                    trichotomy.append('无输出')
                    continue
                tx, ty = frame['truth'][name]
                best = min(
                    frame['obstacles'],
                    key=lambda o: math.hypot(o[0] - tx, o[1] - ty))
                gap = math.hypot(best[0] - tx, best[1] - ty)
                speed = math.hypot(best[2], best[3])
                trichotomy.append(f'{gap:.1f}m/v{speed:.1f}')
            lines.append(f'  ↳ {name} 最近感知障碍（距真值/速度）：' + '  '.join(trichotomy))
        if not names:
            lines.append('（没收到任何 NPC 真值）')
        # 附：z∈[1.25,1.75] 神秘带的 XY 范围（基线云里 +1.5 桶 9326 点是什么）
        band = [(x, y) for f in self.frames for x, y, z in f['points']
                if 1.25 <= z <= 1.75]
        if band:
            xs2 = sorted(x for x, _ in band)
            ys2 = sorted(y for _, y in band)
            lines.append(
                f'z∈[1.25,1.75] 带 {len(band)} 点：x∈[{xs2[0]:.1f},{xs2[-1]:.1f}] '
                f'y∈[{ys2[0]:.1f},{ys2[-1]:.1f}]（自车系）')
        report = '\n'.join(lines)
        with open(f'{self.out_prefix}_report.txt', 'w', encoding='utf-8') as f:
            f.write(report + '\n')
        print(report, flush=True)


def main():
    """入口."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames', type=int, default=5)
    parser.add_argument('--out-prefix', default='/tmp/p9s1')
    parser.add_argument('--timeout-s', type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = CaptureNode(args.frames, args.out_prefix)
    import time
    start = time.monotonic()
    while not node.done() and time.monotonic() - start < args.timeout_s:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.report()


if __name__ == '__main__':
    main()
