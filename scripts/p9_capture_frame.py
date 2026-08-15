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

--timeline PATH（P9-S2 收官谜的仪器）：按 /perception/detections（跟踪器之前的
逐帧检测框，perception_node 的旁路出口）逐帧写一行 —— 真值距离、离真值最近的
**检测**框间距、离真值最近的**确认**航迹间距、检测/确认个数、最近一拍诊断的
剃刀门键。四段流水线（分割/聚类/剃刀门/跟踪确认）在同一行里二分：
  真值旁有检测、无确认 ⟹ 跟踪确认层；连检测都没有 ⟹ 更上游（看邻域点数）。
给了 --timeline 就跑满 --timeout-s（不再采够 N 帧就退）。
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

    def __init__(self, frames_wanted, out_prefix, timeline_path=None):
        super().__init__('p9_capture')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.frames_wanted = frames_wanted
        self.out_prefix = out_prefix
        self.frames = []          # 每帧：base_link 点列表
        self.diag_rows = []       # 每次诊断：dict
        self.truth = {}           # name → (x, y) map 系
        self.ego = None           # (x, y, yaw) map 系
        self.saved_raw = False
        self.raw_counts = []
        self.dyn_saved = 0
        # 时间线（P9-S2 收官谜）：按检测帧写行，见模块 docstring
        self.timeline = None
        self.timeline_rows = 0
        if timeline_path:
            self.timeline = open(timeline_path, 'w', encoding='utf-8')
            self.timeline.write(
                't_s,ego_x,ego_y,n_det,n_conf,'
                'npc_dist,npc_det_gap,npc_det_lw,npc_conf_gap,'
                'ped_dist,ped_det_gap,ped_det_lw,ped_conf_gap,'
                'non_ground,clusters,razor_dropped,razor_max_range,razor_max_top,'
                'ground_found,ground_ratio\n')

        self.create_subscription(PointCloud2, '/lidar/points', self.on_cloud, 10)
        self.obstacles = []   # 最近一帧感知输出：[(x, y, vx, vy)] map 系
        self.create_subscription(
            ObstacleArray, '/perception/obstacles',
            lambda m: setattr(self, 'obstacles', [
                (o.pose.position.x, o.pose.position.y,
                 o.velocity_mps.x, o.velocity_mps.y)
                for o in m.obstacles]), 10)
        self.create_subscription(
            ObstacleArray, '/perception/detections', self.on_detections, 10)
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

    def on_detections(self, msg):
        """时间线一行 = 一帧检测：真值 ↔ 最近检测框 / 最近确认航迹（map 系直接比）."""
        if self.timeline is None or self.ego is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dets = [(o.pose.position.x, o.pose.position.y, o.size_m.x, o.size_m.y)
                for o in msg.obstacles]
        cols = [f'{t:.2f}', f'{self.ego[0]:.2f}', f'{self.ego[1]:.2f}',
                str(len(dets)), str(len(self.obstacles))]
        for name in ('npc_car', 'pedestrian'):
            if name not in self.truth:
                cols += ['', '', '', '']
                continue
            tx, ty = self.truth[name]
            dist = math.hypot(tx - self.ego[0], ty - self.ego[1])
            det_gap, det_lw = '', ''
            if dets:
                best = min(dets, key=lambda d: math.hypot(d[0] - tx, d[1] - ty))
                det_gap = f'{math.hypot(best[0] - tx, best[1] - ty):.2f}'
                det_lw = f'{best[2]:.2f}x{best[3]:.2f}'
            conf_gap = ''
            if self.obstacles:
                best = min(self.obstacles, key=lambda o: math.hypot(o[0] - tx, o[1] - ty))
                conf_gap = f'{math.hypot(best[0] - tx, best[1] - ty):.2f}'
            cols += [f'{dist:.1f}', det_gap, det_lw, conf_gap]
        diag = self.diag_rows[-1] if self.diag_rows else {}
        cols += [f'{diag.get(k, float("nan")):.2f}' for k in (
            'non_ground_points', 'clusters', 'razor_dropped', 'razor_max_range_m',
            'razor_max_top_m', 'ground_found', 'ground_ratio')]
        self.timeline.write(','.join(cols) + '\n')
        self.timeline_rows += 1
        if self.timeline_rows % 50 == 0:
            self.timeline.flush()

    def on_raw(self, msg):
        # 栈内二分（P9-S2 终极）：raw 路的 npc 邻域逐帧计数。raw 有而
        # processed 无 = 预处理器；raw 也无 = 栈的传感器/中继本身。
        if 'npc_car' in self.truth and self.ego is not None and len(self.raw_counts) < 8:
            ex, ey, eyaw = self.ego
            tx, ty = self.truth['npc_car']
            dx, dy = tx - ex, ty - ey
            cos_yaw, sin_yaw = math.cos(eyaw), math.sin(eyaw)
            bx = dx * cos_yaw + dy * sin_yaw
            by = -dx * sin_yaw + dy * cos_yaw
            pts = cloud_points(msg)
            near = sum(1 for x, y, _ in pts
                       if math.hypot(x - bx, y - by) < 3.5)
            self.raw_counts.append((math.hypot(bx, by), near, len(pts)))
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
        # 动态帧落盘（P9-S2 离线化）：车在 20 m 内的时刻抓 processed 整帧，
        # 拉回本地零租金复现聚类桥接 —— 云上单变量盲试到拐点后的正确工具。
        if ('npc_car' in self.truth and self.ego is not None and
                self.dyn_saved < 3):
            ex, ey, _ = self.ego
            tx, ty = self.truth['npc_car']
            if math.hypot(tx - ex, ty - ey) < 25.0:
                self.dyn_saved += 1
                with open(f'{self.out_prefix}_dyn{self.dyn_saved}.csv',
                          'w', encoding='utf-8') as f:
                    f.write(f'# ego {ex:.3f} {ey:.3f} yaw {self.ego[2]:.4f} '
                            f'npc {tx:.3f} {ty:.3f}\n')
                    f.write('x,y,z\n')
                    for x, y, z in cloud_points(msg):
                        f.write(f'{x:.3f},{y:.3f},{z:.3f}\n')
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

    @staticmethod
    def _median_of(rows, key):
        vals = sorted(r.get(key, 0.0) for r in rows)
        return vals[len(vals) // 2] if vals else 0.0

    def done(self):
        if self.timeline is not None:
            return False   # 时间线模式跑满 --timeout-s
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
            lines.append(
                '流水线计数中位：input %.0f → non_ground %.0f → clusters %.0f → '
                'detections %.0f → confirmed %.0f（held %.0f）' % (
                    self._median_of(self.diag_rows, 'input_points'),
                    self._median_of(self.diag_rows, 'non_ground_points'),
                    self._median_of(self.diag_rows, 'clusters'),
                    self._median_of(self.diag_rows, 'detections'),
                    self._median_of(self.diag_rows, 'confirmed_tracks'),
                    self._median_of(self.diag_rows, 'ground_held_count')))
            # 剃刀门覆盖（P9-S2 二条件门对 CARLA 接缝条族的复扫）：被吞框的
            # 簇顶离地 vs 门 razor_max_height_m —— 吞的都是矮条 = 门按物理
            # 先验在收；max_top 逼近门 = 门与目标剖面在拉锯，要看是谁。
            razored = [r for r in self.diag_rows if r.get('razor_dropped', 0.0) > 0]

            def razor_max(key):
                return max((r.get(key, 0.0) for r in razored), default=0.0)
            lines.append(
                f'剃刀门：{len(razored)}/{len(self.diag_rows)} 拍有吞框，'
                f'每拍吞框中位 {self._median_of(razored, "razor_dropped"):.0f}，'
                f'被吞框最远 {razor_max("razor_max_range_m"):.1f} m，'
                f'簇顶离地最高 {razor_max("razor_max_top_m"):.2f} m，'
                f'min(l,w) 最大 {razor_max("razor_min_extent_m"):.3f} m')
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
                # 离地 >0.3 m 的那部分：地面点不算 —— 3 m 邻域里路面本身就有
                # 上百点，只报总数分不出「打到了目标」和「只打到了它脚下的路」
                above = sum(1 for x, y, z in frame['points']
                            if z > 0.3 and math.hypot(x - bx, y - by) < 3.0)
                per_frame.append((math.hypot(bx, by), near, above))
            if per_frame:
                text = '  '.join(f'{d:.1f}m→{n}点/{a}高' for d, n, a in per_frame)
                lines.append(f'真值 {name} 逐帧（距离→3m 邻域点数/其中离地>0.3m）：{text}')
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
                trichotomy.append(
                    f'真值({tx:.0f},{ty:.0f})↔障碍({best[0]:.0f},{best[1]:.0f})'
                    f' {gap:.1f}m/v{speed:.1f}')
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
        if self.raw_counts:
            lines.append('raw 路 npc 邻域逐帧（距离→点数/总点）：' + '  '.join(
                f'{d:.1f}m→{n}/{t}' for d, n, t in self.raw_counts))
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
    parser.add_argument('--timeline', default=None,
                        help='逐检测帧时间线 csv（见模块 docstring）；给了就跑满 --timeout-s')
    args = parser.parse_args()

    rclpy.init()
    node = CaptureNode(args.frames, args.out_prefix, args.timeline)
    import signal
    import time
    # 轮次脚本收尾时发 TERM：要的是报告不是尸体 —— 收到就退出循环、照常出报告
    stop = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('now', True))
    start = time.monotonic()
    while not node.done() and not stop['now'] and time.monotonic() - start < args.timeout_s:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.report()
    if node.timeline is not None:
        node.timeline.close()
        print(f'时间线 {node.timeline_rows} 行 → {args.timeline}', flush=True)


if __name__ == '__main__':
    main()
