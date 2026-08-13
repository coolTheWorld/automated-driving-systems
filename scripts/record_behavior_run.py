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

"""Record one CP-P7-B behavior run and score it against the plan.md table.

判据来自 tasks/plan.md「CP-P7-B」表（①–⑨），**本脚本不重新发明**；
⑩（回归）由既有的 record_control_run / record_obstacle_run 负责，不在这里。

    python3 scripts/record_behavior_run.py --scenario follow    --layer truth --out /tmp/b1.csv
    python3 scripts/record_behavior_run.py --scenario crossing  --layer perception ...
    python3 scripts/record_behavior_run.py --scenario junction  ...

三条量测约定（P7 决策七）：
  * TTC 的距离项用**感知近边**（/perception/obstacles，层 1 时它来自真值
    发布器 —— 同一话题，脚本不分层）；
  * 安全与末态类（间距/停止/恢复/碰撞）一律用**真值**（/ego_pose_gt 与
    /model/<name>/pose_gt —— 感知盲区 3 m 内界，判据带两头都由物理定）；
  * 行为状态（⑨）数 /planning/diagnostics 的 behavior_state 字段。

⚠️ 与 CP-P5-B/P6-B 同一条扩窗协议：记录器**早起**（bringup 完成即起），
   goal 由编排侧的预热发布器在绝对仿真钟 37.0 发出。
"""

import argparse
import csv
import math
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent


# =============================================================================
#  几何（与 record_obstacle_run.py 同一套：精确多边形间距，不用 SAT 下界 ——
#  「拿下界当保守估计」会造成假失败，CLAUDE.md 陷阱表有案）
# =============================================================================
def corners(center_x_m, center_y_m, yaw_rad, length_m, width_m):
    """
    Return the four corners in winding order.

    :return: 四个角点
    """
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    half_l, half_w = 0.5 * length_m, 0.5 * width_m
    return [(center_x_m + dx * cos_yaw - dy * sin_yaw,
             center_y_m + dx * sin_yaw + dy * cos_yaw)
            for dx, dy in ((half_l, half_w), (half_l, -half_w),
                           (-half_l, -half_w), (-half_l, half_w))]


def point_to_segment(point, from_point, to_point):
    """
    Return the minimum distance from a point to a segment.

    :return: 距离，米
    """
    dx, dy = to_point[0] - from_point[0], to_point[1] - from_point[1]
    length_squared = dx * dx + dy * dy
    ratio = 0.0
    if length_squared > 0.0:
        ratio = ((point[0] - from_point[0]) * dx + (point[1] - from_point[1]) * dy)
        ratio = max(0.0, min(1.0, ratio / length_squared))
    return math.hypot(point[0] - (from_point[0] + ratio * dx),
                      point[1] - (from_point[1] + ratio * dy))


def polygon_distance(poly_a, poly_b):
    """
    Return the exact minimum distance between two disjoint convex polygons.

    :return: 距离，米
    """
    best = float('inf')
    for first, second in ((poly_a, poly_b), (poly_b, poly_a)):
        for point in first:
            for i in range(len(second)):
                best = min(best, point_to_segment(
                    point, second[i], second[(i + 1) % len(second)]))
    return best


# =============================================================================
#  配置
# =============================================================================
def load_config():
    """
    Load actor definitions and vehicle geometry.

    :return: (dynamic_actors.yaml, vehicle_params.yaml)
    """
    def read(name):
        return yaml.safe_load((REPO_ROOT / 'config' / name).read_text(encoding='utf-8'))
    return read('dynamic_actors.yaml'), read('vehicle_params.yaml')


SCENARIO_ACTORS = {
    'follow': ['lead_car'],
    'crossing': ['crossing_pedestrian'],
    'junction': ['cross_car_a', 'cross_car_b', 'cross_car_c'],
}


class BehaviorRecorder(Node):
    """Record truth poses + diagnostics for one behavior scenario."""

    def __init__(self, scenario, duration_s):
        """
        Subscribe to everything the CP-P7-B table needs.

        :param scenario: follow / crossing / junction
        :param duration_s: 仿真时长（从第一拍 ego 真值起算）
        """
        super().__init__('behavior_recorder')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.scenario = scenario
        self.duration_s = duration_s
        actors_cfg, vehicle = load_config()
        geo = vehicle['geometry']
        self.vehicle_length_m = float(geo['length_m'])
        self.vehicle_width_m = float(geo['width_m'])
        # pose_gt 的原点在后轴中心地面（自车与道具车同一约定），包围盒中心在
        # 前方 length/2 − rear_overhang。行人的原点在底面中心，纵向不偏。
        self.center_offset_m = self.vehicle_length_m / 2.0 - float(geo['rear_overhang_m'])

        self.actor_dims = {}
        for name in SCENARIO_ACTORS[scenario]:
            actor = actors_cfg['actors'][name]
            if actor.get('classification') == 'vehicle':
                self.actor_dims[name] = (self.vehicle_length_m, self.vehicle_width_m,
                                         self.center_offset_m)
            else:
                self.actor_dims[name] = (float(actor['length_m']), float(actor['width_m']), 0.0)

        from nav_msgs.msg import Odometry
        from diagnostic_msgs.msg import DiagnosticArray
        from ads_msgs.msg import ObstacleArray

        self.rows = []
        self.ego = None            # (x, y, yaw, v)
        self.actor_states = {}     # name -> (x, y, yaw, v)
        self.behavior_state = ''
        self.control_state = ''
        self.control_v = float('nan')
        self.perception_obstacles = []  # [(cx, cy, yaw, lx, ly)]
        self.t0 = None
        self.done = False

        self.create_subscription(Odometry, '/ego_pose_gt', self._on_ego, 50)
        for name in SCENARIO_ACTORS[scenario]:
            self.create_subscription(
                Odometry, f'/model/{name}/pose_gt',
                lambda msg, n=name: self._on_actor(n, msg), 20)
        self.create_subscription(
            DiagnosticArray, '/planning/diagnostics', self._on_planning, 50)
        self.create_subscription(
            DiagnosticArray, '/control/diagnostics', self._on_control, 100)
        self.create_subscription(
            ObstacleArray, '/perception/obstacles', self._on_perception, 20)

    @staticmethod
    def _pose_of(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        return (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, v)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_actor(self, name, msg):
        self.actor_states[name] = self._pose_of(msg)

    def _on_planning(self, msg):
        values = {kv.key: kv.value for kv in msg.status[0].values}
        self.behavior_state = values.get('behavior_state', self.behavior_state)
        self.crossing_ids = values.get('behavior_crossing_ids', getattr(self, 'crossing_ids', '-'))
        self.follow_id = values.get('behavior_follow_id', getattr(self, 'follow_id', '-'))

    def _on_control(self, msg):
        values = {kv.key: kv.value for kv in msg.status[0].values}
        self.control_state = values.get('state', self.control_state)
        try:
            self.control_v = float(values.get('measured_speed_mps', 'nan'))
        except ValueError:
            pass

    def _on_perception(self, msg):
        # 诊断用全量表（id → 位置/尺寸/速度/分类），S4 排查横穿泄漏时开
        self.perception_full = {
            o.id: (o.pose.position.x, o.pose.position.y, o.size_m.x, o.size_m.y,
                   math.hypot(o.velocity_mps.x, o.velocity_mps.y), o.classification)
            for o in msg.obstacles}
        self.perception_obstacles = [
            (o.pose.position.x, o.pose.position.y,
             math.atan2(2.0 * (o.pose.orientation.w * o.pose.orientation.z),
                        1.0 - 2.0 * o.pose.orientation.z * o.pose.orientation.z),
             o.size_m.x, o.size_m.y)
            for o in msg.obstacles]

    def _on_ego(self, msg):
        self.ego = self._pose_of(msg)
        t = self._now_s()
        if self.t0 is None:
            self.t0 = t
        if t - self.t0 >= self.duration_s:
            self.done = True
            return
        row = {'t': t, 'ego_x': self.ego[0], 'ego_y': self.ego[1],
               'ego_yaw': self.ego[2], 'ego_v': self.ego[3],
               'behavior': self.behavior_state, 'ctrl_state': self.control_state,
               'ctrl_v': self.control_v,
               'perc_near_x': self._nearest_perception_edge_x(),
               'crossing_ids': getattr(self, 'crossing_ids', '-'),
               'follow_id': getattr(self, 'follow_id', '-'),
               'perc_count': len(self.perception_obstacles),
               'perc_full': ';'.join(
                   f'{i}:{v[0]:.1f}:{v[1]:.1f}:{v[2]:.1f}:{v[3]:.1f}:{v[4]:.2f}:{v[5]}'
                   for i, v in getattr(self, 'perception_full', {}).items())}
        for name, state in self.actor_states.items():
            row[f'{name}_x'], row[f'{name}_y'] = state[0], state[1]
            row[f'{name}_yaw'], row[f'{name}_v'] = state[2], state[3]
        self.rows.append(row)

    def _nearest_perception_edge_x(self):
        """南侧直道上（自车前方）最近感知目标的近边 x。TTC（判据 ①）用。"""
        if self.ego is None:
            return float('nan')
        best = float('nan')
        for cx, cy, _yaw, lx, _ly in self.perception_obstacles:
            # 只看自车道走廊里、前方的目标（判据 ① 是 S03 的，几何在直道上）。
            if abs(cy - (-51.75)) > 1.75:
                continue
            near = cx - lx / 2.0
            if near > self.ego[0] and (math.isnan(best) or near < best):
                best = near
        return best

    def ego_body(self, x, y, yaw):
        """后轴原点 → 车体外廓多边形。"""
        cx = x + self.center_offset_m * math.cos(yaw)
        cy = y + self.center_offset_m * math.sin(yaw)
        return corners(cx, cy, yaw, self.vehicle_length_m, self.vehicle_width_m)

    def actor_body(self, name, x, y, yaw):
        length, width, offset = self.actor_dims[name]
        cx = x + offset * math.cos(yaw)
        cy = y + offset * math.sin(yaw)
        return corners(cx, cy, yaw, length, width)


# =============================================================================
#  打分 —— 判据编号对应 plan.md CP-P7-B 表
# =============================================================================
FRONT_OFFSET_M = 3.55  # length − rear_overhang，与 planning_node 的推导一致


def min_actor_distance(recorder, rows, names):
    """全程真值最小间距（车体外廓 ↔ 目标外廓）。判据 ⑤⑦⑧ 用。"""
    best = float('inf')
    at = None
    for row in rows:
        ego_poly = recorder.ego_body(row['ego_x'], row['ego_y'], row['ego_yaw'])
        for name in names:
            if f'{name}_x' not in row:
                continue
            poly = recorder.actor_body(name, row[f'{name}_x'], row[f'{name}_y'],
                                       row[f'{name}_yaw'])
            d = polygon_distance(ego_poly, poly)
            if d < best:
                best, at = d, (row['t'], name)
    return best, at


def behavior_transitions(rows):
    """FOLLOW/YIELD 相关的状态切换次数（判据 ⑨）。"""
    seq = []
    for row in rows:
        state = row['behavior']
        if state and (not seq or seq[-1] != state):
            seq.append(state)
    switches = sum(1 for a, b in zip(seq, seq[1:]) if 'FOLLOW' in (a, b) or 'YIELD' in (a, b))
    return switches, seq


def check_table(checks):
    """打印判据表，返回是否全过。"""
    print('\n===== CP-P7-B 判据 =====')
    all_ok = True
    for name, value, ok, limit, note in checks:
        flag = '✅' if ok else '❌'
        all_ok = all_ok and ok
        print(f'  {flag} {name:<28} {value:<24} 判据 {limit}  {note}')
    return all_ok


def score_follow(recorder, rows):
    """S03：① TTC、② 跟停间距、③ 驶离恢复（+ 共通 ⑧⑨）。"""
    lead = 'lead_car'
    checks = []

    # ① TTC（v_rel ≥ 1.5 段，距离用感知近边）。只在两车都还在南直道上时判
    # （出弯后一维近边模型失效 —— 判据的适用范围由几何定，CLAUDE.md 那条）。
    min_ttc, at_t = float('inf'), None
    for prev, row in zip(rows, rows[1:]):
        if math.isnan(row['perc_near_x']) or row['ego_x'] > 76.0:
            continue
        if f'{lead}_x' not in row or row[f'{lead}_x'] > 76.0:
            continue
        v_rel = row['ego_v'] - row[f'{lead}_v']
        if v_rel < 1.5:
            continue
        d = row['perc_near_x'] - (row['ego_x'] + FRONT_OFFSET_M)
        if d <= 0.0:
            continue
        ttc = d / v_rel
        if ttc < min_ttc:
            min_ttc, at_t = ttc, row['t']
    checks.append(('① TTC 最小值（v_rel≥1.5）', f'{min_ttc:.2f} s @t={at_t}',
                   min_ttc > 2.0, '> 2 s', ''))

    # ② 跟停稳态间距：前车停在 x≈70 的窗口里、自车也停住的样本，真值近边间距。
    gaps = []
    for row in rows:
        if f'{lead}_x' not in row:
            continue
        lead_stopped = row[f'{lead}_v'] < 0.05 and abs(row[f'{lead}_x'] - 70.0) < 2.0
        ego_stopped = row['ego_v'] < 0.05
        if lead_stopped and ego_stopped:
            near = row[f'{lead}_x'] - 0.85  # 后轴原点 − 后悬 = 车尾面
            gaps.append(near - (row['ego_x'] + FRONT_OFFSET_M))
    if gaps:
        gap = sum(gaps) / len(gaps)
        checks.append(('② 跟停稳态间距', f'{gap:.2f} m（{len(gaps)} 拍）',
                       4.0 <= gap <= 10.0, '[4, 10] m', ''))
    else:
        checks.append(('② 跟停稳态间距', '无共同停止窗口', False, '[4, 10] m',
                       '场景没激励：前车停时自车没停'))

    # ③ 前车驶离后恢复：驶离时刻 = 停止窗之后 lead_v 首次 > 0.5。
    t_depart, t_recover = None, None
    seen_stop = False
    for row in rows:
        if f'{lead}_x' not in row:
            continue
        if row[f'{lead}_v'] < 0.05 and abs(row[f'{lead}_x'] - 70.0) < 2.0:
            seen_stop = True
        if seen_stop and t_depart is None and row[f'{lead}_v'] > 0.5:
            t_depart = row['t']
        if t_depart is not None and row['t'] > t_depart and row['ego_v'] > 0.5:
            t_recover = row['t']
            break
    if t_depart is None:
        checks.append(('③ 驶离后恢复', '前车没有驶离', False, '≤ 3 s', '场景没激励'))
    else:
        dt = (t_recover - t_depart) if t_recover else float('inf')
        checks.append(('③ 驶离后恢复', f'{dt:.2f} s', dt <= 3.0, '≤ 3 s', ''))
    return checks


def score_crossing(recorder, rows):
    """S05：④ 完全停止、⑤ 最近距离、⑥ 离开后恢复。"""
    ped = 'crossing_pedestrian'
    checks = []

    def in_lane(row):
        return f'{ped}_y' in row and -53.5 <= row[f'{ped}_y'] <= -50.0

    # ④ 完全停止（行人占道期间），停止时刻真值距离 > 3。
    stop_row = next((r for r in rows if in_lane(r) and r['ego_v'] < 0.05), None)
    if stop_row is None:
        checks.append(('④ 占道期间完全停止', '从未停住', False, 'v<0.05 且距离>3', ''))
    else:
        d = polygon_distance(
            recorder.ego_body(stop_row['ego_x'], stop_row['ego_y'], stop_row['ego_yaw']),
            recorder.actor_body(ped, stop_row[f'{ped}_x'], stop_row[f'{ped}_y'],
                                stop_row[f'{ped}_yaw']))
        checks.append(('④ 占道期间完全停止', f'停于 t={stop_row["t"]:.1f}，距 {d:.2f} m',
                       d > 3.0, 'v<0.05 且距离>3 m', ''))

    # ⑤ 全程真值最近距离。
    best, at = min_actor_distance(recorder, rows, [ped])
    checks.append(('⑤ 全程真值最近距离', f'{best:.2f} m @t={at[0]:.1f}' if at else '-',
                   best > 1.0, '> 1 m', ''))

    # ⑥ 行人离开车道后恢复。
    t_exit = next((r['t'] for r in rows
                   if f'{ped}_y' in r and r[f'{ped}_y'] < -53.5), None)
    if t_exit is None:
        checks.append(('⑥ 离开车道后恢复', '行人没离开过车道', False, '≤ 3 s', '场景没激励'))
    else:
        t_recover = next((r['t'] for r in rows if r['t'] > t_exit and r['ego_v'] > 0.5), None)
        dt = (t_recover - t_exit) if t_recover else float('inf')
        checks.append(('⑥ 离开车道后恢复', f'{dt:.2f} s', dt <= 3.0, '≤ 3 s', ''))
    return checks


def score_junction(recorder, rows):
    """junction：⑦ 车流间距 + 通过后恢复（+ 共通 ⑧⑨）。"""
    names = SCENARIO_ACTORS['junction']
    checks = []
    # ⑦ 与每辆车的全程真值间距 > 1.5（「不进冲突区」的可量化形式 ——
    # 冲突区里同时出现两车必然违反 1.5 m，反之守住 1.5 m 就是守住了让行）。
    best, at = min_actor_distance(recorder, rows, names)
    checks.append(('⑦ 对车流的真值最小间距', f'{best:.2f} m @t={at[0]:.1f} vs {at[1]}' if at else '-',
                   best > 1.5, '> 1.5 m', ''))
    # ⑦b 通过后恢复（plan 表原文「通过后恢复」）：让行停车之后 ego 重新起步
    # 并驶入横穿路南行（进入路口以南 ≥ 20 m ⟹ 让行不是永久趴窝）。
    # ⚠️ 初版在这里发明了「GOAL_REACHED」的加严判据 —— 违反「判据来自
    #    plan.md、脚本不重新发明」。末端 goal 逼近是 P3 短跨度 lattice 的
    #    已记录边界（P8 台账），不是让行行为的一部分；末态照印在下面。
    yielded = any(r['ego_v'] < 0.2 and r['ego_y'] > 40.0 for r in rows)
    resumed = any(r['ego_y'] < 19.0 and abs(r['ego_x']) < 3.5 for r in rows)
    checks.append(('⑦b 让行后恢复通过', f'让行停车 {yielded}，南行过路口 20 m {resumed}',
                   yielded and resumed, '恢复行驶且驶入横穿路', ''))
    last = rows[-1]
    print(f'  [照印] 末态 ctrl={last["ctrl_state"]} @({last["ego_x"]:.2f},{last["ego_y"]:.2f}) '
          f'v={last["ego_v"]:.2f}（末端 goal 逼近边界见 P8 台账，不计入判据）')
    return checks


def score(recorder, args):
    """
    Score the recording and print the verdict table.

    :return: 0 = 全过
    """
    rows = recorder.rows
    if len(rows) < 100:
        print(f'✗ 只有 {len(rows)} 拍数据 —— 记录器没有跑起来或仿真死了')
        return 1
    print(f'共 {len(rows)} 拍，仿真 {rows[0]["t"]:.1f} → {rows[-1]["t"]:.1f} s，'
          f'场景 {args.scenario}，层 {args.layer}')

    if args.scenario == 'follow':
        checks = score_follow(recorder, rows)
    elif args.scenario == 'crossing':
        checks = score_crossing(recorder, rows)
    else:
        checks = score_junction(recorder, rows)

    # ⑧ 零碰撞（全场景共通，真值）。
    best, at = min_actor_distance(recorder, rows, SCENARIO_ACTORS[args.scenario])
    checks.append(('⑧ 零碰撞（真值最小间距）', f'{best:.2f} m',
                   best > 0.3, '> 0.3 m', ''))
    # ⑨ 行为状态不振荡。
    switches, seq = behavior_transitions(rows)
    checks.append(('⑨ 行为状态切换次数', f'{switches}（{ "→".join(seq[:10]) }）',
                   switches <= 4, '≤ 4 次', ''))

    return 0 if check_table(checks) else 1


def main():
    """Entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True, choices=list(SCENARIO_ACTORS))
    parser.add_argument('--layer', required=True, choices=['truth', 'perception'])
    parser.add_argument('--duration-s', type=float, default=100.0)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    rclpy.init()
    recorder = BehaviorRecorder(args.scenario, args.duration_s)
    import time
    wall_start = time.monotonic()
    wall_budget = args.duration_s * 2.0 + 180.0  # 铁律：任何等待都有墙钟超时
    while not recorder.done:
        rclpy.spin_once(recorder, timeout_sec=0.1)
        if time.monotonic() - wall_start > wall_budget:
            print(f'⚠️ 墙钟超时（{wall_budget:.0f} s），提前收卷', file=sys.stderr)
            break

    with open(args.out, 'w', newline='') as f:
        if recorder.rows:
            writer = csv.DictWriter(f, fieldnames=sorted({k for r in recorder.rows for k in r}))
            writer.writeheader()
            writer.writerows(recorder.rows)

    code = score(recorder, args)
    recorder.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
