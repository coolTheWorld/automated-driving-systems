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

"""
Record one closed-loop control run and score it against the CP-P2-B criteria.

发一个目标点，把 /control/diagnostics 全程记进 CSV，然后按 CP-P2-B 的
量化判据逐条打分。

    python3 scripts/record_control_run.py --goal 91.75 20.0 --out /tmp/run.csv

为什么要有这个脚本，而不是跑一遍看 RViz
--------------------------------------
SPEC §11 禁止在没有量化判据的情况下声称某功能"做完了"。而**肉眼记下来的
数字下次没法比较** —— 下一次改了增益想知道好了还是坏了，只能重新凭印象。
CSV 落地之后，回归就是 diff 两个 summary。

判据来自 tasks/plan.md 的 CP-P2-B 表，**这里不重新发明判据**。
"""

from __future__ import annotations

import argparse
import csv
import math
import sys

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# CP-P2-B 的量化判据（tasks/plan.md）。键 = CSV/摘要里的名字，值 = (上限, 单位, 说明)。
CRITERIA = {
    'max_lateral_error_m': (0.30, 'm', '越界线 0.850 m，留 2.8 倍余量'),
    'rms_lateral_error_m': (0.10, 'm', ''),
    'steady_speed_error_mps': (0.20, 'm/s', '巡航 5.556 的 3.6%'),
    'max_lateral_accel_mps2': (2.0, 'm/s^2', '规划按 1.5 限，留 33% 给跟踪误差'),
    'max_steer_rate_rad_s': (0.500, 'rad/s', 'vehicle_params.yaml，控制器自己保证'),
    'goal_position_error_m': (1.0, 'm', ''),
    'goal_residual_speed_mps': (0.05, 'm/s', '「停住」不是「慢慢蹭」'),
    'max_cycle_time_ms': (10.0, 'ms', 'SPEC §7'),
}

FIELDS = [
    'time_s', 'state', 'lateral_error_m', 'heading_error_rad', 'curvature_inv_m',
    'path_s_m', 'path_remaining_m', 'goal_distance_m', 'target_speed_mps',
    'feedforward_accel_mps2', 'measured_speed_mps',
    'steer_angle_rad', 'accel_mps2', 'cycle_time_ms',
]


class Recorder(Node):
    """Publish one goal pose and record every /control/diagnostics sample."""

    def __init__(self, goal_xy, goal_delay_s, timeout_s):
        super().__init__('control_run_recorder')
        # ⚠️ use_sim_time 必须开：所有时间戳都是仿真钟，混用墙钟的话
        #    RTF 一偏离 1，算出来的转向速率和单拍耗时就全错了（SPEC §5）。
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])

        self.rows: list[dict] = []
        self.goal_xy = goal_xy
        self.goal_delay_s = goal_delay_s
        self.timeout_s = timeout_s
        self.goal_sent = False
        self.start_s: float | None = None
        self.stopped_since_s: float | None = None
        self.done = False

        # 目标点用 transient_local：map_node 的 /goal_pose 订阅是普通 QoS，
        # 但发布端加上它没有坏处，而且万一 map_node 晚起也能收到。
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(DiagnosticArray, '/control/diagnostics', self.on_diag, 200)
        self.create_timer(0.1, self.on_tick)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_tick(self):
        now = self.now_s()
        if self.start_s is None:
            # 仿真钟还没开始走（/clock 没来）时 now 会是 0，等它动起来再计时。
            if now <= 0.0:
                return
            self.start_s = now
            return
        elapsed = now - self.start_s

        if not self.goal_sent and elapsed >= self.goal_delay_s:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = self.goal_xy[0]
            msg.pose.position.y = self.goal_xy[1]
            msg.pose.orientation.w = 1.0
            self.goal_pub.publish(msg)
            self.goal_sent = True
            self.get_logger().info(f'已发布目标点 ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})')

        if elapsed > self.timeout_s:
            self.get_logger().warn(f'超时 {self.timeout_s:.0f} s，停止记录')
            self.done = True

    def on_diag(self, msg: DiagnosticArray):
        if self.start_s is None:
            return
        # ⚠️ 时间戳必须取**消息头**，不是回调里的 now()。
        #    用 now() 的话记下来的是"本脚本什么时候被调度到"，里面混着
        #    executor 的抖动 —— 而转向速率是拿相邻两拍的 Δδ/Δt 算的，
        #    分母抖 4% 就足以让一个恰好合规的控制器被判成超限（实测踩过一次）。
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for status in msg.status:
            row = {'time_s': stamp_s - self.start_s}
            values = {kv.key: kv.value for kv in status.values}
            row['state'] = values.get('state', '?')
            for field in FIELDS[2:]:
                try:
                    row[field] = float(values[field])
                except (KeyError, ValueError):
                    row[field] = math.nan
            self.rows.append(row)

            # 结束条件：到了终点**并且**真的停住了，持续 2 s。
            # 只看 GOAL_REACHED 会在车还在滑行时就收工，测不出"停住"这件事。
            if row['state'] == 'GOAL_REACHED' and abs(row['measured_speed_mps']) < 0.05:
                if self.stopped_since_s is None:
                    self.stopped_since_s = row['time_s']
                elif row['time_s'] - self.stopped_since_s > 2.0:
                    self.done = True
            else:
                self.stopped_since_s = None


def summarize(rows: list[dict]) -> dict:
    """把逐拍记录压成 CP-P2-B 那张表里的那几个数."""
    # 只用**正常跟踪**的样本算跟踪精度。降级期间车在刹停，把它算进去
    # 等于拿"故障处置过程"给"跟踪精度"打分 —— 两件事。
    tracking = [r for r in rows if r['state'] in ('TRACKING', 'GOAL_REACHED')]
    if not tracking:
        return {'error': '没有任何正常跟踪的样本'}

    errors = [abs(r['lateral_error_m']) for r in tracking]
    out = {
        'samples_total': len(rows),
        'samples_tracking': len(tracking),
        'samples_degraded': len(rows) - len(tracking),
        'duration_s': rows[-1]['time_s'] - rows[0]['time_s'],
        'max_lateral_error_m': max(errors),
        'rms_lateral_error_m': math.sqrt(sum(e * e for e in errors) / len(errors)),
        'max_lateral_accel_mps2': max(
            r['measured_speed_mps'] ** 2 * abs(r['curvature_inv_m']) for r in tracking),
        'max_cycle_time_ms': max(r['cycle_time_ms'] for r in rows),
    }

    # 转向速率：相邻两拍的转角差 / 时间差。**用实际时间戳而不是标称周期** ——
    # 标称周期算出来的永远等于限幅值，那就什么都没验。
    rate = 0.0
    for prev, cur in zip(rows, rows[1:]):
        dt = cur['time_s'] - prev['time_s']
        if dt > 1e-6:
            rate = max(rate, abs(cur['steer_angle_rad'] - prev['steer_angle_rad']) / dt)
    out['max_steer_rate_rad_s'] = rate

    # 稳态速度误差：只取**目标速度已经稳住三个时间常数**、且控制器没顶在限幅上的拍。
    #
    # ⚠️ "稳住多久"不是随便取的：闭环时间常数是 1/K_p，K_p=1.0 → 1 s，
    #    三个时间常数后误差衰减到 5%。窗口取短了（试过 0.4 s）会把**整段暂态**
    #    算成稳态 —— 起步从 0 加到巡航的 3.7 s 里目标恒为 5.556，
    #    终点刹停的 6 s 里目标恒为 0，两段都"目标没动"但显然不是稳态。
    #    实测：0.4 s 窗口给出 4.99 m/s，量到的其实是起步和刹停过程本身。
    #
    # ⚠️ 加速和减速的限幅值**不一样**（+1.5 / −3.0），必须分开取。
    #    用 max(|a|) 当唯一阈值只认得出减速饱和，起步的 +1.5 饱和会漏网。
    settle_samples = 150  # 3 s @ 50 Hz = 3 / K_p
    accel_hi = max(r['accel_mps2'] for r in tracking)
    accel_lo = min(r['accel_mps2'] for r in tracking)
    steady = []
    for i, r in enumerate(tracking):
        error = r['target_speed_mps'] - r['measured_speed_mps']
        if ((abs(r['accel_mps2'] - accel_hi) < 1e-6 and error > 0.0) or
                (abs(r['accel_mps2'] - accel_lo) < 1e-6 and error < 0.0)):
            continue  # 顶在限幅上、误差还在往饱和方向推 —— 物理上消不掉，不是稳态
        if i < settle_samples:
            continue
        window = tracking[i - settle_samples:i]
        # 目标要稳住，**而且**这段窗口里不能出现过饱和 ——
        # 响应只有在执行机构退出饱和之后才开始"收敛"，从目标变常数那一刻起算
        # 会把整段加速过程当成稳态（实测：起步段因此被算出 1.12 m/s）。
        if any(abs(p['accel_mps2'] - accel_hi) < 1e-6 or abs(p['accel_mps2'] - accel_lo) < 1e-6
               for p in window):
            continue
        if all(abs(p['target_speed_mps'] - r['target_speed_mps']) < 0.05 for p in window):
            steady.append(abs(error))
    out['steady_speed_error_mps'] = max(steady) if steady else math.nan
    out['steady_samples'] = len(steady)

    # 终点位置误差 = 前轴到路径末点的**直线距离**，由节点算好发过来。
    #
    # ⚠️ **不能用 path_remaining_m 合成。** 车冲过终点之后投影被夹到末点，
    #    那个量恒为 0 —— "冲过去 20 m"和"恰好停在终点"长得一模一样。
    #    这是 path_tracking 头文件里明确交给 S4 的陷阱，初稿正好踩了它。
    last = rows[-1]
    out['goal_position_error_m'] = last['goal_distance_m']
    out['goal_residual_speed_mps'] = abs(last['measured_speed_mps'])
    out['final_state'] = last['state']
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', nargs=2, type=float, default=[91.75, 20.0],
                        metavar=('X', 'Y'), help='目标点，map 系，单位 m')
    parser.add_argument('--goal-delay-s', type=float, default=6.0,
                        help='仿真起来多久之后再发目标点')
    parser.add_argument('--timeout-s', type=float, default=180.0)
    parser.add_argument('--out', default='/tmp/control_run.csv')
    args = parser.parse_args()

    rclpy.init()
    node = Recorder(tuple(args.goal), args.goal_delay_s, args.timeout_s)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        rows = node.rows
        node.destroy_node()
        rclpy.shutdown()

    if not rows:
        print('没有收到任何 /control/diagnostics —— control_node 起来了吗？', file=sys.stderr)
        return 1

    with open(args.out, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    summary = summarize(rows)
    print(f'\n===== 闭环实测（{len(rows)} 拍，{summary.get("duration_s", 0):.1f} s）=====')
    print(f'CSV: {args.out}')
    print(f'正常跟踪 {summary["samples_tracking"]} 拍，降级 {summary["samples_degraded"]} 拍，'
          f'末态 {summary["final_state"]}，稳态样本 {summary["steady_samples"]} 拍')
    print(f'{"项":<26}{"实测":>12}{"判据":>10}  结果')
    failures = 0
    for key, (limit, unit, note) in CRITERIA.items():
        value = summary.get(key, math.nan)
        # 相对 1e-6 的容差：CSV 存的是 %.9g，时间戳还有量化误差，
        # 硬 `<=` 会把 0.9 ppb 的浮点残渣判成 FAIL（转向速率实测 0.5000000005
        # 就这么被判过一次）。**容差只吸收测量精度，不放宽判据本身。**
        # NaN 与任何数比较都是 False → 记为不通过，正是想要的。
        ok = value <= limit * (1.0 + 1e-6)
        failures += 0 if ok else 1
        mark = 'PASS' if ok else 'FAIL'
        print(f'{key:<26}{value:>12.4f}{limit:>10.3f}  {mark}  {unit} {note}')
    print(f'\n{"全部通过" if failures == 0 else f"{failures} 项未通过"}')
    return 0 if failures == 0 else 2


if __name__ == '__main__':
    sys.exit(main())
