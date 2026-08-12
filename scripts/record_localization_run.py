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

# =============================================================================
#  CP-P4-B 的定位实测记录器 —— **只监听，不驱动**
#
#  与 record_control_run.py 同时跑：那个脚本发目标点、把车开起来并给
#  CP-P2-B 的 8 条判据打分；本脚本在旁边记录定位误差。
#
#  ⚠️ 判据基准是 /ego_pose_gt。**那是真值，只有评测能用** ——
#     localization_node 一旦订阅了它，整个模块的验收就变成自己跟自己比，
#     而它看起来仍然是绿的。
#
#  ⚠️ 「横向误差」量的是**垂直于车头方向**的那一个分量，不是位置误差的模。
#     两者在直道上接近，转弯时能差一倍 —— 而 SPEC §1 写的是横向。
#
#  用法（容器内，需要一个正在跑的、localization:=true 的全栈）：
#      ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false localization:=true
#      python3 scripts/record_localization_run.py --duration-s 60 --out /tmp/loc.csv
# =============================================================================

"""Record localization error against ground truth for the CP-P4-B checkpoint."""

import argparse
import csv
import math
import sys

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# CP-P4-B 的判据（tasks/plan.md）。**这里不重新发明判据。**
_LIMITS = {
    'max_lateral_error_m': (0.30, 'm', 'SPEC §1'),
    'max_heading_error_deg': (2.00, 'deg', 'SPEC §1'),
    'loop_closure_error_m': (0.50, 'm', '绕环一圈回到起点的累积漂移'),
    'max_ndt_time_ms': (100.0, 'ms', '10 Hz 实时性'),
}


def _yaw_from_quaternion(q) -> float:
    """Extract the yaw angle from a ROS quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle_rad: float) -> float:
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class LocalizationRecorder(Node):
    """Listen to the estimate and the ground truth, and score the difference."""

    def __init__(self, duration_s: float):
        super().__init__('localization_run_recorder')
        self._duration_s = duration_s
        self._rows = []
        self._truth = None
        self._prev_truth = None
        self._dropped_unaligned = 0
        self._ndt_ms = 0.0
        self._state = ''
        self._start_ns = None
        self._first_truth = None
        self._last_truth = None

        qos = QoSProfile(depth=10)
        self.create_subscription(Odometry, '/ego_pose_gt', self._on_truth, qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/localization/pose', self._on_estimate, qos)
        self.create_subscription(
            DiagnosticArray, '/localization/diagnostics', self._on_diag, qos)

    def _on_truth(self, msg: Odometry) -> None:
        self._prev_truth = self._truth   # 插值要两条，见 _interpolated_truth
        self._truth = msg
        if self._first_truth is None:
            self._first_truth = msg
        self._last_truth = msg

    def _on_diag(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != 'localization':
                continue
            self._state = status.message
            for kv in status.values:
                if kv.key == 'ndt_time_ms':
                    self._ndt_ms = float(kv.value)

    def _interpolated_truth(self, stamp_ns):
        """Truth pose linearly interpolated to the estimate's own timestamp.

        ⚠️ 2026-08-12 复检补上的缺口：原来直接拿**最后一条**缓存的真值相减，
           真值 50 Hz ⟹ 最多陈旧 20 ms ⟹ 车速 5.5 m/s 时 **~0.11 m** 的
           方法误差混进测量 —— 与 0.30 m 的判据同量级的一大块。
           修法：缓存最近两条真值，把位置线性插值、朝向按角度插值到
           估计消息自己的时间戳上。外推超过一个周期（>25 ms）的样本丢弃
           （插值是内插不是预言）。

        :param stamp_ns: 估计消息的时间戳（ns）
        :return: (x, y, yaw) 或 None（真值不足/太陈旧）
        """
        if self._prev_truth is None or self._truth is None:
            return None
        t0 = rclpy.time.Time.from_msg(self._prev_truth.header.stamp).nanoseconds
        t1 = rclpy.time.Time.from_msg(self._truth.header.stamp).nanoseconds
        if t1 <= t0:
            return None
        # 允许少量外推（真值到达抖动），超过一个真值周期就放弃这个样本。
        if stamp_ns > t1 + (t1 - t0) or stamp_ns < t0:
            return None
        ratio = (stamp_ns - t0) / (t1 - t0)
        p0, p1 = self._prev_truth.pose.pose, self._truth.pose.pose
        x = p0.position.x + ratio * (p1.position.x - p0.position.x)
        y = p0.position.y + ratio * (p1.position.y - p0.position.y)
        yaw0 = _yaw_from_quaternion(p0.orientation)
        yaw1 = _yaw_from_quaternion(p1.orientation)
        yaw = yaw0 + ratio * _wrap(yaw1 - yaw0)
        return (x, y, yaw)

    def _on_estimate(self, msg: PoseWithCovarianceStamped) -> None:
        if self._truth is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if self._start_ns is None:
            self._start_ns = now_ns
        elapsed_s = (now_ns - self._start_ns) * 1e-9

        interpolated = self._interpolated_truth(
            rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds)
        if interpolated is None:
            self._dropped_unaligned += 1
            return
        estimate = msg.pose.pose
        dx = estimate.position.x - truth_pose.position.x
        dy = estimate.position.y - truth_pose.position.y
        truth_yaw = _yaw_from_quaternion(truth_pose.orientation)

        # ⚠️ 横向误差 = 误差矢量在**车体横轴**上的投影，不是误差的模。
        #    两者在直道上接近，转弯时能差一倍，而 SPEC §1 写的是横向。
        lateral_m = -dx * math.sin(truth_yaw) + dy * math.cos(truth_yaw)
        longitudinal_m = dx * math.cos(truth_yaw) + dy * math.sin(truth_yaw)
        heading_err_deg = math.degrees(
            _wrap(_yaw_from_quaternion(estimate.orientation) - truth_yaw))

        self._rows.append({
            't_s': round(elapsed_s, 4),
            'truth_x': round(truth_x, 4),
            'truth_y': round(truth_y, 4),
            'est_x': round(estimate.position.x, 4),
            'est_y': round(estimate.position.y, 4),
            'lateral_error_m': round(lateral_m, 5),
            'longitudinal_error_m': round(longitudinal_m, 5),
            'position_error_m': round(math.hypot(dx, dy), 5),
            'heading_error_deg': round(heading_err_deg, 5),
            'ndt_time_ms': round(self._ndt_ms, 3),
            'state': self._state,
        })

    def finished(self) -> bool:
        """True once the requested duration has elapsed."""
        return (self._start_ns is not None and
                (self.get_clock().now().nanoseconds - self._start_ns) * 1e-9 >=
                self._duration_s)

    def score(self, out_path: str) -> int:
        """Write the CSV and print the CP-P4-B verdict. Returns the failure count."""
        if not self._rows:
            print('✗ 一条样本都没有 —— localization_node 在跑吗？'
                  '/ego_pose_gt 有数据吗？', file=sys.stderr)
            return 1

        with open(out_path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self._rows[0]))
            writer.writeheader()
            writer.writerows(self._rows)

        # 时间对齐丢弃量要**报出来**：偶发几条正常（真值到达抖动），
        # 大量丢弃说明两条流的时钟对不上 —— 那时所有数字都不可信。
        total = len(self._rows) + self._dropped_unaligned
        if self._dropped_unaligned:
            print(f'时间对齐：丢弃 {self._dropped_unaligned}/{total} 条无法内插的样本'
                  f'（>5% 则本次测量不可信）')
        if total > 0 and self._dropped_unaligned / total > 0.05:
            print('✗ 时间对齐丢弃超过 5% —— 真值/估计两条流的时间戳对不上，'
                  '本次运行的测量不可信', file=sys.stderr)
            return 1

        # ⚠️ 只在**已经初始化**之后打分。冷启动的头几拍位置来自单次 GNSS
        #    （σ=2 m），把它算进最大值等于在考核"GNSS 那一枪准不准"。
        settled = [r for r in self._rows if r['state'] in ('NDT_AIDED', 'GNSS_ONLY')]
        if len(settled) < 10:
            print(f'✗ 收敛后的样本只有 {len(settled)} 拍', file=sys.stderr)
            return 1
        # 再去掉最前面 2 s（滤波器把 GNSS 初值拉回来的暂态）。
        t0 = settled[0]['t_s']
        settled = [r for r in settled if r['t_s'] - t0 > 2.0] or settled

        measured = {
            'max_lateral_error_m': max(abs(r['lateral_error_m']) for r in settled),
            'max_heading_error_deg': max(abs(r['heading_error_deg']) for r in settled),
            'loop_closure_error_m': self._loop_closure_m(),
            'max_ndt_time_ms': max(r['ndt_time_ms'] for r in settled),
        }

        states = {}
        for row in self._rows:
            states[row['state']] = states.get(row['state'], 0) + 1

        print(f'\n===== CP-P4-B 定位实测（{len(self._rows)} 拍，'
              f'收敛后 {len(settled)} 拍）=====')
        print(f'CSV: {out_path}')
        print('状态分布：' + '  '.join(f'{k} {v}' for k, v in sorted(states.items())))
        print(f'{"项":<26}{"实测":>12}{"判据":>10}  结果')
        failures = 0
        for key, value in measured.items():
            limit, unit, note = _LIMITS[key]
            # 容差只吸收测量精度，不放宽判据本身（与 record_control_run.py 一致）。
            ok = value <= limit * (1.0 + 1e-9)
            failures += 0 if ok else 1
            print(f'{key:<26}{value:>12.4f}{limit:>10.3f}  '
                  f'{"PASS" if ok else "FAIL"}  {unit} {note}')
        rms = math.sqrt(
            sum(r['lateral_error_m'] ** 2 for r in settled) / len(settled))
        print(f'{"rms_lateral_error_m":<26}{rms:>12.4f}{"—":>10}  参考')
        print(f'\n{"全部通过" if failures == 0 else f"{failures} 项未通过"}')
        return failures

    def _loop_closure_m(self) -> float:
        """Drift measured by how far the estimate and truth diverged end to end."""
        # ⚠️ 这不是「回到起点」的闭合误差 —— 本次验收路线不绕整圈。
        #    量的是**末态的位置误差**，它才是累积漂移的直接度量。
        if not self._rows:
            return float('inf')
        return self._rows[-1]['position_error_m']


def main() -> int:
    """Run the recorder for the requested duration and print the verdict."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=60.0)
    parser.add_argument('--out', default='/tmp/localization_run.csv')
    args = parser.parse_args()

    rclpy.init()
    node = LocalizationRecorder(args.duration_s)
    try:
        while rclpy.ok() and not node.finished():
            rclpy.spin_once(node, timeout_sec=0.1)
        failures = node.score(args.out)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
