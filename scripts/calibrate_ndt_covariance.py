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

"""Calibrate ads_localization's `ndt.covariance_scale` by measuring NEES against truth.

**要解决的问题**：`AlignNdt` 输出的协方差是 `covariance_scale × H⁻¹`，而
`H` 是 Gauss-Newton 信息阵 —— 它只与真实协方差**成正比**，比例未知
（本实现的得分函数不是严格归一化的对数似然，见 localization.md §9.5）。
`covariance_scale` 至今是 **1.0，未标定**。

**没标定的后果不是「不准」，是「权重错」**：这个协方差直接决定 NDT 在 ESKF 里
相对 GNSS（σ=2 m）的话语权。偏小 → 滤波器过度相信 NDT；偏大 → NDT 说了不算，
位姿被 GNSS 拉着走。**两种都不报错。**

**怎么标**：NEES（normalized estimation error squared）

    e     = NDT 位姿 − 真值位姿          （6 维：位置 3 + 姿态 3）
    NEES  = eᵀ Σ⁻¹ e                     （Σ 是 NDT 自报的协方差）
    E[NEES] = 自由度 = 6                  （当且仅当 Σ 是对的）

于是：**实测 NEES 均值 m ⟹ 正确的 scale = 当前 scale × m / 6**
（因为 Σ ∝ scale，NEES ∝ 1/scale）。

⚠️ **姿态误差用 Log(q_ndt⁻¹ ⊗ q_truth)**，不是四元数相减 ——
   后者在大角度下不是李代数上的误差，NEES 会算出一个没有意义的数。

⚠️ **时间对齐是这个脚本最容易错的地方。** NDT 位姿带的是**点云的**时间戳
   （节点里特意这么填的），真值 50 Hz。两者差一拍就是 4 m/s × 20 ms = 8 cm，
   而 NDT 自报的 σ 是毫米级 —— **对齐差一拍，NEES 会大出好几个数量级**。
   本脚本按时间戳最近邻配对，并拒绝时间差 > `--max-dt-s` 的样本。

用法（需要一个正在跑的仿真，localization:=true）：
    ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false localization:=true
    python3 scripts/calibrate_ndt_covariance.py --duration-s 50
"""

import argparse
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

# 自由度：位置 3 + 姿态 3。
DOF = 6


def stamp_to_seconds(stamp) -> float:
    """Convert a ROS time message to float seconds.

    :param stamp: builtin_interfaces/Time
    :return: 秒
    """
    return stamp.sec + stamp.nanosec * 1e-9


def quaternion_log(qw: float, qx: float, qy: float, qz: float) -> list:
    """Log map of a unit quaternion — the 3-vector rotation error in the Lie algebra.

    :param qw: 实部
    :param qx: i 分量
    :param qy: j 分量
    :param qz: k 分量
    :return: 三维旋转向量（rad）
    """
    # ⚠️ q 与 −q 是同一个旋转。不取正实部的话，接近 π 时会得到一个反向的
    #    大角度，NEES 直接爆掉 —— 与 ComparePoses 里那个坑同源。
    if qw < 0.0:
        qw, qx, qy, qz = -qw, -qx, -qy, -qz
    norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        return [2.0 * qx, 2.0 * qy, 2.0 * qz]
    angle = 2.0 * math.atan2(norm, qw)
    return [angle * qx / norm, angle * qy / norm, angle * qz / norm]


def quaternion_inverse_times(a, b) -> list:
    """Rotation error Log(a⁻¹ ⊗ b) as a 3-vector.

    :param a: 估计姿态（geometry_msgs/Quaternion）
    :param b: 真值姿态（geometry_msgs/Quaternion）
    :return: 三维旋转误差
    """
    # a⁻¹ = conj(a)（单位四元数）
    aw, ax, ay, az = a.w, -a.x, -a.y, -a.z
    bw, bx, by, bz = b.w, b.x, b.y, b.z
    return quaternion_log(
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def solve_6x6(matrix: list, rhs: list) -> list:
    """Solve `matrix · x = rhs` by Gauss elimination with partial pivoting.

    自己写而不是依赖 numpy：容器里的宿主 python 没有 numpy，而这是 6×6，
    高斯消元十几行就够，比多一个依赖划算。

    :param matrix: 6×6，按行存的列表
    :param rhs: 长度 6
    :return: 解；矩阵奇异时返回 None
    """
    size = len(rhs)
    augmented = [list(matrix[i]) + [rhs[i]] for i in range(size)]
    for col in range(size):
        pivot = max(range(col, size), key=lambda r: abs(augmented[r][col]))
        if abs(augmented[pivot][col]) < 1e-18:
            return None
        augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        for row in range(size):
            if row == col:
                continue
            factor = augmented[row][col] / augmented[col][col]
            for k in range(col, size + 1):
                augmented[row][k] -= factor * augmented[col][k]
    return [augmented[i][size] / augmented[i][i] for i in range(size)]


class NeesCollector(Node):
    """Pair each raw NDT pose with the nearest ground-truth sample and compute NEES."""

    def __init__(self, max_dt_s: float):
        super().__init__('ndt_covariance_calibrator')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.max_dt_s = max_dt_s
        self.truth = []          # [(t, msg), ...] 环形留最近若干条
        self.nees = []
        self.errors = []         # 每维的误差，用来说明是哪一维在超
        self.skipped_dt = 0
        self.skipped_singular = 0

        # ⚠️ 真值只有评测脚本能订阅（SPEC §4.1）。本脚本正是评测脚本。
        self.create_subscription(Odometry, '/ego_pose_gt', self._on_truth, 100)
        self.create_subscription(
            PoseWithCovarianceStamped, '/localization/ndt_pose', self._on_ndt, 50)

    def _on_truth(self, msg):
        self.truth.append((stamp_to_seconds(msg.header.stamp), msg))
        if len(self.truth) > 400:
            del self.truth[:200]

    def _on_ndt(self, msg):
        if not self.truth:
            return
        t_ndt = stamp_to_seconds(msg.header.stamp)
        best = min(self.truth, key=lambda item: abs(item[0] - t_ndt))
        if abs(best[0] - t_ndt) > self.max_dt_s:
            self.skipped_dt += 1
            return

        truth_pose = best[1].pose.pose
        estimate = msg.pose.pose
        error = [
            estimate.position.x - truth_pose.position.x,
            estimate.position.y - truth_pose.position.y,
            estimate.position.z - truth_pose.position.z,
        ]
        # ⚠️ 姿态误差取 Log(q_est⁻¹ ⊗ q_truth)，**符号方向与位置一致**
        #    （都是「估计相对真值」）—— 混了方向的话交叉项符号错，NEES 偏小。
        error += [-v for v in quaternion_inverse_times(estimate.orientation, truth_pose.orientation)]

        covariance = [[msg.pose.covariance[i * 6 + j] for j in range(6)] for i in range(6)]
        solution = solve_6x6(covariance, error)
        if solution is None:
            self.skipped_singular += 1
            return
        self.nees.append(sum(error[i] * solution[i] for i in range(6)))
        self.errors.append(error)

    def report(self, current_scale: float) -> None:
        """Print the measured NEES and the scale it implies.

        :param current_scale: 本次运行时节点用的 ndt.covariance_scale
        """
        print(f'\n配对样本 {len(self.nees)} 个'
              f'（因时间差过大丢弃 {self.skipped_dt}，协方差奇异丢弃 {self.skipped_singular}）')
        if len(self.nees) < 30:
            print('✗ 样本太少，结论不可用。仿真起来了吗？NDT 有没有一直被拒？')
            return

        ordered = sorted(self.nees)
        mean = sum(self.nees) / len(self.nees)
        median = ordered[len(ordered) // 2]
        print(f'NEES  均值 {mean:.4g}   中位数 {median:.4g}   '
              f'5% {ordered[int(len(ordered) * 0.05)]:.4g}   '
              f'95% {ordered[int(len(ordered) * 0.95)]:.4g}')
        print(f'理论期望 = 自由度 = {DOF}')

        # 每一维的 RMS 误差 —— 说明是哪一维在主导。
        rms = [math.sqrt(sum(e[i] ** 2 for e in self.errors) / len(self.errors)) for i in range(6)]
        print('各维 RMS 误差： x %.4f  y %.4f  z %.4f  rx %.5f  ry %.5f  rz %.5f'
              % tuple(rms))

        # ⚠️ 用**中位数**反解，不用均值：NEES 的分布是重尾的（偶尔一帧 NDT
        #    半收敛就会贡献一个巨大的值），均值会被少数帧主导。
        implied = current_scale * median / DOF
        print(f'\n⟹ 建议的 ndt.covariance_scale = {current_scale} × {median:.4g} / {DOF}'
              f' = **{implied:.4g}**')
        if median > DOF:
            print('   （中位数 > 6 ⟹ NDT **过度自信**，现在的协方差比真实的小，'
                  '滤波器太信它 —— 要调大 scale）')
        else:
            print('   （中位数 < 6 ⟹ NDT **过于保守**，现在的协方差比真实的大，'
                  'GNSS 说了算 —— 要调小 scale）')
        print('\n⚠️ 一轮的结果不能直接填。至少跑两轮看它稳不稳，'
              '\n   并且换一段路线再量一次 —— 一段直路上标出来的系数只在那段路成立。')


def main() -> int:
    """Entry point.

    :return: 进程退出码
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=50.0, help='采样时长（墙钟秒）')
    parser.add_argument(
        '--current-scale', type=float, default=1.0,
        help='本次运行时节点用的 ndt.covariance_scale（默认 1.0）')
    parser.add_argument(
        '--max-dt-s', type=float, default=0.011,
        help=('NDT 与真值配对的最大时间差。真值 50 Hz → 半个周期是 10 ms，'
              '取 11 ms 刚好收下最近邻而拒掉隔了一拍的'))
    args = parser.parse_args()

    rclpy.init()
    node = NeesCollector(args.max_dt_s)
    deadline = time.monotonic() + args.duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.report(args.current_scale)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
