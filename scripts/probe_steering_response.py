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
Identify the vehicle's steering actuator response (open loop, no controller).

**开环**探针：直接下发 /vehicle_cmd 的阶跃转角，量执行机构有多快、多准。
控制器不参与 —— 所以量到的是**被控对象本身**。

    python3 scripts/probe_steering_response.py --step 0.30 --speed 4.0

为什么这个脚本值得单独存在
------------------------
1. **L1 的运动学模型里没有执行机构**，转角是瞬时生效的。真物理里不是。
   CP-P2-B 第一次跑出横向震荡时，靠的就是这个探针把「控制器不行」
   和「执行机构慢」分开 —— 实测 Gazebo 的转向 63% 上升时间是 **1.20 s**，
   比控制器自己的闭环时间常数（1/k_e = 1.0 s）还大。
2. **P0b 对齐 CARLA 时要拿同一把尺子量一遍。** 两个环境的执行机构响应
   不一致，就是 SPEC §4.1 的头号风险「行为漂移」的物理来源，
   而它不会有任何一层报错 —— 只会表现为"本地调好的增益一上 CARLA 就震荡"。

判据来自运动学自行车模型 ω = v·tan(δ)/L：
稳态达成率看**准不准**，上升时间看**快不快**，两者是两回事，都要量。

⚠️ **必须在刚起好的仿真上跑，车要停在路面上。** 阶跃转角保持几秒会让车
   画一个半径 8 m 的圆 —— 起点若不在路中间，车会开到草地上，
   轮胎摩擦一变稳态达成率就掉（实测：路面上 99.7%，跑到草地上只有 86.5%）。
   上升时间对这个不敏感（三次实测都是 1.20–1.36 s），达成率敏感。
"""

from __future__ import annotations

import argparse
import math
import sys

from ads_msgs.msg import VehicleCmd
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEELBASE_M = 2.700  # config/vehicle_params.yaml 的 geometry.wheelbase_m


class SteeringProbe(Node):
    """Hold a straight line, then step the steering and record the response."""

    def __init__(self, step_rad: float, speed_mps: float, settle_s: float, hold_s: float):
        super().__init__('steering_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.step_rad = step_rad
        self.speed_mps = speed_mps
        self.settle_s = settle_s
        self.hold_s = hold_s
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.joint_rad = 0.0
        self.t0: float | None = None
        self.trace: list[tuple[float, float, float]] = []

        self.pub = self.create_publisher(VehicleCmd, '/vehicle_cmd', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 20)
        self.create_timer(0.02, self._tick)

    def _on_odom(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

    def _on_joints(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            # 取内侧（左）前轮。注意它按阿克曼几何比"自行车模型转角"大一点，
            # 左转时 tan(δ_内) = L/(R − 轮距/2) —— 别把这个差当成稳态误差。
            if 'left_steer' in name:
                self.joint_rad = position

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:       # /clock 还没来
            return
        if self.t0 is None:
            self.t0 = now
        elapsed = now - self.t0

        cmd = VehicleCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        # 一个简单的 P 把速度顶到目标，好让转向响应在**恒速**下测量 ——
        # 速度还在变的话，ω = v·tanδ/L 里的 v 也在变，分不清是谁引起的。
        cmd.accel_mps2 = max(-3.0, min(1.5, 1.0 * (self.speed_mps - self.speed)))
        cmd.steer_angle_rad = 0.0 if elapsed < self.settle_s else self.step_rad
        self.pub.publish(cmd)

        if elapsed >= self.settle_s:
            self.trace.append((elapsed - self.settle_s, self.joint_rad, self.yaw_rate))
        if elapsed > self.settle_s + self.hold_s:
            raise SystemExit(0)


def rise_time_s(trace, index: int, final: float, fraction: float) -> float:
    for sample in trace:
        if final != 0.0 and sample[index] / final >= fraction:
            return sample[0]
    return math.nan


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step', type=float, default=0.30, help='阶跃转角，rad')
    parser.add_argument('--speed', type=float, default=4.0, help='测量时保持的车速，m/s')
    parser.add_argument('--settle-s', type=float, default=8.0, help='阶跃前直行多久')
    parser.add_argument('--hold-s', type=float, default=6.0, help='阶跃后记录多久')
    args = parser.parse_args()

    rclpy.init()
    node = SteeringProbe(args.step, args.speed, args.settle_s, args.hold_s)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    trace = node.trace
    node.destroy_node()
    rclpy.shutdown()

    if len(trace) < 100:
        print('样本不足 —— 仿真起来了吗？', file=sys.stderr)
        return 1

    tail = trace[-80:]
    joint_final = sum(s[1] for s in tail) / len(tail)
    yaw_final = sum(s[2] for s in tail) / len(tail)
    expected_yaw = args.speed * math.tan(args.step) / WHEELBASE_M

    print(f'阶跃 δ: 0 → {args.step} rad，保持车速 {args.speed} m/s')
    print(f'  稳态横摆角速度 {yaw_final:.4f} rad/s，运动学期望 {expected_yaw:.4f} '
          f'→ **达成率 {yaw_final / expected_yaw * 100:.1f}%**')
    print(f'  稳态转向关节角 {joint_final:.4f} rad（内侧轮，按阿克曼几何应大于 {args.step}）')
    print(f'  转向关节   63% 上升 {rise_time_s(trace, 1, joint_final, 0.632):.3f} s，'
          f'90% {rise_time_s(trace, 1, joint_final, 0.9):.3f} s')
    print(f'  横摆角速度 63% 上升 {rise_time_s(trace, 2, yaw_final, 0.632):.3f} s，'
          f'90% {rise_time_s(trace, 2, yaw_final, 0.9):.3f} s')
    print('\n两者相差无几 = 滞后**全部来自转向执行机构**，车身动力学几乎不贡献。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
