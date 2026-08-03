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
Align a CARLA vehicle to vehicle_params.yaml and measure its steering response.

P0b **方案 B（最小对齐验证）** 的全部内容，一个脚本跑完：

    ① 按 config/vehicle_params.yaml 调 CARLA 车辆的物理参数
    ② 报告**哪些对齐了、哪些对不齐**（有些量 apply_physics_control 改不了）
    ③ 开环量转向阶跃响应，输出与 scripts/probe_steering_response.py **同一格式**的数

    python3 scripts/carla_align_vehicle.py --dry-run          # 不需要 CARLA，验单位换算
    python3 scripts/carla_align_vehicle.py --host 127.0.0.1   # 需要 CARLA

它要回答的唯一问题
------------------
**CARLA 的转向执行机构响应时间 τ 是多少？** 因为 P2 已经证明这个数值 12.7 倍：
Gazebo 侧 τ 从 1.198 s 降到 0.294 s，CP-P2-B 的最大横向误差就从 0.801 m 降到
0.0633 m —— 同一个控制器、同一组增益、同一条路径。
如果 CARLA 的 τ 与 Gazebo 差得远，现在这组 `k_e = 1.0` 在那边就不成立，
而症状会是「车照样开完全程、只是弯道误差大一个数量级」——
第一反应必然是去调增益，**而那是错的**。

⚠️ **本脚本不经过 ROS，这是有意的。**
   `probe_steering_response.py` 走 `/vehicle_cmd` + `/odom`，那需要 `carla_bridge`，
   而 bridge 是完整 P0b（方案 A）的第 4 步，**不在方案 B 范围内**。
   所以这里直接用 PythonAPI。两者量的是同一件事（下发转角 → 达成的横摆角速度），
   但**命令链路不同**：Gazebo 侧多经过 `gazebo_bridge` 一层。
   比较 τ 时要记得这一点 —— 若两边差异很大，先排除 bridge 那一层的贡献。

⚠️ **本脚本从未在真实 CARLA 上跑过。** 写它的时候手边没有 Vulkan 硬件
   （本机只有 llvmpipe，见 CLAUDE.md）。`--dry-run` 覆盖的是单位换算和参数映射
   ——**那正是最容易错、又最不该在计费机器上调试的部分**；
   连接、spawn、同步模式那些只能上机验。第一次上机请留出排错时间。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
PARAMS_FILE = REPO_ROOT / 'config' / 'vehicle_params.yaml'


def load_targets() -> dict:
    """Read vehicle_params.yaml and convert to the units CARLA's API expects.

    **单位换算集中在这一个函数里**，因为它是本脚本最容易出错的地方，
    而且是唯一能在没有 CARLA 时验证的部分：

    | 量 | 本仓库 | CARLA |
    |---|---|---|
    | 最大转角 | rad | **度** |
    | 车轮半径 | m | **厘米** |
    | 转角指令 | rad（`VehicleCmd.steer_angle_rad`） | **归一化 [-1, 1]** |

    最后一条尤其阴险：`carla.VehicleControl.steer` 不是角度，是"占最大转角的比例"。
    直接把 0.3 rad 填进去，实际转的是 0.3 × max_steer_angle —— 若蓝图默认
    max_steer_angle 是 70°（CARLA 常见），那就是 0.37 rad，差 22%，
    而且**不会报错**，只会让量出来的 τ 和达成率都偏。

    :return: 目标物理参数（已换算到 CARLA 的单位）
    """
    params = yaml.safe_load(PARAMS_FILE.read_text(encoding='utf-8'))
    geo = params['geometry']
    lim = params['limits']
    return {
        'mass_kg': params['mass']['total_kg'],
        'wheelbase_m': geo['wheelbase_m'],
        'track_width_m': geo['track_width_m'],
        'com_height_m': geo['com_height_m'],
        'wheel_radius_cm': params['wheel']['radius_m'] * 100.0,
        'max_steer_angle_deg': math.degrees(lim['max_steer_angle_rad']),
        'max_steer_angle_rad': lim['max_steer_angle_rad'],
        'max_speed_mps': lim['max_speed_mps'],
        # Gazebo 侧的实测值，供上机时**当场对比**，不必回来翻文档。
        'gazebo_tau_s': 0.294,
        'gazebo_steady_ratio': 1.007,
    }


def print_targets(targets: dict) -> None:
    print('=== 目标参数（来自 config/vehicle_params.yaml）===')
    print(f'  质量            {targets["mass_kg"]:.1f} kg')
    print(f'  轴距            {targets["wheelbase_m"]:.3f} m      ← ⚠️ apply_physics_control 改不了，见下')
    print(f'  轮距            {targets["track_width_m"]:.3f} m      ← ⚠️ 同上')
    print(f'  质心高          {targets["com_height_m"]:.3f} m')
    print(f'  车轮半径        {targets["wheel_radius_cm"]:.2f} cm    （YAML 里是 m，CARLA 要 cm）')
    print(f'  最大转角        {targets["max_steer_angle_deg"]:.3f}°     '
          f'（YAML 里是 {targets["max_steer_angle_rad"]:.3f} rad，CARLA 要度）')
    print()
    print('=== 换算自检（这部分不需要 CARLA）===')
    step_rad = 0.30
    norm = step_rad / targets['max_steer_angle_rad']
    print(f'  下发 {step_rad} rad → VehicleControl.steer = {norm:.4f}（归一化，非角度）')
    assert 0.0 < norm <= 1.0, '归一化转角越界 —— 阶跃比最大转角还大？'
    back = norm * targets['max_steer_angle_rad']
    assert abs(back - step_rad) < 1e-12, '来回换算不自洽'
    print(f'  反算回来 {back:.6f} rad ✓')
    print()
    print('=== 已知对不齐的量（**这些是方案 B 的产出之一，不是缺陷**）===')
    print('  轴距 / 轮距：由蓝图的网格与车轮位置决定，apply_physics_control 改不了。')
    print('    → 脚本会**量出来**实际值并报告与 2.700 m 的差。')
    print('    → 差太多的话，对齐手段是**换蓝图**，不是改参数。')
    print('    → 轴距直接进 Stanley 的前轴换算和自行车模型，差 10% 就是控制律的输入错了。')
    print('  轮胎模型：CARLA 有侧偏刚度，Gazebo 的 AckermannSteering 没有。')
    print('    → 这不是"要对齐的参数"，是**两个环境的本质差异**，正是 CP-P2-B 之外')
    print('       还需要云端验收的原因（SPEC §4.1）。')


def measure(args, targets: dict) -> int:
    """Connect to CARLA, align the vehicle, and run the open-loop steering step."""
    try:
        import carla
    except ImportError:
        print('✗ 导入不了 carla —— PythonAPI 装了吗？'
              '（CARLA 0.9.16 官方提供 3.10/3.11/3.12 wheel）', file=sys.stderr)
        return 1

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()

    # 同步模式 + 固定步长：**确定性的前提**。异步模式下每次量出来的 τ 都不一样，
    # 那就没法拿它和 Gazebo 的 0.294 s 比。
    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = args.step_s
    world.apply_settings(settings)

    vehicle = None
    try:
        blueprint = world.get_blueprint_library().find(args.blueprint)
        spawn = world.get_map().get_spawn_points()[0]
        vehicle = world.spawn_actor(blueprint, spawn)
        world.tick()

        physics = vehicle.get_physics_control()

        # ---- 先量再改：轴距/轮距是量出来的，不是设出来的 ----
        wheels = physics.wheels
        # CARLA 的 wheel.position 是**世界坐标、单位 cm**。转成车身系的纵向间距。
        front_x = (wheels[0].position.x + wheels[1].position.x) / 2.0
        rear_x = (wheels[2].position.x + wheels[3].position.x) / 2.0
        front_y = (wheels[0].position.y + wheels[1].position.y) / 2.0
        rear_y = (wheels[2].position.y + wheels[3].position.y) / 2.0
        measured_wheelbase_m = math.hypot(front_x - rear_x, front_y - rear_y) / 100.0
        measured_track_m = abs(wheels[0].position.y - wheels[1].position.y) / 100.0

        print(f'\n=== 蓝图 {args.blueprint} 的固有几何（改不了，只能换蓝图）===')
        print(f'  实测轴距 {measured_wheelbase_m:.3f} m  vs 目标 {targets["wheelbase_m"]:.3f} m'
              f'  → 偏差 {measured_wheelbase_m - targets["wheelbase_m"]:+.3f} m'
              f'（{(measured_wheelbase_m / targets["wheelbase_m"] - 1) * 100:+.1f}%）')
        print(f'  实测轮距 {measured_track_m:.3f} m  vs 目标 {targets["track_width_m"]:.3f} m')
        if abs(measured_wheelbase_m / targets['wheelbase_m'] - 1.0) > 0.05:
            print('  ⚠️ 轴距差超过 5% —— 它直接进 Stanley 的前轴换算和自行车模型，')
            print('     这个偏差会独立于转向响应再贡献一份行为漂移。换个蓝图试试。')

        # ---- 能改的都改 ----
        physics.mass = targets['mass_kg']
        physics.center_of_mass = carla.Vector3D(0.0, 0.0, targets['com_height_m'])
        for wheel in physics.wheels:
            wheel.max_steer_angle = targets['max_steer_angle_deg']
            wheel.radius = targets['wheel_radius_cm']
        physics.wheels = physics.wheels  # CARLA 要求整体回写才生效
        vehicle.apply_physics_control(physics)
        world.tick()
        print(f'\n=== 已对齐：质量 {targets["mass_kg"]} kg、'
              f'最大转角 {targets["max_steer_angle_deg"]:.3f}°、'
              f'轮半径 {targets["wheel_radius_cm"]:.1f} cm ===')

        # ---- 加速到目标速度，转角保持 0 ----
        steer_norm = args.step_rad / targets['max_steer_angle_rad']
        trace = []
        settle_ticks = int(args.settle_s / args.step_s)
        hold_ticks = int(args.hold_s / args.step_s)

        for i in range(settle_ticks + hold_ticks):
            velocity = vehicle.get_velocity()
            speed_mps = math.hypot(velocity.x, velocity.y)
            # 简单的 P 油门，只为把速度稳在目标上 —— 转向响应要在**恒速**下量，
            # 速度还在变的话 ω = v·tanδ/L 里的 v 也在变，分不清是谁引起的。
            error = args.speed_mps - speed_mps
            control = carla.VehicleControl()
            control.throttle = float(max(0.0, min(1.0, 0.5 * error)))
            control.brake = float(max(0.0, min(1.0, -0.5 * error)))
            control.steer = 0.0 if i < settle_ticks else float(steer_norm)
            vehicle.apply_control(control)
            world.tick()

            if i >= settle_ticks:
                # get_angular_velocity() 是**度/秒、世界系**。转成 rad/s。
                yaw_rate = math.radians(vehicle.get_angular_velocity().z)
                trace.append(((i - settle_ticks) * args.step_s, speed_mps, yaw_rate))

        report(trace, args, targets)
        return 0
    finally:
        if vehicle is not None:
            vehicle.destroy()
        world.apply_settings(original)


def report(trace, args, targets) -> None:
    """Print the same numbers, in the same shape, as probe_steering_response.py."""
    if len(trace) < 50:
        print('✗ 样本不足', file=sys.stderr)
        return
    tail = trace[-int(len(trace) * 0.2):]
    speed = sum(s for _, s, _ in tail) / len(tail)
    yaw_final = sum(w for _, _, w in tail) / len(tail)
    expected = speed * math.tan(args.step_rad) / targets['wheelbase_m']

    def rise(fraction):
        for t, _, w in trace:
            if yaw_final != 0.0 and w / yaw_final >= fraction:
                return t
        return math.nan

    tau = rise(0.632)
    print(f'\n=== CARLA 转向阶跃响应（δ: 0 → {args.step_rad} rad，v ≈ {speed:.2f} m/s）===')
    print(f'  稳态横摆角速度 {yaw_final:.4f} rad/s，运动学期望 {expected:.4f}'
          f'  → **达成率 {yaw_final / expected * 100:.1f}%**')
    print(f'  **63% 上升时间 τ = {tau:.3f} s**，90% = {rise(0.9):.3f} s')
    print()
    print('=== 与 Gazebo 侧对比（这就是方案 B 要的那个数）===')
    print(f'  Gazebo τ = {targets["gazebo_tau_s"]:.3f} s  '
          f'（达成率 {targets["gazebo_steady_ratio"] * 100:.1f}%）')
    print(f'  CARLA  τ = {tau:.3f} s')
    if not math.isnan(tau) and targets['gazebo_tau_s'] > 0:
        ratio = tau / targets['gazebo_tau_s']
        print(f'  **比值 {ratio:.2f}×**')
        print()
        if 0.5 <= ratio <= 2.0:
            print('  → 同一量级。现在这组 k_e = 1.0 在 CARLA 上大概率仍然成立，')
            print('     P0b 的紧迫性可以正式下调（有依据，不是侥幸）。')
        else:
            print('  → **不同量级。** P2 实测过：τ 变 4 倍，横向误差变 12.7 倍。')
            print('     现在这组增益不能直接搬过去，而且症状会是"车能开完但弯道误差大"——')
            print('     那种情况下人的第一反应是调 k_e，**方向是错的**。')
        print()
        print('  ⚠️ 比较时记得：Gazebo 侧的数经过 gazebo_bridge 一层，本脚本没有。')
        print('     差异显著时，先排除 bridge 那一层的贡献再下结论。')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true',
                        help='只验参数映射与单位换算，不连 CARLA')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--blueprint', default='vehicle.tesla.model3')
    parser.add_argument('--step-rad', type=float, default=0.30, help='阶跃转角，rad')
    parser.add_argument('--speed-mps', type=float, default=4.0, help='测量时保持的车速')
    parser.add_argument('--step-s', type=float, default=0.02, help='固定仿真步长')
    parser.add_argument('--settle-s', type=float, default=8.0)
    parser.add_argument('--hold-s', type=float, default=6.0)
    args = parser.parse_args()

    targets = load_targets()
    print_targets(targets)
    if args.dry_run:
        print('\n✓ --dry-run：参数映射与单位换算自洽。连接部分未验证（需要 CARLA）。')
        return 0
    return measure(args, targets)


if __name__ == '__main__':
    sys.exit(main())
