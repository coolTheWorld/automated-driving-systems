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
"""CARLA 油门 → 加速度的二维图 a(throttle, v)（P9-S5d「中速段标定戏」的尺子）.

carla_calibrate_throttle.py 只在 v∈[2,6] 整段取一个斜率、三档油门 —— 得到 bias+k 的
线性映射（0.54 + 0.113·a）。P8 入表的 S01 四条跟踪差异（弯道 目标 3.9 → 实测 4.6）
怀疑的是**中速段增益偏大** —— 那是一个 (throttle, v) 二维现象，一维尺子量不出来。
本脚本：油门细分 9 档、每档从静止拉到 6.5 m/s，按速度分箱（1 m/s 一箱）用有限差分
算每箱的加速度，印一张表 + 逐箱反解「要 a=1 m/s² 该给多少油门」，供拍板：
线性映射够不够、要不要按速度调度增益。

用法（云机容器内，**栈没在跑时**；会 load Town10 —— 下一轮栈起来会重载园区世界）：
    python3 scripts/carla_throttle_map_probe.py [--host 127.0.0.1] [--no-load]
异步模式；蓝图 citroen.c3（与 sidecar 同）。
"""
import argparse
import math
import time

import carla


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--no-load', action='store_true', help='不 load Town10，用当前世界')
    parser.add_argument('--throttles', default='0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.9')
    parser.add_argument('--spawn-index', type=int, default=3)
    parser.add_argument('--align', action='store_true',
                        help='对齐实验：发动机零油门阻尼 = 全油门阻尼、扭矩曲线拉平（见脚本头）')
    parser.add_argument('--flat-torque-nm', type=float, default=350.0)
    args = parser.parse_args()

    client = carla.Client(args.host, 2000)
    client.set_timeout(60.0)
    if not args.no_load:
        world = client.load_world('Town10HD_Opt')
        time.sleep(3.0)
    world = client.get_world()
    settings = world.get_settings()
    if settings.synchronous_mode:
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
    # ⚠️ 栈在跑时禁止运行：下面会清掉世界里所有车辆。sidecar 的自车 role_name 是
    #    ego_vehicle —— 见到它就拒跑，别把正在验收的自车销毁掉（复审 #8）。
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name', '') == 'ego_vehicle':
            raise SystemExit('检测到 role_name=ego_vehicle 的车辆 —— 栈还在跑，拒绝执行'
                             '（本脚本会清掉世界里所有车辆）。先收栈再来。')
    # 清掉遗留车辆（上一轮栈收了之后残留的自车/NPC）
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    time.sleep(0.5)

    blueprint = world.get_blueprint_library().find('vehicle.citroen.c3')
    spawn = world.get_map().get_spawn_points()[args.spawn_index]
    vehicle = world.spawn_actor(blueprint, spawn)
    time.sleep(1.0)
    # 与 sidecar 一致：转向曲线拉平（不影响直线油门，但保持同一辆"车"）
    physics = vehicle.get_physics_control()
    physics.steering_curve = [carla.Vector2D(x=0.0, y=1.0), carla.Vector2D(x=50.0, y=1.0)]
    if args.align:
        # PhysX 车辆发动机：净扭矩 = th·T(rpm) − [d0 − (d0 − d1)·th]·ω，d0 = 零油门阻尼
        # （2.0）、d1 = 全油门阻尼（0.15）。零油门阻尼随 ω 长 ⟹ 「顶住阻尼」的油门
        # 偏置随速度变（0.50→0.67）；T(rpm) 峰在 1700 rpm（1 挡 ≈ 4.8 m/s）⟹ 增益在
        # 4–5 m/s 箱最大。两刀拉平成 Gazebo 那种线性力模型：阻尼与油门无关、扭矩恒定。
        physics.damping_rate_zero_throttle_clutch_engaged = physics.damping_rate_full_throttle
        physics.damping_rate_zero_throttle_clutch_disengaged = physics.damping_rate_full_throttle
        physics.torque_curve = [carla.Vector2D(x=pt.x, y=args.flat_torque_nm)
                                for pt in physics.torque_curve]
    vehicle.apply_physics_control(physics)
    print(f'质量 {physics.mass:.0f} kg，最大转速 {physics.max_rpm:.0f}，'
          f'档位 {len(physics.forward_gears)}，蓝图 citroen.c3，spawn #{args.spawn_index}，'
          f'align={args.align}（阻尼 {physics.damping_rate_zero_throttle_clutch_engaged:.2f}/'
          f'{physics.damping_rate_full_throttle:.2f}，扭矩曲线 '
          f'{[round(pt.y) for pt in physics.torque_curve]}）')

    def speed():
        vel = vehicle.get_velocity()
        return math.hypot(vel.x, vel.y)

    def reset():
        vehicle.apply_control(carla.VehicleControl(brake=1.0))
        t0 = time.time()
        while speed() > 0.05 and time.time() - t0 < 8:
            time.sleep(0.05)
        vehicle.set_transform(spawn)
        vehicle.apply_control(carla.VehicleControl(brake=1.0))
        time.sleep(0.8)
        vehicle.apply_control(carla.VehicleControl())
        time.sleep(0.3)

    bins = [(1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 5.0), (5.0, 6.0)]
    table = {}
    for th in [float(v) for v in args.throttles.split(',')]:
        reset()
        vehicle.apply_control(carla.VehicleControl(throttle=th))
        samples = []
        t0 = time.time()
        while time.time() - t0 < 20:
            samples.append((time.time(), speed()))
            if samples[-1][1] > 6.5:
                break
            time.sleep(0.05)
        vehicle.apply_control(carla.VehicleControl(brake=1.0))
        row = {}
        for lo, hi in bins:
            band = [(t, s) for t, s in samples if lo <= s < hi]
            if len(band) >= 4 and band[-1][0] - band[0][0] >= 0.25:
                row[(lo, hi)] = (band[-1][1] - band[0][1]) / (band[-1][0] - band[0][0])
            else:
                row[(lo, hi)] = None
        peak = max(s for _, s in samples)
        table[th] = (row, peak, samples[-1][0] - t0)
        cells = '  '.join(f'{v:5.2f}' if v is not None else '  ---' for v in row.values())
        print(f'throttle {th:.2f}: 峰速 {peak:4.2f} @ {samples[-1][0] - t0:4.1f} s | a(m/s²) 按速度箱 '
              f'[1-2,2-3,3-4,4-5,5-6]: {cells}', flush=True)

    print('\n===== a(throttle, v) 表 =====')
    header = 'throttle | ' + ' | '.join(f'v {lo:.0f}-{hi:.0f}' for lo, hi in bins)
    print(header)
    for th, (row, _, _) in table.items():
        print(f'  {th:.2f}   | ' + ' | '.join(f'{v:6.2f}' if v is not None else '   ---' for v in row.values()))
    print('\n当前映射 throttle = 0.54 + 0.113·a ⟹ a=1 ⟹ 0.653、a=1.5 ⟹ 0.710、a=0.5 ⟹ 0.597；'
          '逐箱线性反解「a=1.0 需要的油门」（用相邻两档内插）：')
    for lo, hi in bins:
        pts = [(th, row[(lo, hi)]) for th, (row, _, _) in table.items() if row[(lo, hi)] is not None]
        pts.sort()
        need = None
        for (t1, a1), (t2, a2) in zip(pts, pts[1:]):
            if a1 <= 1.0 <= a2 and a2 > a1:
                need = t1 + (1.0 - a1) * (t2 - t1) / (a2 - a1)
                break
        slope = None
        if len(pts) >= 2:
            # 最小二乘斜率 da/dthrottle（越大 = 增益越大）
            n = len(pts)
            mx = sum(t for t, _ in pts) / n
            ma = sum(a for _, a in pts) / n
            den = sum((t - mx) ** 2 for t, _ in pts)
            slope = sum((t - mx) * (a - ma) for t, a in pts) / den if den > 0 else None
        print(f'  v {lo:.0f}-{hi:.0f}: a=1.0 需 throttle {need if need is None else round(need, 3)}，'
              f'da/dthrottle {slope if slope is None else round(slope, 2)} m/s² 每 1.0 油门'
              f'（映射假定 1/0.113 = 8.85）')
    vehicle.destroy()


if __name__ == '__main__':
    main()
