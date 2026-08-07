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
#  实测 IMU / GNSS 的噪声是不是真的按 config/vehicle_params.yaml 生效了
#
#  为什么需要这个脚本，而不是"生成物里有 <noise> 就行"
#  ------------------------------------------------------
#  P4 之前这两个传感器**一个噪声参数都没有**，输出的是完美真值。
#  那种情况下 ESKF 无事可做，而 SPEC §1 的「定位误差 < 0.3 m」**恒成立** ——
#  判据全绿，却什么都没验。这个脚本把「噪声确实在」变成一个可量的事实。
#
#  ⚠️ 它还守着一个具体的、已经踩到的坑：**Gazebo 的水平定位噪声按「度」施加**，
#     而 SDF 规范写的是「米」。gen_vehicle_model.py 为此做了 1/111320 的换算。
#     万一哪天 Gazebo 把这个 bug 修成米，那个换算就会把噪声缩小 11 万倍 ——
#     GNSS 悄悄变回完美真值，而 P4 的判据会全部变绿。
#     **那种失效没有任何报错**，只有实测拦得住。
#
#  用法（容器内，需要有一个正在跑的 Gazebo）：
#      ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false
#      python3 scripts/check_sensor_noise.py
#
#  车必须**静止**：静止时陀螺真值为 0、加速度计真值为重力、GNSS 真值为常数，
#  于是量到的散布全部是噪声。车动起来就分不清哪些是噪声、哪些是真实运动。
# =============================================================================

"""Measure IMU and GNSS noise in a running Gazebo and compare with the YAML."""

import argparse
import math
import re
import statistics
import subprocess
import sys
from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parent.parent
_PARAMS_FILE = _REPO_ROOT / 'config' / 'vehicle_params.yaml'

# 判据的宽窄。取 [0.5, 2.0] 倍是有依据的：本脚本的样本量在几百条量级，
# 标准差自身的相对不确定度约 1/sqrt(2n) ≈ 4%，所以 2 倍的窗口极宽松 ——
# 它要抓的不是"标定得准不准"，而是"**噪声在不在**"这种数量级错误
# （少一个噪声块 = 0 倍，量纲搞错 = 十万倍）。
_LO, _HI = 0.5, 2.0

# 一度纬度对应的地面距离。与 gen_vehicle_model.py 里的 _M_PER_DEG_LAT 同源；
# 这里再写一遍是有意的：**两边独立算，才能验出那边的换算是不是对的**。
_M_PER_DEG_LAT = 111320.0


def _sample(topic: str, count: int, timeout_s: float) -> str:
    """Read `count` messages from a Gazebo transport topic."""
    try:
        done = subprocess.run(['gz', 'topic', '-e', '-t', topic, '-n', str(count)],
                              capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return ''
    return done.stdout


def _vectors(text: str, field: str) -> list:
    """Pull every `field { x: .. y: .. z: .. }` block out of the dump."""
    out = []
    for match in re.finditer(field + r'\s*\{([^}]*)\}', text):
        body = match.group(1)
        vals = {}
        for key in ('x', 'y', 'z'):
            hit = re.search(rf'\b{key}:\s*(-?[\d.eE+-]+)', body)
            if hit:
                vals[key] = float(hit.group(1))
        if len(vals) == 3:
            out.append((vals['x'], vals['y'], vals['z']))
    return out


def _scalars(text: str, field: str) -> list:
    """Pull every `field: <number>` value out of the dump."""
    return [float(v) for v in re.findall(rf'\b{field}:\s*(-?[\d.eE+-]+)', text)]


def _verdict(measured: float, nominal: float) -> tuple:
    """Judge one measurement against its nominal value."""
    ok = _LO * nominal <= measured <= _HI * nominal
    return ok, ('✓' if ok else f'✗ 应在 [{_LO * nominal:.3g}, {_HI * nominal:.3g}]')


def main() -> int:
    """Sample the sensors, compare against the YAML, and report a verdict."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model', default='ego_vehicle')
    ap.add_argument('--imu-samples', type=int, default=400)
    ap.add_argument('--gnss-samples', type=int, default=120)
    args = ap.parse_args()

    params = yaml.safe_load(_PARAMS_FILE.read_text(encoding='utf-8'))
    imu_n = params['sensors']['imu']['noise']
    gnss_n = params['sensors']['gnss']['noise']
    failures = []

    # ---- IMU -----------------------------------------------------------
    dump = _sample(f'/model/{args.model}/imu', args.imu_samples, 60.0)
    gyro = _vectors(dump, 'angular_velocity')
    accel = _vectors(dump, 'linear_acceleration')
    if len(gyro) < 20:
        print(f'✗ /model/{args.model}/imu 收不到足够样本（{len(gyro)} 条）—— '
              'Gazebo 在跑吗？', file=sys.stderr)
        return 1

    print(f'IMU：{len(gyro)} 条样本')
    for label, data, nominal in (
            ('陀螺 rad/s', gyro, imu_n['gyro_stddev_rad_s']),
            ('加速度计 m/s²', accel, imu_n['accel_stddev_mps2'])):
        print(f'  {label}  标称 stddev {nominal:.3g}')
        for i, axis in enumerate('xyz'):
            col = [d[i] for d in data]
            sd = statistics.stdev(col)
            ok, note = _verdict(sd, nominal)
            print(f'    {axis}: mean={statistics.fmean(col):+.5f}  '
                  f'stddev={sd:.3e}  {note}')
            if not ok:
                failures.append(f'{label} {axis} 轴 stddev {sd:.3e}')

    # 重力必须还在：噪声块写错位置（比如把 linear_acceleration 写空）时，
    # stddev 可能仍然对，但均值会塌成 0 —— 那是另一种失效。
    gz_mean = statistics.fmean([d[2] for d in accel])
    if not 9.0 <= gz_mean <= 10.5:
        failures.append(f'加速度计 z 均值 {gz_mean:.3f}，重力不见了')
    print(f'  加速度计 z 均值 {gz_mean:.4f} m/s²（应 ≈ 9.8，重力）')

    # ---- GNSS ----------------------------------------------------------
    dump = _sample(f'/model/{args.model}/navsat', args.gnss_samples, 60.0)
    lat, lon = _scalars(dump, 'latitude_deg'), _scalars(dump, 'longitude_deg')
    alt = _scalars(dump, 'altitude')
    if len(lat) < 10:
        print(f'✗ /model/{args.model}/navsat 收不到足够样本', file=sys.stderr)
        return 1

    # 度 → 米。东向要乘 cos(纬度)，因为经线在高纬度收拢。
    north_m = statistics.stdev(lat) * _M_PER_DEG_LAT
    east_m = statistics.stdev(lon) * _M_PER_DEG_LAT * math.cos(math.radians(
        statistics.fmean(lat)))
    vert_m = statistics.stdev(alt)

    print(f'\nGNSS：{len(lat)} 条样本')
    print(f'  标称水平 {gnss_n["position_horizontal_stddev_m"]:.3g} m  '
          f'垂直 {gnss_n["position_vertical_stddev_m"]:.3g} m')
    for label, measured, nominal in (
            ('北向', north_m, gnss_n['position_horizontal_stddev_m']),
            # 东向的标称值要乘 cos(纬度)：SDF 里只有**一个**水平噪声参数，
            # 而它按「度」施加在经纬度上，于是东向的地面误差天然小一个 cos 因子。
            ('东向', east_m, gnss_n['position_horizontal_stddev_m']
             * math.cos(math.radians(statistics.fmean(lat)))),
            ('垂直', vert_m, gnss_n['position_vertical_stddev_m'])):
        ok, note = _verdict(measured, nominal)
        print(f'    {label}: {measured:.3f} m（标称 {nominal:.3f}）  {note}')
        if not ok:
            failures.append(f'GNSS {label} 散布 {measured:.3f} m')

    if failures:
        print(f'\n✗ {len(failures)} 项不达标：', file=sys.stderr)
        for f in failures:
            print(f'    · {f}', file=sys.stderr)
        print('\n  若 GNSS 水平散布小了约 11 万倍，说明 Gazebo 把那个量纲 bug\n'
              '  修成米了，去掉 gen_vehicle_model.py 里 _M_PER_DEG_LAT 那次换算。\n'
              '  若某一项恰好是 0，说明对应的 <noise> 块根本没生效。', file=sys.stderr)
        return 1

    print('\n✓ 全部达标')
    return 0


if __name__ == '__main__':
    sys.exit(main())
