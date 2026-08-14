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

"""CARLA 油门-加速度标定（P8-S5 的那把尺子，需要一个在跑的 CARLA + 空旷地图）.

产出 carla_bridge_params.yaml 的 throttle_per_mps2 / throttle_bias /
brake_per_mps2。测法：v∈[2,6] 工作带内取斜率 —— 避开起步档位瞬态与
高速风阻（2026-08-14 实测：throttle 0.4 顶不动风阻、0.6→0.51、
0.8→2.28 m/s²，全刹 −5.40）。换蓝图/换 CARLA 版本必须重跑。
异步模式下运行；同步模式的世界要先恢复异步。
"""

import math
import time
import carla

c = carla.Client("127.0.0.1", 2000); c.set_timeout(30.0)
w = c.get_world()
bp = w.get_blueprint_library().find("vehicle.citroen.c3")
sp = w.get_map().get_spawn_points()[3]
v = w.spawn_actor(bp, sp)
time.sleep(1.0)

def speed():
    vel = v.get_velocity()
    return math.hypot(vel.x, vel.y)

def reset():
    v.apply_control(carla.VehicleControl(brake=1.0))
    t0 = time.time()
    while speed() > 0.05 and time.time() - t0 < 8:
        time.sleep(0.05)
    v.set_transform(sp)
    time.sleep(0.5)

for th in (0.4, 0.6, 0.8):
    reset()
    v.apply_control(carla.VehicleControl(throttle=th))
    samples = []
    t0 = time.time()
    while time.time() - t0 < 14:
        samples.append((time.time(), speed()))
        if samples[-1][1] > 6.5:
            break
        time.sleep(0.1)
    band = [(t, s) for t, s in samples if 2.0 <= s <= 6.0]
    if len(band) >= 3:
        a = (band[-1][1] - band[0][1]) / (band[-1][0] - band[0][0])
        print(f"throttle={th:.1f}: 工作带斜率 a = {a:.3f} m/s²（{len(band)} 样本，末速 {samples[-1][1]:.1f}）")
    else:
        print(f"throttle={th:.1f}: 没进入工作带（末速 {samples[-1][1]:.2f}）")
v.destroy()
