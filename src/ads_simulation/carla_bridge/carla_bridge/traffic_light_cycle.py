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

"""S06 虚拟红绿灯的相位机 —— 纯函数，L1 直测.

campus.xodr 没有 signal（S06 的激励源设计在 CARLA sidecar，SPEC §4.1），
CARLA 的 generate_opendrive_world 也不会长出灯 —— 灯态由 sidecar 的
状态机**虚拟**发布。最小闭环（决策四）：红/绿两相位定周期循环，
不做黄灯决策、不做相位剩余时间。

「虚拟灯」对判据的充分性：S06 量的是「红灯停在停止线前 0–2 m、绿灯
恢复」—— 刺激 + 行为 + 判据三件俱全，灯有没有 mesh 与判据无关
（与 P5 道具「刺激物」的定位同一条：判据量的是自车对刺激的反应）。
"""


def phase_at(t_s: float, red_s: float, green_s: float, red_first: bool = True) -> str:
    """周期相位：t 时刻是红还是绿.

    :param t_s: 仿真时刻，秒（相对灯启动；负值按 0 处理 —— 灯还没起就是首相位）
    :param red_s: 红灯时长，秒（正）
    :param green_s: 绿灯时长，秒（正）
    :param red_first: 首相位是否为红（默认红 —— 验收场景要先看到停）
    :return: 'RED' 或 'GREEN'
    :raise ValueError: 时长非正或非有限
    """
    import math
    if not (math.isfinite(red_s) and red_s > 0.0 and math.isfinite(green_s) and green_s > 0.0):
        raise ValueError(f'phase_at: 相位时长必须是有限正数（red={red_s}, green={green_s}）')
    cycle_s = red_s + green_s
    t = max(0.0, t_s) % cycle_s
    first_s = red_s if red_first else green_s
    in_first = t < first_s
    if red_first:
        return 'RED' if in_first else 'GREEN'
    return 'GREEN' if in_first else 'RED'
