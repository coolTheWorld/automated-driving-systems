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

"""/vehicle_cmd（转角 rad + 加速度 m/s²）→ CARLA VehicleControl 的纯映射.

与 gazebo_bridge 的 vehicle_cmd_bridge_node **语义对齐**（同一份契约的
另一半实现）：

  · 输入校验在先、喂狗在后 —— 持续 NaN 流在语义上等价于失联
    （CLAUDE.md 陷阱表「先喂狗后校验」，gazebo 侧咬过一次）；
  · 非有限值一律拒，且要判 ±inf 不只是 NaN（比较拦不住 NaN）；
  · 限幅取 vehicle_params 的值，越限 clamp 而不是拒 —— 上游控制器
    自己也限幅，这里是最后防线不是判据。

CARLA 侧特有的部分：**加速度指令没有直接对应物**（VehicleControl 只有
throttle/brake ∈ [0,1]），要经过一层标定映射。P0b 实测的教训
（τ=0.140 s、稳态 86.3%）说明这层映射的常数**必须上机重标**，所以做成
参数而不是常量；本地 dry-run 只钉「方向、边界、单位」这些不随标定变的性质。
"""

import math


class ControlMapping:
    """纯函数集合：无 ROS、无 carla，可 L1 直测."""

    def __init__(
            self, max_steer_rad: float, max_accel_mps2: float, max_decel_mps2: float,
            throttle_per_mps2: float, brake_per_mps2: float):
        """构造并校验参数.

        :param max_steer_rad: 最大转角（rad），来自 vehicle_params limits
        :param max_accel_mps2: 最大加速度（m/s²，正值）
        :param max_decel_mps2: 最大减速度（m/s²，正值）
        :param throttle_per_mps2: 每 1 m/s² 加速需求对应的 throttle 开度（上机标定）
        :param brake_per_mps2: 每 1 m/s² 减速需求对应的 brake 开度（上机标定）
        """
        for name, value in (('max_steer_rad', max_steer_rad),
                            ('max_accel_mps2', max_accel_mps2),
                            ('max_decel_mps2', max_decel_mps2),
                            ('throttle_per_mps2', throttle_per_mps2),
                            ('brake_per_mps2', brake_per_mps2)):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'ControlMapping: {name} 必须是有限正数，得到 {value}')
        self._max_steer_rad = max_steer_rad
        self._max_accel_mps2 = max_accel_mps2
        self._max_decel_mps2 = max_decel_mps2
        self._throttle_per_mps2 = throttle_per_mps2
        self._brake_per_mps2 = brake_per_mps2

    def is_valid(self, steer_rad: float, accel_mps2: float) -> bool:
        """指令是否有效（**校验在先，喂狗在后** —— 调用方按此顺序用）.

        :param steer_rad: 目标转角，rad
        :param accel_mps2: 目标加速度，m/s²
        :return: 两个量都有限才为 True
        """
        return math.isfinite(steer_rad) and math.isfinite(accel_mps2)

    def to_carla(self, steer_rad: float, accel_mps2: float) -> dict:
        """有效指令 → CARLA VehicleControl 字段.

        ⚠️ **CARLA 的 steer 归一化到 [−1, 1] 且左手系（正 = 右转）**；
        SPEC 的转角是弧度、右手系（正 = 左转）—— 又是那个 y 翻转的兄弟。

        :param steer_rad: 目标转角，rad（右手系，正 = 左转）
        :param accel_mps2: 目标加速度，m/s²（负 = 制动）
        :return: {'steer': float, 'throttle': float, 'brake': float}
        :raise ValueError: 指令非有限（调用方应先 is_valid，这里是双保险）
        """
        if not self.is_valid(steer_rad, accel_mps2):
            raise ValueError('to_carla: 指令含非有限值，先用 is_valid 过滤')

        clamped_steer = max(-self._max_steer_rad, min(self._max_steer_rad, steer_rad))
        # 归一化 + 左手系翻转：ROS 正转角（左转）→ CARLA 负 steer
        steer_carla = -clamped_steer / self._max_steer_rad

        clamped_accel = max(-self._max_decel_mps2, min(self._max_accel_mps2, accel_mps2))
        if clamped_accel >= 0.0:
            throttle = min(1.0, clamped_accel * self._throttle_per_mps2)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(1.0, -clamped_accel * self._brake_per_mps2)
        return {'steer': steer_carla, 'throttle': throttle, 'brake': brake}

    @staticmethod
    def full_brake() -> dict:
        """看门狗超时 / 无有效指令时的安全输出：全刹 + 回正.

        与 gazebo 侧看门狗语义一致（0.5 s 无有效指令 → 刹停保持）。

        :return: {'steer': 0.0, 'throttle': 0.0, 'brake': 1.0}
        """
        return {'steer': 0.0, 'throttle': 0.0, 'brake': 1.0}
