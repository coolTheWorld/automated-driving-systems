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

"""NPC 运动学复刻 + 红绿灯相位机的 L1（dry-run 半区）."""

import math
from pathlib import Path

from carla_bridge.npc_kinematics import scenario_actor_names, step_pose
from carla_bridge.traffic_light_cycle import phase_at
import pytest
import yaml


class TestStepPose:
    """gz VelocityControl 语义的复刻：体系速度、立刻生效、前向欧拉."""

    def test_forward_motion_follows_heading(self):
        # 朝北（yaw=π/2）体系 vx=2 走 0.5 s ⟹ 北移 1 m
        x, y, yaw = step_pose(0.0, 0.0, math.pi / 2, 2.0, 0.0, 0.0, 0.5)
        assert (x, y, yaw) == pytest.approx((0.0, 1.0, math.pi / 2))

    def test_body_lateral_velocity_moves_sideways(self):
        # cmd_vel 是**车体系**（npc_controller 文件头那条实测）：
        # 朝东时体系 vy=+1（左）⟹ 世界系 +y
        x, y, _ = step_pose(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0)
        assert (x, y) == pytest.approx((0.0, 1.0))

    def test_yaw_integrates_and_wraps(self):
        _, _, yaw = step_pose(0.0, 0.0, 3.0, 0.0, 0.0, 1.0, 0.5)
        assert -math.pi < yaw <= math.pi
        assert yaw == pytest.approx(3.5 - 2.0 * math.pi)

    def test_zero_dt_is_identity(self):
        assert step_pose(1.0, 2.0, 0.3, 5.0, 0.0, 1.0, 0.0) == \
            pytest.approx((1.0, 2.0, 0.3))

    def test_non_finite_input_raises_instead_of_propagating(self):
        # NaN 会静默污染整条位姿链（比较恒 False，clamp 放行 —— 陷阱表）
        with pytest.raises(ValueError):
            step_pose(float('nan'), 0.0, 0.0, 0.0, 0.0, 0.0, 0.1)
        with pytest.raises(ValueError):
            step_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1)


class TestScenarioActors:
    """与 Gazebo 同一份 dynamic_actors.yaml、同一张场景表（单一来源）."""

    @staticmethod
    def _cfg():
        # 测试读**真实**的 YAML：这条对账（场景表能被 carla 侧解析）正是
        # dry-run 想抓的 —— 表结构变了这里先红，而不是上云才发现。
        repo = Path(__file__).resolve().parents[4]
        return yaml.safe_load(
            (repo / 'config' / 'dynamic_actors.yaml').read_text(encoding='utf-8'))

    def test_junction_scenario_lists_the_three_cross_cars(self):
        names = scenario_actor_names(self._cfg(), 'junction')
        assert names == ['cross_car_a', 'cross_car_b', 'cross_car_c']

    def test_unknown_scenario_raises_loudly(self):
        # 拼错场景名必须是启动失败，不是"零 NPC 正常运行"
        with pytest.raises(KeyError):
            scenario_actor_names(self._cfg(), 'junctoin')


class TestTrafficLightCycle:
    """红/绿定周期。最小闭环：无黄灯决策、无相位剩余时间."""

    def test_red_first_then_green_then_wraps(self):
        assert phase_at(0.0, 10.0, 5.0) == 'RED'
        assert phase_at(9.99, 10.0, 5.0) == 'RED'
        assert phase_at(10.0, 10.0, 5.0) == 'GREEN'
        assert phase_at(14.99, 10.0, 5.0) == 'GREEN'
        assert phase_at(15.0, 10.0, 5.0) == 'RED'   # 回卷

    def test_negative_time_is_first_phase(self):
        # 灯还没"启动"时按首相位（红）—— 保守方向
        assert phase_at(-3.0, 10.0, 5.0) == 'RED'

    def test_green_first_variant(self):
        assert phase_at(0.0, 10.0, 5.0, red_first=False) == 'GREEN'
        assert phase_at(5.0, 10.0, 5.0, red_first=False) == 'RED'

    def test_rejects_non_positive_durations(self):
        with pytest.raises(ValueError):
            phase_at(0.0, 0.0, 5.0)
