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

"""carla_bridge 纯数学层的 L1（无 ROS、无 carla —— 本地 dry-run 的主体）.

这里钉的全是「不随标定变的性质」：坐标翻转方向、互逆性、限幅边界、
看门狗语义。throttle/brake 的标定常数上机重标，这里只验方向与饱和。
"""

import math

from carla_bridge.control_mapping import ControlMapping
from carla_bridge.transforms import (
    position_to_carla, position_to_ros, relative_pose_in_frame, yaw_rate_to_ros,
    yaw_to_carla, yaw_to_quaternion, yaw_to_ros)
import pytest


def make_mapping() -> ControlMapping:
    """vehicle_params.yaml 的 limits 手抄值（L1 不读 YAML，与全仓惯例一致）."""
    return ControlMapping(
        max_steer_rad=0.6, max_accel_mps2=1.5, max_decel_mps2=3.0,
        throttle_per_mps2=0.4, brake_per_mps2=0.33)


class TestCoordinateConversion:
    """左手系 ↔ ENU。漏翻 y 的症状是地图东西镜像、左转变右转 —— 单话题看不出."""

    def test_y_axis_flips_and_x_z_do_not(self):
        assert position_to_ros(1.0, 2.0, 3.0) == (1.0, -2.0, 3.0)

    def test_position_round_trip_is_identity(self):
        x, y, z = position_to_carla(*position_to_ros(1.5, -2.5, 0.3))
        assert (x, y, z) == pytest.approx((1.5, -2.5, 0.3))

    def test_carla_clockwise_yaw_becomes_ros_counterclockwise(self):
        # CARLA yaw +90°（顺时针，即朝它的 +y = 右）→ ROS −π/2
        assert yaw_to_ros(90.0) == pytest.approx(-math.pi / 2)

    def test_yaw_round_trip_is_identity(self):
        assert yaw_to_carla(yaw_to_ros(37.0)) == pytest.approx(37.0)

    def test_yaw_normalizes_beyond_half_turn(self):
        # CARLA 连续旋转不回卷，会给出 270° 这类值 —— 必须归一到 (−π, π]
        assert yaw_to_ros(270.0) == pytest.approx(math.pi / 2)

    def test_yaw_rate_flips_sign(self):
        assert yaw_rate_to_ros(90.0) == pytest.approx(-math.pi / 2)

    def test_quaternion_matches_half_angle(self):
        qx, qy, qz, qw = yaw_to_quaternion(math.pi / 2)
        assert (qx, qy) == (0.0, 0.0)
        assert qz == pytest.approx(math.sin(math.pi / 4))
        assert qw == pytest.approx(math.cos(math.pi / 4))


class TestRelativePose:
    """odom→base_link 的推导：map 位姿相对 spawn 位姿（CLAUDE.md §5 的约定）."""

    def test_at_spawn_the_local_pose_is_zero(self):
        assert relative_pose_in_frame(30.0, -51.75, 1.0, 30.0, -51.75, 1.0) == \
            pytest.approx((0.0, 0.0, 0.0))

    def test_translation_rotates_into_the_frame(self):
        # spawn 朝北（yaw=π/2）：车在 map 里往北走 5 m = 局部系 x 前进 5 m
        x, y, yaw = relative_pose_in_frame(0.0, 5.0, math.pi / 2, 0.0, 0.0, math.pi / 2)
        assert (x, y, yaw) == pytest.approx((5.0, 0.0, 0.0))

    def test_yaw_difference_wraps(self):
        _, _, yaw = relative_pose_in_frame(0.0, 0.0, -3.0, 0.0, 0.0, 3.0)
        assert -math.pi < yaw <= math.pi
        assert yaw == pytest.approx(2.0 * math.pi - 6.0)


class TestControlMapping:
    """与 gazebo 的 vehicle_cmd_bridge 同一份契约的另一半."""

    def test_left_turn_maps_to_negative_carla_steer(self):
        # ROS 正转角 = 左转；CARLA 左手系正 steer = 右转 ⟹ 必须反号。
        # 漏翻的症状与坐标系那条同源：车"会开"，但每个弯都拐向另一边。
        out = make_mapping().to_carla(0.3, 0.0)
        assert out['steer'] == pytest.approx(-0.5)

    def test_steer_saturates_at_unit_magnitude(self):
        out = make_mapping().to_carla(1.2, 0.0)
        assert out['steer'] == pytest.approx(-1.0)

    def test_positive_accel_uses_throttle_only(self):
        out = make_mapping().to_carla(0.0, 1.0)
        assert out['throttle'] == pytest.approx(0.4)
        assert out['brake'] == 0.0

    def test_negative_accel_uses_brake_only(self):
        out = make_mapping().to_carla(0.0, -3.0)
        assert out['throttle'] == 0.0
        assert out['brake'] == pytest.approx(0.99)

    def test_accel_clamps_to_vehicle_limits(self):
        # +10 m/s² 会被 clamp 到 1.5 再映射 —— 不是拒收（上游自己限幅，这是最后防线）
        out = make_mapping().to_carla(0.0, 10.0)
        assert out['throttle'] == pytest.approx(0.6)

    def test_non_finite_command_is_invalid_not_clamped(self):
        # ⚠️ NaN 参与任何比较都是 False —— clamp 会把 NaN 原样放行
        #    （CLAUDE.md 陷阱表，gazebo 侧的指令限幅咬过一次）。
        #    必须显式 isfinite，而且 ±inf 也要拦，不只是 NaN。
        m = make_mapping()
        assert not m.is_valid(float('nan'), 0.0)
        assert not m.is_valid(0.0, float('inf'))
        assert not m.is_valid(float('-inf'), 0.0)
        with pytest.raises(ValueError):
            m.to_carla(float('nan'), 0.0)

    def test_watchdog_output_is_full_brake_and_centered(self):
        assert ControlMapping.full_brake() == \
            {'steer': 0.0, 'throttle': 0.0, 'brake': 1.0}

    def test_bias_shifts_throttle_and_idle_deadband_prevents_creep(self):
        # S5 标定形态：throttle = bias + k·a（发动机+风阻非线性的带内线性化）；
        # 死区防蠕动 —— bias 单独存在时静止小指令会让车爬走。
        m = ControlMapping(0.6, 1.5, 3.0, 0.113, 0.185,
                           throttle_bias=0.54, idle_below_mps2=0.05)
        assert m.to_carla(0.0, 1.0)['throttle'] == pytest.approx(0.54 + 0.113)
        assert m.to_carla(0.0, 0.04)['throttle'] == 0.0   # 死区内怠速
        assert m.to_carla(0.0, -1.0)['brake'] == pytest.approx(0.185)

    def test_standstill_hold_latch(self):
        # S6 实测：静止时 0.185·|a| 的小开度锁不住车（红窗末蠕动 0.11 m/s、
        # 驻车 6 cm 摇晃）。闩锁：近停 + 不要求前进 ⟹ 恒定保持刹车。
        m = ControlMapping(0.6, 1.5, 3.0, 0.113, 0.185,
                           throttle_bias=0.54, idle_below_mps2=0.05)
        held = m.to_carla(0.0, -0.5, speed_mps=0.05)
        assert held['brake'] == pytest.approx(0.30) and held['throttle'] == 0.0
        # 怠速指令（非负但在死区内）同样闩住 —— 否则红灯下溜坡
        assert m.to_carla(0.0, 0.0, speed_mps=0.0)['brake'] == pytest.approx(0.30)
        # 控制器一要求加速立刻放开（绿灯起步不能被闩死）
        go = m.to_carla(0.0, 1.0, speed_mps=0.05)
        assert go['brake'] == 0.0 and go['throttle'] > 0.5
        # 车还在动（≥ 门限）不闩 —— 行车中制动必须按标定映射走
        assert m.to_carla(0.0, -1.0, speed_mps=2.0)['brake'] == pytest.approx(0.185)
        # 未知车速（NaN）退回纯映射 —— 利用「NaN 比较恒假」的安全方向
        assert m.to_carla(0.0, -1.0)['brake'] == pytest.approx(0.185)

    def test_rejects_non_positive_parameters(self):
        with pytest.raises(ValueError):
            ControlMapping(0.0, 1.5, 3.0, 0.4, 0.33)
