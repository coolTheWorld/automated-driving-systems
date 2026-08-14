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
Shared fixtures for the L3-G closed-loop launch tests.

两个 L3-G 用例（无障碍物的 test_closed_loop.py、带障碍物的
test_closed_loop_obstacle.py）共用起点/终点常量与三份参数搬运逻辑。

⚠️ **抽出来是因为它有第三份的风险，不是为了少打字。**
   这三个函数与 stack.launch.py 里的同名函数已经是**两份**了
   （本包不 depend ads_bringup，反过来依赖会成环，见 _control_params 的说明）。
   再让两个测试各抄一遍就是四份，而参数搬运漂移的症状是
   「测试跑的参数和实际部署的不一样」—— 测试全绿，上车不对。
"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


# 自车起点：与 worlds/campus_loop.sdf 的 spawn 一致（南侧道路东行车道）。
# 用同一个起点是有意的 —— 这样本测试和 CP-P2-B 跑的是同一条路的同一段，
# 两者的数出现分歧时能直接对比。
START_X_M = 30.0
START_Y_M = -51.75
START_YAW_RAD = 0.0

# 目标点：东侧道路上，绕过东南角。约 95 m，包含一个 R≈13.75 的环线弯角。
# **刻意不选一条纯直路** —— 直路上横向误差恒为 0，投影、曲率、限速全都测不到。
GOAL_X_M = 91.75
GOAL_Y_M = -20.0

# 假车相对墙钟的倍率。3 倍下 ~20 s 仿真 ≈ 7 s 墙钟。
REAL_TIME_FACTOR = 3.0


def control_params() -> dict:
    """
    Collect control_node parameters from the two YAML files.

    与 `stack.launch.py` 的 `control_node_params()` 同一套。
    这里**重新读一遍**而不是 import stack.launch.py：本包不依赖 ads_bringup，
    反过来依赖会形成环（ads_bringup 已经 exec_depend 了 ads_control）。
    代价是这段搬运逻辑有两份 —— 但它没有算法，只是键名映射，
    而且一旦漂移，control_node 的构造函数会指名报错（参数默认值全是 0）。

    :return: 传给 control_node 的参数字典
    """
    share = Path(get_package_share_directory('ads_control')) / 'config'
    vehicle = yaml.safe_load((share / 'vehicle_params.yaml').read_text(encoding='utf-8'))
    control = yaml.safe_load((share / 'control_params.yaml').read_text(encoding='utf-8'))
    lim = vehicle['limits']
    return {
        'geometry.wheelbase_m': vehicle['geometry']['wheelbase_m'],
        'limits.max_steer_angle_rad': lim['max_steer_angle_rad'],
        'limits.max_steer_rate_rad_s': lim['max_steer_rate_rad_s'],
        'limits.max_accel_mps2': lim['max_accel_mps2'],
        'limits.max_decel_mps2': lim['max_decel_mps2'],
        'lateral.gain': control['lateral']['gain'],
        'lateral.soft_speed_mps': control['lateral']['soft_speed_mps'],
        'lateral.search_window': control['lateral']['search_window'],
        'longitudinal.kp': control['longitudinal']['kp'],
        'longitudinal.ki': control['longitudinal']['ki'],
        'goal.stop_distance_m': control['goal']['stop_distance_m'],
        'safety.max_lateral_error_m': control['safety']['max_lateral_error_m'],
        'safety.odom_timeout_s': control['safety']['odom_timeout_s'],
        'safety.trajectory_timeout_s': control['safety']['trajectory_timeout_s'],
        'control_rate_hz': control['control_rate_hz'],
        'use_sim_time': True,
    }


def planning_params() -> dict:
    """
    Collect planning_node parameters from the two YAML files.

    ⚠️ **P3-S4 起 L3-G 的链路里多了 planning_node。**
    control_node 不再直接订阅 /route/path，而是吃 /planning/trajectory ——
    不把规划器拉进来的话，控制器永远收不到轨迹，闭环测试会以
    「车一直不动」的形式失败，而根因看起来像控制器坏了。

    与 `stack.launch.py` 的 `planning_node_params()` 同一套，理由见 `control_params`。

    :return: 传给 planning_node 的参数字典
    """
    share = Path(get_package_share_directory('ads_planning')) / 'config'
    vehicle = yaml.safe_load((share / 'vehicle_params.yaml').read_text(encoding='utf-8'))
    planning = yaml.safe_load((share / 'planning_params.yaml').read_text(encoding='utf-8'))
    geo = vehicle['geometry']
    lim = vehicle['limits']
    return {
        'lateral.max_offset_m': planning['lateral']['max_offset_m'],
        'lateral.offset_step_m': planning['lateral']['offset_step_m'],
        # 运动学准入上限（P8-S2d）——推导链与 stack.launch.py 同一条。
        'lateral.max_curvature_inv_m': (
            math.tan(lim['max_steer_angle_rad']) / geo['wheelbase_m']
            * planning['lateral']['steering_authority_fraction']),
        'longitudinal.min_horizon_m': planning['longitudinal']['min_horizon_m'],
        'longitudinal.max_horizon_m': planning['longitudinal']['max_horizon_m'],
        'longitudinal.horizon_step_m': planning['longitudinal']['horizon_step_m'],
        'trajectory.resample_step_m': planning['trajectory']['resample_step_m'],
        'safety.margin_m': planning['safety']['margin_m'],
        'safety.floor_m': planning['safety']['floor_m'],
        'safety.stop_margin_m': planning['safety']['stop_margin_m'],
        'cost.weight_offset': planning['cost']['weight_offset'],
        'cost.weight_curvature': planning['cost']['weight_curvature'],
        'cost.weight_clearance': planning['cost']['weight_clearance'],
        'cost.weight_consistency': planning['cost']['weight_consistency'],
        'vehicle.length_m': geo['length_m'],
        'vehicle.width_m': geo['width_m'],
        'vehicle.rear_overhang_m': geo['rear_overhang_m'],
        'speed.cruise_speed_mps': lim['cruise_speed_mps'],
        'speed.max_lateral_accel_mps2': planning['speed']['max_lateral_accel_mps2'],
        'speed.max_accel_mps2': lim['max_accel_mps2'],
        'speed.max_decel_mps2': lim['max_decel_mps2'],
        'behavior.corridor_half_m': planning['behavior']['corridor_half_m'],
        'behavior.stand_off_m': planning['behavior']['stand_off_m'],
        'behavior.yield_margin_m': planning['behavior']['yield_margin_m'],
        'behavior.time_margin_s': planning['behavior']['time_margin_s'],
        'behavior.sigma_inflation_cap_m': planning['behavior']['sigma_inflation_cap_m'],
        'behavior.release_cycles': planning['behavior']['release_cycles'],
        'behavior.prediction_timeout_s': planning['behavior']['prediction_timeout_s'],
        'use_sim_time': True,
    }


def vehicle_params() -> dict:
    share = Path(get_package_share_directory('ads_control')) / 'config'
    vehicle = yaml.safe_load((share / 'vehicle_params.yaml').read_text(encoding='utf-8'))
    return {
        'geometry.wheelbase_m': vehicle['geometry']['wheelbase_m'],
        'limits.max_steer_angle_rad': vehicle['limits']['max_steer_angle_rad'],
        'limits.max_speed_mps': vehicle['limits']['max_speed_mps'],
        'initial.x_m': START_X_M,
        'initial.y_m': START_Y_M,
        'initial.heading_rad': START_YAW_RAD,
        'real_time_factor': REAL_TIME_FACTOR,
    }
