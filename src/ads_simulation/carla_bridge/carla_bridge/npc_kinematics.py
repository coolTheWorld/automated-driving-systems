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

"""NPC 道具的运动学步进 —— gz VelocityControl 语义在 CARLA 侧的复刻.

## 为什么是「复刻语义」而不是「重写驱动」

驱动 NPC 的**控制器**是 gazebo_bridge 的 npc_controller_node —— 它只认
ROS 话题（/model/<name>/pose_gt 进、/model/<name>/cmd_vel 出），本来就
仿真器无关。CARLA 侧复用**同一个节点**（行为逐字节相同 ⟹ 场景判据的
时间窗与 Gazebo 对得上 —— 用户拍板「sidecar 复刻航点驱动」的实质），
sidecar 只补两样：

    pose_gt  ← CARLA actor 位姿（sidecar 发）
    cmd_vel  → 本模块积分成位姿 → set_transform（sidecar 收）

## 语义对齐点（与 gz-sim-velocity-control-system 逐条对照）

  · cmd_vel 是**车体系**（npc_controller 文件头 2026-08-11 实测确认那条）；
  · 指令速度**立刻生效**（运动学，无力学延迟）—— 道具是感知的目标不是
    被测对象；
  · 无指令 = 保持不动（gz 插件收不到话题时不动，看门狗语义交给上游）。

⚠️ CARLA actor 要 set_simulate_physics(False)：物理开着的话 set_transform
   与物理引擎抢位姿，症状是道具抖动。与「关重力必须同时删 collision」
   那条陷阱不同源但同一个教训 —— 道具的位姿要么全归物理、要么全归脚本，
   一人一半必出事。CARLA 的雷达是射线打 mesh，不吃物理状态，感知不受影响。
"""

import math

# ---- CARLA 侧 NPC 车的蓝图与真值尺寸（launch 与 sidecar 共用一处）-------------
# Gazebo 的 NPC 车模型是从 vehicle_params 生成的 4.4×1.8×1.5 盒子、原点在后轴，
# 所以 Gazebo launch 给真值发布器的是 length=4.4、offset_x=+1.35（后轴→中心）。
# CARLA 侧道具要**尽量贴这份几何**（P9-S3 收口，2026-08-16 拍板换蓝图）：
#   感知的车辆形状先验（tracker.vehicle_prior_length_m = 4.4，ODD 车长）会把观测框
#   的长度垫到 4.4 —— 道具比 4.4 短多少，框就往两头各多出一半：nissan.micra 3.633
#   时近边恒偏近 (4.4−3.4)/2 ≈ 0.5 m，CP-P5-B 近边 p95 0.60/0.60 只差这一项红。
#   33 个四轮蓝图里离 4.4×1.8×1.5 最近的是 seat.leon（bbox_survey 2026-08-16）：
#     bounding_box.extent = (2.097, 0.908, 0.737) ⟹ 4.193 × 1.816 × 1.474（Δ 0.25）
#     bounding_box.location x = 0.000 ⟹ actor 原点就在包围盒 xy 中心，offset_x = 0
#   （micra 是 P8-S4b 随手挑的，当时只考虑「道具不需要对齐动力学」，没考虑几何。）
#   ⚠️ 这一换也遮住了先验的一个真实偏置：现实里 3.6 m 的车会被垫到 4.4，近边偏近
#      ~0.4 m —— 方向保守（障碍物看起来更近），记在 perception.md §11。
# 之前 CARLA launch 照抄 Gazebo 的 4.4/+1.35：真值框中心比物理车中心**前移 1.35 m**，
# 教训同「判据只量被测对象不量刺激物」那条：真值也是刺激物的一部分。
# sidecar spawn 后会拿实际 bounding_box 与这里对账，差 >2 cm 报错（蓝图换了
# 或 CARLA 升级改了模型时，真值不能悄悄跟着错）。
CARLA_NPC_VEHICLE_BLUEPRINT = 'vehicle.seat.leon'
CARLA_NPC_VEHICLE_SIZE_M = (4.193, 1.816, 1.474)   # (长, 宽, 高)


def step_pose(
        x_m: float, y_m: float, yaw_rad: float,
        body_vx_mps: float, body_vy_mps: float, yaw_rate_rad_s: float,
        dt_s: float) -> tuple:
    """车体系速度指令 × dt → 新位姿（ENU，前向欧拉）.

    gz 的 VelocityControl 同样是每步按当前朝向施加体系速度 —— 前向欧拉
    在 50 Hz 下的弧误差 ~v·ω·dt²/2 ≈ 0.1 mm 量级，远小于判据分辨率。

    :param x_m: 当前 x，ENU，米
    :param y_m: 当前 y，ENU，米
    :param yaw_rad: 当前朝向，弧度
    :param body_vx_mps: 车体系纵向速度，m/s
    :param body_vy_mps: 车体系横向速度，m/s（行人可能有）
    :param yaw_rate_rad_s: 偏航角速度，rad/s
    :param dt_s: 步长，秒（非负）
    :return: (x, y, yaw) 新位姿
    :raise ValueError: 任一输入非有限或 dt 为负
    """
    values = (x_m, y_m, yaw_rad, body_vx_mps, body_vy_mps, yaw_rate_rad_s, dt_s)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('step_pose: 输入含非有限值 —— NaN 会静默污染整条位姿链')
    if dt_s < 0.0:
        raise ValueError(f'step_pose: dt 为负（{dt_s}）—— 时钟倒流，别当正常值积分')

    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    new_x = x_m + (body_vx_mps * cos_yaw - body_vy_mps * sin_yaw) * dt_s
    new_y = y_m + (body_vx_mps * sin_yaw + body_vy_mps * cos_yaw) * dt_s
    new_yaw = yaw_rad + yaw_rate_rad_s * dt_s
    while new_yaw > math.pi:
        new_yaw -= 2.0 * math.pi
    while new_yaw <= -math.pi:
        new_yaw += 2.0 * math.pi
    return (new_x, new_y, new_yaw)


def scenario_actor_names(actors_cfg: dict, scenario: str) -> list:
    """dynamic_actors.yaml 的场景表 → 该场景要 spawn 的 actor 名单.

    与 Gazebo 侧同一份 YAML、同一张 scenarios 表（单一来源）——
    「没有 none 场景」的约定照旧：未知场景名抛异常而不是空手而归，
    拼错场景名的症状必须是启动失败，不是一辆 NPC 都没有的"正常"运行。

    :param actors_cfg: dynamic_actors.yaml 解析后的 dict
    :param scenario: 场景名（follow / crossing / junction / ...）
    :return: actor 名字列表
    :raise KeyError: 场景不存在
    """
    scenarios = actors_cfg['scenarios']
    if scenario not in scenarios:
        raise KeyError(
            f'未知场景 {scenario!r}（可用：{sorted(scenarios)}）——'
            '拼错场景名必须是启动失败，不是零 NPC 的"正常"运行')
    return list(scenarios[scenario]['actors'])
