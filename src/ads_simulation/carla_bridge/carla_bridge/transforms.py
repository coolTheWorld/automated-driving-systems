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

"""CARLA 左手系 ↔ ROS ENU 的坐标换算（P8-S4 的纯数学核心）.

⚠️ **CARLA 是左手系**：x 前、y **右**、z 上（Unreal 惯例）；ROS/SPEC 是
右手 ENU：x 前、y **左**、z 上。两者差一个 y 翻转，连带着：

    位置:      y_ros = −y_carla
    朝向 yaw:  CARLA 的 yaw 顺时针为正（度），ROS 逆时针为正（弧度）
               ⟹ yaw_ros = −radians(yaw_carla)
    角速度:    z 分量同样反号
    速度:      v_y 反号

这是所有 carla bridge 的**头号经典 bug**：漏翻 y 的症状是地图东西镜像、
车"沿路开"但左转变右转 —— 单看任何一个话题都"数值合理"，只有对照地图
才露馅。所以本文件是纯函数 + L1 用例钉死，不进 ROS、不 import carla ——
本机 dry-run 就能验（这正是"本地把最容易错的调完、上机只调环境"）。

同类先例：Gazebo 的 GNSS 水平噪声按度施加（CLAUDE.md 陷阱表）——
凡是跨引擎的单位/坐标约定，都值得一个不依赖引擎的对账用例。
"""

import math


def position_to_ros(x_carla_m: float, y_carla_m: float, z_carla_m: float) -> tuple:
    """CARLA 位置 → ROS ENU（y 翻转）.

    :param x_carla_m: CARLA x，米（前）
    :param y_carla_m: CARLA y，米（**右**为正）
    :param z_carla_m: CARLA z，米（上）
    :return: (x, y, z)，ROS ENU，米
    """
    return (x_carla_m, -y_carla_m, z_carla_m)


def yaw_to_ros(yaw_carla_deg: float) -> float:
    """CARLA yaw（度，顺时针正）→ ROS yaw（弧度，逆时针正，归一到 (−π, π]）.

    :param yaw_carla_deg: CARLA 朝向角，度
    :return: ROS 朝向角，弧度
    """
    yaw_rad = -math.radians(yaw_carla_deg)
    # 归一化：CARLA 会给出 [−180, 180] 之外的值（连续旋转不回卷）
    while yaw_rad > math.pi:
        yaw_rad -= 2.0 * math.pi
    while yaw_rad <= -math.pi:
        yaw_rad += 2.0 * math.pi
    return yaw_rad


def velocity_to_ros(vx_carla_mps: float, vy_carla_mps: float, vz_carla_mps: float) -> tuple:
    """CARLA 速度矢量 → ROS ENU（y 翻转）.

    :param vx_carla_mps: CARLA x 向速度，m/s
    :param vy_carla_mps: CARLA y 向速度，m/s
    :param vz_carla_mps: CARLA z 向速度，m/s
    :return: (vx, vy, vz)，ROS ENU，m/s
    """
    return (vx_carla_mps, -vy_carla_mps, vz_carla_mps)


def yaw_rate_to_ros(yaw_rate_carla_deg_s: float) -> float:
    """CARLA 偏航角速度（度/秒，顺时针正）→ ROS（弧度/秒，逆时针正）.

    :param yaw_rate_carla_deg_s: CARLA 偏航角速度，度/秒
    :return: ROS 偏航角速度，弧度/秒
    """
    return -math.radians(yaw_rate_carla_deg_s)


def position_to_carla(x_ros_m: float, y_ros_m: float, z_ros_m: float) -> tuple:
    """ROS ENU → CARLA（spawn 车辆 / 道具时用；与 position_to_ros 互逆）.

    :param x_ros_m: ROS x，米
    :param y_ros_m: ROS y，米（左为正）
    :param z_ros_m: ROS z，米
    :return: (x, y, z)，CARLA 左手系，米
    """
    return (x_ros_m, -y_ros_m, z_ros_m)


def yaw_to_carla(yaw_ros_rad: float) -> float:
    """ROS yaw（弧度）→ CARLA yaw（度）；与 yaw_to_ros 互逆.

    :param yaw_ros_rad: ROS 朝向角，弧度
    :return: CARLA 朝向角，度
    """
    return -math.degrees(yaw_ros_rad)


def yaw_to_quaternion(yaw_rad: float) -> tuple:
    """平面 yaw → 四元数 (x, y, z, w)。园区场景 roll/pitch ≈ 0（SPEC §2 ODD）.

    :param yaw_rad: 朝向角，弧度
    :return: 四元数 (x, y, z, w)
    """
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


def relative_pose_in_frame(
        x_map_m: float, y_map_m: float, yaw_map_rad: float,
        frame_x_m: float, frame_y_m: float, frame_yaw_rad: float) -> tuple:
    """map 系位姿 → 以 (frame) 为原点的局部系位姿（odom→base_link 用）.

    map→odom 是 spawn 位姿（与 gazebo_bridge 的 map_to_odom_static 同一约定，
    CLAUDE.md §5：发单位变换等于宣称「地图原点 = 出生点」——P0a 踩过）。

    :param x_map_m: 目标位姿 x，map 系，米
    :param y_map_m: 目标位姿 y，map 系，米
    :param yaw_map_rad: 目标朝向，map 系，弧度
    :param frame_x_m: 局部系原点 x，map 系，米
    :param frame_y_m: 局部系原点 y，map 系，米
    :param frame_yaw_rad: 局部系朝向，map 系，弧度
    :return: (x, y, yaw)，局部系
    """
    dx = x_map_m - frame_x_m
    dy = y_map_m - frame_y_m
    cos_f = math.cos(-frame_yaw_rad)
    sin_f = math.sin(-frame_yaw_rad)
    local_x = dx * cos_f - dy * sin_f
    local_y = dx * sin_f + dy * cos_f
    local_yaw = yaw_map_rad - frame_yaw_rad
    while local_yaw > math.pi:
        local_yaw -= 2.0 * math.pi
    while local_yaw <= -math.pi:
        local_yaw += 2.0 * math.pi
    return (local_x, local_y, local_yaw)
