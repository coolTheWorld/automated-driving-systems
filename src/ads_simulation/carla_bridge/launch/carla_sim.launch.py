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

"""CARLA 侧仿真 launch（SPEC §4.1 环境 B，P8-S4）.

与 gazebo_sim.launch.py 镜像对应：

    gazebo:  gz sim + parameter_bridge + lidar_preprocessor + vehicle_cmd_bridge
             + robot_state_publisher + map_to_odom_static
    carla:   CARLA 服务端（--ros2，外部起）+ carla_sidecar
             + lidar_preprocessor（**同一个节点**，input_topic 换 /carla 前缀）
             + robot_state_publisher

CARLA 服务端不由本 launch 拉起 —— 它是重进程（20 GB、Vulkan），生命周期
由云机上的 tmux/systemd 管，与 ROS 栈解耦（断线重连不用重启仿真器）。

⚠️ gui:=false / rviz:=false 参数被接受但忽略：CARLA 无头与否由服务端
   `-RenderOffScreen` 决定。接受它们是为了 verify_ros_bridge.sh 能原样
   传参（它对两个 bridge 用同一条命令行）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """组装 CARLA 侧最小节点集."""
    share = Path(get_package_share_directory('carla_bridge'))
    params_file = str(share / 'config' / 'carla_bridge_params.yaml')

    # campus.xodr 从 ads_map 的安装目录拿 —— 与 map_node 用同一份（单一来源）。
    xodr_path = str(
        Path(get_package_share_directory('ads_map')) / 'maps' / 'campus.xodr')

    # 自车包围盒：与 gazebo_sim.launch 同一推导（车身在 base_link 系的占据），
    # CARLA 雷达同样打得到自家车身（S5 实测 1040 点/帧落在盒内 —— 引擎换了，
    # 自车反射这件事不换）。
    vehicle_yaml = (
        Path(get_package_share_directory('ads_planning')) / 'config' / 'vehicle_params.yaml')
    geo = yaml.safe_load(vehicle_yaml.read_text(encoding='utf-8'))['geometry']
    ego_box = {
        'ego_box.x_min': -geo['rear_overhang_m'],
        'ego_box.x_max': geo['wheelbase_m'] + geo['front_overhang_m'],
        'ego_box.y_min': -geo['width_m'] / 2.0,
        'ego_box.y_max': geo['width_m'] / 2.0,
        'ego_box.z_min': 0.0,
        'ego_box.z_max': geo['height_m'],
    }

    # URDF 与 gazebo 侧完全同一份（车辆单一来源，SPEC §4.1 表格那两行）。
    urdf_path = (
        Path(get_package_share_directory('ads_visualization')) / 'urdf' /
        'ego_vehicle.urdf')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='忽略（CARLA 无头由服务端定），为 verify 脚本兼容'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='同上'),

        Node(
            package='carla_bridge',
            executable='carla_sidecar_node',
            name='carla_sidecar',
            parameters=[params_file, {
                'map_xodr_path': xodr_path,
                'vehicle_params_yaml': str(
                    Path(get_package_share_directory('ads_planning')) / 'config' /
                    'vehicle_params.yaml'),
            }],
            output='screen'),

        # 点云中间话题 → 规范话题：与 gazebo 侧**同一个 C++ 节点**，
        # 只有 input_topic 不同（bridge_topics.yaml 当年就为此留了 /carla 前缀）。
        Node(
            package='gazebo_bridge',
            executable='lidar_preprocessor',
            name='lidar_preprocessor',
            parameters=[{
                'use_sim_time': True,
                # ⚠️ 实测（2026-08-14，云机）：原生话题名恒为
                #    /carla//lidar/point_cloud（双斜杠，issue #2）——而 rclcpp
                #    **拒绝**重复斜杠，原生流对 ROS 节点不可达。点云由 sidecar
                #    的 listen 回调中继到本合法中间名（y 已翻成 ENU）。
                'input_topic': '/carla/lidar/points_raw',
                'output_topic': '/lidar/points',
                **ego_box,
            }],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': urdf_path.read_text(encoding='utf-8'),
            }],
            output='screen'),
    ])
