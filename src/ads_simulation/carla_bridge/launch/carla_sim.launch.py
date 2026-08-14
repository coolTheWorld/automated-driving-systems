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


def generate_launch_description():
    """组装 CARLA 侧最小节点集."""
    share = Path(get_package_share_directory('carla_bridge'))
    params_file = str(share / 'config' / 'carla_bridge_params.yaml')

    # campus.xodr 从 ads_map 的安装目录拿 —— 与 map_node 用同一份（单一来源）。
    xodr_path = str(
        Path(get_package_share_directory('ads_map')) / 'maps' / 'campus.xodr')

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
            parameters=[params_file, {'map_xodr_path': xodr_path}],
            output='screen'),

        # 点云中间话题 → 规范话题：与 gazebo 侧**同一个 C++ 节点**，
        # 只有 input_topic 不同（bridge_topics.yaml 当年就为此留了 /carla 前缀）。
        Node(
            package='gazebo_bridge',
            executable='lidar_preprocessor_node',
            name='lidar_preprocessor',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/carla/lidar/points_raw',
                'output_topic': '/lidar/points',
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
