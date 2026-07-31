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

# =============================================================================
#  gazebo_sim.launch.py —— 一条命令拉起「Gazebo + 桥接 + TF + RViz」
#
#      ros2 launch gazebo_bridge gazebo_sim.launch.py
#      ros2 launch gazebo_bridge gazebo_sim.launch.py gui:=false rviz:=false
#
#  这个 launch 的边界：它只负责**仿真数据源**这一侧。
#  感知/规划/控制等算法节点不在这里起 —— S5 的 ads_bringup/stack.launch.py
#  才是全栈入口，它会 include 本文件（或换成 carla_bridge 的同名 launch）。
#  这个分层就是 SPEC §4.1「切换仿真源 = 换一个 launch 参数」的落地方式。
# =============================================================================

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def ego_box_params(vehicle_params: dict) -> dict:
    """
    由车辆几何参数算出自车包围盒（base_link 系，单位 m）.

    base_link 在**后轴中心、地面高度**，所以车身占据：

        x：从 -rear_overhang 到 wheelbase + front_overhang
        y：±width/2
        z：从 0（地面）到 height

    宽度用 width_m（含轮胎的**保守**包络）而不是 SDF 里建模用的
    「轮距 - 轮宽」。自车滤除宁可多滤一点自己，也不要漏出自车反射点 ——
    漏出来的后果是感知把自己的车顶当成零距离障碍物，直接触发急刹。

    ⚠️ 这里只做算术，不引入任何新数字。所有输入都来自
    config/vehicle_params.yaml（SPEC §4.1 车辆参数单一来源）。
    """
    geo = vehicle_params['geometry']
    half_width = geo['width_m'] / 2.0
    return {
        'ego_box.x_min': -geo['rear_overhang_m'],
        'ego_box.x_max': geo['wheelbase_m'] + geo['front_overhang_m'],
        'ego_box.y_min': -half_width,
        'ego_box.y_max': half_width,
        'ego_box.z_min': 0.0,
        'ego_box.z_max': geo['height_m'],
    }


def vehicle_limit_params(vehicle_params: dict) -> dict:
    """
    vehicle_cmd_bridge 需要的车辆参数.

    同样只是搬运，不引入任何新数字 —— 全部来自 config/vehicle_params.yaml。
    参数名保持和 YAML 里的层级一致（geometry.* / limits.*），
    这样看到日志里的参数名就知道去 YAML 的哪一段改。
    """
    geo = vehicle_params['geometry']
    lim = vehicle_params['limits']
    return {
        'geometry.wheelbase_m': geo['wheelbase_m'],
        'limits.max_steer_angle_rad': lim['max_steer_angle_rad'],
        'limits.max_speed_mps': lim['max_speed_mps'],
        'limits.max_accel_mps2': lim['max_accel_mps2'],
        'limits.max_decel_mps2': lim['max_decel_mps2'],
        'limits.emergency_decel_mps2': lim['emergency_decel_mps2'],
    }


def generate_launch_description():
    bridge_share = get_package_share_directory('gazebo_bridge')
    viz_share = get_package_share_directory('ads_visualization')

    bridge_config = str(Path(bridge_share) / 'config' / 'bridge_topics.yaml')
    urdf_file = Path(viz_share) / 'urdf' / 'ego_vehicle.urdf'
    rviz_config = str(Path(viz_share) / 'rviz' / 'default.rviz')

    # robot_state_publisher 要的是 URDF 的**内容字符串**，不是路径。
    # 在这里一次读进来，比让节点自己去读省事，也能在文件缺失时立刻报错。
    robot_description = urdf_file.read_text(encoding='utf-8')

    # 车辆参数由 CMakeLists 装进本包 share（symlink-install 下就是仓库里那一份）。
    # 在 launch 里读、算好了传给节点，而不是让 C++ 节点自己解析 YAML ——
    # 这样节点不必依赖 yaml-cpp，也不必知道参数文件在哪。
    vehicle_params = yaml.safe_load(
        (Path(bridge_share) / 'config' / 'vehicle_params.yaml').read_text(encoding='utf-8'))

    # -------------------------------------------------------------------------
    # 所有节点都必须 use_sim_time=true（SPEC §3.3）。
    #
    # 为什么这条不能有例外：仿真时间和真实时间是两条独立的时间轴，RTF 不等于 1
    # 时两者流速不同。只要有一个节点用了真实时间，它盖的时间戳就和别人对不上，
    # TF 会报 extrapolation 错误，多传感器同步会静默错配。
    # 而且这类问题在 RTF≈1.0 时几乎看不出来 —— 等场景变复杂 RTF 掉下去才爆发。
    # -------------------------------------------------------------------------
    use_sim_time = {'use_sim_time': True}

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='campus_minimal.sdf',
            description='世界文件名。靠 GZ_SIM_RESOURCE_PATH 解析，不用写绝对路径'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否开 Gazebo 图形界面。CI 里必须 false'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否开 RViz2'),

        # ---------------------------------------------------------------------
        # 1. Gazebo
        #
        # -r 表示加载后立刻开始仿真。不加的话世界是暂停的，表现为
        #    所有话题都存在但永远没数据 —— 很容易误判成桥接坏了。
        # -s 表示只起服务端（headless）。CI 和批量跑场景测试时用。
        #
        # 写成两个互斥的 ExecuteProcess 而不是用替换拼参数：
        # 拼出来的空字符串会作为一个空参数传给 gz，gz 会把它当成文件名去找。
        # ---------------------------------------------------------------------
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            condition=IfCondition(gui),
            output='screen',
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r', world],
            condition=UnlessCondition(gui),
            output='screen',
        ),

        # ---------------------------------------------------------------------
        # 2. 话题桥接（3.4 的主体）
        #    翻译表在 config/bridge_topics.yaml，那个文件才是契约所在。
        # ---------------------------------------------------------------------
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_bridge',
            parameters=[{'config_file': bridge_config}, use_sim_time],
            output='screen',
        ),

        # ---------------------------------------------------------------------
        # 3. 点云预处理：坐标变换 + 自车滤除 + 无效点滤除
        #    见 src/lidar_preprocessor_node.cpp 顶部的说明。
        #
        #    自车滤除不是可选项：雷达装在 z=1.6，车顶在 z=1.5，只高 10 cm，
        #    实测 34% 的点打在自己车顶上。不滤的话感知会把自车车顶
        #    当成一个零距离障碍物。
        # ---------------------------------------------------------------------
        Node(
            package='gazebo_bridge',
            executable='lidar_preprocessor',
            name='lidar_preprocessor',
            parameters=[ego_box_params(vehicle_params), use_sim_time],
            output='screen',
        ),

        # ---------------------------------------------------------------------
        # 3b. 控制指令桥接：/vehicle_cmd → Gazebo 的 Twist
        #
        #     这是唯一一条 ROS → Gazebo 的链路。它把 SPEC §4.1 的车辆物理量
        #     （转角 rad + 加速度 m/s²）换算成 AckermannSteering 插件吃的
        #     Twist（速度 + 横摆角速度）。见 src/vehicle_cmd_bridge_node.cpp。
        #
        #     没有它的话，上游算法就得直接发 Gazebo 约定的 Twist，
        #     等于把仿真器的执行器细节泄漏进控制器。
        # ---------------------------------------------------------------------
        Node(
            package='gazebo_bridge',
            executable='vehicle_cmd_bridge',
            name='vehicle_cmd_bridge',
            parameters=[vehicle_limit_params(vehicle_params), use_sim_time],
            output='screen',
        ),

        # ---------------------------------------------------------------------
        # 4. robot_state_publisher
        #    读 URDF + 订阅 /joint_states → 发布 base_link 以下的整棵 TF 子树
        #    （含 base_link→lidar_link 这个外参，上面那个节点要用）。
        # ---------------------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}, use_sim_time],
            output='screen',
        ),

        # ---------------------------------------------------------------------
        # 5. map → odom（单位变换）
        #
        # 真实系统里这一段由**定位模块**发布，它表示的是「轮式里程计累积了
        # 多少漂移」。P4 之前没有定位，所以这里发单位变换 ——
        # 含义是「我们假装里程计不漂移」。这不是敷衍：TF 树必须连通，
        # 否则 RViz 以 map 为固定坐标系时什么都画不出来。
        #
        # ⚠️ P4 接上定位后必须删掉这个节点，否则会和定位模块抢着发同一段 TF，
        #    症状是车在 RViz 里疯狂跳动。
        # ---------------------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_identity',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[use_sim_time],
            output='screen',
        ),

        # ---------------------------------------------------------------------
        # 6. RViz2
        # ---------------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
