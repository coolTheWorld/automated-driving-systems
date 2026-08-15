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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from carla_bridge.npc_kinematics import CARLA_NPC_VEHICLE_SIZE_M


# 与 gazebo_sim.launch 的 _CLASSIFICATION **同一张表**（obstacle_truth 认 int）。
# 抄表而不是 import：launch 文件不是可导入模块，两处都从 ads_msgs 的常量语义来。
_CLASSIFICATION = {
    'unknown': 0,
    'pedestrian': 1,
    'bicycle': 2,
    'vehicle': 3,
}


def _dynamic_specs(context):
    """dynamic:=<场景> 的目标清单 —— gazebo_sim._dynamic_actor_specs 的移植.

    解析逻辑必须逐字段一致（npc_controller 与 obstacle_truth 原样复用，
    参数语义在那两个节点里定死）。CARLA 侧不 spawn gz 模型 —— 道具由
    sidecar 按同一份 yaml spawn 并做 pose_gt/cmd_vel 两头（S4b）。
    """
    scenario = LaunchConfiguration('dynamic').perform(context)
    if scenario in ('', 'none'):
        return []
    share = Path(get_package_share_directory('gazebo_bridge'))
    config = yaml.safe_load(
        (share / 'config' / 'dynamic_actors.yaml').read_text(encoding='utf-8'))
    if scenario not in config['scenarios']:
        raise RuntimeError(f'dynamic:={scenario} 未定义（可选：none、'
                           f'{"、".join(config["scenarios"])}）')
    specs = []
    for name in config['scenarios'][scenario]['actors']:
        actor = config['actors'][name]
        # carla_waypoints 副本优先（P9-S2 拍板）：真车化后路外段撞墙，
        # CARLA 侧吃收进路内的副本；Gazebo launch 不认这个键（基线不动）。
        waypoints = [
            (float(x), float(y))
            for x, y in actor.get('carla_waypoints', actor['waypoints'])]
        if actor['classification'] == 'vehicle':
            # CARLA 侧道具是 nissan.micra，真值按**它的**包围盒给、原点即中心
            # （P9 窗口 4 定案，推导见 npc_kinematics.CARLA_NPC_VEHICLE_SIZE_M）。
            # Gazebo launch 那套「vehicle_params 的 4.4 + 后轴偏移 1.35」在这里
            # 是错的刺激物：真值框比物理车前移 1.35 m、长 0.77 m。
            length, width, height = CARLA_NPC_VEHICLE_SIZE_M
            offset_x = 0.0
        else:
            length = float(actor['length_m'])
            width = float(actor['width_m'])
            height = float(actor['height_m'])
            offset_x = 0.0
        dwell = [float(v) for v in actor.get('dwell_s', [0.0] * len(waypoints))]
        dwell[0] += float(actor.get('depart_delay_s', 0.0))
        specs.append({
            'name': name,
            'waypoints': waypoints,
            'speed_mps': float(actor['speed_mps']),
            'loop': bool(actor.get('loop', True)),
            'dwell_s': dwell,
            'length_m': length, 'width_m': width, 'height_m': height,
            'offset_x_m': offset_x, 'offset_z_m': height / 2.0,
            'classification': _CLASSIFICATION[actor['classification']],
        })
    return specs


def _dynamic_actor_nodes(context, *args, **kwargs):
    """每个动态目标一个 npc_controller —— 与 gazebo 侧同一可执行同一参数.

    Gazebo 侧另有 spawn + parameter_bridge 两个动作，这里都不需要：
    道具与话题两头由 sidecar 包办（_spawn_npcs / _tick_npcs）。
    """
    actions = []
    for actor in _dynamic_specs(context):
        npc_params = {
            'model_name': actor['name'],
            'waypoints_x_m': [w[0] for w in actor['waypoints']],
            'waypoints_y_m': [w[1] for w in actor['waypoints']],
            'speed_mps': actor['speed_mps'],
            'loop': actor['loop'],
            'use_sim_time': True,
        }
        if any(v > 0.0 for v in actor['dwell_s']):
            npc_params['dwell_s'] = actor['dwell_s']
        actions.append(Node(
            package='gazebo_bridge', executable='npc_controller',
            name=f'npc_controller_{actor["name"]}',
            parameters=[npc_params], output='screen'))
    return actions


def _obstacle_truth_nodes(context, *args, **kwargs):
    """S04 静态 + 行为动态的真值发布器 —— 与 gazebo_sim.launch 同一换算同一节点.

    obstacle_truth 是仿真器无关的（参数进、话题出），照抄扁平化逻辑：
    车道坐标 → 世界坐标用 yaml 自带 lane 段（单一来源）。
    ⚠️ obstacles:=none 时**不能早退**（gazebo 侧同款教训）：dynamic 非 none
    时真值发布器还负责动态目标，早退 = 那一跑没有任何真值、判据报空。
    """
    scenario = LaunchConfiguration('obstacles').perform(context)
    actors = _dynamic_specs(context)
    if scenario in ('', 'none') and not actors:
        return []
    # perception:=true 时真值不再兼发 /perception/obstacles（交给感知栈）。
    publish_as_perception = (
        LaunchConfiguration('perception').perform(context).lower() not in ('true', '1'))
    truth_params = {'frame_id': 'map', 'use_sim_time': True,
                    'publish_as_perception': publish_as_perception}
    if scenario not in ('', 'none'):
        config_path = (Path(get_package_share_directory('gazebo_bridge')) / 'config'
                       / 'obstacles.yaml')
        config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        if scenario not in config['scenarios']:
            raise RuntimeError(f'obstacles:={scenario} 未定义（可选：none、'
                               f'{"、".join(config["scenarios"])}）')
        lane = config['lane']
        xs, ys, yaws, ls, ws, hs = [], [], [], [], [], []
        for obstacle in config['scenarios'][scenario]['obstacles']:
            xs.append(float(obstacle['along_x_m']))
            ys.append(float(lane['center_y_m']) + float(obstacle['lateral_offset_m']))
            yaws.append(float(lane['heading_rad']))
            ls.append(float(obstacle['length_m']))
            ws.append(float(obstacle['width_m']))
            hs.append(float(obstacle['height_m']))
        truth_params.update({
            'obstacles.center_x_m': xs, 'obstacles.center_y_m': ys,
            'obstacles.yaw_rad': yaws, 'obstacles.length_m': ls,
            'obstacles.width_m': ws, 'obstacles.height_m': hs})
    dynamic_arrays = {
        'dynamic.names': [a['name'] for a in actors],
        'dynamic.length_m': [a['length_m'] for a in actors],
        'dynamic.width_m': [a['width_m'] for a in actors],
        'dynamic.height_m': [a['height_m'] for a in actors],
        'dynamic.offset_x_m': [a['offset_x_m'] for a in actors],
        'dynamic.offset_z_m': [a['offset_z_m'] for a in actors],
        'dynamic.classification': [a['classification'] for a in actors],
    }
    for key, value in dynamic_arrays.items():
        if value:  # 空数组不传（launch 推断不出元素类型，gazebo 侧实测）
            truth_params[key] = value
    return [Node(package='gazebo_bridge', executable='obstacle_truth',
                 name='obstacle_truth', parameters=[truth_params], output='screen')]


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
        # +0.5 头顶余量（P9-S1 实测）：c3 传感器旁结构的自反射点在 z 1.25–1.75，
        # 只裁到车高 1.5 时 z>1.5 的半个 blob 漏网 —— 占全云 27.5%，是常驻的
        # 车内幻影源。Gazebo 模型 1.5 以上没有任何东西，所以从没暴露。
        'ego_box.z_max': geo['height_m'] + 0.5,
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
        DeclareLaunchArgument('obstacles', default_value='',
                              description='S04 静态障碍场景（avoid/block，空=无）'),
        DeclareLaunchArgument('dynamic', default_value='',
                              description='行为动态场景（follow/crossing/junction 等，空=无）'),
        DeclareLaunchArgument('perception', default_value='false',
                              description='true 时真值不兼发 /perception/obstacles（与 gazebo 同语义）'),

        Node(
            package='carla_bridge',
            executable='carla_sidecar_node',
            name='carla_sidecar',
            parameters=[params_file, {
                'map_xodr_path': xodr_path,
                'obstacles_yaml': str(
                    Path(get_package_share_directory('gazebo_bridge')) / 'config' /
                    'obstacles.yaml'),
                'obstacles_scenario': LaunchConfiguration('obstacles'),
                # NPC 道具（S4b 机件，P8-S6 补上这根线 —— 实测 dynamic:= 一直
                # 没接到 sidecar，三个行为场景全是「场景没激励」）
                'dynamic_actors_yaml': str(
                    Path(get_package_share_directory('gazebo_bridge')) / 'config' /
                    'dynamic_actors.yaml'),
                'scenario': LaunchConfiguration('dynamic'),
                'vehicle_params_yaml': str(
                    Path(get_package_share_directory('ads_planning')) / 'config' /
                    'vehicle_params.yaml'),
            }],
            output='screen'),

        OpaqueFunction(function=_dynamic_actor_nodes),

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

        OpaqueFunction(function=_obstacle_truth_nodes),

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
