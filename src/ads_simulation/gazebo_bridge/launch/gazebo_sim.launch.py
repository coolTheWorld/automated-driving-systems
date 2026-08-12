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

import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# 自车在世界文件里的模型名。世界文件的 <include><name> 用的就是它。
EGO_MODEL_NAME = 'ego_vehicle'


# ads_msgs/Obstacle 的 CLASSIFICATION_* 常量。
# ⚠️ 写在这里而不是让 YAML 直接填数字：数字对不上时症状是「行人被标成车辆」，
#    而 P6 预测会按 classification 选运动模型（车走车道跟随、人走恒速椭圆），
#    选错的后果是预测轨迹荒谬 —— 而那时人会去查预测模块。
_CLASSIFICATION = {
    'unknown': 0,
    'pedestrian': 1,
    'bicycle': 2,
    'vehicle': 3,
    'static': 4,
}


def _resolve_world_file(world_name: str) -> Path:
    """
    Locate a world file the same way Gazebo does.

    照抄 Gazebo 的解析规则（沿 GZ_SIM_RESOURCE_PATH 逐个目录找），
    而不是假设它一定在 /workspace/worlds —— 那个路径只在本容器成立。

    :param world_name: 世界文件名，例如 campus_loop.sdf
    :return: 世界文件的绝对路径
    """
    candidate = Path(world_name)
    if candidate.is_absolute() and candidate.is_file():
        return candidate
    for entry in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(os.pathsep):
        if not entry:
            continue
        found = Path(entry) / world_name
        if found.is_file():
            return found
    raise RuntimeError(
        f'在 GZ_SIM_RESOURCE_PATH 里找不到世界文件 "{world_name}"。\n'
        f'当前 GZ_SIM_RESOURCE_PATH = {os.environ.get("GZ_SIM_RESOURCE_PATH", "<未设置>")}')


def _ego_spawn_pose(world_file: Path) -> list[str]:
    """
    Read the ego vehicle's spawn pose out of a world file.

    SDF 的 <pose> 是六个数：x y z roll pitch yaw，单位分别是米和弧度。

    :param world_file: 世界文件路径
    :return: 六个字符串，顺序同上
    """
    root = ET.parse(world_file).getroot()
    for include in root.iter('include'):
        name = include.find('name')
        if name is None or name.text != EGO_MODEL_NAME:
            continue
        pose = include.find('pose')
        if pose is None or pose.text is None:
            # 没写 <pose> 在 SDF 里是合法的（默认原点），但对我们不是：
            # 它意味着 map→odom 该用什么值这件事被留在了一个默认值里。
            raise RuntimeError(f'{world_file} 里的 {EGO_MODEL_NAME} 没有 <pose>')
        values = pose.text.split()
        if len(values) != 6:
            raise RuntimeError(f'{world_file} 里的 <pose> 应当是 6 个数，实际是 "{pose.text}"')
        return values
    raise RuntimeError(f'{world_file} 里没有名为 {EGO_MODEL_NAME} 的模型')


def _map_to_odom_from_spawn_pose(context, *args, **kwargs):
    """
    Publish map -> odom as the ego vehicle's spawn pose.

    详细理由见调用处的注释。一句话：Gazebo 把 odom 原点放在自车出生点，
    所以 map→odom 就是自车出生点在世界坐标里的位姿。

    :param context: launch 运行时上下文
    :return: 要执行的 launch 动作列表
    """
    # ⚠️ P4-S4 起这段静态 TF 是**可关的**：localization:=true 时由
    #    localization_node 发动态的 map→odom。两者同时发的症状是 TF 树上
    #    同一段变换有两个发布者，tf2 会按时间戳挑一个 —— 位姿于是随机地
    #    在真值与估计之间跳，而**没有任何报错**。
    if LaunchConfiguration('localization').perform(context).lower() in ('true', '1'):
        return []

    world_name = LaunchConfiguration('world').perform(context)
    x, y, z, roll, pitch, yaw = _ego_spawn_pose(_resolve_world_file(world_name))
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=[
                '--x', x, '--y', y, '--z', z,
                '--roll', roll, '--pitch', pitch, '--yaw', yaw,
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ]


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


def raw_cloud_topic(bridge_topics: list) -> str:
    """
    从桥接表里取出「仿真器原始点云」那条中间话题的 ROS 名.

    为什么要在这里查表，而不是让 lidar_preprocessor 直接用它的默认值：
    话题名一旦在 bridge_topics.yaml 和 C++ 默认值里各写一份，就会漂移。
    而漂移的症状极其安静 —— 预处理节点订阅了一个没人发布的话题，
    它只在收到点云时才打日志，于是**一条日志都没有**，/lidar/points 静默为空，
    RViz 一片空白。和「仿真器崩了」的表象完全一致（见 CLAUDE.md 陷阱表）。

    所以让 bridge_topics.yaml 当唯一来源，这里读出来传进去。这与本文件
    从 vehicle_params.yaml 派生自车包围盒是同一个做法。

    按「类型 + 方向」而不是按名字查：按名字查等于又写死一次名字，白做。
    匹配不到唯一一条就直接抛异常 —— 桥接表被改乱时应当立刻炸，
    而不是退回某个默认值继续跑（那又是一次静默失败）。
    """
    matches = [
        entry['ros_topic_name']
        for entry in bridge_topics
        if entry.get('ros_type_name') == 'sensor_msgs/msg/PointCloud2'
        and entry.get('direction') == 'GZ_TO_ROS'
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f'bridge_topics.yaml 里 GZ_TO_ROS 方向的 PointCloud2 条目应当恰好有 1 条，'
            f'实际找到 {len(matches)} 条：{matches}')
    return matches[0]


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


def _dynamic_actor_specs(context) -> list[dict]:
    """
    Resolve which dynamic actors are on stage and everything launch needs about them.

    ⚠️ **动态目标不进 worlds/campus_loop.sdf**，与静态障碍物同一条规矩：
       那个世界是 CP-P2-B / CP-P3-B / CP-P4-B **三个**检查点的回归基线。
       往里加一辆会动的车，点云、NDT 的 inlier、规划的碰撞检查全都会变。

    :param context: launch 上下文
    :return: 每个出场目标的一组字典；dynamic:=none 时为空列表
    """
    scenario = LaunchConfiguration('dynamic').perform(context)
    if scenario == 'none':
        return []

    config_path = (Path(get_package_share_directory('gazebo_bridge')) / 'config'
                   / 'dynamic_actors.yaml')
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if scenario not in config['scenarios']:
        raise RuntimeError(
            f'dynamic:={scenario} 在 {config_path} 里没有定义。'
            f'可选：none、{"、".join(config["scenarios"])}')

    vehicle_path = (Path(get_package_share_directory('gazebo_bridge')) / 'config'
                    / 'vehicle_params.yaml')
    vehicle_geo = yaml.safe_load(vehicle_path.read_text(encoding='utf-8'))['geometry']

    specs = []
    for name in config['scenarios'][scenario]['actors']:
        actor = config['actors'][name]
        waypoints = [(float(x), float(y)) for x, y in actor['waypoints']]

        if name == 'npc_car':
            # ⚠️ 尺寸从 vehicle_params 读，与 gen_dynamic_actors.py **同一个源头**。
            length = float(vehicle_geo['length_m'])
            width = float(vehicle_geo['width_m'])
            height = float(vehicle_geo['height_m'])
            # 模型原点在**后轴中心地面**（与自车同一个约定），
            # 包围盒中心在它前方 length/2 − rear_overhang。
            # 这与 gen_vehicle_model.derive() 里的 chassis_center_x 是同一个式子 ——
            # 那边建模型、这边发真值，两处必须一致，否则真值会恒定偏一个常量。
            offset_x = length / 2.0 - float(vehicle_geo['rear_overhang_m'])
        else:
            length = float(actor['length_m'])
            width = float(actor['width_m'])
            height = float(actor['height_m'])
            # 行人的模型原点在**底面中心**，所以纵向不偏。
            offset_x = 0.0

        # 车头朝向第二个航点。⚠️ 必须给对：cmd_vel 是**车体系**，
        #    spawn 时朝向反了的话第一拍会原地掉头（控制器会纠正，但点云里
        #    开头几帧的朝向是错的，而那几帧正好在判据窗口里）。
        heading_rad = math.atan2(waypoints[1][1] - waypoints[0][1],
                                 waypoints[1][0] - waypoints[0][0])

        specs.append({
            'name': name,
            'waypoints': waypoints,
            'speed_mps': float(actor['speed_mps']),
            'length_m': length,
            'width_m': width,
            'height_m': height,
            'offset_x_m': offset_x,
            'offset_z_m': height / 2.0,
            'classification': _CLASSIFICATION[actor['classification']],
            'spawn': (waypoints[0][0], waypoints[0][1], heading_rad),
        })
    return specs


def _obstacle_actions(context, *args, **kwargs):
    """
    Spawn the obstacle model and start the ground-truth publisher for a scenario.

    ⚠️ **障碍物不进 worlds/campus_loop.sdf**，而是运行时注入。
       那个世界是 CP-P2-B 的验收基线，往里加东西会让 P2 的实测数字全部作废，
       而 CP-P3-B 的第三个场景（无障碍物回归）恰恰要求世界与 P2 时**一模一样**。

    ⚠️ 真值发布器读的是 config/obstacles.yaml —— 与 Gazebo 模型**同一个源头**
       （模型由 scripts/gen_obstacles.py 从它生成）。于是「车看到的」与
       「车会撞上的」根本不存在第二份数据，也就不可能漂移。

    :param context: launch 上下文
    :return: 该场景对应的 launch 动作列表；obstacles:=none 时为空
    """
    scenario = LaunchConfiguration('obstacles').perform(context)
    actors = _dynamic_actor_specs(context)

    # ⚠️ **不能在 obstacles:=none 时直接返回。** P5-S1 起真值发布器还负责
    #    动态目标，而 `dynamic:=both obstacles:=none` 是一个合法且常用的组合
    #    （感知的验收场景不需要静态锥桶）。早退的话那一跑没有任何真值，
    #    而判据会因为「没有基准」而无法计算 —— 现场表现是脚本报空数据。
    if scenario == 'none' and not actors:
        return []

    center_x, center_y, yaw, length, width, height = [], [], [], [], [], []
    if scenario != 'none':
        config_path = (Path(get_package_share_directory('gazebo_bridge')) / 'config'
                       / 'obstacles.yaml')
        config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        if scenario not in config['scenarios']:
            raise RuntimeError(
                f'obstacles:={scenario} 在 {config_path} 里没有定义。'
                f'可选：none、{"、".join(config["scenarios"])}')

        lane = config['lane']
        spec = config['scenarios'][scenario]

        # 车道坐标 → 世界坐标。与 gen_obstacles.py 里的换算**必须一致**，
        # 否则真值和 Gazebo 模型会差一个偏移 —— 而那正是本节要避免的漂移。
        # 两处都只支持 heading = 0（南侧直道），生成器会显式拒绝其他值。
        for obstacle in spec['obstacles']:
            center_x.append(float(obstacle['along_x_m']))
            center_y.append(float(lane['center_y_m']) + float(obstacle['lateral_offset_m']))
            yaw.append(float(lane['heading_rad']))
            length.append(float(obstacle['length_m']))
            width.append(float(obstacle['width_m']))
            height.append(float(obstacle['height_m']))

    actions = []
    if scenario != 'none':
        actions.append(
            # 把障碍物模型 spawn 进已经在跑的 Gazebo。
            # 用 ros_gz_sim create 而不是生成一个"带障碍物的世界"，理由见上面的第一条 ⚠️。
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_obstacles_{scenario}',
                arguments=['-file', f'model://campus_obstacles_{scenario}',
                           '-name', f'campus_obstacles_{scenario}'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ))

    # ---- 动态目标：spawn、桥接、驱动（P5-S1）--------------------------------
    for actor in actors:
        sx, sy, syaw = actor['spawn']
        actions.append(Node(
            package='ros_gz_sim', executable='create',
            name=f'spawn_{actor["name"]}',
            arguments=['-file', f'model://{actor["name"]}', '-name', actor['name'],
                       '-x', str(sx), '-y', str(sy), '-z', '0', '-Y', str(syaw)],
            parameters=[{'use_sim_time': True}], output='screen',
        ))
        # 桥接。⚠️ **单独一个 bridge，不并进 bridge_topics.yaml** ——
        #    那张表是所有场景共用的，往里加两个只在 dynamic 非 none 时才存在的
        #    话题，等于让 CP-P2-B 的基线跑也多出两个空订阅。
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'bridge_{actor["name"]}',
            arguments=[
                # 真值：GZ → ROS（`[`）
                f'/model/{actor["name"]}/pose_gt@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # 速度指令：ROS → GZ（`]`）
                f'/model/{actor["name"]}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
            parameters=[{'use_sim_time': True}], output='screen',
        ))
        actions.append(Node(
            package='gazebo_bridge', executable='npc_controller',
            name=f'npc_controller_{actor["name"]}',
            parameters=[{
                'model_name': actor['name'],
                'waypoints_x_m': [w[0] for w in actor['waypoints']],
                'waypoints_y_m': [w[1] for w in actor['waypoints']],
                'speed_mps': actor['speed_mps'],
                # ⚠️ 用**墙钟**定时器（节点里是 create_wall_timer）——
                #    它是仿真道具，不是算法时序，不受 SPEC §5 那条约束。
                'use_sim_time': True,
            }],
            output='screen',
        ))

    # ---- 真值发布器：静态 + 动态合在一个节点里发 ----------------------------
    #
    # ⚠️ **空列表参数不能传给 launch**（2026-08-11 实测）：
    #    ROS 2 launch 对 `[]` 推断不出元素类型，报
    #    `Expected 'value' to be one of [float, int, str, bool, bytes], but got '()'`。
    #    而 `obstacles:=none dynamic:=both` 恰恰会让前六个数组是空的。
    #    所以空的就**不传**，让节点用它自己的默认值（也是空 vector）——
    #    节点那边的长度一致性校验对「全空」是通过的。
    # ⚠️ perception:=true 时真值**不再**兼发 /perception/obstacles ——
    #    那个话题交给 perception_node。两个发布者同时发不会报错，
    #    而 P4 实测过这类错误数值上也不一定看得出来（见 SPEC §3.3）。
    publish_as_perception = (
        LaunchConfiguration('perception').perform(context).lower() not in ('true', '1'))
    truth_params = {
        'frame_id': 'map',
        'use_sim_time': True,
        'publish_as_perception': publish_as_perception,
    }
    static_arrays = {
        'obstacles.center_x_m': center_x,
        'obstacles.center_y_m': center_y,
        'obstacles.yaw_rad': yaw,
        'obstacles.length_m': length,
        'obstacles.width_m': width,
        'obstacles.height_m': height,
    }
    dynamic_arrays = {
        'dynamic.names': [a['name'] for a in actors],
        'dynamic.length_m': [a['length_m'] for a in actors],
        'dynamic.width_m': [a['width_m'] for a in actors],
        'dynamic.height_m': [a['height_m'] for a in actors],
        'dynamic.offset_x_m': [a['offset_x_m'] for a in actors],
        'dynamic.offset_z_m': [a['offset_z_m'] for a in actors],
        'dynamic.classification': [a['classification'] for a in actors],
    }
    for arrays in (static_arrays, dynamic_arrays):
        for key, value in arrays.items():
            if value:
                truth_params[key] = value

    actions.append(Node(
        package='gazebo_bridge',
        executable='obstacle_truth',
        name='obstacle_truth',
        parameters=[truth_params],
        output='screen',
    ))
    return actions


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

    # 桥接表同样在这里读一次：点云中间话题名要从它取（见 raw_cloud_topic 的说明），
    # 而 parameter_bridge 那边拿的是文件路径，两者用的是同一个文件。
    bridge_topics = yaml.safe_load(Path(bridge_config).read_text(encoding='utf-8'))

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
        DeclareLaunchArgument(
            'localization', default_value='false',
            description=('true 时由 ads_localization 发动态 map→odom，'
                         '并**不再**发那条来自 spawn 位姿的静态 TF（P4-S4）')),
        DeclareLaunchArgument(
            'obstacles', default_value='none',
            description=('P3 验收场景的静态障碍物：none / avoid / block。'
                         '**默认 none** —— 那是 CP-P2-B 的回归基线，'
                         '世界必须与 P2 时一模一样')),
        DeclareLaunchArgument(
            'perception', default_value='false',
            description=('P5：true 时真值发布器**不再**兼发 /perception/obstacles，'
                         '那个话题交给 perception_node（两者互斥）。'
                         '真值始终走 /perception/obstacles_gt')),
        DeclareLaunchArgument(
            'dynamic', default_value='none',
            description=('P5 感知场景的**动态**目标：none / oncoming / cross / both。'
                         '**默认 none** —— 与 obstacles 同一条规矩：'
                         'CP-P2-B / CP-P3-B / CP-P4-B 三个检查点的回归基线'
                         '要求世界里没有会动的东西。两个开关**互相独立**，'
                         '可以自由组合（CP-P5-B 第 9 条跑 obstacles:=avoid dynamic:=none）')),

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
            parameters=[
                ego_box_params(vehicle_params),
                # 输入话题名从桥接表取，不依赖节点里的默认值 —— 两处漂移时
                # 的症状是「订阅了没人发的话题」，安静得没有任何日志。
                {'input_topic': raw_cloud_topic(bridge_topics)},
                use_sim_time,
            ],
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
        # 5. map → odom
        #
        # 真实系统里这一段由**定位模块**发布。P4 之前没有定位，用静态变换顶上。
        #
        # ⚠️ 它**不是单位变换**，这一点很容易搞错，而且搞错了不报任何错。
        #    Gazebo 的 AckermannSteering 把 odom 原点放在**自车 spawn 的位置**，
        #    所以 map→odom 的正确取值就是「自车 spawn 位姿在世界里的坐标」。
        #    发单位变换等于宣称「地图原点 = 自车出生点」。
        #
        #    P0a 一直用的是单位变换，而 campus_minimal 里自车 spawn 在
        #    (0, −1.75)，几乎就是原点，所以从没露过马脚 —— 反正那个世界里
        #    没有任何东西依赖世界坐标。到了 campus_loop（自车在 (30, −51.75)）
        #    症状立刻出来：RViz 里车画在园区正中央的草地上，而 Gazebo 里它
        #    好端端停在南边那条路上，两边差 60 m。**没有任何一层会报错。**
        #
        #    spawn 位姿从**世界文件里读**，不在这里写死 —— 写死就等于给
        #    「换个世界忘了改这里」留了一个必然会踩的坑，而症状是全局路径
        #    从一个错误的起点出发，看起来完全正常。
        #
        # ⚠️ P4 接上定位后必须删掉这个节点，否则会和定位模块抢着发同一段 TF，
        #    症状是车在 RViz 里疯狂跳动。
        # ---------------------------------------------------------------------
        OpaqueFunction(function=_map_to_odom_from_spawn_pose),

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

        # ---------------------------------------------------------------------
        # 7. P3 验收场景的静态障碍物（obstacles:=avoid / block）
        # ---------------------------------------------------------------------
        OpaqueFunction(function=_obstacle_actions),
    ])
