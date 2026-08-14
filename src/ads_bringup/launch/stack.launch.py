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
#  stack.launch.py —— 全栈唯一入口
#
#      ros2 launch ads_bringup stack.launch.py                    # 默认 sim:=gazebo
#      ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false
#      ros2 launch ads_bringup stack.launch.py --show-args        # 看有哪些参数
#
#  职责边界：**只做装配，不含任何算法逻辑，也不定义任何参数默认值**。
#  车辆参数在 config/vehicle_params.yaml，话题名在各 bridge 的配置里，
#  这个文件只负责「把谁和谁接起来」。
#
#  为什么在只有一个仿真源的时候就要有这一层
#  ----------------------------------------
#  现在 SIM_SOURCES 里只有 gazebo 一项，这层看着像多余的间接。但它存在的
#  意义不是"现在有几个数据源"，而是**把切换点固定在一个地方**：
#  P0b 接 CARLA 时只在 SIM_SOURCES 加一行，上游算法节点一行都不用改。
#  等到有三个数据源才来抽这一层，届时算法节点已经散落着"这里是 Gazebo"
#  的隐含假设了——那正是 SPEC §4.1 要防的「行为漂移」。
#
#  与 gazebo_sim.launch.py 的分工
#  ------------------------------
#  gazebo_sim.launch.py 只管**仿真数据源那一侧**（仿真器 + 桥接 + TF + RViz）。
#  本文件在它之上再挂算法节点。所以调试仿真链路时直接跑前者更快，
#  跑整个系统才用本文件。
# =============================================================================

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

# 仿真数据源注册表：sim 参数值 → (包名, launch 文件名)。
#
# 这张表就是 SPEC §4.1「切换仿真源 = 换一个 launch 参数」的落地点。
# 新增数据源 = 在这里加一行，前提是它对外发布的话题符合 §4.1 的规范话题名
# （/lidar/points、/imu、/gnss、/odom、/vehicle_cmd …）。
# 话题名不一致的话，这一层换得再干净，上游照样收不到数据。
SIM_SOURCES = {
    'gazebo': ('gazebo_bridge', 'gazebo_sim.launch.py'),
}

# SPEC 里已规划、但当前阶段还没实现的数据源。
#
# 为什么要单独列一张表而不是让它落进"未知值"分支：
# 「还没做」和「你拼错了」的排查方向完全不同。用户敲 sim:=carla 时应该
# 立刻知道"这个功能在 P0b"，而不是对着"未知的 sim 值"去检查拼写。
PLANNED_SOURCES = {
    'carla': 'P0b 阶段实现（云端 CARLA 验收环境）',
    'bag': 'P1 之后实现（离线 rosbag 回放）',
}


def _resolve_sim_source(context, *args, **kwargs):
    """
    按 sim 参数选出仿真数据源的 launch 并 include 进来.

    用 OpaqueFunction 而不是 IfCondition + EqualsSubstitution 的声明式写法，
    是为了能在**参数非法时直接报错**。声明式写法下所有条件都不满足时，
    launch 会安安静静地什么都不起：进程活着、话题名一个不少（都是别的节点
    声明的），就是永远没有数据。这种故障和"仿真器崩了"长得一模一样，
    本项目已经吃过一次亏（见 CLAUDE.md 陷阱表 "gz sim 崩溃被 launch 报成干净退出"）。

    :param context: launch 运行时上下文，用来 perform 出参数的实际取值
    :return: 要执行的 launch 动作列表
    """
    sim = LaunchConfiguration('sim').perform(context)

    if sim in PLANNED_SOURCES:
        raise RuntimeError(
            f'仿真数据源 "{sim}" 尚未实现 —— {PLANNED_SOURCES[sim]}。\n'
            f'当前可用：{", ".join(sorted(SIM_SOURCES))}')

    if sim not in SIM_SOURCES:
        raise RuntimeError(
            f'未知的 sim 值 "{sim}"。\n'
            f'当前可用：{", ".join(sorted(SIM_SOURCES))}；'
            f'已规划但未实现：{", ".join(sorted(PLANNED_SOURCES))}')

    package, launch_file = SIM_SOURCES[sim]
    launch_path = str(Path(get_package_share_directory(package)) / 'launch' / launch_file)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            # 参数原样透传。这里**不设默认值** —— 默认值只在下面的
            # DeclareLaunchArgument 里定义一次，避免两处各写一份然后慢慢漂移。
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'obstacles': LaunchConfiguration('obstacles'),
                'dynamic': LaunchConfiguration('dynamic'),
                'perception': LaunchConfiguration('perception'),
                'localization': LaunchConfiguration('localization'),
            }.items(),
        ),
    ]


def planning_node_params() -> dict:
    """
    planning_node 的全部参数，来自两个 YAML 且**不引入任何新数字**.

    分工与 control_node 一致，判据同样是「换一辆车它会不会变」：

      * vehicle_params.yaml —— 车**外廓**（长/宽/后悬，碰撞检查用）和
        **能力**（巡航速度、最大加减速）。换车会变。
      * planning_params.yaml —— 采样网格、安全间距、代价权重。换驾驶风格才变。

    ⚠️ **车宽绝不在 planning_params.yaml 里再抄一份。** 抄一份的症状是
       「改了车宽之后碰撞检查还用旧值」，而没有任何一层报错。

    :return: 传给 planning_node 的参数字典
    """
    share = Path(get_package_share_directory('ads_planning')) / 'config'
    vehicle = yaml.safe_load((share / 'vehicle_params.yaml').read_text(encoding='utf-8'))
    planning = yaml.safe_load((share / 'planning_params.yaml').read_text(encoding='utf-8'))

    geo = vehicle['geometry']
    lim = vehicle['limits']
    return {
        # 采样网格
        'lateral.max_offset_m': planning['lateral']['max_offset_m'],
        'lateral.offset_step_m': planning['lateral']['offset_step_m'],
        # 运动学准入上限（P8-S2d）：车辆能力 × 转向余量系数。
        # 轴距/最大转角来自 vehicle_params（单一来源），系数来自 planning_params。
        'lateral.max_curvature_inv_m': (
            math.tan(lim['max_steer_angle_rad']) / geo['wheelbase_m']
            * planning['lateral']['steering_authority_fraction']),
        'longitudinal.min_horizon_m': planning['longitudinal']['min_horizon_m'],
        'longitudinal.max_horizon_m': planning['longitudinal']['max_horizon_m'],
        'longitudinal.horizon_step_m': planning['longitudinal']['horizon_step_m'],
        'trajectory.resample_step_m': planning['trajectory']['resample_step_m'],
        # 安全（准入条件，不是代价项）
        'safety.margin_m': planning['safety']['margin_m'],
        'safety.stop_margin_m': planning['safety']['stop_margin_m'],
        # 代价权重
        'cost.weight_offset': planning['cost']['weight_offset'],
        'cost.weight_curvature': planning['cost']['weight_curvature'],
        'cost.weight_clearance': planning['cost']['weight_clearance'],
        'cost.weight_consistency': planning['cost']['weight_consistency'],
        # 车辆外廓与能力 —— 规划器不得重新定义
        'vehicle.length_m': geo['length_m'],
        'vehicle.width_m': geo['width_m'],
        'vehicle.rear_overhang_m': geo['rear_overhang_m'],
        'speed.cruise_speed_mps': lim['cruise_speed_mps'],
        'speed.max_lateral_accel_mps2': planning['speed']['max_lateral_accel_mps2'],
        'speed.max_accel_mps2': lim['max_accel_mps2'],
        # ⚠️ 用的是 max_decel（3.0，舒适约束）而**不是** emergency_decel（5.0）。
        'speed.max_decel_mps2': lim['max_decel_mps2'],
        # 行为决策（P7-S3）。front_offset 是推导量，planning_node 自己从车辆几何算。
        'behavior.corridor_half_m': planning['behavior']['corridor_half_m'],
        'behavior.stand_off_m': planning['behavior']['stand_off_m'],
        'behavior.yield_margin_m': planning['behavior']['yield_margin_m'],
        'behavior.time_margin_s': planning['behavior']['time_margin_s'],
        'behavior.sigma_inflation_cap_m': planning['behavior']['sigma_inflation_cap_m'],
        'behavior.release_cycles': planning['behavior']['release_cycles'],
        'behavior.prediction_timeout_s': planning['behavior']['prediction_timeout_s'],
    }


def control_node_params() -> dict:
    """
    control_node 的全部参数，来自两个 YAML 且**不引入任何新数字**.

    分工是刻意的，判据是「换一辆车它会不会变」：

      * vehicle_params.yaml —— **车能做什么**（轴距、转角、转向速率、加减速）。
        换车会变，所以它必须和 Gazebo/CARLA 用同一份，否则就是 SPEC §4.1
        的头号风险「行为漂移」：本地调好的参数一上 CARLA 就震荡，
        而你分不清是算法错了还是环境不同。
      * control_params.yaml —— **我们想怎么开**（增益、限速策略、安全阈值）。
        换车不变，换驾驶风格才变。

    参数名保持与 YAML 的层级一致，这样看到日志里的参数名就知道去哪一段改。

    :return: 传给 control_node 的参数字典
    """
    share = Path(get_package_share_directory('ads_control')) / 'config'
    vehicle = yaml.safe_load((share / 'vehicle_params.yaml').read_text(encoding='utf-8'))
    control = yaml.safe_load((share / 'control_params.yaml').read_text(encoding='utf-8'))

    geo = vehicle['geometry']
    lim = vehicle['limits']
    return {
        # 车辆能力 —— 控制器不得重新定义
        'geometry.wheelbase_m': geo['wheelbase_m'],
        'limits.max_steer_angle_rad': lim['max_steer_angle_rad'],
        'limits.max_steer_rate_rad_s': lim['max_steer_rate_rad_s'],
        'limits.max_accel_mps2': lim['max_accel_mps2'],
        # ⚠️ 注意用的是 max_decel（3.0，舒适约束）而**不是** emergency_decel（5.0，
        #    物理能力）。后者只允许安全模块下发；常规控制器自己不得越过 3.0。
        'limits.max_decel_mps2': lim['max_decel_mps2'],
        # 控制器调参
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
    }


def _load_gazebo_launch_module():
    """
    Import gazebo_sim.launch.py by path so we can reuse its world parsing.

    为什么复用而不是再写一份 XML 解析：自车 spawn 朝向是 localization_node 的
    冷启动航向先验。**在 launch 里写死那个数就是给「换个世界忘了改」留一个
    必然会踩的坑**（CLAUDE.md 在 map→odom 那条上已经记过一次）。

    ⚠️ 按路径加载模块时必须先塞进 sys.modules 再 exec —— 见
       src/ads_map/test/test_gen_map.py 里那段说明（dataclass + 字符串注解）。
       这里没有 dataclass，但保持同一个写法免得下次有人照抄踩坑。
    """
    import importlib.util
    import sys

    path = (Path(get_package_share_directory('gazebo_bridge')) / 'launch' /
            'gazebo_sim.launch.py')
    spec = importlib.util.spec_from_file_location('gazebo_sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _perception_nodes(context, *args, **kwargs):
    """
    Build the P5 perception node, or nothing when the switch is off.

    ⚠️ **它与 `obstacle_truth` 的 `/perception/obstacles` 互斥。**
       两者同时发同一个话题不会报错，而 P4 实测过这类错误
       **数值上也不一定看得出来**（多一个 map→odom 发布者时误差只从
       0.012 变成 0.043 m，全部判据仍绿）。所以由这个开关二选一：
       `perception:=true` 时把真值发布器的 `publish_as_perception` 关掉
       （见 gazebo_sim.launch.py），真值只走 `/perception/obstacles_gt`。

    :param context: launch 运行时上下文
    :return: 要执行的 launch 动作列表；perception:=false 时为空
    """
    if LaunchConfiguration('perception').perform(context).lower() not in ('true', '1'):
        return []

    params_yaml = (Path(get_package_share_directory('ads_perception')) / 'config'
                   / 'perception_params.yaml')
    config = yaml.safe_load(params_yaml.read_text(encoding='utf-8'))

    # 平铺成 ROS 参数名。**逐个列出而不是自动展开**，理由与 control_node 一样：
    # 自动展开时 YAML 里多一个键就会静默变成一个节点不认识的参数，
    # 而 ROS 2 对未声明的参数默认是**忽略**的 —— 于是"改了配置没生效"。
    flat = {}
    for section in ('ground', 'cluster', 'lshape', 'tracker'):
        for key, value in config[section].items():
            flat[f'{section}.{key}'] = value
    flat['max_cloud_age_s'] = config['max_cloud_age_s']
    flat['use_sim_time'] = True

    return [
        Node(
            package='ads_perception',
            executable='perception_node',
            name='perception_node',
            parameters=[flat],
            output='screen',
        ),
    ]


def _prediction_nodes(context, *args, **kwargs):
    """
    Build the P6 prediction node, or nothing when the switch is off.

    prediction_node 是**旁路**：订阅 /perception/obstacles、发布
    /prediction/trajectories 与 markers，不改动任何既有话题 ——
    所以它不需要透传给 gazebo_sim.launch.py（perception 需要透传是
    因为要关掉真值发布器的兼发，这里没有那样的互斥）。
    /perception/obstacles 在 perception:=false 时来自 obstacle_truth、
    true 时来自 perception_node —— 两种模式下预测都有输入，这正是
    CP-P6-B 双层评测协议（真值层 / 感知层）的开关组合。

    :param context: launch 运行时上下文
    :return: 要执行的 launch 动作列表；prediction:=false 时为空
    """
    if LaunchConfiguration('prediction').perform(context).lower() not in ('true', '1'):
        return []

    params_yaml = (Path(get_package_share_directory('ads_prediction')) / 'config'
                   / 'prediction_params.yaml')
    config = yaml.safe_load(params_yaml.read_text(encoding='utf-8'))

    # 平铺：prediction_params.yaml 本身就是平的（无分节），逐个搬 ——
    # 理由与 perception 一样：不自动展开，多出来的键要在这里显式亮相。
    flat = dict(config)
    flat['use_sim_time'] = True

    return [
        Node(
            package='ads_prediction',
            executable='prediction_node',
            name='prediction_node',
            parameters=[flat],
            output='screen',
        ),
    ]


def _localization_nodes(context, *args, **kwargs):
    """
    Build the P4 localization node, or nothing when the switch is off.

    :param context: launch 运行时上下文
    :return: 要执行的 launch 动作列表
    """
    if LaunchConfiguration('localization').perform(context).lower() not in ('true', '1'):
        return []

    map_yaml = Path(get_package_share_directory('ads_map')) / 'config' / 'campus_map.yaml'
    if not map_yaml.is_file():
        # 装到 share 之外的兜底：源码树里的那一份。
        map_yaml = Path(__file__).resolve().parents[3] / 'config' / 'campus_map.yaml'
    campus = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    geo = campus['geo_origin']

    vehicle_yaml = (Path(get_package_share_directory('ads_localization')) / 'config' /
                    'vehicle_params.yaml')
    vehicle = yaml.safe_load(vehicle_yaml.read_text(encoding='utf-8'))
    imu_noise = vehicle['sensors']['imu']['noise']
    gnss_cfg = vehicle['sensors']['gnss']
    gnss_noise = gnss_cfg['noise']

    # 冷启动的航向先验：从**世界文件**读自车 spawn 朝向，不写死。
    gazebo_launch = _load_gazebo_launch_module()
    world_name = LaunchConfiguration('world').perform(context)
    spawn = gazebo_launch._ego_spawn_pose(gazebo_launch._resolve_world_file(world_name))
    initial_yaw_rad = float(spawn[5])

    # NDT 的先验点云地图。与 models/campus_structures 同源（scripts/gen_map.py）。
    cloud_pcd = Path(__file__).resolve().parents[3] / 'maps' / 'campus_cloud.pcd'

    return [
        Node(
            package='ads_localization',
            executable='localization_node',
            name='localization_node',
            parameters=[{
                # ⚠️ 大地原点必须与世界文件的 <spherical_coordinates> 一致。
                #    不一致的症状是定位稳定地偏一个常量，而没有任何模块报错。
                'geo_origin.latitude_deg': float(geo['latitude_deg']),
                'geo_origin.longitude_deg': float(geo['longitude_deg']),
                'geo_origin.elevation_m': float(geo['elevation_m']),
                # ESKF 的过程噪声与传感器噪声同源 —— 两处各填一遍就会漂移，
                # 而漂移的症状是滤波器过度自信或过度保守，都不报错。
                'eskf.gyro_noise_rad_s': float(imu_noise['gyro_stddev_rad_s']),
                'eskf.accel_noise_mps2': float(imu_noise['accel_stddev_mps2']),
                'eskf.gyro_bias_rw_rad_s': float(imu_noise['gyro_dynamic_bias_stddev_rad_s']),
                'eskf.accel_bias_rw_mps2': float(imu_noise['accel_dynamic_bias_stddev_mps2']),
                'eskf.init_gyro_bias_std_rad_s': float(imu_noise['gyro_bias_stddev_rad_s']),
                'eskf.init_accel_bias_std_mps2': float(imu_noise['accel_bias_stddev_mps2']),
                # ⚠️ GNSS 的 σ 用 YAML 里的**米**，不是 SDF 里那个除过 111320 的数
                #    （Gazebo 把水平噪声按度施加，见 CLAUDE.md 陷阱表）。
                'gnss.horizontal_std_m': float(gnss_noise['position_horizontal_stddev_m']),
                'gnss.vertical_std_m': float(gnss_noise['position_vertical_stddev_m']),
                # ⚠️ 杆臂：GNSS 报的是**天线**的位置。不减掉它就是一个系统性
                #    偏差 —— 竖直 1.6 m 常量、水平 0.5 m 随航向旋转。
                #    它不会让滤波器发散，只会让它稳定地偏一点。
                'gnss.lever_arm_x_m': float(gnss_cfg['mount_x_m']),
                'gnss.lever_arm_y_m': float(gnss_cfg['mount_y_m']),
                'gnss.lever_arm_z_m': float(gnss_cfg['mount_z_m']),
                'initial_yaw_rad': initial_yaw_rad,
                'map_pcd_path': str(cloud_pcd),
                'use_sim_time': True,
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    """装配全栈：仿真数据源 + 算法节点（P1 地图、P2 控制）."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='gazebo',
            description='仿真数据源：gazebo（可用）/ carla（P0b）/ bag（P1 之后）'),
        # P1 起默认 campus_loop.sdf：全栈跑起来时，地图节点画出来的车道图必须
        # 和脚下的路面对得上。留在 campus_minimal.sdf 的话，RViz 里会是一张
        # 悬在空处的车道图 —— 数据全对，但看着像坏了。
        #
        # ⚠️ **只改这里，不要动 gazebo_sim.launch.py 的默认值**。
        #    verify_sim / verify_ros_bridge / verify_teleop 三个脚本都不传 world，
        #    直接吃那一个默认值，而它们的实测基线（RTF 0.970、点云 10.00 Hz…）
        #    全部建立在 campus_minimal 上。改了那些数字一次性作废，
        #    而那是判断「环境有没有退化」的唯一依据。
        DeclareLaunchArgument(
            'world', default_value='campus_loop.sdf',
            description='世界文件名，仅对 sim:=gazebo 有效'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否开仿真器图形界面。CI 里必须 false'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否开 RViz2'),
        DeclareLaunchArgument(
            'obstacles', default_value='none',
            description=('P3 验收场景的静态障碍物：none / avoid / block。'
                         '**默认 none** —— 那是 CP-P2-B 的回归基线，'
                         '世界必须与 P2 时一模一样')),

        DeclareLaunchArgument(
            'dynamic', default_value='none',
            description=('动态目标场景（清单以 config/dynamic_actors.yaml 的 scenarios 为准）：'
                         'none；P5 感知 oncoming / cross / both；P6 预测 curve；'
                         'P7 行为 follow / crossing / junction。'
                         '默认 none —— 三个检查点的回归基线要求世界里没有会动的东西。'
                         '与 obstacles 互相独立，可自由组合'),
        ),
        DeclareLaunchArgument(
            'perception', default_value='false',
            description=('P5：true 时起 perception_node，由它发 /perception/obstacles，'
                         '同时**关掉**真值发布器往那个话题上发（两者互斥）。\n'
                         '默认 false —— 那是 CP-P2-B / CP-P3-B / CP-P4-B 的回归基线，'
                         '它们建立在真值障碍物上')),
        DeclareLaunchArgument(
            'prediction', default_value='false',
            description=('P6：true 时起 prediction_node（订阅 /perception/obstacles，'
                         '发布 /prediction/trajectories 与 RViz markers）。\n'
                         '⚠️ P7-S3 起**不再是纯旁路**：planning_node 消费预测做横穿'
                         '让行（行为层），且 expect_prediction 会随本开关置位 ——'
                         '开了它而预测链路没接上时规划器指名报错并不发轨迹。\n'
                         '默认 false —— 所有既有检查点的回归基线里都没有预测；'
                         '关掉时行为层只剩跟车（感知近边），降级方向正确')),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description=('P4：true 时起 localization_node，由它发动态 map→odom，'
                         '同时**关掉**仿真侧那条来自 spawn 位姿的静态 TF。\n'
                         '默认 false —— 那是 CP-P2-B / CP-P3-B 的回归基线，'
                         '它们的实测值全部建立在真值 TF 上。')),

        OpaqueFunction(function=_resolve_sim_source),
        OpaqueFunction(function=_localization_nodes),
        OpaqueFunction(function=_perception_nodes),
        OpaqueFunction(function=_prediction_nodes),

        # ---------------------------------------------------------------------
        # 算法节点挂在这里
        #
        # 顺序按 SPEC §12 的路线图：P1 地图 → P2 控制 → P3 规划 → P4 定位 → P5 感知。
        # 每个节点都只订阅 §4.1 的规范话题，**不允许**出现任何
        # "这是 Gazebo 所以……" 的判断 —— 那种判断一旦写进算法节点，
        # 上面那张 SIM_SOURCES 表就白做了。
        # ---------------------------------------------------------------------

        # P1：地图与路由。
        #
        # 它对仿真源**一无所知** —— 只读 .xodr、只要 TF map→base_link。
        # 所以 sim:=carla 那天它一行都不用改，这正是 §4.1 那套契约要买的东西。
        #
        # 这里不传 map_file：默认值由节点自己从 ads_map 的 share 里取，
        # 在这儿再写一遍路径就等于把地图位置写了第二份。
        Node(
            package='ads_map',
            executable='map_node',
            name='map_node',
            # SPEC §5：所有节点 use_sim_time=true，禁止用墙钟做算法时序。
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # P3：运动规划（Frenet 采样 + 碰撞检测 + 速度剖面）。
        #
        # 它插在 map_node 与 control_node 之间：把全局路径 + 障碍物
        # 变成一条**带速度**的轨迹。
        #
        # ⚠️ **/perception/obstacles 现在还没有发布者**（真值障碍物发布器在 P3-S5）。
        #    这不影响启动：没有障碍物时它就按空障碍物列表规划，沿车道中心线走。
        #    「等一个可能永远不来的话题」是很常见的死法，本节点不这么做。
        Node(
            package='ads_planning',
            executable='planning_node',
            name='planning_node',
            parameters=[planning_node_params(), {
                'use_sim_time': True,
                # 启动告知（P7-S3 收口）：launch 知道这一跑开了哪些链路，
                # 告诉规划器，让「链路该在而从未到达」从静默变成指名报错。
                # ⚠️ ParameterValue(…, bool)：裸 LaunchConfiguration 解析成
                #    **字符串** "true"，与节点声明的 bool 类型不符会直接抛。
                'expect_perception': ParameterValue(
                    LaunchConfiguration('perception'), value_type=bool),
                'expect_prediction': ParameterValue(
                    LaunchConfiguration('prediction'), value_type=bool),
            }],
            output='screen',
        ),

        # P2：路径跟踪控制（横向 Stanley + 纵向 PI）。
        #
        # ⚠️ **P3-S4 起它吃 /planning/trajectory 而不是 /route/path** ——
        #    速度剖面已经移到规划侧（算目标是规划，跟目标才是控制）。
        #
        # 它同样对仿真源**一无所知** —— 只订阅 /planning/trajectory、/odom 和 TF，
        # 只发 /vehicle_cmd。这些都是 SPEC §4.1 意义上的规范接口，
        # 所以 sim:=carla 那天它也一行不用改。
        #
        # ⚠️ 参数**全部**从 YAML 读，这个文件里一个数字都不写。
        #    在 launch 里写死"顺手改一下试试"的那个值，是参数漂移最常见的起点：
        #    它不在任何一个 config 文件里，grep 不到，而且会盖掉 YAML。
        Node(
            package='ads_control',
            executable='control_node',
            name='control_node',
            parameters=[control_node_params(), {'use_sim_time': True}],
            output='screen',
        ),
    ])
