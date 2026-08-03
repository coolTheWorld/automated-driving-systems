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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
            }.items(),
        ),
    ]


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
        'limits.cruise_speed_mps': lim['cruise_speed_mps'],
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
        'profile.max_lateral_accel_mps2': control['profile']['max_lateral_accel_mps2'],
        'goal.stop_distance_m': control['goal']['stop_distance_m'],
        'safety.max_lateral_error_m': control['safety']['max_lateral_error_m'],
        'safety.odom_timeout_s': control['safety']['odom_timeout_s'],
        'control_rate_hz': control['control_rate_hz'],
    }


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

        OpaqueFunction(function=_resolve_sim_source),

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

        # P2：路径跟踪控制（横向 Stanley + 纵向速度剖面/PI）。
        #
        # 它同样对仿真源**一无所知** —— 只订阅 /route/path、/odom 和 TF，
        # 只发 /vehicle_cmd。这三个都是 SPEC §4.1 的规范话题，
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
