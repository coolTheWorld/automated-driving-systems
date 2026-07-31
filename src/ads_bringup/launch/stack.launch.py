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


def generate_launch_description():
    """装配全栈：仿真数据源 + （待 P2 起）算法节点."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='gazebo',
            description='仿真数据源：gazebo（可用）/ carla（P0b）/ bag（P1 之后）'),
        DeclareLaunchArgument(
            'world', default_value='campus_minimal.sdf',
            description='世界文件名，仅对 sim:=gazebo 有效'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否开仿真器图形界面。CI 里必须 false'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否开 RViz2'),

        OpaqueFunction(function=_resolve_sim_source),

        # ---------------------------------------------------------------------
        # 算法节点挂在这里（P2 起）
        #
        # 顺序按 SPEC §12 的路线图：P2 控制 → P3 规划 → P4 定位 → P5 感知。
        # 每个节点都只订阅 §4.1 的规范话题，**不允许**出现任何
        # "这是 Gazebo 所以……" 的判断 —— 那种判断一旦写进算法节点，
        # 上面那张 SIM_SOURCES 表就白做了。
        # ---------------------------------------------------------------------
    ])
