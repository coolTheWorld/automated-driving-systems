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
#  scenario.launch.py —— 按场景号起栈（SPEC §6 承诺的那条命令，P8-S2 兑现）
#
#      ros2 launch ads_bringup scenario.launch.py scenario:=S05
#      ros2 launch ads_bringup scenario.launch.py scenario:=S04b sim:=gazebo
#
#  ## 它只是一张映射表
#
#  场景 = stack.launch.py 的一组参数（世界不变，运行时注入 —— 三个检查点的
#  回归基线要求世界逐字节相同，见 CLAUDE.md）。本文件把「S05」翻译成那组
#  参数，**不含任何新配置**：改场景去 config/*.yaml，改接线去 stack.launch.py。
#
#  ⚠️ 它起的是**栈**，不含记录器与 goal —— 判定归 record_*_run.py
#     （由 scripts/run_all_scenarios.sh 编排）。「跑起来看看」用本文件，
#     「跑出判定」用 run_all_scenarios.sh。
#
#  ⚠️ S06（红绿灯）不在表里：环境 A 无灯态激励源（SPEC §8 注解，P7 拍板），
#     它只存在于 L3-C。sim:=carla 侧的场景注入在 carla_bridge（P8-S4+）。
# =============================================================================

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 场景号 → stack.launch.py 参数组。与 scripts/run_all_scenarios.sh 的注册表
# 同一套对应关系（那边多带记录器与 goal；两处的参数组若漂移，run_all 的
# 判定就不再代表本命令起的栈 —— 改一处必改另一处，两处都引用了对方）。
SCENARIOS = {
    # S01 直道巡航 + S02 弯道 + S07 到达：CP-P2-B 的 95 m 基线路线，裸栈。
    'S01': {},
    'S02': {},
    'S07': {},
    # S03 前车跟停（P7 follow）。
    'S03': {'perception': 'true', 'prediction': 'true', 'dynamic': 'follow'},
    # S04 静止障碍物：a 可绕 / b 堵死。
    'S04': {'obstacles': 'avoid'},
    'S04a': {'obstacles': 'avoid'},
    'S04b': {'obstacles': 'block'},
    # S05 行人横穿（P7 crossing）。
    'S05': {'perception': 'true', 'prediction': 'true', 'dynamic': 'crossing'},
    # 无信号路口让行（SPEC 表外，P7 的让行原语路口实例）。
    'junction': {'perception': 'true', 'prediction': 'true', 'dynamic': 'junction'},
}


def _resolve(context, *args, **kwargs):
    """
    Translate scenario:=Sxx into a stack.launch.py include.

    未知场景**直接报错**而不是起一个空栈 —— 「进程活着、话题都在、永远没
    数据」的静默空转与仿真器崩溃在现场长得一模一样（CLAUDE.md 陷阱表）。

    :param context: launch 上下文
    :return: 要执行的 launch 动作
    """
    scenario = LaunchConfiguration('scenario').perform(context)
    if scenario == 'S06':
        raise RuntimeError(
            'S06（红绿灯）只存在于 L3-C：环境 A 无灯态激励源（SPEC §8 注解，P7 拍板）。')
    if scenario not in SCENARIOS:
        raise RuntimeError(
            f'未知场景 "{scenario}"。可用：{"、".join(sorted(SCENARIOS))}（S06 仅 L3-C）')

    stack = str(Path(get_package_share_directory('ads_bringup')) / 'launch' / 'stack.launch.py')
    arguments = {'sim': LaunchConfiguration('sim'),
                 'gui': LaunchConfiguration('gui'),
                 'rviz': LaunchConfiguration('rviz')}
    arguments.update(SCENARIOS[scenario])
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stack), launch_arguments=arguments.items())]


def generate_launch_description():
    """按场景号装配 stack.launch.py."""
    return LaunchDescription([
        DeclareLaunchArgument('scenario', description='场景号：' + '、'.join(sorted(SCENARIOS))),
        DeclareLaunchArgument('sim', default_value='gazebo'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ] + [OpaqueFunction(function=_resolve)])
