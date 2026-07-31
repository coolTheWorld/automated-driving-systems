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
#  stack.launch.py 的 L1 测试 —— 只测「仿真源注册表」的解析
#
#  为什么这几行装配代码值得单独测
#  ------------------------------
#  stack.launch.py 里那个 OpaqueFunction 是**故意**用命令式写法换来的：
#  声明式的 IfCondition 在参数非法时会安安静静地什么都不起，而那种故障
#  和「仿真器崩了」长得一模一样（见 CLAUDE.md 陷阱表）。
#
#  但那段报错逻辑此前**没有任何自动化覆盖** —— CI 只跑 `--show-args`，
#  而 OpaqueFunction 是在 launch 真正启动时才执行的，`--show-args` 碰不到它。
#  于是出现一个讽刺的局面：为了消灭静默失败而写的代码，自己没被验证过。
#
#  为什么是 pytest 而不是 launch_testing（L2）
#  -------------------------------------------
#  被测的是**纯函数**：给一个 sim 字符串，要么返回 include 动作，要么抛异常。
#  它不需要 ROS 图、不需要仿真器、不需要起任何进程。用 L2 去测等于为了
#  几个 if 分支付出秒级启动开销 —— 而 SPEC §8 的金字塔明确要求
#  「能在 L1 测的不要放到 L2」。整个文件跑完是毫秒级。
#
#  ⚠️ 端到端那一层（`ros2 launch` 真的以非零码退出）由 CI 的 shell 断言覆盖，
#     见 .github/workflows/ci.yml。两层职责不同，缺一不可：
#     本文件证明「函数会抛」，CI 证明「抛出来之后 launch 确实失败了」。
# =============================================================================

"""Unit tests for the simulation-source registry in stack.launch.py."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import pytest

# 测试文件在 <pkg>/test/，launch 文件在 <pkg>/launch/
_LAUNCH_FILE = Path(__file__).resolve().parent.parent / 'launch' / 'stack.launch.py'


def _load_stack_launch():
    """
    Import stack.launch.py by path.

    文件名里的点让它不是合法的 Python 模块名，`import stack.launch` 会被
    当成包路径解析而失败，所以只能按路径加载。

    这里加载的是**源码树里那一份**（不是 install/ 下的），这样测试跑的
    就是你刚改的代码，不必先 colcon build 才能看到改动的效果。
    """
    spec = importlib.util.spec_from_file_location('stack_launch', _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


stack = _load_stack_launch()


def _context_with_sim(value):
    """
    Build a LaunchContext whose ``sim`` configuration equals ``value``.

    _resolve_sim_source 只会 perform 'sim' 这一个参数；world/gui/rviz 三个
    是作为 LaunchConfiguration **对象**原样透传给子 launch 的，构造时不求值，
    所以这里不必给它们赋值。
    """
    context = LaunchContext()
    context.launch_configurations['sim'] = value
    return context


def _registered_launch_path(sim_name):
    """
    Resolve the launch file path that SIM_SOURCES claims for ``sim_name``.

    ⚠️ 这里**独立地**按注册表算一遍路径，而不是去读 IncludeLaunchDescription
    内部的 launch_description_source.location。

    实测那个属性存的是**尚未求值的替换（substitution）对象** ——
    str() 出来是 "<launch.substitutions.text_substitution.TextSubstitution
    object at 0x...>"，拿它去 is_file() 永远是 False。
    而且它属于 launch 的实现细节，版本一变断言就可能变成假过。
    """
    package, launch_file = stack.SIM_SOURCES[sim_name]
    return Path(get_package_share_directory(package)) / 'launch' / launch_file


def _declared_default(name):
    """Return the default value declared for launch argument ``name``."""
    for entity in stack.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name == name:
            # default_value 是替换序列，要 perform 才是字符串
            return ''.join(sub.perform(LaunchContext()) for sub in entity.default_value)
    raise AssertionError(f'launch 参数 "{name}" 没有被 DeclareLaunchArgument 声明')


# -----------------------------------------------------------------------------
# 可用数据源
# -----------------------------------------------------------------------------

# 参数化而不是硬写 'gazebo'：P0b 往 SIM_SOURCES 加 carla 时，这条用例
# **自动**开始覆盖它，不需要有人记得回来补测试。
@pytest.mark.parametrize('sim_name', sorted(stack.SIM_SOURCES))
def test_every_registered_source_resolves(sim_name):
    """Each registered source must resolve to exactly one include action."""
    actions = stack._resolve_sim_source(_context_with_sim(sim_name))

    assert len(actions) == 1
    assert isinstance(actions[0], IncludeLaunchDescription)


@pytest.mark.parametrize('sim_name', sorted(stack.SIM_SOURCES))
def test_every_registered_source_points_at_a_real_file(sim_name):
    """
    The package and launch file named in SIM_SOURCES must actually exist.

    包名或文件名写错时，报错要等到真正 launch 才出现，而那时的错误信息是
    ament_index 的「找不到包」—— 看着像环境没装好，而不是像表里写错了。
    这条用例把它提前到 colcon test。
    """
    path = _registered_launch_path(sim_name)
    assert path.is_file(), f'{sim_name} 在 SIM_SOURCES 里指向的 launch 文件不存在：{path}'


def test_default_sim_is_a_registered_source():
    """
    The declared default for ``sim`` must exist in SIM_SOURCES.

    防的是「改了默认值忘了改注册表」：那会让**不带任何参数**的
    `ros2 launch ads_bringup stack.launch.py` 直接报错 —— 而这条命令
    是 README 里给新人的第一条命令，坏了影响面最大。
    """
    assert _declared_default('sim') in stack.SIM_SOURCES


# -----------------------------------------------------------------------------
# 两条报错分支 —— 它们必须**互相可区分**
# -----------------------------------------------------------------------------

@pytest.mark.parametrize('sim_name', sorted(stack.PLANNED_SOURCES))
def test_planned_source_names_its_roadmap_phase(sim_name):
    """
    A planned-but-unimplemented source must say so and name the phase.

    「还没做」和「你拼错了」的排查方向完全不同：前者该去看路线图，
    后者该去检查拼写。所以这里不只断言「抛了异常」，还断言异常信息里
    确实带着阶段说明 —— 否则报错信息退化成一句无用的「失败」也能测过。
    """
    with pytest.raises(RuntimeError) as excinfo:
        stack._resolve_sim_source(_context_with_sim(sim_name))

    message = str(excinfo.value)
    assert '尚未实现' in message
    assert stack.PLANNED_SOURCES[sim_name] in message


def test_unknown_source_is_distinguishable_from_planned():
    """A typo must not be reported as "not implemented yet"."""
    with pytest.raises(RuntimeError) as excinfo:
        # 'gazbo' 是 'gazebo' 的常见手误
        stack._resolve_sim_source(_context_with_sim('gazbo'))

    message = str(excinfo.value)
    assert '未知的' in message
    assert '尚未实现' not in message


# -----------------------------------------------------------------------------
# 注册表不变量 —— 这条是给 P0b 埋的
# -----------------------------------------------------------------------------

def test_registries_do_not_overlap():
    """
    A source must never be in both registries.

    ⚠️ 这条是本文件里最重要的用例，防的是一个**只在 P0b 才会发作**的陷阱。

    _resolve_sim_source 先查 PLANNED_SOURCES 再查 SIM_SOURCES。所以 P0b 实现
    carla_bridge、往 SIM_SOURCES 加了 'carla' 之后，如果忘了从 PLANNED_SOURCES
    删掉它，第一个判断会先命中 —— `sim:=carla` 永远报「尚未实现」，
    哪怕代码早就写好了。

    症状是「我明明实现了，它说我没实现」，而所有人的第一反应都是去查
    carla_bridge 有没有装上、包名对不对 —— 没人会想到病根在这张表里。
    """
    overlap = set(stack.SIM_SOURCES) & set(stack.PLANNED_SOURCES)
    assert not overlap, (
        f'这些数据源同时出现在两张表里：{sorted(overlap)}。'
        f'实现完成后必须从 PLANNED_SOURCES 删除，否则它永远被当成「尚未实现」。')
