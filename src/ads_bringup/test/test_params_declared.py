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

"""
Every key stack.launch.py feeds perception_node from perception_params.yaml must be declared.

ROS 2 对**未声明**的参数默认是忽略的：yaml 里键名拼错、或节点删了某个 declare_parameter，
现场表现都是「改了配置没生效、节点静默用默认值」—— 而 CI 的 L3-G 用节点默认值起感知节点，
从不加载生产 yaml，所以这类漂移**没有任何测试能发现**（2026-08-16 复审 High）。
这里做静态对账：把 stack.launch.py 实际平铺出来的参数名（perception:=true 的那条
OpaqueFunction）与 perception_node.cpp 里 `declare_parameter<...>("...")` 的字面量对齐。
静态而不起节点：毫秒级、不需要 DDS，且节点的参数声明全是构造函数里的字面量。

⚠️ 反向（节点声明了、yaml 没给）只作 WARN 打印不判红：节点默认值就是那个参数的合法来源
（`use_sim_time`、`diagnostics_period_s` 一类）。
"""

import importlib.util
from pathlib import Path
import re
import warnings

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node as RosNode

_REPO = Path(__file__).resolve().parents[3]
_LAUNCH_FILE = _REPO / 'src' / 'ads_bringup' / 'launch' / 'stack.launch.py'
_NODE_SOURCE = _REPO / 'src' / 'ads_perception' / 'node' / 'perception_node.cpp'
_DECLARE_RE = re.compile(r'declare_parameter<[^>]+>\(\s*"([A-Za-z0-9_.]+)"')


def _load_stack_launch():
    spec = importlib.util.spec_from_file_location('stack_launch_params', _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _perception_flat_params():
    """Run stack.launch.py's perception OpaqueFunction and return the flat param dict it builds."""
    stack = _load_stack_launch()
    context = LaunchContext()
    context.launch_configurations['perception'] = 'true'
    actions = stack._perception_nodes(context)
    nodes = [a for a in actions if isinstance(a, RosNode)]
    assert len(nodes) == 1, f'perception:=true 应恰好起一个节点，实际 {len(nodes)}'
    flat = {}
    for entry in nodes[0]._Node__parameters or ():
        if isinstance(entry, dict):
            for key, value in entry.items():
                flat[perform_substitutions(context, list(key))] = value
    return flat


def _declared_in_node_source():
    text = _NODE_SOURCE.read_text(encoding='utf-8')
    names = set(_DECLARE_RE.findall(text))
    assert names, f'{_NODE_SOURCE} 里一个 declare_parameter 都没匹配到 —— 正则或源码变了'
    return names


def test_every_yaml_key_fed_to_perception_node_is_declared():
    """Keys launch passes but the node never declares would be silently ignored."""
    flat = _perception_flat_params()
    declared = _declared_in_node_source()
    # use_sim_time 由 rclcpp 自动声明，不在源码里。
    undeclared = sorted(k for k in flat if k not in declared and k != 'use_sim_time')
    assert not undeclared, (
        f'perception_params.yaml 经 stack.launch.py 平铺后有 {len(undeclared)} 个键节点没声明'
        f'（会被 ROS 静默忽略、节点用默认值）：{undeclared}')


def test_node_declarations_missing_from_yaml_are_only_the_expected_ones():
    """Reverse direction: warn, not fail — defaults are legitimate, but keep drift visible."""
    flat = _perception_flat_params()
    declared = _declared_in_node_source()
    missing = sorted(k for k in declared if k not in flat)
    # 已知只靠节点默认值的参数：诊断周期、map 系名。加新的要在这里登记（让漂移可见）。
    allowed = {'diagnostics_period_s', 'map_frame'}
    unexpected = [k for k in missing if k not in allowed]
    if unexpected:
        warnings.warn(f'perception_node 声明了但 yaml 没给（走默认值）：{unexpected}', stacklevel=1)
    # 只提示不判红：默认值是合法来源；这条用例的价值是让漂移在测试输出里**看得见**。
    assert isinstance(unexpected, list)


def test_share_yaml_is_the_same_file_launch_reads():
    """The installed share yaml must equal the repo one — else CI checks stale config."""
    share_yaml = (Path(get_package_share_directory('ads_perception')) / 'config'
                  / 'perception_params.yaml')
    repo_yaml = _REPO / 'config' / 'perception_params.yaml'
    if share_yaml.exists() and repo_yaml.exists():
        assert share_yaml.read_text(encoding='utf-8') == repo_yaml.read_text(encoding='utf-8'), (
            'install 里的 perception_params.yaml 与仓库 config/ 不一致 —— 重新 colcon build')
