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

"""dry-run 的骨架检查：没有 CARLA 的机器上能验证的一切结构性质.

「本地 dry-run 全过才上机」（P8 风险表第一条）的机械化：
惰性 import、launch 可加载、节点集与 gazebo 侧镜像对应。
"""

import importlib.util
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node as RosNode


def test_sidecar_module_imports_without_carla():
    """惰性 import 的机械证明：模块级 import 不得触碰 carla.

    这条红了 = 有人把 import carla 提到了模块顶层 —— 本包在无 CARLA 的
    机器上会连构建/launch 加载都做不到，dry-run 战略整个失效。
    """
    assert 'carla' not in sys.modules
    import carla_bridge.carla_sidecar_node  # noqa: F401
    assert 'carla' not in sys.modules, (
        'carla_sidecar_node 在模块层 import 了 carla —— 必须放进 main()（惰性）')


def _load_launch_module():
    """按路径加载 carla_sim.launch.py（与 test_sim_source.py 同一套做法）."""
    path = (Path(get_package_share_directory('carla_bridge')) / 'launch' /
            'carla_sim.launch.py')
    spec = importlib.util.spec_from_file_location('carla_sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    # dataclass/注解解析需要模块先进 sys.modules（CLAUDE.md lint 陷阱表末行）
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_launch_description_loads_and_mirrors_gazebo_side():
    """launch 可加载，且节点集与 gazebo 侧镜像：sidecar + 预处理 + RSP.

    lidar_preprocessor 必须**复用 gazebo_bridge 的同一个可执行**、只换
    input_topic 前缀（bridge_topics.yaml 当年为 /carla 前缀留的位置）——
    抄一份 C++ 就是两处漂移。
    """
    module = _load_launch_module()
    nodes = [e for e in module.generate_launch_description().entities
             if isinstance(e, RosNode)]
    by_package = {}
    for node in nodes:
        by_package.setdefault(node.node_package, []).append(node.node_executable)
    assert by_package.get('carla_bridge') == ['carla_sidecar_node']
    assert by_package.get('gazebo_bridge') == ['lidar_preprocessor_node']
    assert by_package.get('robot_state_publisher') == ['robot_state_publisher']


def test_lidar_preprocessor_input_uses_carla_prefix():
    """中间话题必须带 /carla 前缀 —— 防两套 bridge 误同起时点云串台."""
    module = _load_launch_module()
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, RosNode) and entity.node_executable == 'lidar_preprocessor_node':
            # 私有属性 _Node__parameters：与 test_sim_source.py 同一条注释 ——
            # launch_ros 没有公开读取口，升级时宁可 AttributeError 响亮地炸。
            flat = {}
            for chunk in entity._Node__parameters or ():
                if isinstance(chunk, dict):
                    for key, value in chunk.items():
                        flat[perform_substitutions(context, list(key))] = value
            def resolved(key):
                value = flat.get(key)
                if isinstance(value, (tuple, list)):
                    value = perform_substitutions(context, list(value))
                # launch 的参数规范化会在字符串值上带出 YAML 序列化痕迹
                # （尾随换行与省略标记），只取首行比较。
                return str(value).strip().splitlines()[0]
            assert resolved('input_topic') == '/carla/lidar/points_raw'
            assert resolved('output_topic') == '/lidar/points'
            return
    raise AssertionError('launch 里没有 lidar_preprocessor_node')
