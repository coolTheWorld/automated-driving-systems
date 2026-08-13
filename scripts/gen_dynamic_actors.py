#!/usr/bin/env python3
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

"""Generate the P5 dynamic-actor Gazebo models from config/dynamic_actors.yaml.

从 `config/dynamic_actors.yaml`（唯一手写源头）生成两个 Gazebo 模型：

    models/npc_car/model.sdf       对向行驶的 NPC 车
    models/pedestrian/model.sdf    行人（一个盒子，理由见 YAML 顶部）

⚠️ **生成的是模型，不是场景。** 一个 `VelocityControl` 插件只能驱动**一个**
模型，所以两个目标必须是两个独立模型 —— 把它们塞进同一个 SDF 的话，
它们只能一起动，而「车经过时挡住行人」这个遮挡场景就没了。
场景（哪些目标出场）由 launch 的 `dynamic:=` 参数决定，见 gazebo_sim.launch.py。

⚠️ **NPC 车的几何从 config/vehicle_params.yaml 读，本脚本一个尺寸都不写死。**
与自车同一份几何是有意的（SPEC §4.1 单一来源）：两份各写一遍的话，
感知判据里的「尺寸误差」会把生成器的笔误算成算法误差，而人会去查聚类参数。

用法：
    python3 scripts/gen_dynamic_actors.py            # 生成
    python3 scripts/gen_dynamic_actors.py --check    # 逐字节比对（CI 用）
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
ACTORS_YAML = REPO / 'config' / 'dynamic_actors.yaml'
VEHICLE_YAML = REPO / 'config' / 'vehicle_params.yaml'


def _load_vehicle_model_module():
    """Load gen_vehicle_model.py by path so we can reuse its derive().

    ⚠️ 必须先 `sys.modules[spec.name] = module` 再 `exec_module` ——
    那个模块用了 `from __future__ import annotations`，注解变成字符串后
    dataclass 要回 sys.modules 里查模块才能解析它们，否则报
    `AttributeError: 'NoneType' object has no attribute '__dict__'`，
    而报错完全不提根因。见 CLAUDE.md 的 lint 陷阱表最后一条。

    :return: 已加载的 gen_vehicle_model 模块
    """
    path = REPO / 'scripts' / 'gen_vehicle_model.py'
    spec = importlib.util.spec_from_file_location('gen_vehicle_model', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


# =============================================================================
#  机械校验 —— 与 gen_obstacles.py 校验可行性不等式是同一个思路
# =============================================================================
def derive_inner_loop_clockwise(loop: dict) -> list[tuple[float, float]]:
    """Derive the inner clockwise lane circuit from the loop parameters.

    与 dynamic_actors.yaml 的 `loop_lane` 注释同一套推导：
    直道中心线向环内偏 lane_offset、四角圆弧半径 = corner_radius − lane_offset，
    首点是北直道上的 spawn（相位锚点），四角各"入口 + 弧上按 arc_step_deg
    采样 + 出口"，loop=true 下最后一点沿北车道接回首点。

    :param loop: dynamic_actors.yaml 的 loop_lane 段
    :return: [(x, y), ...] 世界坐标航点
    """
    half_len = float(loop['half_length_m'])
    half_wid = float(loop['half_width_m'])
    radius_ref = float(loop['corner_radius_m'])
    offset = float(loop['lane_offset_m'])
    step_deg = int(loop['arc_step_deg'])
    spawn_x = float(loop['spawn_x_m'])

    r = radius_ref - offset          # 内圈弧半径
    cx = half_len - radius_ref       # 弯心 |x|
    cy = half_wid - radius_ref       # 弯心 |y|
    lane_y = half_wid - offset       # 南北直道内圈车道 |y|
    lane_x = half_len - offset       # 东西路内圈车道 |x|

    def arc(center_x: float, center_y: float, a0_deg: int, a1_deg: int):
        step = -step_deg if a1_deg < a0_deg else step_deg
        pts = []
        a = a0_deg + step
        while (step < 0 and a > a1_deg) or (step > 0 and a < a1_deg):
            pts.append((center_x + r * math.cos(math.radians(a)),
                        center_y + r * math.sin(math.radians(a))))
            a += step
        return pts

    waypoints = [(spawn_x, lane_y)]
    waypoints += [(cx, lane_y)] + arc(cx, cy, 90, 0) + [(lane_x, cy)]        # NE
    waypoints += [(lane_x, -cy)] + arc(cx, -cy, 0, -90) + [(cx, -lane_y)]    # SE
    waypoints += [(-cx, -lane_y)] + arc(-cx, -cy, -90, -180) + [(-lane_x, -cy)]  # SW
    waypoints += [(-lane_x, cy)] + arc(-cx, cy, 180, 90) + [(-cx, lane_y)]   # NW
    return waypoints


def validate(cfg: dict, vehicle: dict) -> None:
    """Reject configurations that would silently ruin the acceptance runs.

    ⚠️ 校验而不是信任填的人，理由与 gen_obstacles.py 一样：
    这两条违反了都**不会报错**，只会让实测得出一个错误的结论，
    而人会去查算法。

    :param cfg: dynamic_actors.yaml 的内容
    :param vehicle: vehicle_params.yaml 的内容
    :raises SystemExit: 校验不通过
    """
    lane = cfg['lane']
    ego_y = float(lane['ego_center_y_m'])
    half = float(lane['ego_lane_half_width_m'])
    x_lo, x_hi = float(lane['valid_from_x_m']), float(lane['valid_to_x_m'])

    for name, actor in cfg['actors'].items():
        if actor.get('route') == 'inner_loop_clockwise':
            # ④ 环线航点是**推导量**：按 loop_lane 重推一遍逐点对账。
            #    手改任何一个数（或改了 loop_lane 忘了重贴航点）都在这里被拒。
            #    「锚定车道中心线」由此机械保证 —— 内圈车道与自车车道全程
            #    平行、间隔 = 2×lane_offset = 3.5 m，天然不侵入，
            #    所以直道区间/侵入两条检查对它不适用（也检查不了弯段）。
            derived_wps = derive_inner_loop_clockwise(cfg['loop_lane'])
            given = [(float(x), float(y)) for x, y in actor['waypoints']]
            if len(given) != len(derived_wps):
                sys.exit(
                    f'✗ {name} 的航点数 {len(given)} ≠ 推导值 {len(derived_wps)}。\n'
                    f'  route: inner_loop_clockwise 的航点是推导量，'
                    f'用 derive_inner_loop_clockwise() 重新生成后整段替换。')
            for i, ((gx, gy), (dx, dy)) in enumerate(zip(given, derived_wps)):
                if abs(gx - dx) > 1e-3 or abs(gy - dy) > 1e-3:
                    sys.exit(
                        f'✗ {name} 航点 #{i} = ({gx}, {gy}) 偏离推导值 '
                        f'({dx:.4f}, {dy:.4f}) 超过 1 mm。\n'
                        f'  这些航点锚定车道中心线，是推导量不是配置项。')
            continue

        if name == 'npc_car':
            half_width = float(vehicle['geometry']['width_m']) / 2.0
        else:
            half_width = float(actor['width_m']) / 2.0

        for wx, wy in actor['waypoints']:
            # ① 航点必须在这条直道的有效区间内。
            #    跑到路面外面的目标在 RViz 里看着完全正常，只是自车永远遇不到它，
            #    于是「感知测试通过」变成一句空话。
            if not (x_lo <= float(wx) <= x_hi):
                sys.exit(
                    f'✗ {name} 的航点 x={wx} 超出直道有效区间 [{x_lo}, {x_hi}]。\n'
                    f'  跑到路面外的目标自车永远遇不到，判据会变成空话。')

            # ② 目标的**外廓**不许侵入自车车道。
            #    这一条不是"怕撞"：规划器会拿 /perception/obstacles 做碰撞检查，
            #    侵入车道的目标会让车减速或绕行，而 P5 **不做**避让（那是 P7）。
            #    现场表现是「车莫名其妙停了」→ 所有人去查规划器，而错在场景设定。
            #    与 P3「障碍物放在车道中间几何无解」是同一类问题。
            gap_m = abs(float(wy) - ego_y) - half - half_width
            if gap_m < 0.0:
                sys.exit(
                    f'✗ {name} 的航点 y={wy} 侵入自车车道 '
                    f'(中心 {ego_y}，半宽 {half})，外廓重叠 {-gap_m:.3f} m。\n'
                    f'  规划器会把它当障碍物 → 车会减速/绕行，而 P5 不做避让。\n'
                    f'  症状是「车莫名其妙停了」，人会去查规划器 —— 错却在这里。')

    # ③ 场景引用的目标必须存在。写错名字的话那个目标就是**静默地不出场**。
    for scen, spec in cfg['scenarios'].items():
        for name in spec['actors']:
            if name not in cfg['actors']:
                sys.exit(f'✗ 场景 {scen} 引用了未定义的目标 {name}')


# =============================================================================
#  SDF 渲染
# =============================================================================
def _box_inertia(mass: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Solid-box inertia about its own center.

    :param mass: 质量 kg
    :param x: 长 m
    :param y: 宽 m
    :param z: 高 m
    :return: (ixx, iyy, izz)
    """
    k = mass / 12.0
    return k * (y * y + z * z), k * (x * x + z * z), k * (x * x + y * y)


def _plugins(name: str) -> str:
    """The two plugins every dynamic actor needs: drive it, and report the truth.

    :param name: 模型名
    :return: SDF 片段
    """
    return f"""
    <!-- 驱动：订阅 gz.msgs.Twist，直接设置模型速度。
         ⚠️ 它是**运动学**的 —— 绕过力与轮胎，指令速度立刻生效。
            这对道具正合适：NPC 是感知的**目标**，不是被测对象。
            真车的动力学延迟属于 ads_control 的问题，P2 已经量过了
            （转向执行机构 τ=0.294 s），不该在这里重复一遍。
         ⚠️ 轮子不会转（模型里根本没有转动关节）。对点云无影响 ——
            轮子在原地也是那个形状，而感知看的是形状不是运动模糊。 -->
    <plugin filename="gz-sim-velocity-control-system"
            name="gz::sim::systems::VelocityControl">
      <topic>/model/{name}/cmd_vel</topic>
    </plugin>

    <!-- 真值：与自车的 /ego_pose_gt 走**完全相同**的插件与约定。
         一套真值机制而不是两套，是把 actor 换成 model 的主要收益之一
         （见 dynamic_actors.yaml 顶部）。
         ⚠️ <tf_topic> 指向一个没人订阅的话题：这个插件默认会发
            odom_frame → robot_base_frame 的 TF，而那会让 TF 树上多出
            一个 map 的子节点，与自车的 base_link 重名冲突。真值只走话题。 -->
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>map</odom_frame>
      <robot_base_frame>{name}_base</robot_base_frame>
      <odom_topic>/model/{name}/pose_gt</odom_topic>
      <tf_topic>/model/{name}/pose_gt_tf_unused</tf_topic>
      <dimensions>3</dimensions>
      <odom_publish_frequency>50</odom_publish_frequency>
    </plugin>"""


def render_npc_car(vehicle: dict, derived: dict, name: str = 'npc_car',
                   title: str = 'NPC 车：P5 感知的动态目标') -> str:
    """Render a vehicle prop model — same geometry as ego, **no sensors**.

    ⚠️ 「去掉传感器」不是可选项。复用自车模型时若把 `<sensor>` 块也带上，
    世界里就会多出一个 32 线雷达在渲染，RTF 直接腰斩 ——
    而症状是「加了辆车怎么就卡了」，人会去查 GPU 而不是查模型。

    ⚠️ P6-S1 起同一个模板渲染两辆道具车（npc_car / curve_car）：
    只有模型名与标题参数化，其余逐字节相同 —— npc_car 的产物必须与
    P5 基线**逐字节一致**（--check 守着）。

    :param vehicle: vehicle_params.yaml 的内容
    :param derived: gen_vehicle_model.derive() 的结果
    :param name: 模型名（模型/link/插件话题都用它）
    :param title: SDF 头部注释的第一行标题
    :return: model.sdf 的内容
    """
    geo = vehicle['geometry']
    d = derived
    mass = float(vehicle['mass']['total_kg'])
    ixx, iyy, izz = _box_inertia(mass, d['length'], geo['width_m'], d['height'])

    # 车轮只做 visual：让它在 RViz / 点云里像辆车，而不是一块砖。
    # ⚠️ **不做 collision** —— 碰撞体只需要车身盒（VelocityControl 下
    #    根本不靠碰撞行走），多四个碰撞体只是白白增加物理开销。
    wheels = []
    for sx, sy in ((0.0, 1.0), (0.0, -1.0), (1.0, 1.0), (1.0, -1.0)):
        x = sx * d['wheelbase']
        y = sy * d['half_track']
        wheels.append(f"""      <visual name="wheel_{'f' if sx else 'r'}{'l' if sy > 0 else 'r'}">
        <pose>{x:.6g} {y:.6g} {d['wheel_r']:.6g} {math.pi / 2:.9g} 0 0</pose>
        <geometry><cylinder>
          <radius>{d['wheel_r']:.6g}</radius><length>{d['wheel_w']:.6g}</length>
        </cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>""")

    return f"""<?xml version="1.0" ?>
<!-- 由 scripts/gen_dynamic_actors.py 从 config/dynamic_actors.yaml 与
     config/vehicle_params.yaml 生成 —— **不要手改**。改了下次 --check 就红。

     {title}。几何与自车**同源**（同一份 vehicle_params），
     区别只有三处，每一处都有理由：
       ① **没有任何传感器**   —— 多一个 32 线雷达在渲染，RTF 腰斩
       ② **没有转向/驱动关节** —— VelocityControl 直接推模型，不需要
       ③ **原点仍在后轴中心地面** —— 与自车同一个约定，真值换算才对得上
     ⚠️ 原点不是包围盒中心：包围盒中心在它前方 {d['chassis_center_x']:.3f} m。
        真值发布器负责这个换算（Obstacle.msg 要的是**包围盒中心**）。
        不换算的症状是一个恒定的纵向偏差被算成检测误差。 -->
<sdf version="1.9">
  <model name="{name}">
    <link name="{name}_base">
      <!-- ⚠️ **关掉重力。** 这一条是实测逼出来的（2026-08-11 探针）：
           VelocityControl 只设置线速度，物理引擎仍在算重力与接触力，于是
             · NPC 车以 −0.0098 m/s **恒速下沉**（穿透地面，5 s 沉 5 cm）；
             · 行人**直接翻倒卡死** —— 0.4×0.4 的底面配 1.7 m 高，
               高宽比 4.25，碰上任何接触扰动都站不住。实测它走了 1.44 m
               就躺下不动了，z 从 0 变成 0.20（躺倒后的半宽）。
           两个症状看起来毫不相干，根因是同一个。
           关掉重力之后模型不压在地面上，接触力消失，纯由 cmd_vel 驱动 ——
           这正是「道具」该有的行为：它是感知的**目标**，不是物理交互对象。

      ⚠️ **同时删掉了 <collision>，两件事必须一起做。**（2026-08-12 实测逼出来的）
         只关重力、留着 collision 的后果是一颗地雷：关掉重力**也关掉了受扰后
         回到地面的唯一机制**，于是任何一次接触给的角速度都**永不衰减** ——
         而 VelocityControl 是沿模型**自己的车体 x 轴**推它的，俯仰角一旦不为零，
         道具就斜着往上飞，速度恰好是 v·sin(pitch)。
         实测：两个 NPC 在 t=46 s 相撞，此后俯仰角以 1.48°/s 匀速增长、
         高度按 t² 上升，20 s 后飞到 20 m 高。而 CP-P5-B 的现场症状是
         **「检测率低、分类错、位置误差大」** —— 三条判据同时红，指向感知，
         而错在道具。查了两轮才找到（见 docs/modules/perception.md §8）。
         `dynamic:=oncoming`（只有一个道具、不可能相撞）下 50 s 全程 z=0.000，
         这是那次的对照组。

      ⚠️ 雷达仍然打得到它：gpu_lidar 走的是**渲染**（visual），不是 collision。
         这一条不是推理，是删 collision 之后**重新实测过**的（点云里目标仍在）。
      ⚠️ 代价：自车会从道具身上**穿过去**。可以接受 —— CP-P5-B 量的是感知质量，
         「撞没撞上」由记录脚本按真值算最小间距来判，本来就不依赖物理引擎。 -->
      <gravity>false</gravity>
      <inertial>
        <pose>{d['chassis_center_x']:.6g} 0 {d['com_h']:.6g} 0 0 0</pose>
        <mass>{mass:.6g}</mass>
        <inertia><ixx>{ixx:.6g}</ixx><iyy>{iyy:.6g}</iyy><izz>{izz:.6g}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>

      <!-- 车身。用**整车外廓宽度** width_m 而不是 chassis_width：
           自车模型那边取「轮距 − 轮宽」是为了调试时能看见前轮转向，
           而这里没有转向可看，用外廓宽度让点云的宽度与真值尺寸一致 ——
           感知的「尺寸误差」判据量的正是这个。 -->
      <visual name="body">
        <pose>{d['chassis_center_x']:.6g} 0 {d['chassis_center_z']:.6g} 0 0 0</pose>
        <geometry><box>
          <size>{d['length']:.6g} {geo['width_m']:.6g} {d['chassis_len_z']:.6g}</size>
        </box></geometry>
        <material><ambient>0.2 0.3 0.7 1</ambient><diffuse>0.2 0.3 0.7 1</diffuse></material>
      </visual>
      <!-- ⚠️ **没有 <collision>，这是有意的。** 见上面那段关于重力的注释的第二半。 -->

{chr(10).join(wheels)}
    </link>
{_plugins(name)}
  </model>
</sdf>
"""


def render_pedestrian(actor: dict) -> str:
    """Render the pedestrian model — a box, deliberately.

    为什么是盒子而不是 Gazebo 的 `<actor>`，见 dynamic_actors.yaml 顶部
    那三条实测理由（不带 skin 的 actor 会让 Gazebo segfault）。

    :param actor: dynamic_actors.yaml 里 pedestrian 的定义
    :return: model.sdf 的内容
    """
    lx = float(actor['length_m'])
    wy = float(actor['width_m'])
    hz = float(actor['height_m'])
    mass = 70.0  # 成年人量级。VelocityControl 下质量不影响运动，只影响物理稳定性。
    ixx, iyy, izz = _box_inertia(mass, lx, wy, hz)

    return f"""<?xml version="1.0" ?>
<!-- 由 scripts/gen_dynamic_actors.py 从 config/dynamic_actors.yaml 生成 —— **不要手改**。

     行人。它是**一个盒子**，不是 Gazebo 的 <actor> —— 三条实测理由写在
     config/dynamic_actors.yaml 顶部，最硬的一条是：不带 skin 的 actor 会让
     SceneBroadcaster 在发布位姿时 segfault（2026-08-11 对照实验）。

     ⚠️ 代价不只是观感：真人形的点云比盒子**更稀疏、更不规则**（四肢分开、
        中间有空隙），欧式聚类有可能把一个人分成两簇。盒子不会出现这个失效
        模式，所以本阶段的检测率判据是**偏乐观的**。那一条等 P8 的 CARLA。

     ⚠️ 原点在**底面中心**（z=0），不是几何中心。与 NPC 车「原点在后轴中心
        地面」同一个精神：所有目标的原点都贴地，真值换算只有一条竖直偏移。 -->
<sdf version="1.9">
  <model name="pedestrian">
    <link name="pedestrian_base">
      <!-- ⚠️ **关掉重力。** 这一条是实测逼出来的（2026-08-11 探针）：
           VelocityControl 只设置线速度，物理引擎仍在算重力与接触力，于是
             · NPC 车以 −0.0098 m/s **恒速下沉**（穿透地面，5 s 沉 5 cm）；
             · 行人**直接翻倒卡死** —— 0.4×0.4 的底面配 1.7 m 高，
               高宽比 4.25，碰上任何接触扰动都站不住。实测它走了 1.44 m
               就躺下不动了，z 从 0 变成 0.20（躺倒后的半宽）。
           两个症状看起来毫不相干，根因是同一个。
           关掉重力之后模型不压在地面上，接触力消失，纯由 cmd_vel 驱动 ——
           这正是「道具」该有的行为：它是感知的**目标**，不是物理交互对象。

      ⚠️ **同时删掉了 <collision>，两件事必须一起做。**（2026-08-12 实测逼出来的）
         只关重力、留着 collision 的后果是一颗地雷：关掉重力**也关掉了受扰后
         回到地面的唯一机制**，于是任何一次接触给的角速度都**永不衰减** ——
         而 VelocityControl 是沿模型**自己的车体 x 轴**推它的，俯仰角一旦不为零，
         道具就斜着往上飞，速度恰好是 v·sin(pitch)。
         实测：两个 NPC 在 t=46 s 相撞，此后俯仰角以 1.48°/s 匀速增长、
         高度按 t² 上升，20 s 后飞到 20 m 高。而 CP-P5-B 的现场症状是
         **「检测率低、分类错、位置误差大」** —— 三条判据同时红，指向感知，
         而错在道具。查了两轮才找到（见 docs/modules/perception.md §8）。
         `dynamic:=oncoming`（只有一个道具、不可能相撞）下 50 s 全程 z=0.000，
         这是那次的对照组。

      ⚠️ 雷达仍然打得到它：gpu_lidar 走的是**渲染**（visual），不是 collision。
         这一条不是推理，是删 collision 之后**重新实测过**的（点云里目标仍在）。
      ⚠️ 代价：自车会从道具身上**穿过去**。可以接受 —— CP-P5-B 量的是感知质量，
         「撞没撞上」由记录脚本按真值算最小间距来判，本来就不依赖物理引擎。 -->
      <gravity>false</gravity>
      <inertial>
        <pose>0 0 {hz / 2.0:.6g} 0 0 0</pose>
        <mass>{mass:.6g}</mass>
        <inertia><ixx>{ixx:.6g}</ixx><iyy>{iyy:.6g}</iyy><izz>{izz:.6g}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="body">
        <pose>0 0 {hz / 2.0:.6g} 0 0 0</pose>
        <geometry><box><size>{lx:.6g} {wy:.6g} {hz:.6g}</size></box></geometry>
        <material><ambient>0.8 0.4 0.1 1</ambient><diffuse>0.8 0.4 0.1 1</diffuse></material>
      </visual>
      <!-- ⚠️ **没有 <collision>，这是有意的。** 见上面那段关于重力的注释的第二半。 -->
    </link>
{_plugins('pedestrian')}
  </model>
</sdf>
"""


def render_config(name: str, description: str) -> str:
    """Render model.config — Gazebo needs it to resolve `model://`.

    :param name: 模型名
    :param description: 一句话说明
    :return: model.config 的内容
    """
    return f"""<?xml version="1.0"?>
<!-- 由 scripts/gen_dynamic_actors.py 生成 —— 不要手改。 -->
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{description}</description>
</model>
"""


# =============================================================================
def main() -> int:
    """Entry point.

    :return: 进程退出码
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='只比对，不写入（CI 用）')
    args = parser.parse_args()

    cfg = yaml.safe_load(ACTORS_YAML.read_text(encoding='utf-8'))
    vehicle = yaml.safe_load(VEHICLE_YAML.read_text(encoding='utf-8'))
    validate(cfg, vehicle)

    derived = _load_vehicle_model_module().derive(vehicle)

    outputs = {
        REPO / 'models' / 'npc_car' / 'model.sdf': render_npc_car(vehicle, derived),
        REPO / 'models' / 'npc_car' / 'model.config': render_config(
            'npc_car', 'P5 感知的动态目标：对向行驶的 NPC 车（与自车同源几何，无传感器）'),
        REPO / 'models' / 'pedestrian' / 'model.sdf': render_pedestrian(
            cfg['actors']['pedestrian']),
        REPO / 'models' / 'pedestrian' / 'model.config': render_config(
            'pedestrian', 'P5 感知的动态目标：行人（一个盒子，理由见 dynamic_actors.yaml）'),
        REPO / 'models' / 'curve_car' / 'model.sdf': render_npc_car(
            vehicle, derived, name='curve_car',
            title='curve_car：P6 预测的过弯激励 —— 沿内圈车道顺时针绕整圈，永不掉头'),
        REPO / 'models' / 'curve_car' / 'model.config': render_config(
            'curve_car', 'P6 预测的动态目标：沿内圈车道绕圈的车（过弯激励，理由见 dynamic_actors.yaml）'),
    }

    failed = False
    for path, content in outputs.items():
        rel = path.relative_to(REPO)
        if args.check:
            if not path.is_file() or path.read_text(encoding='utf-8') != content:
                print(f'✗ {rel} 与参数文件不同步 —— 请重跑 gen_dynamic_actors.py')
                failed = True
            else:
                print(f'✓ {rel} 与参数文件同步')
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding='utf-8')
            print(f'✓ 已生成 {rel}')

    if args.check and failed:
        return 1
    if not args.check:
        print(f'\n场景：{"、".join(cfg["scenarios"])}（launch 的 dynamic:= 参数）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
