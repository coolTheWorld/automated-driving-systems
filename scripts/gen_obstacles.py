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

"""
Generate the Gazebo obstacle models used by the P3 acceptance scenarios.

从 config/obstacles.yaml 生成 models/campus_obstacles_<场景>/{model.sdf,model.config}。

    python3 scripts/gen_obstacles.py            # 生成
    python3 scripts/gen_obstacles.py --check    # 只校验是否同步（供 CI 用）

**只有 Gazebo 那一侧是生成物。** 真值发布器直接读同一个 YAML（由 launch 搬运成
ROS 参数），所以「车看到的」和「车会撞上的」根本不存在第二份数据 ——
这比 CLAUDE.md §3b 那种"两份生成物"更彻底。

⚠️ 本脚本还做一件比生成更重要的事：**机械校验 docs/modules/planning.md §6
   的可行性不等式**，并与 YAML 里声明的 expect 对账。对不上就拒绝生成。

   为什么值得这么做：那个不等式（障碍物左缘 ≤ W/2 − w − g）的余量在本项目
   只有零点几米。有人把障碍物挪 0.3 m 就可能让「可绕」场景变成几何上无解，
   而症状是**实测时车停住了，然后所有人去查规划器** —— 而错在场景设定。
   让脚本算，错就每次都错；而每次都错的东西，测试抓得住。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
OBSTACLES_YAML = REPO_ROOT / "config" / "obstacles.yaml"
MAP_YAML = REPO_ROOT / "config" / "campus_map.yaml"
VEHICLE_YAML = REPO_ROOT / "config" / "vehicle_params.yaml"
PLANNING_YAML = REPO_ROOT / "config" / "planning_params.yaml"
MODELS_DIR = REPO_ROOT / "models"


def precise(value: float) -> str:
    """
    Format a float deterministically for byte-exact --check comparison.

    用 %.12g 而不是 %.6f：见 gen_map.py 里同名函数的说明
    （同一个格式串套不同量纲是想当然，曲率那类小数值会被截掉有效位）。
    这里其实只有坐标，但保持两个生成器一致，免得有人以为它们的精度约定不同。

    :param value: 待格式化的浮点数
    :return: 确定性的字符串表示
    """
    return f"{value:.12g}"


def feasible_sampled_offsets(lateral_offset_m: float, width_m: float,
                             lane_width_m: float, vehicle_width_m: float,
                             safety_margin_m: float,
                             max_offset_m: float, offset_step_m: float) -> tuple[list[float], str]:
    """
    Find which of the planner's sampled lateral offsets can clear an obstacle.

    实现的是 docs/modules/planning.md §6 的不等式，**但判据比它更严一档**：
    不只问"连续意义上有没有解"，而是问"**规划器真会采样到的那些 d 里**有没有解"。

    ⚠️ 这一档是实测逼出来的。把障碍物从 −1.2 挪到 −0.8，连续不等式
       （o_l = −0.55 ≤ 阈值 −0.55）仍判"可绕"，但可行区间退化成**单点 d = 0.85**，
       而采样网格是 k×0.2、上限 0.85 ⟹ 最大只到 0.80，**一个候选都落不进去**。
       于是生成器说"可绕"、规划器报"绕不过去"，而人会去查规划器。

       连续判据是**必要不充分**的。要预测规划器的行为，就得按它的网格算。

    可行区间（d 轴左正）：
        从左绕：d ∈ [o_l + g + w/2,  W/2 − w/2]
        从右绕：d ∈ [−W/2 + w/2,     o_r − g − w/2]

    :param lateral_offset_m: 障碍物中心的横向偏移，左正
    :param width_m: 障碍物横向尺寸
    :param lane_width_m: 车道宽 W
    :param vehicle_width_m: 车宽 w
    :param safety_margin_m: 安全间距 g
    :param max_offset_m: 采样的 |d_T| 上限
    :param offset_step_m: 采样步长
    :return: (能绕开的采样偏移列表, 供打印的说明)
    """
    left_edge_m = lateral_offset_m + 0.5 * width_m
    right_edge_m = lateral_offset_m - 0.5 * width_m
    half_vehicle_m = 0.5 * vehicle_width_m
    lane_limit_m = 0.5 * lane_width_m - half_vehicle_m

    left_lo_m = left_edge_m + safety_margin_m + half_vehicle_m
    right_hi_m = right_edge_m - safety_margin_m - half_vehicle_m

    # 与 ads_planning::sample_offsets 完全一致：k×step，|k×step| ≤ max。
    # 用 k 取整数而不是从 −max 累加，是为了让 0 精确存在（同 lattice.cpp 的说明）。
    steps = int(math.floor(max_offset_m / offset_step_m))
    sampled = [k * offset_step_m for k in range(-steps, steps + 1)]

    good = [d for d in sampled
            if (left_lo_m - 1e-9 <= d <= lane_limit_m + 1e-9)
            or (-lane_limit_m - 1e-9 <= d <= right_hi_m + 1e-9)]

    detail = (f"左缘 {left_edge_m:+.3f} / 右缘 {right_edge_m:+.3f} m；"
              f"可行区间 左绕 [{left_lo_m:+.3f}, {lane_limit_m:+.3f}] "
              f"右绕 [{-lane_limit_m:+.3f}, {right_hi_m:+.3f}]；"
              f"采样网格里可用的 d_T：{[round(d, 3) for d in good] if good else '无'}")
    return good, detail


def build_model_sdf(scenario: str, spec: dict, lane: dict) -> str:
    """
    Render one scenario's obstacles as a static Gazebo model.

    :param scenario: 场景名
    :param spec: 该场景的配置（description / obstacles）
    :param lane: 车道参考（center_y_m / heading_rad / 有效 x 区间）
    :return: model.sdf 的完整内容
    """
    lines = [
        '<?xml version="1.0" ?>',
        "<!-- ===========================================================",
        f"     campus_obstacles_{scenario} —— P3 验收场景的静态障碍物",
        "",
        "     ⚠️ 本文件由 scripts/gen_obstacles.py 从 config/obstacles.yaml 生成，",
        "        请勿手改。真值发布器读的是**同一个 YAML**，所以",
        "        「车看到的」与「车会撞上的」不存在第二份数据。",
        "",
        f"     场景：{spec['description']}",
        "",
        "     有 <collision>（与 campus_road 不同）：障碍物是要真撞上的，",
        "     没有碰撞体的话 CP-P3-B 的「碰撞次数 = 0」这条判据就永远成立，",
        "     而那正是它最该抓的东西。",
        "     =========================================================== -->",
        '<sdf version="1.9">',
        f'  <model name="campus_obstacles_{scenario}">',
        "    <static>true</static>",
    ]

    heading_rad = float(lane["heading_rad"])
    center_y_m = float(lane["center_y_m"])
    # 车道系 → 世界系：沿 heading 前进 along，再沿**左**法向偏 lateral。
    # 本项目的南侧直道 heading = 0，所以这就是 (x, y0 + lateral)，
    # 但仍然写成一般形式 —— 写死的话换一条道就要改代码，而那时没人记得这里有假设。
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)

    for obstacle in spec["obstacles"]:
        along_m = float(obstacle["along_x_m"])
        lateral_m = float(obstacle["lateral_offset_m"])
        # 直道上 along_x_m 就是世界 x；一般情形下要沿参考线走弧长，
        # 那需要地图几何 —— 见文件头的边界说明。
        x_m = along_m
        y_m = center_y_m + lateral_m * cos_h
        # 上一行对 heading = 0 是精确的。非零 heading 需要真正的车道参考线，
        # 本脚本**不支持**，所以下面显式拒绝。
        if abs(sin_h) > 1e-9:
            raise SystemExit(
                "gen_obstacles: lane.heading_rad 目前只支持 0（南侧直道）。\n"
                "  非零朝向要沿参考线走弧长，那需要解析地图几何（ads_map 的活）。\n"
                "  与其在这里写一个只在直道上对的近似，不如明确拒绝。"
            )
        height_m = float(obstacle["height_m"])
        lines += [
            f'    <link name="{obstacle["name"]}">',
            f"      <pose>{precise(x_m)} {precise(y_m)} {precise(0.5 * height_m)}"
            f" 0 0 {precise(heading_rad)}</pose>",
            '      <collision name="collision">',
            f'        <geometry><box><size>{precise(float(obstacle["length_m"]))}'
            f' {precise(float(obstacle["width_m"]))} {precise(height_m)}</size></box></geometry>',
            "      </collision>",
            '      <visual name="visual">',
            f'        <geometry><box><size>{precise(float(obstacle["length_m"]))}'
            f' {precise(float(obstacle["width_m"]))} {precise(height_m)}</size></box></geometry>',
            # 橙色：交通锥的常规配色，在灰色路面上一眼能找到。
            "        <material><ambient>0.9 0.35 0.05 1</ambient>"
            "<diffuse>1.0 0.45 0.1 1</diffuse></material>",
            "      </visual>",
            "    </link>",
        ]

    lines += ["  </model>", "</sdf>", ""]
    return "\n".join(lines)


def build_model_config(scenario: str, spec: dict) -> str:
    """
    Render the Gazebo model metadata file.

    :param scenario: 场景名
    :param spec: 该场景的配置
    :return: model.config 的完整内容
    """
    return (
        '<?xml version="1.0"?>\n'
        "<!-- Gazebo 模型元数据。model.sdf 是生成物，本文件也由 gen_obstacles.py 生成\n"
        "     （它含场景描述，而描述在 YAML 里，手写就会漂移）。 -->\n"
        "<model>\n"
        f"  <name>campus_obstacles_{scenario}</name>\n"
        "  <version>1.0</version>\n"
        "  <sdf version=\"1.9\">model.sdf</sdf>\n"
        "  <description>\n"
        f"    {spec['description']}\n"
        "    由 scripts/gen_obstacles.py 从 config/obstacles.yaml 生成。\n"
        "  </description>\n"
        "</model>\n"
    )


def collect_outputs() -> list[tuple[Path, str]]:
    """
    Build every output file's content, validating the scenarios along the way.

    :return: (路径, 内容) 列表
    """
    config = yaml.safe_load(OBSTACLES_YAML.read_text(encoding="utf-8"))
    lane = config["lane"]

    lane_width_m = float(
        yaml.safe_load(MAP_YAML.read_text(encoding="utf-8"))["lanes"]["width_m"])
    vehicle_width_m = float(
        yaml.safe_load(VEHICLE_YAML.read_text(encoding="utf-8"))["geometry"]["width_m"])
    planning = yaml.safe_load(PLANNING_YAML.read_text(encoding="utf-8"))
    safety_margin_m = float(planning["safety"]["margin_m"])

    outputs: list[tuple[Path, str]] = []
    for scenario, spec in config["scenarios"].items():
        for obstacle in spec["obstacles"]:
            along_m = float(obstacle["along_x_m"])
            if not (float(lane["valid_from_x_m"]) <= along_m <= float(lane["valid_to_x_m"])):
                raise SystemExit(
                    f"gen_obstacles: 场景 {scenario} 的 {obstacle['name']} "
                    f"along_x_m = {along_m} 落在这条直道之外 "
                    f"[{lane['valid_from_x_m']}, {lane['valid_to_x_m']}]。\n"
                    "  放到路面外的障碍物在 RViz 里看着完全正常，只是车永远遇不到它，\n"
                    "  于是「绕障测试通过」变成一句空话。"
                )

        # -------------------------------------------------------------------
        #  机械校验 planning.md §6 的可行性不等式，并与 expect 对账
        # -------------------------------------------------------------------
        # 一个场景里有多个障碍物时，只要**任一**个绕不过去，整个场景就是 blocked。
        max_offset_m = float(planning["lateral"]["max_offset_m"])
        offset_step_m = float(planning["lateral"]["offset_step_m"])
        avoidable = True
        detail: list[str] = []
        for obstacle in spec["obstacles"]:
            good, text = feasible_sampled_offsets(
                float(obstacle["lateral_offset_m"]), float(obstacle["width_m"]),
                lane_width_m, vehicle_width_m, safety_margin_m,
                max_offset_m, offset_step_m)
            avoidable = avoidable and bool(good)
            detail.append(f"      {obstacle['name']}: {text}")

        expected_avoidable = spec["expect"] == "avoidable"
        if spec["expect"] not in ("avoidable", "blocked"):
            raise SystemExit(
                f"gen_obstacles: 场景 {scenario} 的 expect = {spec['expect']!r}，"
                "只允许 avoidable 或 blocked")
        if avoidable != expected_avoidable:
            raise SystemExit(
                f"gen_obstacles: 场景 {scenario} 声称 expect = {spec['expect']}，"
                f"但按 §6 的不等式 + 规划器的采样网格算出来是"
                f"{'可绕' if avoidable else '绕不过去'}。\n"
                + "\n".join(detail) + "\n"
                "  余量只有零点几米，挪 0.3 m 就可能翻过来。**不要改 expect 去迁就位置** ——\n"
                "  先想清楚这个场景到底要验什么，再改位置。"
            )
        print(f"  场景 {scenario}（expect={spec['expect']}）✓")
        for line in detail:
            print(line)

        directory = MODELS_DIR / f"campus_obstacles_{scenario}"
        outputs.append((directory / "model.sdf", build_model_sdf(scenario, spec, lane)))
        outputs.append((directory / "model.config", build_model_config(scenario, spec)))

    return outputs


def main() -> int:
    """
    Entry point.

    :return: 进程退出码
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="只校验生成物是否与 YAML 同步，不写文件（CI 用）")
    args = parser.parse_args()

    outputs = collect_outputs()

    if args.check:
        stale = []
        for path, content in outputs:
            if not path.exists() or path.read_text(encoding="utf-8") != content:
                stale.append(path)
        if stale:
            print("以下生成物与 config/obstacles.yaml 不同步：", file=sys.stderr)
            for path in stale:
                print(f"  {path.relative_to(REPO_ROOT)}", file=sys.stderr)
            print("请跑 python3 scripts/gen_obstacles.py", file=sys.stderr)
            return 1
        print("✓ 全部生成物与 config/obstacles.yaml 同步")
        return 0

    for path, content in outputs:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        print(f"  写入 {path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
