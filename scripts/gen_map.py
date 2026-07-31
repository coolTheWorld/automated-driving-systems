#!/usr/bin/env python3
"""从 config/campus_map.yaml 生成园区地图 —— OpenDRIVE 的 .xodr 和 Gazebo 的道路模型。

为什么要生成而不是手写
----------------------
SPEC §4.1 把「地图单一来源」列为强制要求：两个仿真环境必须共用同一份 OpenDRIVE。

但「共用同一份 .xodr」只解决了一半问题。Gazebo **不认识 OpenDRIVE** ——
它需要一份三维的路面几何才能把路画出来。于是路面几何和车道语义天然是两份东西，
一旦各写各的就会漂移。漂移的症状特别阴险：

    路由算得出来、RViz 画得出来、车也开得动，
    **但车沿着一条肉眼看不见的车道压着绿化带走**，全程没有任何模块报错。

所以两份产物都从同一个 YAML 生成：

    maps/campus.xodr              → ads_map 解析 → 车道图 → 路由
                                  → CARLA 的 generate_opendrive_world()（P0b）
    models/campus_road/model.sdf  → Gazebo 把路画出来

这与 gen_vehicle_model.py（vehicle_params.yaml → SDF + URDF）是同一套结构。
两者防的是同一类 bug，只是一个防车、一个防路。

关于「手写路口」为什么不可接受
------------------------------
OpenDRIVE 的路口不是一个点，是一块区域：每条腿在离路口中心 cutback 处断开，
区域内部由若干条**连接道路**填充，每条对应一个转向动作。3 条腿的 T 型路口
有 6 条连接道路（直行 ×2 + 转弯 ×4），每条都要手工算圆弧的切点、曲率、
起始朝向，再手填 laneLink。

一个数填错的症状是：路由照样算得出来，但车经过**那一个**路口时拐进绿化带。
它只在经过那个路口时发作，而且看起来像控制问题。
让程序算，错就每次都错 —— 而每次都错的东西，测试抓得住。

用法
----
    python3 scripts/gen_map.py            # 生成
    python3 scripts/gen_map.py --check    # 只校验是否与 YAML 同步（供 CI 用）
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
PARAMS_FILE = REPO_ROOT / "config" / "campus_map.yaml"
XODR_FILE = REPO_ROOT / "maps" / "campus.xodr"
# 只生成 model.sdf；同目录的 model.config 是手写的元数据，不含任何几何，
# 与 models/ego_vehicle/ 的分工完全一致。
ROAD_SDF_FILE = REPO_ROOT / "models" / "campus_road" / "model.sdf"

# 浮点输出精度。两个用途：
#   1. .xodr 和 SDF 里的坐标 —— 6 位小数 = 微米级，远超任何仿真需要；
#   2. --check 是**逐字节**比对，所以格式必须完全确定性，不能依赖 repr()。
FMT = "%.6f"


def num(v: float) -> str:
    """把浮点数格式化成确定性的字符串.

    专门处理 -0.0：Python 里 -0.0 会打印成 "-0.000000"，
    而同一个几何量算法路径稍变就可能在 +0.0 / -0.0 之间跳，
    于是 --check 会报「不同步」而实际几何一模一样。
    """
    if abs(v) < 5e-7:
        v = 0.0
    return FMT % v


# =============================================================================
#  第一层：几何内核
#
#  OpenDRIVE 的参考线是一串首尾相接的几何记录，每条记录自带起点位姿。
#  本项目只支持 line 和 arc 两种（curvature == 0 即直线），
#  这是**有意的子集**：地图由我们自己生成，用到哪些原语完全可控。
#
#  ⚠️ ads_map 的 C++ 解析器遇到 spiral / poly3 / paramPoly3 必须**显式报错**，
#     不能静默跳过 —— 静默跳过的症状是路网少了一段而无人知晓。
# =============================================================================


@dataclass(frozen=True)
class Geom:
    """参考线上的一段几何。字段与 OpenDRIVE 的 <geometry> 一一对应。"""

    s0: float          # 该段起点在整条参考线上的弧长坐标
    x: float           # 起点世界坐标
    y: float
    hdg: float         # 起点朝向（弧度，ENU，x 轴为 0，逆时针为正）
    length: float      # 该段弧长
    curvature: float   # 曲率 1/R，正 = 左转（逆时针）；0 = 直线

    def pose_at(self, ds: float) -> tuple[float, float, float]:
        """求该段起点后 ds 处的位姿 (x, y, heading).

        直线：平凡。
        圆弧：由 dx/ds = cos(h + k·s)、dy/ds = sin(h + k·s) 积分而来 ——

            x(ds) = x₀ + [sin(h₀ + k·ds) − sin(h₀)] / k
            y(ds) = y₀ − [cos(h₀ + k·ds) − cos(h₀)] / k
            h(ds) = h₀ + k·ds

        这个闭式解对 k 的正负都成立，不需要分左右转两种写法。
        **k → 0 时它是 0/0 型**，所以直线必须单独走一支，不能靠 k 取很小来近似 ——
        那会在 s 大的时候产生肉眼可见的偏差。
        """
        if self.curvature == 0.0:
            return (self.x + ds * math.cos(self.hdg),
                    self.y + ds * math.sin(self.hdg),
                    self.hdg)
        k = self.curvature
        h1 = self.hdg + k * ds
        return (self.x + (math.sin(h1) - math.sin(self.hdg)) / k,
                self.y - (math.cos(h1) - math.cos(self.hdg)) / k,
                h1)

    def end_pose(self) -> tuple[float, float, float]:
        """该段终点位姿。"""
        return self.pose_at(self.length)


class GeomBuilder:
    """按「从当前位姿继续往前修路」的方式累积几何记录。

    这样写的好处是**接续关系由构造过程保证**：每一段的起点就是上一段的终点，
    不可能出现两段之间差几厘米的缝。手写 .xodr 时这类缝是最常见的错误，
    而它在 RViz 里根本看不出来 —— 路看着是连的，但车道中心线在缝处会跳变。
    """

    def __init__(self, x: float, y: float, hdg: float) -> None:
        self._x, self._y, self._hdg = x, y, hdg
        self._s = 0.0
        self.geoms: list[Geom] = []

    def line(self, length: float) -> "GeomBuilder":
        if length <= 0.0:
            raise ValueError(f"直线段长度必须为正，得到 {length}")
        self._append(length, 0.0)
        return self

    def arc(self, radius: float, angle_rad: float) -> "GeomBuilder":
        """接一段圆弧。angle_rad > 0 为左转（逆时针），< 0 为右转。"""
        if radius <= 0.0:
            raise ValueError(f"圆弧半径必须为正，得到 {radius}")
        if angle_rad == 0.0:
            raise ValueError("圆弧转角为 0，应该用 line()")
        # 曲率的符号来自转角，半径只提供大小 —— 把两者分开是为了让调用处读起来
        # 是「以 12 m 半径左转 90°」这种人话，而不是「曲率 +0.0833」。
        self._append(abs(angle_rad) * radius, math.copysign(1.0 / radius, angle_rad))
        return self

    def _append(self, length: float, curvature: float) -> None:
        g = Geom(self._s, self._x, self._y, self._hdg, length, curvature)
        self.geoms.append(g)
        self._x, self._y, self._hdg = g.end_pose()
        self._s += length

    @property
    def length(self) -> float:
        return self._s

    @property
    def pose(self) -> tuple[float, float, float]:
        return (self._x, self._y, self._hdg)


def unit(hdg: float) -> tuple[float, float]:
    """朝向角 → 单位向量。"""
    return (math.cos(hdg), math.sin(hdg))


def normalize_angle(a: float) -> float:
    """把角度归一化到 (-π, π]。

    用 math.remainder 而不是手写 while 循环：后者在输入很大时要转很多圈，
    且浮点误差会累积。ads_common 的 C++ 版本用的是同一个思路（std::remainder）。
    """
    return math.remainder(a, 2.0 * math.pi)


def intersect(p1: tuple[float, float], d1: tuple[float, float],
              p2: tuple[float, float], d2: tuple[float, float]) -> tuple[float, float]:
    """两条参数直线的交点。p + t·d 形式，d 为单位方向。

    用于求路口转弯圆弧的**圆心**：圆心必然同时落在「进入车道线向内偏移 R」
    和「离开车道线向内偏移 R」这两条平行偏移线上。

    平行（det ≈ 0）意味着调用方要求了一个掉头动作 —— 那没有有限半径的圆弧解，
    直接抛异常而不是返回一个巨大的数。
    """
    det = d2[0] * d1[1] - d1[0] * d2[1]
    if abs(det) < 1e-9:
        raise ValueError("两条直线平行，无法求交点（是不是要求了掉头动作？）")
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    t = (d2[0] * dy - dx * d2[1]) / det
    return (p1[0] + t * d1[0], p1[1] + t * d1[1])


# =============================================================================
#  第二层：路网数据模型
#
#  字段命名刻意贴近 OpenDRIVE 的元素名，这样第三层渲染 XML 时几乎是逐字段搬运，
#  中间不做任何「聪明」的转换 —— 转换正是错误藏身的地方。
# =============================================================================


@dataclass
class Road:
    """一条道路。常规路和路口内的连接道路都用它。"""

    rid: int
    name: str
    geoms: list[Geom]
    length: float
    # 链接。elem_type 取 "road" / "junction"；contact 取 "start" / "end"。
    predecessor: tuple[str, int, str] | None = None
    successor: tuple[str, int, str] | None = None
    # 所属路口 id；-1 表示这不是连接道路（OpenDRIVE 规定用 -1 而不是省略）
    junction: int = -1
    # 车道：双向路给 [+1, -1]，单向的连接道路只给 [-1]
    lane_ids: tuple[int, ...] = (1, -1)
    # 连接道路的车道级链接：lane -1 在前驱/后继road上分别接哪条车道
    lane_predecessor: int | None = None
    lane_successor: int | None = None


@dataclass
class Leg:
    """路口的一条「腿」：某条常规道路接在路口上的那一端。"""

    road: Road
    contact: str        # "start" | "end" —— 该道路的哪一端贴着路口
    x: float            # 路口边界点（= 该道路对应端点）的世界坐标
    y: float
    hdg_out: float      # 由路口中心指向外侧的朝向

    @property
    def inbound_lane(self) -> int:
        """这条腿上「驶向路口」的车道 id。

        右侧通行下 lane −1 沿 s 增大方向行驶：
          · 道路以 **end** 贴路口 → 沿 +s 走就是驶向路口 → lane −1
          · 道路以 **start** 贴路口 → 驶向路口是逆 s 方向 → lane +1
        """
        return -1 if self.contact == "end" else 1

    @property
    def outbound_lane(self) -> int:
        """这条腿上「驶离路口」的车道 id。恒与 inbound_lane 相反。"""
        return 1 if self.contact == "end" else -1


@dataclass
class Connection:
    """路口内的一条连接关系（一个转向动作）。"""

    incoming_road: int
    connecting_road: int
    contact_point: str        # 连接道路的哪一端贴着 incoming_road
    lane_from: int
    lane_to: int


@dataclass
class Junction:
    jid: int
    name: str
    x: float                  # 路口中心（仅用于生成 Gazebo 的路面块）
    y: float
    connections: list[Connection] = field(default_factory=list)
    legs: list[Leg] = field(default_factory=list)


@dataclass
class Network:
    roads: list[Road] = field(default_factory=list)
    junctions: list[Junction] = field(default_factory=list)


# =============================================================================
#  第三层：由 YAML 推导 + 构建路网
# =============================================================================


def derive(p: dict) -> dict:
    """由 YAML 原始参数推导出建模需要的中间量。

    与 gen_vehicle_model.py 的 derive() 是同一个角色：**把约束做成推导关系，
    而不是让人在配置文件里填两个必须互相匹配的数。**
    """
    lanes, loop, jun = p["lanes"], p["loop"], p["junction"]
    half_lane = lanes["width_m"] / 2.0

    # 路口每条腿从路口中心退多远。
    #
    # 下界推导：转弯的**参考线**圆弧必须与两条腿的参考线相切，切点落在路口区域内。
    # 转弯半径 turn_radius 定义在**车道中心线**上（那才是车实际走的轨迹），
    # 而参考线在车道中心线左侧 half_lane 处，于是：
    #     左转（曲率 > 0，圆心在左）：车道中心在圆心外侧 → R_ref = R_lane − half_lane
    #     右转（曲率 < 0，圆心在右）：车道中心在圆心内侧 → R_ref = R_lane + half_lane
    # 右转那支更大，所以切点最远处 = R_lane + half_lane，cutback 必须 ≥ 它。
    # 再留 clearance 的余量，免得生成出长度只有几厘米的几何记录。
    cutback = jun["turn_radius_m"] + half_lane + jun["clearance_m"]

    # 环线中心线包络的半宽/半高，以及直线段长度。
    half_len, half_wid = loop["length_m"] / 2.0, loop["width_m"] / 2.0
    r = loop["corner_radius_m"]
    if r >= min(half_len, half_wid):
        raise ValueError(f"转弯半径 {r} 过大，环线放不下（需 < {min(half_len, half_wid)}）")

    return {
        "half_lane_m": half_lane,
        "cutback_m": cutback,
        # 单侧直线段（从路口边界到圆弧起点）；横穿路在 x_m 处把上下两条边各切成两半
        "loop_straight_x_m": half_len - r - cutback,
        "loop_straight_y_m": 2.0 * (half_wid - r),
        "corner_arc_len_m": r * math.pi / 2.0,
        "cross_len_m": loop["width_m"] - 2.0 * cutback,
        # 环线周长（用于自洽校验）：四条直边 + 四个圆弧
        "loop_perimeter_m": (2.0 * (loop["length_m"] - 2.0 * r)
                             + 2.0 * (loop["width_m"] - 2.0 * r)
                             + 2.0 * math.pi * r),
        # 路面视觉宽度：双向车道 + 两侧路肩
        "surface_width_m": (2.0 * lanes["count_per_direction"] * lanes["width_m"]
                            + 2.0 * p["visual"]["shoulder_m"]),
    }


def build_network(p: dict, d: dict) -> Network:
    """构建整张路网。

    拓扑是固定的「圆角矩形环线 + 一条横穿路 = 2 个 T 型路口」，
    但**几何全部由参数推导**，改 YAML 里的尺寸即可整体缩放。

    ⚠️ 这个函数是拓扑专用的，不是通用的路网描述语言。
       将来要造别的地图（比如带停车场），做法是再写一个 build_xxx()，
       复用下面的 t_junction() 和整个几何内核 —— 而不是把本函数改成万能的。
       现在就做成通用的属于臆测需求：真正通用的路网描述语言就是 OpenDRIVE 本身。
    """
    loop, cross = p["loop"], p["cross_road"]
    r = loop["corner_radius_m"]
    cut = d["cutback_m"]
    quarter = math.pi / 2.0

    half_len, half_wid = loop["length_m"] / 2.0, loop["width_m"] / 2.0
    cx = cross["x_m"]

    # ---- 两条常规环线道路 ----------------------------------------------
    # 环线整体按**逆时针**走 s：下边向东、右边向北、上边向西、左边向南。
    # 于是 lane −1（沿 s 行驶）在环线**外侧**，lane +1 在内侧。
    # 统一一个绕行方向是有意的：混着来的话，某条路的 lane −1 在外侧、
    # 另一条在内侧，车道图接边时极易接反，而接反的症状是路由让车瞬间横移一个车道宽。
    #
    # loop_east：从下方路口向东出发，绕右半圈，回到上方路口。
    east = GeomBuilder(cx + cut, -half_wid, 0.0)
    east.line(half_len - r - (cx + cut))       # 下边东段
    east.arc(r, quarter)                       # 右下角
    east.line(d["loop_straight_y_m"])          # 右边
    east.arc(r, quarter)                       # 右上角
    east.line(half_len - r - (cx + cut))       # 上边东段（此时朝向 −x）
    road_east = Road(1, "loop_east", east.geoms, east.length)

    # loop_west：从上方路口向西出发，绕左半圈，回到下方路口。
    west = GeomBuilder(cx - cut, half_wid, math.pi)
    west.line((cx - cut) - (-half_len + r))    # 上边西段
    west.arc(r, quarter)                       # 左上角
    west.line(d["loop_straight_y_m"])          # 左边
    west.arc(r, quarter)                       # 左下角
    west.line((cx - cut) - (-half_len + r))    # 下边西段（此时朝向 +x）
    road_west = Road(2, "loop_west", west.geoms, west.length)

    # ---- 横穿路 --------------------------------------------------------
    # 由下方路口向北到上方路口。
    cross_b = GeomBuilder(cx, -half_wid + cut, quarter)
    cross_b.line(d["cross_len_m"])
    road_cross = Road(3, "cross", cross_b.geoms, cross_b.length)

    net = Network(roads=[road_east, road_west, road_cross])

    # ---- 两个 T 型路口 --------------------------------------------------
    # 腿的定义规则（对所有路口一致）：
    #   contact "start" → 边界点 = 道路起点，hdg_out = 起点朝向
    #   contact "end"   → 边界点 = 道路终点，hdg_out = 终点朝向 + π
    # hdg_out 一律指向**远离路口中心**的方向，这样连接道路的推导对三条腿完全对称。
    j_bot = Junction(100, "j_south", cx, -half_wid)
    j_bot.legs = [
        _leg(road_east, "start"),
        _leg(road_west, "end"),
        _leg(road_cross, "start"),
    ]
    j_top = Junction(101, "j_north", cx, half_wid)
    j_top.legs = [
        _leg(road_east, "end"),
        _leg(road_west, "start"),
        _leg(road_cross, "end"),
    ]

    # 常规道路两端都接在路口上，所以链接的 elementType 是 junction。
    # 车道级的链接由路口的 laneLink 负责，道路本身不需要写 lane <link>。
    road_east.predecessor = ("junction", j_bot.jid, "")
    road_east.successor = ("junction", j_top.jid, "")
    road_west.predecessor = ("junction", j_top.jid, "")
    road_west.successor = ("junction", j_bot.jid, "")
    road_cross.predecessor = ("junction", j_bot.jid, "")
    road_cross.successor = ("junction", j_top.jid, "")

    next_rid = 10
    for junction in (j_bot, j_top):
        next_rid = t_junction(net, junction, p, d, next_rid)
        net.junctions.append(junction)

    return net


def _leg(road: Road, contact: str) -> Leg:
    """由「道路 + 哪一端」构造一条路口腿。"""
    if contact == "start":
        g = road.geoms[0]
        return Leg(road, contact, g.x, g.y, g.hdg)
    x, y, hdg = road.geoms[-1].end_pose()
    return Leg(road, contact, x, y, normalize_angle(hdg + math.pi))


def t_junction(net: Network, jun: Junction, p: dict, d: dict, next_rid: int) -> int:
    """为一个路口生成全部连接道路（每个有序腿对一条），返回下一个可用的道路 id。

    n 条腿 → n·(n−1) 条连接道路（排除掉头）。T 型路口 3 条腿 → 6 条。
    """
    for a in jun.legs:
        for b in jun.legs:
            if a is b:
                continue      # 同一条腿进出 = 掉头，本项目不建模
            road = _connecting_road(a, b, p, d, next_rid, jun.jid)
            net.roads.append(road)
            jun.connections.append(Connection(
                incoming_road=a.road.rid,
                connecting_road=road.rid,
                contact_point="start",     # 连接道路总是从 a 腿开始修
                lane_from=a.inbound_lane,
                lane_to=-1,                # 连接道路是单向的，只有 lane −1
            ))
            next_rid += 1
    return next_rid


def _connecting_road(a: Leg, b: Leg, p: dict, d: dict, rid: int, jid: int) -> Road:
    """构造从腿 a 驶向腿 b 的一条连接道路。

    关键点：连接道路的**参考线**从 a 的边界点连到 b 的边界点（都在道路中心线上），
    而不是从车道中心连到车道中心。这样它的 lane −1（在参考线右侧 half_lane 处）
    会自动落在两端正确的车道上 —— 因为右侧通行下，「驶入路口的车道」和
    「驶离路口的车道」本来就都在各自行驶方向的右侧。

    换句话说，**车道偏移这件事交给 OpenDRIVE 的车道机制去做，参考线只管连中心线。**
    自己去算车道中心的连线是绕远路，而且会在两端与相邻道路的车道对不齐。
    """
    hdg_in = normalize_angle(a.hdg_out + math.pi)     # 驶入路口的朝向
    hdg_out = b.hdg_out                                # 驶离路口的朝向
    turn = normalize_angle(hdg_out - hdg_in)

    p_a, p_b = (a.x, a.y), (b.x, b.y)
    dir_in, dir_out = unit(hdg_in), unit(hdg_out)

    builder = GeomBuilder(a.x, a.y, hdg_in)

    if abs(turn) < 1e-9:
        # 直行：两条腿共线，直接一段直线。
        builder.line(math.dist(p_a, p_b))
    else:
        # 转弯：参考线的圆弧半径由车道中心半径换算而来（推导见 derive()）。
        side = 1.0 if turn > 0.0 else -1.0
        r_ref = p["junction"]["turn_radius_m"] - side * d["half_lane_m"]
        if r_ref <= 0.0:
            raise ValueError(
                f"路口转弯半径 {p['junction']['turn_radius_m']} 小于车道半宽 "
                f"{d['half_lane_m']}，参考线半径会变成负数")

        # 圆心 = 两条「向内偏移 r_ref」的直线的交点。
        n_in = unit(hdg_in + side * math.pi / 2.0)
        n_out = unit(hdg_out + side * math.pi / 2.0)
        center = intersect((p_a[0] + r_ref * n_in[0], p_a[1] + r_ref * n_in[1]), dir_in,
                           (p_b[0] + r_ref * n_out[0], p_b[1] + r_ref * n_out[1]), dir_out)

        # 切点 = 圆心沿反法线投影回各自的直线。
        t_in = (center[0] - r_ref * n_in[0], center[1] - r_ref * n_in[1])
        t_out = (center[0] - r_ref * n_out[0], center[1] - r_ref * n_out[1])

        # 首尾两小段直线的长度。负数意味着切点落到了路口外面 ——
        # 那是 cutback 不够大，属于参数配错，必须炸而不是生成一条歪掉的路。
        lead = (t_in[0] - p_a[0]) * dir_in[0] + (t_in[1] - p_a[1]) * dir_in[1]
        tail = (p_b[0] - t_out[0]) * dir_out[0] + (p_b[1] - t_out[1]) * dir_out[1]
        eps = 1e-6
        if lead < -eps or tail < -eps:
            raise ValueError(
                f"路口 {jid} 的连接道路 {rid} 几何不成立（前导 {lead:.3f} m、"
                f"后随 {tail:.3f} m 出现负值）。cutback 太小，"
                f"请增大 junction.clearance_m 或减小 junction.turn_radius_m")

        if lead > eps:
            builder.line(lead)
        builder.arc(r_ref, turn)
        if tail > eps:
            builder.line(tail)

    return Road(
        rid=rid,
        name=f"j{jid}_{a.road.name}_to_{b.road.name}",
        geoms=builder.geoms,
        length=builder.length,
        predecessor=("road", a.road.rid, a.contact),
        successor=("road", b.road.rid, b.contact),
        junction=jid,
        lane_ids=(-1,),                       # 单向，只有一条行驶车道
        lane_predecessor=a.inbound_lane,
        lane_successor=b.outbound_lane,
    )


# =============================================================================
#  第四层之一：渲染 OpenDRIVE
# =============================================================================


def render_xodr(p: dict) -> str:
    """生成 maps/campus.xodr。"""
    d = derive(p)
    net = build_network(p, d)
    lanes, geo = p["lanes"], p["geo_origin"]

    out: list[str] = []
    out.append('<?xml version="1.0" encoding="UTF-8"?>')
    out.append("<!-- ===========================================================")
    out.append(f"     {p['name']} —— 园区 OpenDRIVE 地图")
    out.append("")
    out.append("     ⚠️ 本文件由 scripts/gen_map.py 从 config/campus_map.yaml 生成，")
    out.append("        请勿手改。改参数请改 YAML 后重新运行生成器。")
    out.append("")
    out.append("     两个仿真环境共用本文件（SPEC §4.1 强制要求）：")
    out.append("       · ads_map  解析它 → 车道图 → 路由")
    out.append("       · CARLA    generate_opendrive_world() 直接加载（P0b）")
    out.append("     =========================================================== -->")

    # header 的 north/south/east/west 是路网包围盒，给消费者做快速裁剪用。
    xs, ys = [], []
    for road in net.roads:
        for g in road.geoms:
            for ds in (0.0, g.length):
                x, y, _ = g.pose_at(ds)
                xs.append(x)
                ys.append(y)
    out.append('<OpenDRIVE>')
    out.append(f'  <header revMajor="1" revMinor="6" name="{p["name"]}" version="1.00"'
               f' date="generated" north="{num(max(ys))}" south="{num(min(ys))}"'
               f' east="{num(max(xs))}" west="{num(min(xs))}" vendor="ads">')
    # geoReference 把地图局部坐标 (0,0) 钉在一个真实经纬度上。
    # ⚠️ 必须与 worlds/*.sdf 的 <spherical_coordinates> 一致，否则两个仿真环境里
    #    同一个 x/y 会换算出不同经纬度，P4 的 GNSS 定位结果无法互比。
    out.append(f'    <geoReference><![CDATA[+proj=tmerc +lat_0={geo["latitude_deg"]}'
               f' +lon_0={geo["longitude_deg"]} +k=1 +x_0=0 +y_0=0'
               f' +datum=WGS84 +units=m +no_defs]]></geoReference>')
    out.append('  </header>')

    for road in net.roads:
        out.extend(_render_road(road, p, d))

    for jun in net.junctions:
        out.append(f'  <junction id="{jun.jid}" name="{jun.name}">')
        for i, c in enumerate(jun.connections):
            out.append(f'    <connection id="{i}" incomingRoad="{c.incoming_road}"'
                       f' connectingRoad="{c.connecting_road}"'
                       f' contactPoint="{c.contact_point}">')
            out.append(f'      <laneLink from="{c.lane_from}" to="{c.lane_to}"/>')
            out.append('    </connection>')
        out.append('  </junction>')

    out.append('</OpenDRIVE>')
    return "\n".join(out) + "\n"


def _render_road(road: Road, p: dict, d: dict) -> list[str]:
    """渲染一条 <road>。"""
    lanes = p["lanes"]
    out = [f'  <road name="{road.name}" length="{num(road.length)}" id="{road.rid}"'
           f' junction="{road.junction}">']

    if road.predecessor or road.successor:
        out.append('    <link>')
        for tag, link in (("predecessor", road.predecessor), ("successor", road.successor)):
            if link is None:
                continue
            elem_type, elem_id, contact = link
            attr = f' contactPoint="{contact}"' if contact else ""
            out.append(f'      <{tag} elementType="{elem_type}" elementId="{elem_id}"{attr}/>')
        out.append('    </link>')

    # <type> 携带限速。P1 不读它，第一个消费者是 P2 的速度规划。
    out.append('    <type s="0.0" type="town">')
    out.append(f'      <speed max="{num(lanes["speed_limit_mps"])}" unit="m/s"/>')
    out.append('    </type>')

    out.append('    <planView>')
    for g in road.geoms:
        out.append(f'      <geometry s="{num(g.s0)}" x="{num(g.x)}" y="{num(g.y)}"'
                   f' hdg="{num(g.hdg)}" length="{num(g.length)}">')
        if g.curvature == 0.0:
            out.append('        <line/>')
        else:
            out.append(f'        <arc curvature="{num(g.curvature)}"/>')
        out.append('      </geometry>')
    out.append('    </planView>')

    out.append('    <lanes>')
    out.append('      <laneSection s="0.0">')
    # 左侧车道（id > 0，逆 s 行驶）
    left = [i for i in road.lane_ids if i > 0]
    if left:
        out.append('        <left>')
        for lid in sorted(left, reverse=True):
            out.extend(_render_lane(lid, road, lanes, indent=10))
        out.append('        </left>')
    # 中心车道 id=0：宽度恒为 0，不可行驶，只承载分道线样式
    out.append('        <center>')
    out.append('          <lane id="0" type="none" level="false">')
    out.append('            <roadMark sOffset="0.0" type="broken" weight="standard"'
               ' color="standard" width="0.15" laneChange="both"/>')
    out.append('          </lane>')
    out.append('        </center>')
    # 右侧车道（id < 0，沿 s 行驶）
    right = [i for i in road.lane_ids if i < 0]
    if right:
        out.append('        <right>')
        for lid in sorted(right, reverse=True):
            out.extend(_render_lane(lid, road, lanes, indent=10))
        out.append('        </right>')
    out.append('      </laneSection>')
    out.append('    </lanes>')
    out.append('  </road>')
    return out


def _render_lane(lid: int, road: Road, lanes: dict, indent: int) -> list[str]:
    """渲染一条 <lane>。"""
    pad = " " * indent
    out = [f'{pad}<lane id="{lid}" type="driving" level="false">']
    # 车道级链接只有连接道路需要：常规道路两端接的是路口，
    # 由路口的 <laneLink> 负责，写在这里反而会出现两处定义。
    if road.junction >= 0 and lid == -1:
        out.append(f'{pad}  <link>')
        out.append(f'{pad}    <predecessor id="{road.lane_predecessor}"/>')
        out.append(f'{pad}    <successor id="{road.lane_successor}"/>')
        out.append(f'{pad}  </link>')
    else:
        out.append(f'{pad}  <link/>')
    # 车道宽是 ds 的三次多项式 w(ds) = a + b·ds + c·ds² + d·ds³。
    # 等宽车道只需 a，其余为 0。变宽车道（展宽段、渐变段）才用得上高次项。
    out.append(f'{pad}  <width sOffset="0.0" a="{num(lanes["width_m"])}"'
               ' b="0.0" c="0.0" d="0.0"/>')
    out.append(f'{pad}  <roadMark sOffset="0.0" type="solid" weight="standard"'
               ' color="standard" width="0.12" laneChange="none"/>')
    out.append(f'{pad}</lane>')
    return out


# =============================================================================
#  第四层之二：渲染 Gazebo 道路模型
#
#  Gazebo 不认识 OpenDRIVE，需要真实的三维几何。这里把每段参考线摊成一串
#  贴地的薄盒子。圆弧按 arc_segments_per_quarter 分段用直盒子近似。
#
#  ⚠️ 分段数**只影响画面**。ads_map 用的是 .xodr 里的解析圆弧，不受它影响。
#     所以「路看起来有点棱角」永远不是算法问题。
# =============================================================================


def _geom_boxes(g: Geom, segs_per_quarter: int) -> list[tuple[float, float, float, float]]:
    """把一段几何摊成若干个 (中心x, 中心y, yaw, 长度) 的盒子。

    直线一个盒子搞定；圆弧按转角比例分段，每段用**弦**代替弧。
    弦与弧的最大偏差（矢高）= R(1 − cos(θ/2))，θ 为单段转角。
    默认参数下 R=12 m、θ=7.5° → 偏差 2.6 cm，肉眼不可见。
    """
    if g.curvature == 0.0:
        n = 1
    else:
        turn = abs(g.length * g.curvature)          # 该段总转角（弧度）
        n = max(1, math.ceil(turn / (math.pi / 2.0) * segs_per_quarter))

    boxes = []
    for i in range(n):
        x0, y0, _ = g.pose_at(g.length * i / n)
        x1, y1, _ = g.pose_at(g.length * (i + 1) / n)
        boxes.append(((x0 + x1) / 2.0, (y0 + y1) / 2.0,
                      math.atan2(y1 - y0, x1 - x0), math.hypot(x1 - x0, y1 - y0)))
    return boxes


def render_road_sdf(p: dict) -> str:
    """生成 models/campus_road/model.sdf。"""
    d = derive(p)
    net = build_network(p, d)
    vis = p["visual"]
    z = vis["elevation_m"]
    th = vis["thickness_m"]
    width = d["surface_width_m"]

    out: list[str] = []
    out.append('<?xml version="1.0" ?>')
    out.append("<!-- ===========================================================")
    out.append("     campus_road —— 园区道路的 Gazebo 可视几何")
    out.append("")
    out.append("     ⚠️ 本文件由 scripts/gen_map.py 从 config/campus_map.yaml 生成，")
    out.append("        请勿手改。它与 maps/campus.xodr 同源，两者不会漂移。")
    out.append("")
    out.append("     只有 <visual> 没有 <collision>：车在 ground_plane 上跑，")
    out.append("     再给路面加一层碰撞体是白算一份接触力，且两层几乎重合的")
    out.append("     碰撞面会让求解器抖动。沿用 P0a 的做法。")
    out.append("     =========================================================== -->")
    out.append('<sdf version="1.9">')
    out.append('  <model name="campus_road">')
    out.append('    <static>true</static>')
    out.append('    <link name="link">')

    def box(name: str, x: float, y: float, zz: float, yaw: float,
            dx: float, dy: float, dz: float, mat: str) -> None:
        out.append(f'      <visual name="{name}">')
        out.append(f'        <pose>{num(x)} {num(y)} {num(zz)} 0 0 {num(yaw)}</pose>')
        out.append(f'        <geometry><box><size>{num(dx)} {num(dy)} {num(dz)}'
                   '</size></box></geometry>')
        out.append(f'        <material>{mat}</material>')
        out.append('      </visual>')

    asphalt = ('<ambient>0.22 0.22 0.24 1</ambient>'
               '<diffuse>0.28 0.28 0.30 1</diffuse>')
    paint = ('<ambient>0.85 0.85 0.80 1</ambient>'
             '<diffuse>0.95 0.95 0.90 1</diffuse>')

    # ---- 常规道路：路面 + 中心分道线 -----------------------------------
    # 连接道路（junction >= 0）不单独画：6 条连接道路在路口内互相交叠，
    # 各画一条会产生大量共面重叠面，在 GPU 上表现为 z-fighting（闪烁条纹）。
    # 路口区域改为整块铺装，见下。
    n = 0
    for road in net.roads:
        if road.junction >= 0:
            continue
        for g in road.geoms:
            for bx, by, yaw, length in _geom_boxes(g, p["visual"]["arc_segments_per_quarter"]):
                box(f"surface_{n}", bx, by, z, yaw, length, width, th, asphalt)
                # 分道线画在参考线正上方（双向车道之间），抬高 1 mm 压住路面。
                box(f"centerline_{n}", bx, by, z + 0.001, yaw,
                    length, vis["center_line_width_m"], th, paint)
                n += 1

    # ---- 路口铺装 -------------------------------------------------------
    # 取所有腿矩形的**轴对齐包围盒**，一整块画完。
    # 比按腿逐块画简单得多，也避免了重叠；多出来的边角在视觉上正是真实路口的样子。
    for jun in net.junctions:
        xs, ys = [], []
        for leg in jun.legs:
            dirx, diry = unit(leg.hdg_out)
            nx, ny = -diry, dirx                     # 腿的法向
            for t in (0.0, math.dist((jun.x, jun.y), (leg.x, leg.y))):
                for side in (-1.0, 1.0):
                    xs.append(jun.x + t * dirx + side * width / 2.0 * nx)
                    ys.append(jun.y + t * diry + side * width / 2.0 * ny)
        box(f"junction_{jun.jid}", (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0,
            z, 0.0, max(xs) - min(xs), max(ys) - min(ys), th, asphalt)

    out.append('    </link>')
    out.append('  </model>')
    out.append('</sdf>')
    return "\n".join(out) + "\n"


# =============================================================================
#  入口
# =============================================================================

# 输出物清单。--check 会逐个比对，任何一个与 YAML 不同步都算失败。
OUTPUTS = [
    (XODR_FILE, render_xodr),
    (ROAD_SDF_FILE, render_road_sdf),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="不写文件，只校验生成物是否与 YAML 同步；不同步则退出码 1")
    args = ap.parse_args()

    params = yaml.safe_load(PARAMS_FILE.read_text(encoding="utf-8"))

    if args.check:
        stale = 0
        for path, fn in OUTPUTS:
            rel = path.relative_to(REPO_ROOT)
            if not path.exists():
                print(f"✗ {rel} 不存在，请运行 scripts/gen_map.py 生成", file=sys.stderr)
                stale = 1
            elif path.read_text(encoding="utf-8") != fn(params):
                print(f"✗ {rel} 与 {PARAMS_FILE.relative_to(REPO_ROOT)} 不同步。\n"
                      f"  要么有人手改了生成物，要么改了 YAML 忘了重新生成。\n"
                      f"  运行：python3 scripts/gen_map.py", file=sys.stderr)
                stale = 1
            else:
                print(f"✓ {rel} 与参数文件同步")
        return stale

    for path, fn in OUTPUTS:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(fn(params), encoding="utf-8")
        print(f"已生成 {path.relative_to(REPO_ROOT)}")

    d = derive(params)
    net = build_network(params, d)
    normal = [r for r in net.roads if r.junction < 0]
    conn = [r for r in net.roads if r.junction >= 0]
    print(f"  环线 {params['loop']['length_m']}×{params['loop']['width_m']} m  "
          f"转弯半径 {params['loop']['corner_radius_m']} m  "
          f"周长 {d['loop_perimeter_m']:.3f} m")
    print(f"  常规道路 {len(normal)} 条（"
          + "、".join(f"{r.name} {r.length:.3f} m" for r in normal) + "）")
    print(f"  路口 {len(net.junctions)} 个，连接道路 {len(conn)} 条，"
          f"每腿退让 {d['cutback_m']:.3f} m")
    print(f"  车道宽 {params['lanes']['width_m']} m  "
          f"路面视觉宽 {d['surface_width_m']:.3f} m  "
          f"限速 {params['lanes']['speed_limit_mps']} m/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
