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
#  scripts/gen_map.py 的 L1 测试
#
#  为什么生成器必须有单元测试，而不是只有 --check
#  ---------------------------------------------
#  `--check` 只能证明「生成物与 YAML 同步」，证明不了「生成的几何是对的」——
#  **同步地错着也是同步**。gen_vehicle_model.py 目前就只有 --check，
#  这是一个已知缺口，不要在新生成器上重复它。
#
#  本文件测三类东西，覆盖三种不同的失效方式：
#
#    1. 几何内核对不对        → 与解析解比对（圆弧的闭式解、直线的平凡解）
#    2. 路网结构对不对        → 周长自洽、路口连接道路条数
#    3. **车道有没有接上**    → 路口两侧的车道中心点与朝向是否连续
#
#  第 3 类是本文件的核心。它防的是这样一种故障：
#  路由算得出来、RViz 画得出来、车也开得动，**但车经过某个路口时横移一个车道宽**，
#  而 .xodr 完全合法、没有任何模块报错。这种错误肉眼在 RViz 里看不出来
#  （一个车道宽的偏移在俯视图上就是一条稍微偏一点的线），
#  要等到 P2 车真的开上去、贴着路沿走，才会被发现。
# =============================================================================

"""Unit tests for the campus map generator in scripts/gen_map.py."""

import importlib.util
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

_REPO_ROOT = Path(__file__).resolve().parents[3]
_GEN_SCRIPT = _REPO_ROOT / 'scripts' / 'gen_map.py'

# 位置比对容差。所有坐标都出自同一批浮点运算，真正接上的话残差在 1e-12 量级；
# 取 1e-9 是给累积舍入留的余量，同时远小于任何有物理意义的偏差
# （最小的有意义偏差是「差一个车道宽」= 3.5 m，差了 9 个数量级）。
POS_TOL_M = 1e-9
ANG_TOL_RAD = 1e-9


def _load_gen_map():
    """
    Import scripts/gen_map.py by path.

    它不是一个包，没法 `import`；按路径加载是本仓库既有的做法
    （见 ads_bringup/test/test_sim_source.py）。

    ⚠️ 必须先把模块塞进 sys.modules 再 exec_module，顺序反了会炸在
    `@dataclass` 上，报 `AttributeError: 'NoneType' object has no attribute
    '__dict__'`。原因：gen_map.py 用了 `from __future__ import annotations`，
    于是类型注解全是字符串，dataclass 要回 `sys.modules[cls.__module__]`
    去解析它们 —— 而 module_from_spec 造出来的模块默认**不在** sys.modules 里。
    报错信息完全没提 sys.modules，很容易误以为是 dataclass 用错了。
    （test_sim_source.py 不需要这一步，因为 launch 文件里没有 dataclass。）
    """
    spec = importlib.util.spec_from_file_location('gen_map', _GEN_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


gen = _load_gen_map()


@pytest.fixture(scope='module')
def params():
    """Load config/campus_map.yaml once for the whole module."""
    return yaml.safe_load(gen.PARAMS_FILE.read_text(encoding='utf-8'))


@pytest.fixture(scope='module')
def network(params):
    """Build the road network once for the whole module."""
    return gen.build_network(params, gen.derive(params))


# =============================================================================
#  1. 几何内核：与解析解比对
# =============================================================================


def test_line_matches_analytic_solution():
    """A straight geometry advances along its heading and keeps it."""
    g = gen.Geom(s0=0.0, x=3.0, y=-4.0, hdg=math.radians(30.0), length=10.0, curvature=0.0)
    x, y, hdg = g.pose_at(10.0)
    assert x == pytest.approx(3.0 + 10.0 * math.cos(math.radians(30.0)), abs=POS_TOL_M)
    assert y == pytest.approx(-4.0 + 10.0 * math.sin(math.radians(30.0)), abs=POS_TOL_M)
    assert hdg == pytest.approx(math.radians(30.0), abs=ANG_TOL_RAD)


@pytest.mark.parametrize('sign', [1.0, -1.0])
def test_quarter_arc_matches_analytic_solution(sign):
    """
    A quarter arc from the origin lands on the circle's other axis.

    从原点朝 +x 出发、曲率 ±1/R 走过 R·π/2：
      左转（+）终点 (R, +R)，朝向 +90°；右转（−）终点 (R, −R)，朝向 −90°。
    这是圆弧闭式解最容易手推的特例 —— 一旦曲率符号或积分公式写反，这条立刻红。
    """
    radius = 12.0
    g = gen.Geom(s0=0.0, x=0.0, y=0.0, hdg=0.0,
                 length=radius * math.pi / 2.0, curvature=sign / radius)
    x, y, hdg = g.end_pose()
    assert x == pytest.approx(radius, abs=POS_TOL_M)
    assert y == pytest.approx(sign * radius, abs=POS_TOL_M)
    assert hdg == pytest.approx(sign * math.pi / 2.0, abs=ANG_TOL_RAD)


def test_arc_stays_on_its_circle():
    """
    Every point of an arc keeps a constant distance to the circle centre.

    这条比端点比对更强：端点对了不代表中间也对。圆心位于起点左侧（左转）
    半径处，弧上任意点到圆心的距离都应恒等于半径。
    """
    radius, hdg0 = 8.0, math.radians(37.0)
    g = gen.Geom(s0=0.0, x=1.0, y=2.0, hdg=hdg0,
                 length=radius * math.pi / 2.0, curvature=1.0 / radius)
    cx = 1.0 + radius * math.cos(hdg0 + math.pi / 2.0)
    cy = 2.0 + radius * math.sin(hdg0 + math.pi / 2.0)
    for i in range(21):
        x, y, _ = g.pose_at(g.length * i / 20.0)
        assert math.hypot(x - cx, y - cy) == pytest.approx(radius, abs=POS_TOL_M)


def test_builder_segments_are_continuous():
    """
    Consecutive geometries share an exact pose, leaving no seam.

    接缝是手写 .xodr 最常见的错误，而它在 RViz 里根本看不出来 ——
    路看着是连的，但车道中心线在接缝处跳变。GeomBuilder 的存在就是为了
    让接续关系由构造过程保证，本条是对那个保证的验证。
    """
    b = gen.GeomBuilder(0.0, 0.0, 0.0)
    b.line(10.0).arc(12.0, math.pi / 2.0).line(5.0).arc(8.0, -math.pi / 3.0)
    for prev, nxt in zip(b.geoms, b.geoms[1:]):
        px, py, phdg = prev.end_pose()
        assert px == pytest.approx(nxt.x, abs=POS_TOL_M)
        assert py == pytest.approx(nxt.y, abs=POS_TOL_M)
        assert gen.normalize_angle(phdg - nxt.hdg) == pytest.approx(0.0, abs=ANG_TOL_RAD)
        assert prev.s0 + prev.length == pytest.approx(nxt.s0, abs=POS_TOL_M)


# =============================================================================
#  2. 路网结构
# =============================================================================


def test_loop_perimeter_is_self_consistent(params, network):
    """
    Loop road lengths plus junction through-movements equal the derived perimeter.

    环线周长有两条独立的算法：
      (a) derive() 按圆角矩形的解析公式算：直边 + 2πR
      (b) 把两条环线道路的长度、加上两个路口的直行段长度加起来
    两者必须相等。**这条能抓住「圆弧接续算错了半径」** ——
    半径错了周长就对不上，而单看某一段几何是看不出来的。
    """
    d = gen.derive(params)
    loop_roads = [r for r in network.roads if r.name.startswith('loop_')]
    assert len(loop_roads) == 2

    # 路口内的直行连接道路长度 = 2 × cutback（两条腿各退让一次）
    through = 2.0 * d['cutback_m']
    total = sum(r.length for r in loop_roads) + len(network.junctions) * through
    assert total == pytest.approx(d['loop_perimeter_m'], abs=1e-9)


def test_each_junction_has_six_connecting_roads(network):
    """
    A three-leg junction produces exactly 6 one-way connecting roads.

    n 条腿两两有序配对、排除掉头 → n(n−1)。T 型路口 3 条腿 → 6 条
    （直行 2 条 + 转弯 4 条）。数目不对意味着有配对被漏掉或多算，
    症状是路由在这个路口少一种转向选择 —— 而 Dijkstra 会安静地绕远路。
    """
    for junction in network.junctions:
        assert len(junction.legs) == 3
        assert len(junction.connections) == 6
        connecting = [r for r in network.roads if r.junction == junction.jid]
        assert len(connecting) == 6


def test_no_connection_is_a_u_turn(network):
    """A connecting road never returns to the road it came from."""
    for junction in network.junctions:
        for conn in junction.connections:
            road = _road_by_id(network, conn.connecting_road)
            assert road.successor[1] != conn.incoming_road, (
                f'路口 {junction.jid} 的连接道路 {road.rid} 回到了来路 '
                f'{conn.incoming_road} —— 那是掉头动作，本项目不建模')


def test_connecting_roads_are_one_way(network):
    """
    Connecting roads carry a single driving lane.

    连接道路对应**一个**转向动作，必须是单向的。给它加一条反向车道，
    车道图就会多出一条谁都没打算提供的通路，Dijkstra 可能选中它，
    结果是车在路口里逆行。
    """
    for road in network.roads:
        if road.junction >= 0:
            assert road.lane_ids == (-1,)


# =============================================================================
#  3. 车道连续性 —— 本文件的核心
# =============================================================================


def _road_by_id(network, rid):
    """Look up a road by its OpenDRIVE id."""
    for road in network.roads:
        if road.rid == rid:
            return road
    raise AssertionError(f'路网里没有 id={rid} 的道路')


def _leg_of(junction, rid):
    """Find the leg of a junction that belongs to road ``rid``."""
    for leg in junction.legs:
        if leg.road.rid == rid:
            return leg
    raise AssertionError(f'路口 {junction.jid} 没有属于道路 {rid} 的腿')


def _lane_center(x, y, hdg, lane_id, lane_width):
    """
    Offset a reference-line pose sideways onto a lane centre.

    OpenDRIVE 的横向坐标 t 以参考线**左侧**为正。第 |id| 条车道的中心在
        t = sign(id) · (|id| − 0.5) · width
    于是 lane −1（右侧第一条）落在 −0.5·width，lane +1 落在 +0.5·width。
    """
    t = math.copysign((abs(lane_id) - 0.5) * lane_width, lane_id)
    return (x + t * math.cos(hdg + math.pi / 2.0),
            y + t * math.sin(hdg + math.pi / 2.0))


def _end_pose(road, contact):
    """Reference-line pose at whichever end of ``road`` touches the junction."""
    if contact == 'start':
        g = road.geoms[0]
        return (g.x, g.y, g.hdg)
    return road.geoms[-1].end_pose()


def test_lane_centres_are_continuous_across_junctions(params, network):
    """
    Lane centres and travel headings match on both sides of every junction.

    ⚠️ **这是本文件最重要的用例。**

    对每一条连接道路，检查两个接缝：
      入口 —— 来路的「驶入车道」中心点 vs 连接道路 lane −1 在 s=0 的中心点
      出口 —— 连接道路 lane −1 在 s=L 的中心点 vs 去路的「驶离车道」中心点

    位置和**行驶朝向**都要连续。只查位置是不够的：位置对、朝向差 180°
    意味着车要原地掉头才能接上，而那在几何上看起来完全正常。

    它能抓住的错误包括：车道 id 正负号搞反、inbound/outbound 判断写反、
    连接道路的参考线接到了车道中心线上（而不是道路中心线上）。
    这些错误全都产出合法的 .xodr，没有任何工具会报错。
    """
    lane_width = params['lanes']['width_m']

    for junction in network.junctions:
        for conn in junction.connections:
            connecting = _road_by_id(network, conn.connecting_road)
            leg_in = _leg_of(junction, conn.incoming_road)

            # ---- 入口接缝 ----
            x, y, hdg = _end_pose(leg_in.road, leg_in.contact)
            want = _lane_center(x, y, hdg, leg_in.inbound_lane, lane_width)
            g0 = connecting.geoms[0]
            got = _lane_center(g0.x, g0.y, g0.hdg, -1, lane_width)
            _assert_same_point(want, got, f'路口 {junction.jid} 连接道路 '
                                          f'{connecting.rid} 的入口')

            # 行驶朝向：来路以 end 贴路口时车沿 +s 走，以 start 贴路口时沿 −s 走。
            travel_in = hdg if leg_in.contact == 'end' else hdg + math.pi
            assert gen.normalize_angle(travel_in - g0.hdg) == pytest.approx(
                0.0, abs=ANG_TOL_RAD), (
                f'路口 {junction.jid} 连接道路 {connecting.rid} 的入口朝向不连续')

            # ---- 出口接缝 ----
            out_rid, out_contact = connecting.successor[1], connecting.successor[2]
            leg_out = _leg_of(junction, out_rid)
            assert leg_out.contact == out_contact
            x, y, hdg = _end_pose(leg_out.road, out_contact)
            want = _lane_center(x, y, hdg, leg_out.outbound_lane, lane_width)
            gx, gy, ghdg = connecting.geoms[-1].end_pose()
            got = _lane_center(gx, gy, ghdg, -1, lane_width)
            _assert_same_point(want, got, f'路口 {junction.jid} 连接道路 '
                                          f'{connecting.rid} 的出口')

            travel_out = hdg if out_contact == 'start' else hdg + math.pi
            assert gen.normalize_angle(travel_out - ghdg) == pytest.approx(
                0.0, abs=ANG_TOL_RAD), (
                f'路口 {junction.jid} 连接道路 {connecting.rid} 的出口朝向不连续')


def _assert_same_point(want, got, where):
    """Assert two points coincide, reporting the gap in metres."""
    gap = math.dist(want, got)
    assert gap < POS_TOL_M, (
        f'{where}车道中心不连续：相差 {gap:.6f} m。\n'
        f'  期望 ({want[0]:.6f}, {want[1]:.6f})，实际 ({got[0]:.6f}, {got[1]:.6f})。\n'
        f'  差值接近一个车道宽的话，多半是车道 id 的正负号搞反了。')


def test_turning_lane_radius_equals_the_configured_turn_radius(params, network):
    """
    The driven lane centre turns at exactly ``junction.turn_radius_m``.

    ⚠️ 这条补的是一个**连续性测试抓不到**的洞。

    turn_radius_m 定义在**车道中心线**上（那才是车实际走的轨迹），
    而参考线在车道中心线左侧半个车道处，于是参考线半径要按转向分别换算：
        左转（曲率 > 0，圆心在左）：车道中心在圆心外侧 → r_ref = r_lane − 半车道
        右转（曲率 < 0，圆心在右）：车道中心在圆心内侧 → r_ref = r_lane + 半车道

    把这个加减号写反会怎样？**几何依然处处连续** —— 因为圆心是按 r_ref 求交点
    算出来的，端点照样精确落在路口边界上，上面那条连续性用例全绿。
    唯一的变化是车实际的转弯半径变成了 4.5 m 或 11.5 m 而不是 8 m。
    症状要到 P2 才出现：车过路口时转角需求突变，你会以为是控制器的问题。
    """
    half_lane = params['lanes']['width_m'] / 2.0
    want = params['junction']['turn_radius_m']

    arcs = 0
    for road in network.roads:
        if road.junction < 0:
            continue
        for g in road.geoms:
            if g.curvature == 0.0:
                continue
            r_ref = 1.0 / abs(g.curvature)
            # 车道 −1 在参考线右侧：左转时离圆心更远，右转时更近。
            r_lane = r_ref + math.copysign(half_lane, g.curvature)
            assert r_lane == pytest.approx(want, abs=1e-9), (
                f'连接道路 {road.rid} 的车道中心转弯半径是 {r_lane:.3f} m，'
                f'配置要求 {want} m —— 参考线半径的换算加减号写反了？')
            arcs += 1

    # 每个 T 路口 4 条转弯 + 2 条直行，两个路口共 8 段圆弧。
    # 断言条数是为了防止「一条都没检查也算通过」—— 空循环永远是绿的。
    assert arcs == 8


def test_lane_links_point_at_the_lane_that_actually_connects(network):
    """
    The junction's laneLink and the connecting road's own lane link agree.

    ⚠️ **这条用例的证明力有限，别高估它。** 它两边取的都是 `leg.inbound_lane`，
    等于拿生成器和它自己比 —— 如果 inbound_lane 本身的正负号写反了，
    这里照样全绿（已用故意注入验证过）。

    它能抓的是**两处使用同一个值时不一致**：t_junction() 写进 laneLink 的
    车道号，与 _connecting_road() 存进 lane_predecessor 的不是同一个。
    这在把这段逻辑拆开重构时是真实存在的风险。

    「车道号本身对不对」由 test_lane_centres_are_continuous_across_junctions
    独立地保证 —— 那条用例比对的是**几何位置**，而连接道路 lane −1 的位置
    只由参考线决定，与 inbound_lane 无关，所以它是真正的外部裁判。
    """
    for junction in network.junctions:
        for conn in junction.connections:
            leg_in = _leg_of(junction, conn.incoming_road)
            connecting = _road_by_id(network, conn.connecting_road)
            assert conn.lane_from == leg_in.inbound_lane
            assert conn.lane_to == -1
            assert connecting.lane_predecessor == leg_in.inbound_lane

            leg_out = _leg_of(junction, connecting.successor[1])
            assert connecting.lane_successor == leg_out.outbound_lane


# =============================================================================
#  4. 参数校验：错误配置必须炸，而不是生成一张歪掉的地图
# =============================================================================


def test_clearance_too_small_is_rejected(params):
    """
    An impossible junction geometry raises instead of bending the road.

    cutback = turn_radius + 车道半宽 + clearance。把 clearance 设成足够负的值，
    转弯圆弧的切点就会落到路口区域外面，几何接不上。
    此时必须抛异常 —— 静默生成一条歪掉的连接道路是最坏的结果，
    因为 .xodr 依然合法，错误要到车开进那个路口才暴露。
    """
    bad = yaml.safe_load(yaml.safe_dump(params))     # 深拷贝，别污染其他用例
    bad['junction']['clearance_m'] = -3.0
    with pytest.raises(ValueError, match='cutback'):
        gen.build_network(bad, gen.derive(bad))


def test_corner_radius_larger_than_the_loop_is_rejected(params):
    """A corner radius that cannot fit inside the loop raises."""
    bad = yaml.safe_load(yaml.safe_dump(params))
    bad['loop']['corner_radius_m'] = bad['loop']['width_m']
    with pytest.raises(ValueError, match='放不下'):
        gen.derive(bad)


# =============================================================================
#  5. 跨文件一致性：地图原点与 Gazebo 世界必须重合
# =============================================================================


def test_world_geo_origin_matches_the_map(params):
    """
    The Gazebo world's spherical coordinates equal the map's geo origin.

    ⚠️ 这条跨文件比对防的是一个**没有任何工具会报错**的错误。

    OpenDRIVE 的 (0,0)、Gazebo 世界原点、ROS 的 map 系原点必须是同一个点。
    Gazebo 侧的经纬度写在 worlds/campus_loop.sdf 的 <spherical_coordinates>，
    地图侧写在 config/campus_map.yaml 的 geo_origin，两处各写一份 ——
    正是「同一个数抄两遍」的经典配置。

    写歪了会怎样：两个仿真环境里同一个 x/y 换算出不同的经纬度。
    P1-P3 完全无感（都用局部坐标），要到 P4 拿 GNSS 做定位时才发作，
    而那时你已经在调 ESKF 的噪声参数了，不会想到是地图原点。

    ⚠️ 这里只比对 campus_loop.sdf。campus_minimal.sdf 是冻结的 P0a 回归基线，
       它用同一个原点纯属沿用；把它也纳入比对等于给它加了一条约束，
       将来想改地图原点时会被一个与地图无关的文件挡住。
    """
    world = ET.parse(_REPO_ROOT / 'worlds' / 'campus_loop.sdf').getroot()
    sc = world.find('.//spherical_coordinates')
    assert sc is not None, 'campus_loop.sdf 里没有 <spherical_coordinates>'

    origin = params['geo_origin']
    for tag, key in (('latitude_deg', 'latitude_deg'),
                     ('longitude_deg', 'longitude_deg'),
                     ('elevation', 'elevation_m')):
        node = sc.find(tag)
        assert node is not None, f'<spherical_coordinates> 缺少 <{tag}>'
        assert float(node.text) == pytest.approx(origin[key], abs=1e-9), (
            f'worlds/campus_loop.sdf 的 {tag} = {node.text} 与 '
            f'config/campus_map.yaml 的 geo_origin.{key} = {origin[key]} 不一致。\n'
            f'  两者必须逐字相同 —— OpenDRIVE 原点与 Gazebo 世界原点是同一个点。')

    # ENU 是 ROS map 系的约定（x 东、y 北、z 上）。写成 NED 的话所有 y 会反号，
    # 症状是车往北开而里程计说它在往南。
    frame = sc.find('world_frame_orientation')
    assert frame is not None and frame.text.strip() == 'ENU'


def test_ego_spawn_pose_sits_on_a_lane_centre(params):
    """
    The ego vehicle starts exactly on a lane centre, not on the lane marking.

    起始摆位是手写在 worlds/campus_loop.sdf 里的常数（它属于「摆位」而非
    「地图」，没做成生成物）。但改了车道宽或环线尺寸后它就会失效，
    表现是车一开始就骑在车道线上甚至草地上 —— 而 P2 一上来就会得到一个
    非零的横向误差，让人以为是控制器的初始化有问题。
    """
    world = ET.parse(_REPO_ROOT / 'worlds' / 'campus_loop.sdf').getroot()
    pose = None
    for inc in world.iter('include'):
        name = inc.find('name')
        if name is not None and name.text.strip() == 'ego_vehicle':
            pose = [float(v) for v in inc.find('pose').text.split()]
    assert pose is not None, 'campus_loop.sdf 里没有名为 ego_vehicle 的 include'

    # 环线下边的参考线在 y = −width/2；右侧通行的顺行车道（lane −1）
    # 在其右侧半个车道处。车头朝 +x 时右侧是 −y。
    want_y = -params['loop']['width_m'] / 2.0 - params['lanes']['width_m'] / 2.0
    assert pose[1] == pytest.approx(want_y, abs=1e-9), (
        f'自车起始 y = {pose[1]}，顺行车道中心应在 {want_y}')
    assert pose[5] == pytest.approx(0.0, abs=1e-9), '自车应车头朝 +x（沿 s 方向）'


# =============================================================================
#  6. 生成物与 YAML 同步
# =============================================================================


@pytest.mark.parametrize('path,render_name', [
    (gen.XODR_FILE, 'render_xodr'),
    (gen.ROAD_SDF_FILE, 'render_road_sdf'),
    # 采样基准也必须随 YAML 重新生成 —— 否则改了地图之后，C++ 侧的对账
    # 用的是旧基准，CP-P1-A 会给出**虚假的通过**。
    (gen.SAMPLES_FILE, 'render_samples'),
])
def test_committed_artifacts_are_in_sync(params, path, render_name):
    """
    The checked-in artefacts match what the generator produces today.

    与 CI 里的 `gen_map.py --check` 是同一件事，但放在 colcon test 里意味着
    本地 `colcon test` 就能拦下「改了 YAML 忘了重新生成」，
    不用等推到 CI 才发现。
    """
    assert path.exists(), f'{path} 不存在，请运行 python3 scripts/gen_map.py'
    expected = getattr(gen, render_name)(params)
    assert path.read_text(encoding='utf-8') == expected, (
        f'{path.name} 与 config/campus_map.yaml 不同步。\n'
        f'  运行：python3 scripts/gen_map.py')
