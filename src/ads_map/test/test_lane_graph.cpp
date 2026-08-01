// Copyright 2026 孙帅
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// =============================================================================
//  车道图的 L1 测试
//
//  本文件里最有分量的一条是 EveryEdgeIsGeometricallyContinuous。
//
//  为什么它比「边数对不对」强得多
//  ------------------------------
//  数边数只能证明**数量**对，证明不了每条边接到了正确的地方。而建边这件事里
//  真正容易错的恰恰是「接到哪」：contactPoint 说的是目标道路的哪一端、
//  laneLink 的 from 指的是来路的车道还是连接道路的车道、正编号车道的出口
//  在 s=0 还是 s=length —— 这四处任意一处理解反了，边数都还是 24 条。
//
//  几何连续性不看这些约定，它只问一件事：**上一条车道的出口点，
//  是不是就是下一条车道的入口点，朝向是不是也接得上。**
//  这个判据独立于建边的全部推理过程，所以推理错了它一定红。
//
//  判据里的朝向用的是**行驶朝向**（正编号车道要翻 180°），不是参考线朝向。
//  只比位置不比朝向的话，一条方向接反的边照样能过 —— 两条车道在路口处
//  位置本来就重合。
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ads_map/lane_graph.hpp"
#include "ads_map/opendrive_parser.hpp"

namespace
{

using ads_map::ContactPoint;
using ads_map::ElementType;
using ads_map::Geometry;
using ads_map::Lane;
using ads_map::LaneGraph;
using ads_map::LaneId;
using ads_map::LaneNode;
using ads_map::LaneSection;
using ads_map::LaneWidth;
using ads_map::Pose2D;
using ads_map::Road;
using ads_map::RoadLink;
using ads_map::RoadMap;

// -----------------------------------------------------------------------------
//  来自 config/campus_map.yaml 的配置值。
//
//  **有意手写一遍，而不是从被测代码里读出来。** 从实现里取值再拿去断言，
//  等于让实现自己判自己的卷子：把车道宽改成 4.0 也照样绿。
// -----------------------------------------------------------------------------
constexpr double kLaneWidthM = 3.500;
constexpr double kTurnRadiusM = 8.000;

/// 期望的节点数：3 条常规路各有双向 2 条车道 + 12 条单向连接道路。
constexpr std::size_t kExpectedNodeCount = 3 * 2 + 12;

/// 期望的边数：6 条常规车道每条 2 条出边（T 型路口 3 个方向，去掉掉头剩 2 个）
/// + 12 条连接道路每条 1 条出边（连接道路只通往一个出口）。
constexpr std::size_t kExpectedEdgeCount = 6 * 2 + 12;

/// 几何连续性的判据。实测最大断裂见测试自己打印的数字。
///
/// 定在 1 µm / 1e-8 rad：要抓的是**约定层面**的理解错误，那种错误的量级是
/// 米和弧度级（接错一端 = 差整条道路，方向反了 = 差 π），不是微米级。
/// 定得再松就抓不住「差半个车道宽」这类偏移了。
constexpr double kContinuityTolM = 1e-6;
constexpr double kContinuityTolRad = 1e-8;

/// @brief 车道在给定处的**行驶**位姿：正编号车道逆 s 行驶，朝向翻 180°。
Pose2D travel_pose(const RoadMap & map, const LaneId & lane, double s_m)
{
  Pose2D pose = map.road(lane.road_id).lane_center_pose_at(lane.lane_id, s_m);
  if (lane.lane_id > 0) {
    pose.heading_rad += M_PI;
  }
  return pose;
}

/// @brief 把角度差折进 [-π, π]。
///
/// 这里刻意用 std::remainder 而不是 ads_common::angle_diff —— 被测代码用的
/// 就是后者，测试再用一遍等于拿同一个函数验证它自己。
double wrapped_diff(double a_rad, double b_rad) { return std::remainder(a_rad - b_rad, 2 * M_PI); }

/// @brief 独立重算一条车道的中心线长度：Σ 直线段 + Σ (R_lane × θ)。
///
/// 与实现用的 Δs·(1 − t·k) 是**两种不同的写法**：这里走「半径 × 圆心角」，
/// 实现走「弧长 × 缩放因子」。两者数学等价但形式不同，抄不到一起去。
double expected_lane_length(const Road & road, int lane_id)
{
  const double half_lane_m = kLaneWidthM / 2.0;
  double total_m = 0.0;
  for (const Geometry & segment : road.geometries) {
    if (segment.curvature_inv_m == 0.0) {
      total_m += segment.length_m;
      continue;
    }
    const double radius_ref_m = 1.0 / std::fabs(segment.curvature_inv_m);
    const double angle_rad = segment.length_m / radius_ref_m;
    // 左转（k > 0）时参考线左侧在内圈，右侧车道在外圈，半径更大；右转反之。
    const bool lane_is_outside = (segment.curvature_inv_m > 0.0) == (lane_id < 0);
    const double radius_lane_m =
      lane_is_outside ? radius_ref_m + half_lane_m : radius_ref_m - half_lane_m;
    total_m += radius_lane_m * angle_rad;
  }
  return total_m;
}

/// @brief 造一条只有一个车道段的直线道路，供反例测试用。
Road make_straight_road(int id, double x_m, double y_m, double heading_rad, double length_m)
{
  Road road;
  road.id = id;
  road.length_m = length_m;
  road.geometries.push_back(Geometry{0.0, x_m, y_m, heading_rad, length_m, 0.0});
  LaneSection section;
  for (const int lane_id : {1, -1}) {
    Lane lane;
    lane.id = lane_id;
    lane.type = "driving";
    lane.widths.push_back(LaneWidth{0.0, kLaneWidthM, 0.0, 0.0, 0.0});
    (lane_id > 0 ? section.left : section.right).push_back(lane);
  }
  road.lane_sections.push_back(section);
  return road;
}

/// @brief 把一条道路塞进一张新地图。
RoadMap single_road_map(Road road)
{
  RoadMap map;
  const int id = road.id;
  map.roads.emplace(id, std::move(road));
  return map;
}

// -----------------------------------------------------------------------------
//  真实园区地图上的测试
// -----------------------------------------------------------------------------
class CampusLaneGraph : public ::testing::Test
{
protected:
  void SetUp() override
  {
    graph_ = std::make_unique<LaneGraph>(ads_map::load_opendrive(ADS_MAP_XODR_PATH));
  }
  std::unique_ptr<LaneGraph> graph_;
};

TEST_F(CampusLaneGraph, NodeCountMatchesTheHandCount)
{
  EXPECT_EQ(graph_->node_count(), kExpectedNodeCount);

  // 常规道路是双向的，两个方向各是一个独立节点。
  EXPECT_TRUE(graph_->contains(LaneId{1, -1}));
  EXPECT_TRUE(graph_->contains(LaneId{1, 1}));
  EXPECT_TRUE(graph_->contains(LaneId{3, -1}));
  EXPECT_TRUE(graph_->contains(LaneId{3, 1}));
  // 连接道路是单向的，**没有**对向车道。要是有，路口里就能逆行了。
  EXPECT_TRUE(graph_->contains(LaneId{10, -1}));
  EXPECT_FALSE(graph_->contains(LaneId{10, 1}));
  // 中心车道（id=0）宽度恒为 0、不可行驶，永远不该成为节点。
  EXPECT_FALSE(graph_->contains(LaneId{1, 0}));
}

TEST_F(CampusLaneGraph, EdgeCountMatchesTheHandCount)
{
  EXPECT_EQ(graph_->edge_count(), kExpectedEdgeCount);

  // 每条常规车道在路口处正好有 2 个去向（T 型路口的另外两条腿）。
  EXPECT_EQ(graph_->node(graph_->index_of(LaneId{1, -1})).successors.size(), 2u);
  EXPECT_EQ(graph_->node(graph_->index_of(LaneId{1, 1})).successors.size(), 2u);
  // 每条连接道路只有 1 个去向。
  EXPECT_EQ(graph_->node(graph_->index_of(LaneId{10, -1})).successors.size(), 1u);
}

TEST_F(CampusLaneGraph, EveryEdgeIsGeometricallyContinuous)
{
  const RoadMap & map = graph_->road_map();
  double max_gap_m = 0.0;
  double max_heading_gap_rad = 0.0;
  std::size_t checked = 0;

  for (std::size_t i = 0; i < graph_->node_count(); ++i) {
    const LaneNode & from = graph_->node(i);
    const Pose2D exit = travel_pose(map, from.id, from.exit_s_m);
    for (const std::size_t j : from.successors) {
      const LaneNode & to = graph_->node(j);
      const Pose2D entry = travel_pose(map, to.id, to.entry_s_m);

      const double gap_m = std::hypot(entry.x_m - exit.x_m, entry.y_m - exit.y_m);
      const double heading_gap_rad = std::fabs(wrapped_diff(entry.heading_rad, exit.heading_rad));
      EXPECT_LT(gap_m, kContinuityTolM) << from.id << " → " << to.id << " 位置接不上";
      EXPECT_LT(heading_gap_rad, kContinuityTolRad) << from.id << " → " << to.id << " 朝向接不上";

      max_gap_m = std::fmax(max_gap_m, gap_m);
      max_heading_gap_rad = std::fmax(max_heading_gap_rad, heading_gap_rad);
      ++checked;
    }
  }

  // 防「一条边都没检查」冒充通过 —— 建图要是返回空图，上面的循环一次都不进，
  // 这个用例照样是绿的，而它本该是全文件最强的一条。
  ASSERT_EQ(checked, kExpectedEdgeCount);
  std::cout << "  [连续性] 检查 " << checked << " 条边，最大位置断裂 " << max_gap_m * 1e3
            << " mm，最大朝向断裂 " << max_heading_gap_rad << " rad\n";
}

TEST_F(CampusLaneGraph, NoEdgeIsAnInPlaceUTurn)
{
  for (std::size_t i = 0; i < graph_->node_count(); ++i) {
    const LaneNode & from = graph_->node(i);
    for (const std::size_t j : from.successors) {
      const LaneId to = graph_->node(j).id;
      // 同一条道路上的一对反向车道之间不该有边。有的话，Dijkstra 会发现
      // 「原地掉头」是通往对向车道的最短路 —— 而路径在 RViz 里平滑正常。
      const bool is_u_turn = from.id.road_id == to.road_id && from.id.lane_id * to.lane_id < 0;
      EXPECT_FALSE(is_u_turn) << "掉头边 " << from.id << " → " << to;
    }
  }
}

TEST_F(CampusLaneGraph, TurningLanesHaveTheConfiguredTurnRadius)
{
  // config/campus_map.yaml 写的是**车道中心线**的转弯半径 8 m，
  // 而 .xodr 里参考线的半径是 6.25 或 9.75（左转右转各偏半个车道）。
  // 只有把 (1 − t·k) 这步做对，才能把 8 m 还原出来。
  // 这条判据的源头是配置文件，与实现毫无关系 —— 抄不到。
  const double expected_arc_m = kTurnRadiusM * M_PI / 2.0;
  std::size_t checked = 0;

  for (const auto & entry : graph_->road_map().roads) {
    const Road & road = entry.second;
    if (road.junction_id < 0) {
      continue;  // 只看路口里的连接道路
    }
    for (const Geometry & segment : road.geometries) {
      if (segment.curvature_inv_m == 0.0) {
        continue;
      }
      const double arc_m = road.lane_arc_length(-1, segment.s0_m, segment.s0_m + segment.length_m);
      EXPECT_NEAR(arc_m, expected_arc_m, 1e-6)
        << "道路 " << road.id << " 的转弯圆弧半径不是配置的 " << kTurnRadiusM << " m";
      ++checked;
    }
  }
  // 两个 T 型路口各有 4 条转弯连接道路（每条腿的左转 + 右转）。
  ASSERT_EQ(checked, 8u);
}

TEST_F(CampusLaneGraph, LaneCostIsTheLaneCentreLengthNotTheReferenceLength)
{
  const Road & loop_east = graph_->road_map().road(1);
  const LaneNode & outer = graph_->node(graph_->index_of(LaneId{1, -1}));
  const LaneNode & inner = graph_->node(graph_->index_of(LaneId{1, 1}));

  EXPECT_NEAR(outer.length_m, expected_lane_length(loop_east, -1), 1e-9);
  EXPECT_NEAR(inner.length_m, expected_lane_length(loop_east, 1), 1e-9);

  // 实测值，写在这里是为了让改动一眼可见（改地图会让它们变）。
  EXPECT_NEAR(outer.length_m, 253.196899, 1e-5);
  EXPECT_NEAR(inner.length_m, 242.201325, 1e-5);

  // 关键点：两个方向的长度**必须不同**，且参考线长度落在两者之间。
  // 用参考线长度当代价的话，一圈的内外两条车道会被算成一样长，
  // 而实际相差 11 m —— 差不多就是路由在两条候选路线之间做取舍的余量。
  const double reference_m = loop_east.length_m;
  EXPECT_GT(outer.length_m, reference_m + 5.0);
  EXPECT_LT(inner.length_m, reference_m - 5.0);
  // 差值有闭式解：每个 90° 弯上，外圈车道比内圈车道多绕「一整个车道宽 × 圆心角」，
  // 而这条路有 2 个弯。注意它**与转弯半径无关** —— 半径在相减时抵消掉了。
  const double difference_m = outer.length_m - inner.length_m;
  EXPECT_NEAR(difference_m, 2.0 * kLaneWidthM * (M_PI / 2.0), 1e-6);
}

TEST_F(CampusLaneGraph, NearestLaneRecoversAPointTakenFromALaneCentre)
{
  const Road & road = graph_->road_map().road(1);
  const double s_m = 100.0;
  const Pose2D on_lane = road.lane_center_pose_at(-1, s_m);

  const auto match = graph_->nearest_lane(on_lane.x_m, on_lane.y_m);
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(match->lane, (LaneId{1, -1}));
  EXPECT_NEAR(match->s_m, s_m, 1e-6);
  EXPECT_LT(match->distance_m, 1e-6);
}

TEST_F(CampusLaneGraph, NearestLaneWithoutHeadingPicksThePhysicallyNearestLane)
{
  const Road & road = graph_->road_map().road(1);
  const Pose2D on_opposite = road.lane_center_pose_at(1, 100.0);

  const auto match = graph_->nearest_lane(on_opposite.x_m, on_opposite.y_m);
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(match->lane, (LaneId{1, 1}));
}

TEST_F(CampusLaneGraph, NearestLaneWithHeadingPicksTheDirectionYouAreFacing)
{
  const Road & road = graph_->road_map().road(1);
  const double s_m = 100.0;
  // 站在**对向**车道的中心线上（距离 0），但车头朝着顺行方向。
  const Pose2D on_opposite = road.lane_center_pose_at(1, s_m);
  const double forward_heading_rad = road.reference_pose_at(s_m).heading_rad;

  const auto match = graph_->nearest_lane(on_opposite.x_m, on_opposite.y_m, forward_heading_rad);
  ASSERT_TRUE(match.has_value());
  // 必须选 3.5 m 外那条顺行车道，而不是脚下这条逆行车道。
  // 选错的后果不是「路径长一点」，而是**整条路径都在逆行**。
  EXPECT_EQ(match->lane, (LaneId{1, -1}));
  EXPECT_NEAR(match->distance_m, kLaneWidthM, 1e-6);
  EXPECT_NEAR(match->s_m, s_m, 1e-6);
}

TEST_F(CampusLaneGraph, NearestLaneReportsHowFarOffTheLaneYouAre)
{
  const Road & road = graph_->road_map().road(1);
  const Pose2D on_lane = road.lane_center_pose_at(-1, 100.0);
  // 沿法向（朝向左转 90°）挪 0.4 m。挪得比半车道宽小，才不会跨到别的车道上。
  const double normal_rad = on_lane.heading_rad + M_PI_2;
  const double offset_m = 0.4;

  const auto match = graph_->nearest_lane(
    on_lane.x_m + offset_m * std::cos(normal_rad), on_lane.y_m + offset_m * std::sin(normal_rad));
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(match->lane, (LaneId{1, -1}));
  EXPECT_NEAR(match->distance_m, offset_m, 1e-6);
}

// -----------------------------------------------------------------------------
//  反例：地图讲不通时必须**抛异常**，而不是丢一条边继续跑
// -----------------------------------------------------------------------------

TEST(LaneGraphRejects, MultipleLaneSections)
{
  Road road = make_straight_road(1, 0.0, 0.0, 0.0, 100.0);
  road.lane_sections.push_back(road.lane_sections.front());
  road.lane_sections.back().s0_m = 50.0;

  // 解析和几何求值都支持多段，是建图这一层不支持。悄悄只用第一段的后果是
  // 后半条路的车道对应关系被凭空捏造，而路由照样算得出结果。
  EXPECT_THROW(LaneGraph{single_road_map(std::move(road))}, std::invalid_argument);
}

TEST(LaneGraphRejects, ContactPointContradictingTheLaneSign)
{
  Road first = make_straight_road(1, 0.0, 0.0, 0.0, 100.0);
  Road second = make_straight_road(2, 100.0, 0.0, 0.0, 100.0);
  // 从第二条路的 start 端驶入，却指向正编号车道 —— 正编号车道逆 s 行驶，
  // 它的入口在 end 端。两者矛盾，放过去就是图里多一条方向相反的边。
  first.successor = RoadLink{ElementType::kRoad, 2, ContactPoint::kStart};
  first.lane_sections.front().right.front().successor_id = 1;

  RoadMap map;
  map.roads.emplace(1, std::move(first));
  map.roads.emplace(2, std::move(second));
  EXPECT_THROW(LaneGraph{std::move(map)}, std::invalid_argument);
}

TEST(LaneGraphRejects, LinkToALaneThatDoesNotExist)
{
  Road first = make_straight_road(1, 0.0, 0.0, 0.0, 100.0);
  Road second = make_straight_road(2, 100.0, 0.0, 0.0, 100.0);
  first.successor = RoadLink{ElementType::kRoad, 2, ContactPoint::kStart};
  first.lane_sections.front().right.front().successor_id = -2;  // 第二条路没有 −2

  RoadMap map;
  map.roads.emplace(1, std::move(first));
  map.roads.emplace(2, std::move(second));
  EXPECT_THROW(LaneGraph{std::move(map)}, std::invalid_argument);
}

TEST(LaneGraphRejects, LinkWithoutALaneLevelLink)
{
  Road first = make_straight_road(1, 0.0, 0.0, 0.0, 100.0);
  Road second = make_straight_road(2, 100.0, 0.0, 0.0, 100.0);
  first.successor = RoadLink{ElementType::kRoad, 2, ContactPoint::kStart};
  // 道路级链接有了，车道级链接没有 —— 不知道接的是哪条车道。
  RoadMap map;
  map.roads.emplace(1, std::move(first));
  map.roads.emplace(2, std::move(second));
  EXPECT_THROW(LaneGraph{std::move(map)}, std::invalid_argument);
}

TEST(LaneGraphRejects, VaryingLaneWidth)
{
  Road road = make_straight_road(1, 0.0, 0.0, 0.0, 100.0);
  road.lane_sections.front().right.front().widths.front().b = 0.01;  // 每米宽 1 cm
  // 变宽车道的弧长是椭圆积分。按常宽算完返回一个偏小的值，症状是路由
  // 「莫名其妙偏爱某条路」，而没有任何一层会报错。
  EXPECT_THROW(LaneGraph{single_road_map(std::move(road))}, std::invalid_argument);
}

TEST(LaneGraphNodes, NonDrivingLanesAreNotNodes)
{
  Road road = make_straight_road(1, 0.0, 0.0, 0.0, 100.0);
  Lane sidewalk;
  sidewalk.id = -2;
  sidewalk.type = "sidewalk";
  sidewalk.widths.push_back(LaneWidth{0.0, 2.0, 0.0, 0.0, 0.0});
  road.lane_sections.front().right.push_back(sidewalk);

  const LaneGraph graph{single_road_map(std::move(road))};
  // 人行道进了图的话，路由可能规划出一条走人行道的「捷径」。
  EXPECT_EQ(graph.node_count(), 2u);
  EXPECT_FALSE(graph.contains(LaneId{1, -2}));
}

TEST(LaneGraphNearest, ReturnsNulloptWhenHeadingRulesOutEveryLane)
{
  // 只有一条单向车道的地图：车道沿 +x 行驶。
  Road road = make_straight_road(1, 0.0, 0.0, 0.0, 100.0);
  road.lane_sections.front().left.clear();
  const LaneGraph graph{single_road_map(std::move(road))};

  EXPECT_TRUE(graph.nearest_lane(50.0, 0.0).has_value());
  // 车头朝 −x：与唯一一条车道的行驶方向差 180°，没有任何候选。
  // 返回一个「最近但方向反了」的车道比返回 nullopt 危险得多 ——
  // 那会让路由从一次逆行开始，而调用方毫不知情。
  EXPECT_FALSE(graph.nearest_lane(50.0, 0.0, M_PI).has_value());
}

}  // namespace
