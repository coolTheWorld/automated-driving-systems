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
//  车道级路由的 L1 测试
//
//  期望的路径与长度是**独立算出来的**，不是把实现跑一遍抄下来的：
//  用一个穷举所有简单路径的脚本枚举，而不是再写一遍 Dijkstra。
//  （非负边权下最短游走必是简单路径，所以枚举不会漏掉最优解。）
//  拿实现的输出当期望值，测试就只能证明「今天和昨天一样」，
//  而昨天可能本来就是错的。
//
//  本文件里分量最重的是 OppositeDirectionOfTheSameRoadRequiresGoingAround：
//  它是「车道图必须有向」这条约束的验收。起点和终点相距 3.5 m，
//  无向图会给出一条约 3.5 m 的「路径」，而正确答案是 674.7 m。
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ads_map/opendrive_parser.hpp"
#include "ads_map/routing.hpp"

namespace
{

using ads_map::Geometry;
using ads_map::Lane;
using ads_map::LaneGraph;
using ads_map::LaneId;
using ads_map::LaneSection;
using ads_map::LaneWidth;
using ads_map::Pose2D;
using ads_map::Road;
using ads_map::RoadMap;
using ads_map::Route;
using ads_map::RouteStep;

constexpr double kLaneWidthM = 3.500;

/// 路径长度的判据：0.1 mm。
///
/// 期望值来自穷举脚本，两边都是 double 累加同一批闭式解，差异只可能是
/// 末位误差。定得松一点没有好处 —— 真出错时的量级是米（走了另一条路线）。
constexpr double kLengthTolM = 1e-4;

/// @brief 车道在给定处的**行驶**位姿：正编号车道逆 s 行驶，朝向翻 180°。
Pose2D travel_pose(const RoadMap & map, const LaneId & lane, double s_m)
{
  Pose2D pose = map.road(lane.road_id).lane_center_pose_at(lane.lane_id, s_m);
  if (lane.lane_id > 0) {
    pose.heading_rad += M_PI;
  }
  return pose;
}

/// @brief 图上是否真的存在 from → to 这条边。
bool has_edge(const LaneGraph & graph, const LaneId & from, const LaneId & to)
{
  const std::size_t to_index = graph.index_of(to);
  for (const std::size_t successor : graph.node(graph.index_of(from)).successors) {
    if (successor == to_index) {
      return true;
    }
  }
  return false;
}

std::vector<LaneId> lane_sequence(const Route & route)
{
  std::vector<LaneId> lanes;
  lanes.reserve(route.steps.size());
  for (const RouteStep & step : route.steps) {
    lanes.push_back(step.lane);
  }
  return lanes;
}

/// @brief 造一条只有一个车道段的直线双向道路，供反例测试用。
Road make_straight_road(int id, double x_m, double y_m, double length_m)
{
  Road road;
  road.id = id;
  road.length_m = length_m;
  road.geometries.push_back(Geometry{0.0, x_m, y_m, 0.0, length_m, 0.0});
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

class CampusRouting : public ::testing::Test
{
protected:
  void SetUp() override
  {
    graph_ = std::make_unique<LaneGraph>(ads_map::load_opendrive(ADS_MAP_XODR_PATH));
  }
  std::unique_ptr<LaneGraph> graph_;
};

TEST_F(CampusRouting, SameLaneWithTheGoalAheadIsASingleStep)
{
  const auto route = ads_map::find_route(*graph_, LaneId{1, -1}, 20.0, LaneId{1, -1}, 200.0);
  ASSERT_TRUE(route.has_value());
  ASSERT_EQ(route->steps.size(), 1u);
  EXPECT_EQ(route->steps.front().lane, (LaneId{1, -1}));
  EXPECT_DOUBLE_EQ(route->steps.front().entry_s_m, 20.0);
  EXPECT_DOUBLE_EQ(route->steps.front().exit_s_m, 200.0);
  // 20→200 跨过两个 90° 弯，车道中心线比参考线的 180 m 长 5.5 m。
  EXPECT_NEAR(route->length_m, 185.497787, kLengthTolM);
}

TEST_F(CampusRouting, SameLaneWithTheGoalBehindGoesAround)
{
  // 本地图没有掉头连接，往回走只能绕。最近的一圈是借横穿路兜回来。
  const auto route = ads_map::find_route(*graph_, LaneId{1, -1}, 200.0, LaneId{1, -1}, 20.0);
  ASSERT_TRUE(route.has_value());
  EXPECT_EQ(
    lane_sequence(*route), (std::vector<LaneId>{{1, -1}, {17, -1}, {3, 1}, {14, -1}, {1, -1}}));
  EXPECT_NEAR(route->length_m, 189.831853, kLengthTolM);
  // 起点车道在路径里出现两次：一次作起点，一次作终点。这正是「起点节点不
  // 预置距离 0、而是引入虚拟源点」这个写法要支持的情形。
  EXPECT_EQ(route->steps.front().lane, route->steps.back().lane);
}

TEST_F(CampusRouting, OppositeDirectionOfTheSameRoadRequiresGoingAround)
{
  // ★ 「车道图必须有向」的验收。
  // 起点与终点是同一条物理道路的两个方向，相距一个车道宽 3.5 m。
  // 建成无向图的话，最短路会是「原地掉头」，长度约 0 ——
  // 而它在 RViz 里是一条平滑的短线，看不出任何异常。
  const auto route = ads_map::find_route(*graph_, LaneId{1, -1}, 100.0, LaneId{1, 1}, 100.0);
  ASSERT_TRUE(route.has_value());

  EXPECT_NEAR(route->length_m, 674.732290, kLengthTolM);
  EXPECT_GT(route->length_m, 600.0) << "出现了短路径 —— 图很可能被建成了无向图";
  EXPECT_EQ(
    lane_sequence(*route),
    (std::vector<LaneId>{{1, -1}, {17, -1}, {3, 1}, {15, -1}, {2, 1}, {18, -1}, {1, 1}}));

  // 这条用例同时也是「代价必须用车道中心线长度」的验收：
  // 另一条候选路线（绕整圈，经 loop_west 的 −1 车道）长 685.73 m。
  // 若拿参考线长度当代价，两条路线**恰好并列**（都是 680.23 m），
  // 返回哪条取决于堆的遍历顺序 —— 一个看起来正常、却会随实现变的路由。
  for (const RouteStep & step : route->steps) {
    EXPECT_NE(step.lane, (LaneId{2, -1})) << "走了更长的那条候选路线";
  }
}

TEST_F(CampusRouting, RouteStepsAreContiguousAndFollowGraphEdges)
{
  const auto route = ads_map::find_route(*graph_, LaneId{1, -1}, 100.0, LaneId{1, 1}, 100.0);
  ASSERT_TRUE(route.has_value());
  ASSERT_GE(route->steps.size(), 2u);
  const RoadMap & map = graph_->road_map();

  double summed_m = 0.0;
  for (std::size_t i = 0; i < route->steps.size(); ++i) {
    const RouteStep & step = route->steps[i];
    summed_m += step.length_m;

    // 每一段都必须**顺着**自己的车道走。负编号车道 s 递增，正编号车道 s 递减。
    // 少了这条检查，一条「在某条车道上倒着开一段」的路径也能满足总长自洽。
    if (step.lane.lane_id < 0) {
      EXPECT_GE(step.exit_s_m, step.entry_s_m) << "第 " << i << " 段在负编号车道上逆行";
    } else {
      EXPECT_LE(step.exit_s_m, step.entry_s_m) << "第 " << i << " 段在正编号车道上逆行";
    }
    EXPECT_GE(step.length_m, 0.0);

    if (i + 1 == route->steps.size()) {
      break;
    }
    const RouteStep & next = route->steps[i + 1];

    // 相邻两段必须是图上真实存在的一条边 —— 否则路径在拓扑上是断的。
    EXPECT_TRUE(has_edge(*graph_, step.lane, next.lane))
      << step.lane << " → " << next.lane << " 不是图上的边";

    // 并且几何上也接得上：上一段的终点就是下一段的起点。
    const Pose2D leave = travel_pose(map, step.lane, step.exit_s_m);
    const Pose2D enter = travel_pose(map, next.lane, next.entry_s_m);
    EXPECT_LT(std::hypot(enter.x_m - leave.x_m, enter.y_m - leave.y_m), 1e-6)
      << "第 " << i << " 段与下一段接不上";
  }
  // 总长必须与逐段之和一致。对不上的话，下游拿「总长」做进度估计、
  // 拿「逐段」做采样，两者会缓慢分家而不报错。
  EXPECT_NEAR(route->length_m, summed_m, 1e-9);
}

TEST_F(CampusRouting, AlreadyAtTheGoalReturnsAZeroLengthRouteNotAnEmptyOne)
{
  const auto route = ads_map::find_route(*graph_, LaneId{3, -1}, 40.0, LaneId{3, -1}, 40.0);
  ASSERT_TRUE(route.has_value());
  // **非空**是接口契约的一部分：空路径与「不可达」都表现成「没有点」，
  // 下游只会发布一条空的 nav_msgs/Path，RViz 一片空白且无人报错。
  ASSERT_EQ(route->steps.size(), 1u);
  EXPECT_DOUBLE_EQ(route->length_m, 0.0);
  EXPECT_EQ(route->steps.front().lane, (LaneId{3, -1}));
}

TEST_F(CampusRouting, RoutesAcrossTheJunctionMatchTheHandComputedLength)
{
  const auto route = ads_map::find_route(*graph_, LaneId{3, -1}, 40.0, LaneId{2, -1}, 120.0);
  ASSERT_TRUE(route.has_value());
  EXPECT_EQ(lane_sequence(*route), (std::vector<LaneId>{{3, -1}, {21, -1}, {2, -1}}));
  EXPECT_NEAR(route->length_m, 182.815264, kLengthTolM);
}

TEST_F(CampusRouting, UnknownLaneThrows)
{
  // 图里没有这个节点是**调用方的 bug**，不是「没有路」。
  // 混成 nullopt 的话，写错车道号会被当成「目的地到不了」，查起来完全走错方向。
  EXPECT_THROW(
    ads_map::find_route(*graph_, LaneId{999, -1}, 0.0, LaneId{1, -1}, 0.0), std::out_of_range);
  EXPECT_THROW(
    ads_map::find_route(*graph_, LaneId{1, -1}, 0.0, LaneId{1, -3}, 0.0), std::out_of_range);
}

TEST_F(CampusRouting, OutOfRangeStationThrows)
{
  const double length_m = graph_->road_map().road(3).length_m;
  EXPECT_THROW(
    ads_map::find_route(*graph_, LaneId{3, -1}, length_m + 1.0, LaneId{3, -1}, 10.0),
    std::out_of_range);
  EXPECT_THROW(
    ads_map::find_route(*graph_, LaneId{3, -1}, 10.0, LaneId{3, -1}, -1.0), std::out_of_range);
}

TEST(RoutingUnreachable, ReturnsNulloptRatherThanAnEmptyRoute)
{
  // 两条互不相连的路。
  RoadMap map;
  map.roads.emplace(1, make_straight_road(1, 0.0, 0.0, 100.0));
  map.roads.emplace(2, make_straight_road(2, 500.0, 500.0, 100.0));
  const LaneGraph graph{std::move(map)};

  const auto route = ads_map::find_route(graph, LaneId{1, -1}, 10.0, LaneId{2, -1}, 10.0);
  EXPECT_FALSE(route.has_value());

  // 同一张图上「已经在终点」必须仍然返回一条路径 —— 两者必须能区分开。
  const auto trivial = ads_map::find_route(graph, LaneId{1, -1}, 10.0, LaneId{1, -1}, 10.0);
  ASSERT_TRUE(trivial.has_value());
  EXPECT_EQ(trivial->steps.size(), 1u);
}

TEST(RoutingUnreachable, GoalBehindOnADeadEndLaneIsUnreachable)
{
  // 一条断头路：走过去就没法回头了，因为没有掉头连接。
  RoadMap map;
  map.roads.emplace(1, make_straight_road(1, 0.0, 0.0, 100.0));
  const LaneGraph graph{std::move(map)};

  EXPECT_FALSE(ads_map::find_route(graph, LaneId{1, -1}, 80.0, LaneId{1, -1}, 20.0).has_value());
  // 同一条车道上往前走则完全没问题 —— 证明上一条失败不是因为图是空的。
  EXPECT_TRUE(ads_map::find_route(graph, LaneId{1, -1}, 20.0, LaneId{1, -1}, 80.0).has_value());
}

}  // namespace
