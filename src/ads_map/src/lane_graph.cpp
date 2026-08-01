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

#include "ads_map/lane_graph.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ads_common/angles.hpp"

namespace ads_map
{

namespace
{

/// 最近车道查询的粗采样步长，单位 m。
///
/// 它决定的是「会不会落进错误的局部极小」，**不决定最终精度** ——
/// 精度由后面的三分法细化保证。判据是步长要远小于车道的曲率半径，
/// 本地图最小的参考线半径是 6.25 m，0.5 m 步长对应 4.6° 的转角，足够密。
/// 调大：线性变快，但在急弯或 S 弯上可能选到另一侧的局部极小，
///       症状是「自车明明在这条路上，却被定位到隔壁那条」。
/// 调小：线性变慢。全图约 1400 m 车道，0.5 m 步长约 2800 次求值。
constexpr double kNearestSampleStepM = 0.5;

/// 三分法细化的迭代次数。每次把区间缩到 2/3，60 次可把 1 m 的初始区间
/// 缩到 (2/3)^60 ≈ 1e-11 m，远超浮点有效位，属于「足够到不用再想」。
constexpr int kRefineIterations = 60;

/// 朝向过滤的阈值，单位 rad。
///
/// 只要车头朝向与车道行驶方向的夹角小于直角，就认为「顺着这条车道」。
/// 调小（比如 45°）：转弯中的自车可能一条车道都匹配不上，路由直接失败。
/// 调大（接近 180°）：等于没有过滤，对向车道会被选中 → 路由第一步就是逆行。
constexpr double kHeadingToleranceRad = M_PI_2;

/// @brief 车道在给定处的**行驶**朝向。
///
/// 参考线朝向不等于行驶朝向：正编号车道逆着 s 走，方向要翻 180°。
/// 这个 180° 是有向图的全部物理含义所在，漏掉它，朝向过滤就成了摆设。
double travel_heading(const Road & road, int lane_id, double s_m)
{
  const double heading_rad = road.lane_center_pose_at(lane_id, s_m).heading_rad;
  return (lane_id > 0) ? heading_rad + M_PI : heading_rad;
}

}  // namespace

std::ostream & operator<<(std::ostream & os, const LaneId & lane)
{
  return os << "(road " << lane.road_id << ", lane " << lane.lane_id << ")";
}

LaneGraph::LaneGraph(RoadMap map) : map_(std::move(map))
{
  build_nodes();
  build_edges();
}

void LaneGraph::build_nodes()
{
  for (const auto & entry : map_.roads) {
    const Road & road = entry.second;
    if (road.lane_sections.size() != 1) {
      throw std::invalid_argument(
        "道路 " + std::to_string(road.id) + " 有 " + std::to_string(road.lane_sections.size()) +
        " 个 <laneSection>，车道图目前只支持恰好 1 个。"
        "解析和几何求值都支持多段，不支持的是**建图**这一层："
        "跨段时车道编号的对应关系需要读 <lane><link>，尚未实现");
    }
    const LaneSection & section = road.lane_sections.front();
    // 先左后右只是为了输出稳定，图的语义与顺序无关。
    for (const std::vector<Lane> * side : {&section.left, &section.right}) {
      for (const Lane & lane : *side) {
        // 只有 driving 类型参与路由。人行道、路肩也在 <lanes> 里，
        // 把它们当节点的后果是路由可能规划出一条走人行道的「捷径」。
        if (lane.type != "driving") {
          continue;
        }
        const LaneId id{road.id, lane.id};
        LaneNode node;
        node.id = id;
        // 代价用车道中心线长度而非道路参考线长度，理由见 Road::lane_arc_length()。
        node.length_m = road.lane_arc_length(lane.id, 0.0, road.length_m);
        node.entry_s_m = (lane.id < 0) ? 0.0 : road.length_m;
        node.exit_s_m = (lane.id < 0) ? road.length_m : 0.0;
        if (!index_.emplace(id, nodes_.size()).second) {
          throw std::invalid_argument(
            "道路 " + std::to_string(road.id) + " 里出现了重复的车道编号 " +
            std::to_string(lane.id));
        }
        nodes_.push_back(std::move(node));
      }
    }
  }
}

void LaneGraph::build_edges()
{
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const LaneId id = nodes_[i].id;
    const Road & road = map_.road(id.road_id);
    // 出口在哪一端由行驶方向决定：负编号车道沿 s 走，出口是 successor 那一端；
    // 正编号车道逆 s 走，出口在 s=0，也就是 predecessor 那一端。
    const bool forward = id.lane_id < 0;
    const std::optional<RoadLink> & link = forward ? road.successor : road.predecessor;
    if (!link.has_value()) {
      continue;  // 断头路。合法，只是没有后继，不是错误
    }

    if (link->element_type == ElementType::kJunction) {
      // 接的是路口：车道级的去向由 <junction><connection><laneLink> 描述。
      const Junction & junction = map_.junction(link->element_id);
      for (const JunctionConnection & connection : junction.connections) {
        if (connection.incoming_road_id != id.road_id) {
          continue;
        }
        for (const LaneLink & lane_link : connection.lane_links) {
          if (lane_link.from_lane_id != id.lane_id) {
            continue;
          }
          add_edge(
            i, LaneId{connection.connecting_road_id, lane_link.to_lane_id},
            connection.contact_point);
        }
      }
      continue;
    }

    // 直接接到另一条道路（本项目里是「连接道路 → 出口道路」）：
    // 车道级的去向在 <lane><link> 里，路口那套表在这里用不上。
    // 节点是从这条车道建出来的，所以 find_lane 必定命中。
    const Lane * lane = road.lane_sections.front().find_lane(id.lane_id);
    const std::optional<int> & next_lane_id = forward ? lane->successor_id : lane->predecessor_id;
    if (!next_lane_id.has_value()) {
      throw std::invalid_argument(
        "道路 " + std::to_string(id.road_id) + " 的车道 " + std::to_string(id.lane_id) +
        " 直接接到道路 " + std::to_string(link->element_id) +
        "，但没有车道级的 <link> 说明接的是哪条车道。"
        "静默跳过的后果是路网在这里断成两半，而路由只会说「不可达」");
    }
    if (!link->contact_point.has_value()) {
      throw std::invalid_argument(
        "道路 " + std::to_string(id.road_id) + " 到道路 " + std::to_string(link->element_id) +
        " 的链接缺少 contactPoint，无法判断从目标道路的哪一端驶入");
    }
    add_edge(i, LaneId{link->element_id, *next_lane_id}, *link->contact_point);
  }
}

void LaneGraph::add_edge(std::size_t from_index, const LaneId & to, ContactPoint contact_point)
{
  // contactPoint 说明我们从目标道路的**哪一端**驶入。从 start 驶入意味着此后
  // 沿 s 增大方向行驶，按 OpenDRIVE 约定那必然是负编号车道；从 end 驶入则
  // 必然是正编号。两者对不上说明地图的链接写错了。
  //
  // 这条检查不是形式主义：放过去的后果是图里多一条方向相反的边，
  // 路由据此规划出的路径在 RViz 里平滑正常，车开上去却在逆行。
  const bool expects_forward = (contact_point == ContactPoint::kStart);
  if ((to.lane_id < 0) != expects_forward) {
    throw std::invalid_argument(
      "驶入道路 " + std::to_string(to.road_id) + " 的车道 " + std::to_string(to.lane_id) +
      " 时 contactPoint=" + (expects_forward ? "start" : "end") +
      "，两者矛盾：从 start 驶入必须是负编号车道，从 end 驶入必须是正编号车道");
  }

  const auto it = index_.find(to);
  if (it == index_.end()) {
    throw std::invalid_argument(
      "地图引用了道路 " + std::to_string(to.road_id) + " 的车道 " + std::to_string(to.lane_id) +
      "，但它不是图中的节点（不存在，或 type 不是 driving）");
  }
  nodes_[from_index].successors.push_back(it->second);
}

std::size_t LaneGraph::edge_count() const
{
  std::size_t total = 0;
  for (const LaneNode & node : nodes_) {
    total += node.successors.size();
  }
  return total;
}

const LaneNode & LaneGraph::node(std::size_t index) const
{
  if (index >= nodes_.size()) {
    throw std::out_of_range(
      "车道图节点下标 " + std::to_string(index) + " 越界，共 " + std::to_string(nodes_.size()) +
      " 个节点");
  }
  return nodes_[index];
}

std::size_t LaneGraph::index_of(const LaneId & lane) const
{
  const auto it = index_.find(lane);
  if (it == index_.end()) {
    throw std::out_of_range(
      "车道图里没有道路 " + std::to_string(lane.road_id) + " 的车道 " +
      std::to_string(lane.lane_id) + " 这个节点");
  }
  return it->second;
}

std::optional<LaneMatch> LaneGraph::nearest_lane(
  double x_m, double y_m, std::optional<double> heading_rad) const
{
  std::optional<LaneMatch> best;

  for (const LaneNode & graph_node : nodes_) {
    const Road & road = map_.road(graph_node.id.road_id);
    const int lane_id = graph_node.id.lane_id;

    const auto distance2_at = [&road, lane_id, x_m, y_m](double s_m) {
      const Pose2D pose = road.lane_center_pose_at(lane_id, s_m);
      const double dx_m = pose.x_m - x_m;
      const double dy_m = pose.y_m - y_m;
      return dx_m * dx_m + dy_m * dy_m;
    };

    // 第一步：粗采样。目的只是**框住**全局最小落在哪一段，不求精度。
    double best_s_m = 0.0;
    double best_distance2 = std::numeric_limits<double>::infinity();
    const int steps = static_cast<int>(std::ceil(road.length_m / kNearestSampleStepM));
    for (int i = 0; i <= steps; ++i) {
      const double s_m = std::fmin(road.length_m, i * kNearestSampleStepM);
      if (heading_rad.has_value()) {
        const double diff_rad =
          ads_common::angle_diff(*heading_rad, travel_heading(road, lane_id, s_m));
        if (std::fabs(diff_rad) > kHeadingToleranceRad) {
          continue;
        }
      }
      const double distance2 = distance2_at(s_m);
      if (distance2 < best_distance2) {
        best_distance2 = distance2;
        best_s_m = s_m;
      }
    }
    if (!std::isfinite(best_distance2)) {
      continue;  // 整条车道都被朝向过滤掉了，它不是候选
    }

    // 第二步：在最近采样点两侧各一个步长内用三分法细化。
    // 距离平方在最小值附近是单峰的（这一小段上曲线近似直线），三分法适用。
    //
    // 细化过程**不再检查朝向**：区间只有 ±0.5 m，行驶朝向最多变化
    // 0.5 / 6.25 = 0.08 rad，翻不过 90° 的判据。为这点变化把过滤塞进
    // 三分法，只会让这段本就不直观的代码更难读。
    double lo_m = std::fmax(0.0, best_s_m - kNearestSampleStepM);
    double hi_m = std::fmin(road.length_m, best_s_m + kNearestSampleStepM);
    for (int i = 0; i < kRefineIterations; ++i) {
      const double third_m = (hi_m - lo_m) / 3.0;
      if (distance2_at(lo_m + third_m) < distance2_at(hi_m - third_m)) {
        hi_m -= third_m;
      } else {
        lo_m += third_m;
      }
    }

    const double s_m = 0.5 * (lo_m + hi_m);
    const double distance_m = std::sqrt(distance2_at(s_m));
    if (!best.has_value() || distance_m < best->distance_m) {
      best = LaneMatch{graph_node.id, s_m, distance_m};
    }
  }

  return best;
}

}  // namespace ads_map
