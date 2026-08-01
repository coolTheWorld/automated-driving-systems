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

#ifndef ADS_MAP__ROUTING_HPP_
#define ADS_MAP__ROUTING_HPP_

// =============================================================================
//  车道级全局路由（Dijkstra）—— 纯 C++17，**不依赖 ROS**
//
//  刻意写成自由函数而不是 LaneGraph 的成员：路由是**算法**，图是**数据**。
//  分开之后，将来加 A*、加带转向惩罚的代价函数，都不必动图这一侧；
//  而图这一侧的结构性测试也不会被算法的改动波及。
// =============================================================================

#include <optional>
#include <vector>

#include "ads_map/lane_graph.hpp"

namespace ads_map
{

/// @brief 路径上的一段：在某条车道上从 entry_s_m 行驶到 exit_s_m。
struct RouteStep
{
  LaneId lane;
  /// 进入本段时的参考线弧长，单位 m。
  double entry_s_m{0.0};
  /// 离开本段时的参考线弧长，单位 m。
  /// @note **正编号车道上 exit_s_m < entry_s_m**，因为它逆着 s 行驶。
  ///       下游采样这一段时必须按 entry→exit 的方向走，不能想当然地
  ///       从小到大遍历 —— 那会画出一条方向反了的路径。
  double exit_s_m{0.0};
  /// 本段沿车道中心线的长度，单位 m，恒为非负。
  double length_m{0.0};
};

/// @brief 一条完整的车道级路径。
struct Route
{
  /// 依次经过的车道段。非空。
  std::vector<RouteStep> steps;
  /// 总长度，单位 m，等于各段之和。
  double length_m{0.0};
};

/// @brief 求从 (start, start_s_m) 到 (goal, goal_s_m) 的最短车道级路径。
/// @param graph 车道图。
/// @param start 起点所在的有向车道。
/// @param start_s_m 起点处的参考线弧长，单位 m，须在起点道路的 [0, length] 内。
/// @param goal 终点所在的有向车道。
/// @param goal_s_m 终点处的参考线弧长，单位 m。
/// @return 找到返回路径；**不可达返回 std::nullopt**。
/// @throw std::out_of_range start / goal 不是图中的节点，或 s 越界。
///
/// @note **不可达与「已经在终点」是两回事，返回值必须能区分。**
///       已经在终点时返回的是一条**只有一段、长度为 0** 的路径，而不是空路径；
///       不可达时返回 nullopt。把两者都表示成「空」的话，下游只会看到
///       「路径里没有点」，然后照常发布一条空的 nav_msgs/Path ——
///       RViz 上什么都不显示，而没有任何一层报错说路由失败了。
///       所以 Route::steps **保证非空**。
///
/// @note 起点和终点在**同一条车道**上时分两种情况：
///       目标在行驶方向的前方 → 直接一段走过去；
///       目标在后方 → 本地图不含掉头连接，必须绕行，
///       算法会把起点车道当作普通节点重新访问，自然得出绕行路径。
///
/// @note 代价是车道中心线长度（见 Road::lane_arc_length()），
///       **不含**转向惩罚、限速、信号灯。P3 若要「少拐弯优先」，
///       是在这里加代价项，而不是在下游对结果做后处理。
std::optional<Route> find_route(
  const LaneGraph & graph, const LaneId & start, double start_s_m, const LaneId & goal,
  double goal_s_m);

}  // namespace ads_map

#endif  // ADS_MAP__ROUTING_HPP_
