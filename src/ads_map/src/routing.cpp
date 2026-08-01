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

#include "ads_map/routing.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace ads_map
{

namespace
{

/// s 比较的容差，单位 m。与 road_map.cpp 里的 kEps 同源同理由：
/// 只用来吸收浮点末位误差，让「目标恰好就在脚下」判成「在前方」而不是「在后方」。
constexpr double kEps = 1e-9;

/// 前驱数组里表示「没有前驱」。
constexpr std::size_t kNoPredecessor = std::numeric_limits<std::size_t>::max();

/// 优先队列的元素：(到该节点入口的距离, 节点下标)。
/// 用 pair 而不是自定义比较器：距离相同时按下标比，**结果就是确定的**。
/// 不这么做的话，代价并列的两条路线返回哪条取决于堆的内部顺序，
/// 换个标准库实现就可能变 —— 那种不稳定极难在测试里复现。
using QueueEntry = std::pair<double, std::size_t>;

}  // namespace

std::optional<Route> find_route(
  const LaneGraph & graph, const LaneId & start, double start_s_m, const LaneId & goal,
  double goal_s_m)
{
  const std::size_t start_index = graph.index_of(start);  // 不在图里直接抛
  const std::size_t goal_index = graph.index_of(goal);
  const RoadMap & map = graph.road_map();
  const LaneNode & start_node = graph.node(start_index);
  const LaneNode & goal_node = graph.node(goal_index);
  const Road & start_road = map.road(start.road_id);
  const Road & goal_road = map.road(goal.road_id);

  // 起点车道上还要走的距离 + 终点车道上已经走过的距离。
  // 放在最前面算，顺带把 s 越界挡住：lane_arc_length() 两端都检查。
  // 先算再搜索，是为了让「起点 s 写错了」表现成异常，而不是一条算得出来的错路。
  const double head_m = start_road.lane_arc_length(start.lane_id, start_s_m, start_node.exit_s_m);
  const double tail_m = goal_road.lane_arc_length(goal.lane_id, goal_node.entry_s_m, goal_s_m);

  // ---------------------------------------------------------------------------
  // 特例：起点终点在同一条车道，且目标在行驶方向的**前方** —— 直接走过去。
  // ---------------------------------------------------------------------------
  if (start_index == goal_index) {
    const bool goal_is_ahead =
      (start.lane_id < 0) ? (goal_s_m >= start_s_m - kEps) : (goal_s_m <= start_s_m + kEps);
    if (goal_is_ahead) {
      const double length_m = start_road.lane_arc_length(start.lane_id, start_s_m, goal_s_m);
      return Route{{RouteStep{start, start_s_m, goal_s_m, length_m}}, length_m};
    }
    // 目标在后方：落到下面的搜索。本地图没有掉头连接，只能绕一圈回来。
  }

  // ---------------------------------------------------------------------------
  // Dijkstra。dist[v] = 从起点位置出发、抵达节点 v **入口**的距离。
  //
  // 起点节点自己不置 0 —— 我们不是从它的入口出发，而是从 start_s_m 出发。
  // 于是引入一个**虚拟源点**（下标 = node_count()），它到起点车道各后继的
  // 边权就是 head_m。这样起点节点就是一个普普通通的节点，可以被重新访问，
  // 「同车道、目标在后方」于是自动成立，不需要任何特判。
  //
  // 换个写法（把 dist[start]=0）会在 start == goal 时把前驱链绕成一个环，
  // 回溯时死循环 —— 这类 bug 只在「终点在起点后方」时触发，很容易漏测。
  // ---------------------------------------------------------------------------
  const std::size_t node_count = graph.node_count();
  const std::size_t source_index = node_count;

  std::vector<double> dist(node_count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> predecessor(node_count, kNoPredecessor);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;

  for (const std::size_t next_index : start_node.successors) {
    if (head_m < dist[next_index]) {
      dist[next_index] = head_m;
      predecessor[next_index] = source_index;
      queue.push({head_m, next_index});
    }
  }

  while (!queue.empty()) {
    const auto [current_dist, current] = queue.top();
    queue.pop();
    if (current_dist > dist[current]) {
      continue;  // 过期条目：这个节点后来被更短的路径松弛过
    }
    if (current == goal_index) {
      break;  // 弹出即最优，非负边权下不会再变短
    }
    // 从 current 的入口走到它的出口，要走完整条车道 —— 这就是边权。
    const LaneNode & current_node = graph.node(current);
    const double next_dist = current_dist + current_node.length_m;
    for (const std::size_t next_index : current_node.successors) {
      if (next_dist < dist[next_index]) {
        dist[next_index] = next_dist;
        predecessor[next_index] = current;
        queue.push({next_dist, next_index});
      }
    }
  }

  if (!std::isfinite(dist[goal_index])) {
    return std::nullopt;  // 真的走不到。**不是**返回一条空路径，见头文件说明
  }

  // ---------------------------------------------------------------------------
  // 回溯。前驱链从终点一直退到虚拟源点，中途不会碰到 kNoPredecessor ——
  // 能退回来就说明每一步都被松弛过。
  // ---------------------------------------------------------------------------
  std::vector<std::size_t> chain;
  for (std::size_t current = goal_index; current != source_index; current = predecessor[current]) {
    chain.push_back(current);
  }
  std::reverse(chain.begin(), chain.end());

  Route route;
  route.steps.reserve(chain.size() + 1);
  // 第一段：起点车道，从 start_s_m 走到出口。
  route.steps.push_back(RouteStep{start, start_s_m, start_node.exit_s_m, head_m});
  // 中间各段：整条走完。chain 的最后一个是终点，单独处理。
  for (std::size_t i = 0; i + 1 < chain.size(); ++i) {
    const LaneNode & mid = graph.node(chain[i]);
    route.steps.push_back(RouteStep{mid.id, mid.entry_s_m, mid.exit_s_m, mid.length_m});
  }
  // 最后一段：终点车道，从入口走到 goal_s_m。
  route.steps.push_back(RouteStep{goal, goal_node.entry_s_m, goal_s_m, tail_m});

  // 总长直接累加各段，而不是用 dist[goal] + tail —— 两者数学上相等，
  // 但累加各段保证了「总长」与「逐段长度」自洽，下游拿哪个都不会对不上。
  for (const RouteStep & step : route.steps) {
    route.length_m += step.length_m;
  }
  return route;
}

}  // namespace ads_map
