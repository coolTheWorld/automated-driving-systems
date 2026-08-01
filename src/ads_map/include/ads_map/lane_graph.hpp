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

#ifndef ADS_MAP__LANE_GRAPH_HPP_
#define ADS_MAP__LANE_GRAPH_HPP_

// =============================================================================
//  车道级**有向**图 —— 纯 C++17，**不依赖 ROS**
//
//  为什么必须是有向图（这是本切片的头号风险，不是风格问题）
//  ------------------------------------------------------
//  OpenDRIVE 的车道 −1 沿 s 增大方向行驶，+1 逆 s 行驶。同一条物理道路的
//  两个方向在这里是**两个不同的节点**，它们之间没有边。
//
//  如果偷懒把双向车道建成一个无向节点，Dijkstra 会算出「原地掉头更近」的
//  路径 —— 而它在 RViz 里看起来是一条完全正常的平滑曲线，长度也合理。
//  要等到 P2 车真的开上去，才会发现它在对向车道上逆行。
//  本文件的 kNoUTurn 相关测试就是为了把这件事钉死。
//
//  一个节点 = 一条道路上的一条有向车道，而**不是**「一小段车道」。
//  理由：本项目的地图里每条道路只有一个 laneSection，车道从头到尾不分叉，
//  所以「整条车道」就是最小的不可分割行驶单元。真遇到多 laneSection 的地图
//  （比如 CARLA 导出的），建图会直接抛异常而不是猜 —— 见构造函数的说明。
// =============================================================================

#include <cstddef>
#include <map>
#include <optional>
#include <ostream>
#include <vector>

#include "ads_map/road_map.hpp"

namespace ads_map
{

/// @brief 车道图里一个节点的标识：某条道路上的某条有向车道。
///
/// 行驶方向由 lane_id 的符号决定，不额外存一个字段 —— 存两份就有对不上的可能。
struct LaneId
{
  int road_id{-1};
  /// 车道编号，非 0。负 = 沿 s 增大方向行驶，正 = 逆 s 行驶。
  int lane_id{0};

  bool operator==(const LaneId & other) const
  {
    return road_id == other.road_id && lane_id == other.lane_id;
  }
  bool operator!=(const LaneId & other) const { return !(*this == other); }
  /// 为了能做 std::map 的键。顺序本身无意义，只要求稳定。
  bool operator<(const LaneId & other) const
  {
    return (road_id != other.road_id) ? (road_id < other.road_id) : (lane_id < other.lane_id);
  }
};

/// @brief 让 gtest 在断言失败时打印出 "(road 1, lane -1)" 而不是原始字节。
std::ostream & operator<<(std::ostream & os, const LaneId & lane);

/// @brief 车道图的一个节点。
struct LaneNode
{
  LaneId id;
  /// 沿**车道中心线**走完这条车道的长度，单位 m。这就是路由的边代价。
  /// 注意它不等于道路参考线长度，弯道上可差 14%，见 Road::lane_arc_length()。
  double length_m{0.0};
  /// 行驶方向上的**入口**处的参考线弧长，单位 m。
  /// 负编号车道是 0，正编号车道是道路长度 —— 因为它逆着 s 走。
  double entry_s_m{0.0};
  /// 行驶方向上的**出口**处的参考线弧长，单位 m。
  double exit_s_m{0.0};
  /// 出边：可以合法驶入的下一条车道，存的是 LaneGraph 内部的节点下标。
  std::vector<std::size_t> successors;
};

/// @brief 最近车道查询的结果。
struct LaneMatch
{
  LaneId lane;
  /// 最近点处的**参考线**弧长，单位 m。
  double s_m{0.0};
  /// 查询点到该车道中心线的距离，单位 m。
  /// 调用方应当拿它做合理性判断 —— 点在 50 m 外还硬凑一条路，
  /// 症状是「路径从天而降接到某条路上」，而没有任何一层会报错。
  double distance_m{0.0};
};

/// @brief 由路网构建的车道级有向图。
class LaneGraph
{
public:
  /// @brief 建图。
  /// @param map 路网。**按值取并移入** —— 图持有它，避免调用方先销毁路网、
  ///            图里却还留着悬垂引用。这类悬垂在 ROS 节点里尤其容易发生：
  ///            路网常是个临时对象，图却活到节点析构。
  /// @throw std::invalid_argument 地图在车道图这一层讲不通时，例如：
  ///        道路有多个 laneSection（跨段的车道对应关系尚未实现）、
  ///        路口连接指向不存在或不可行驶的车道、
  ///        contactPoint 与目标车道编号的符号自相矛盾。
  ///        **一律抛异常而不是丢掉这条边** —— 少一条边的症状是
  ///        「某个转弯路由死活算不出来」，而没有任何日志说少了什么。
  explicit LaneGraph(RoadMap map);

  /// @brief 取回建图所用的路网。求车道中心线坐标要用到它。
  const RoadMap & road_map() const { return map_; }

  std::size_t node_count() const { return nodes_.size(); }

  /// @brief 图中有向边的总数。用于「节点/边数与手数一致」这类结构性断言。
  std::size_t edge_count() const;

  /// @brief 按下标取节点。
  /// @throw std::out_of_range 下标越界。
  const LaneNode & node(std::size_t index) const;

  /// @brief 车道标识 → 节点下标。
  /// @throw std::out_of_range 该车道不是图中的节点（不存在，或不是 driving 类型）。
  std::size_t index_of(const LaneId & lane) const;

  bool contains(const LaneId & lane) const { return index_.find(lane) != index_.end(); }

  /// @brief 找离给定世界坐标最近的可行驶车道。
  /// @param x_m 地图系（ENU）x，单位 m。
  /// @param y_m 地图系 y，单位 m。
  /// @param heading_rad 可选的车头朝向，单位 rad。给了它就**只在行驶方向大致
  ///        一致的车道里找**（夹角 < 90°）。
  /// @return 找到返回匹配结果；一条候选都没有时返回 std::nullopt。
  ///
  /// @note **朝向参数不是可有可无的。** 双向道路的两条车道只差 3.5 m，
  ///       不给朝向的话，自车停在偏左一点就会被判到对向车道上，
  ///       于是路由第一步就要求它掉头 —— 而路径本身看起来平滑正常。
  ///       S4 从 TF 拿到的自车位姿是带朝向的，应当传进来。
  ///       终点（RViz 的 2D Goal Pose）同样带朝向，传不传取决于
  ///       「用户是想指定一个方向，还是只想指个地方」。
  std::optional<LaneMatch> nearest_lane(
    double x_m, double y_m, std::optional<double> heading_rad = std::nullopt) const;

private:
  /// 把所有可行驶车道变成节点。
  void build_nodes();
  /// 按前后继链接与路口连接连边。必须在 build_nodes() 之后调用。
  void build_edges();
  /// 加一条 from → to 的边，并检查 contactPoint 与目标车道编号的符号是否自洽。
  void add_edge(std::size_t from_index, const LaneId & to, ContactPoint contact_point);

  RoadMap map_;
  std::vector<LaneNode> nodes_;
  std::map<LaneId, std::size_t> index_;
};

}  // namespace ads_map

#endif  // ADS_MAP__LANE_GRAPH_HPP_
