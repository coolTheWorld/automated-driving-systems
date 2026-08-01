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

#include "ads_map/road_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace ads_map
{

namespace
{

/// 浮点比较的容差。
///
/// 取 1e-9 m（= 1 纳米）的理由：s 的来源是解析 .xodr 文本得到的 double，
/// 与道路长度的比较只需要吸收「文本里写 247.699000、累加得 247.698999…」
/// 这一类末位误差。放大到毫米级会掩盖真正的越界，缩小到 0 则会在
/// s == length 这个完全合法的边界上误报。
constexpr double kEps = 1e-9;

/// @brief 检查「求 lane_id 的横向偏移会用到的每一条车道」在本车道段内是否等宽。
///
/// 这是 Road::lane_arc_length() 闭式解的前提条件。与其在宽度会变的地图上
/// 悄悄按常宽算完、返回一个偏小的长度，不如在这里直接停下来 ——
/// 路由代价偏小的症状是「算出来的路径莫名其妙偏爱某条路」，
/// 而没有任何一层会报错。
void require_constant_width(const LaneSection & section, int road_id, int lane_id)
{
  const int step = (lane_id > 0) ? 1 : -1;
  for (int id = step;; id += step) {
    const Lane * lane = section.find_lane(id);
    // 车道缺失不在这里报 —— lane_offset_at() 的报错信息更全，交给它。
    if (lane == nullptr) {
      return;
    }
    // 与 0.0 精确比较是有意的：这些值直接来自 .xodr 文本里的 "0.0"，
    // 不是任何浮点运算的结果，没有末位误差可言。
    const bool constant = lane->widths.size() == 1 && lane->widths.front().b == 0.0 &&
                          lane->widths.front().c == 0.0 && lane->widths.front().d == 0.0;
    if (!constant) {
      throw std::invalid_argument(
        "道路 " + std::to_string(road_id) + " 的车道 " + std::to_string(id) +
        " 在车道段 s0=" + std::to_string(section.s0_m) +
        " 内不是等宽的，弧长的闭式解不成立（变宽车道的弧长是椭圆积分，尚未实现）");
    }
    if (id == lane_id) {
      return;
    }
  }
}

}  // namespace

double LaneWidth::width_at(double ds_m) const
{
  // 霍纳法则展开 a + b·x + c·x² + d·x³。比逐项 pow() 少两次乘法，
  // 数值上也更稳 —— 不过在这里两者差别可以忽略，写成这样纯粹是习惯。
  return a + ds_m * (b + ds_m * (c + ds_m * d));
}

double Lane::width_at(double ds_m) const
{
  if (widths.empty()) {
    throw std::invalid_argument("车道 " + std::to_string(id) + " 没有任何 <width> 记录");
  }
  // 找最后一条 s_offset ≤ ds 的多项式。宽度记录按 s_offset 递增排列，
  // 所以倒着找命中的就是生效的那一条。
  for (auto it = widths.rbegin(); it != widths.rend(); ++it) {
    if (ds_m >= it->s_offset_m - kEps) {
      return it->width_at(ds_m - it->s_offset_m);
    }
  }
  // 走到这里说明所有记录的 s_offset 都大于 ds —— 第一条没有从 0 开始。
  // 规范要求第一条的 sOffset 必须是 0，所以这是地图坏了。
  throw std::invalid_argument(
    "车道 " + std::to_string(id) + " 的第一条 <width> 未从 sOffset=0 开始");
}

const Lane * LaneSection::find_lane(int lane_id) const
{
  const std::vector<Lane> & side = (lane_id > 0) ? left : right;
  for (const Lane & lane : side) {
    if (lane.id == lane_id) {
      return &lane;
    }
  }
  return nullptr;
}

const LaneSection & Road::lane_section_at(double s_m) const
{
  if (lane_sections.empty()) {
    throw std::out_of_range("道路 " + std::to_string(id) + " 没有任何 <laneSection>");
  }
  if (s_m < -kEps || s_m > length_m + kEps) {
    throw std::out_of_range(
      "s=" + std::to_string(s_m) + " 超出道路 " + std::to_string(id) + " 的长度 " +
      std::to_string(length_m));
  }
  for (auto it = lane_sections.rbegin(); it != lane_sections.rend(); ++it) {
    if (s_m >= it->s0_m - kEps) {
      return *it;
    }
  }
  throw std::out_of_range("道路 " + std::to_string(id) + " 的第一个 <laneSection> 未从 s=0 开始");
}

const Geometry & Road::geometry_at(double s_m) const
{
  if (geometries.empty()) {
    throw std::out_of_range("道路 " + std::to_string(id) + " 的 <planView> 里没有任何几何段");
  }
  if (s_m < -kEps || s_m > length_m + kEps) {
    throw std::out_of_range(
      "s=" + std::to_string(s_m) + " 超出道路 " + std::to_string(id) + " 的长度 " +
      std::to_string(length_m));
  }
  for (auto it = geometries.rbegin(); it != geometries.rend(); ++it) {
    if (s_m >= it->s0_m - kEps) {
      return *it;
    }
  }
  throw std::out_of_range("道路 " + std::to_string(id) + " 的第一段几何未从 s=0 开始");
}

Pose2D Road::reference_pose_at(double s_m) const
{
  const Geometry & segment = geometry_at(s_m);
  // 夹到 [0, 段长]：s 恰好等于道路长度时应当落在最后一段的终点，
  // 而不是因为末位浮点误差多走出去一点点。
  const double ds = std::fmax(0.0, std::fmin(s_m - segment.s0_m, segment.length_m));
  return segment.pose_at(ds);
}

double Road::lane_offset_at(int lane_id, double s_m) const
{
  // 先查车道段，顺带把 s 的越界检查做掉 —— 放在 lane_id == 0 的早退**之前**，
  // 否则中心车道会静默接受任意越界的 s，而文档说它会抛。
  // 「大部分情况会检查」的检查最难查：出问题的永远是没检查的那一支。
  const LaneSection & section = lane_section_at(s_m);
  if (lane_id == 0) {
    return 0.0;  // 中心车道就是参考线本身，宽度恒为 0
  }
  const double ds = s_m - section.s0_m;

  // 从中心向外逐条累加：内侧车道算整宽，本车道算半宽。
  double offset_m = 0.0;
  const int step = (lane_id > 0) ? 1 : -1;
  for (int lane_index = step;; lane_index += step) {
    const Lane * lane = section.find_lane(lane_index);
    if (lane == nullptr) {
      throw std::invalid_argument(
        "道路 " + std::to_string(id) + " 在 s=" + std::to_string(s_m) + " 处没有车道 " +
        std::to_string(lane_index) + "（求车道 " + std::to_string(lane_id) +
        " 的中心线时需要累加它的宽度）");
    }
    const double width = lane->width_at(ds);
    offset_m += (lane_index == lane_id) ? width * 0.5 : width;
    if (lane_index == lane_id) {
      break;
    }
  }
  // t 以参考线左侧为正，右侧车道取负。
  return (lane_id < 0) ? -offset_m : offset_m;
}

Pose2D Road::lane_center_pose_at(int lane_id, double s_m) const
{
  const Pose2D ref = reference_pose_at(s_m);
  const double offset_m = lane_offset_at(lane_id, s_m);
  // 沿参考线法向平移。法向 = 航向逆时针转 90°，即指向左侧。
  const double normal_rad = ref.heading_rad + M_PI_2;
  return Pose2D{
    ref.x_m + offset_m * std::cos(normal_rad), ref.y_m + offset_m * std::sin(normal_rad),
    ref.heading_rad};
}

double Road::lane_arc_length(int lane_id, double s_a_m, double s_b_m) const
{
  const double lo = std::fmin(s_a_m, s_b_m);
  const double hi = std::fmax(s_a_m, s_b_m);
  // 借 geometry_at() 做越界检查：两端都必须落在道路上。
  // 只查一端的话，「起点合法、终点越界」会算出一个偏短但看不出问题的长度。
  (void)geometry_at(lo);
  (void)geometry_at(hi);
  if (lane_id == 0) {
    return hi - lo;  // 中心车道即参考线，t = 0，缩放因子恒为 1
  }

  // 切点 = 几何段边界 ∪ 车道段边界。这两处分别是曲率 k 与横向偏移 t
  // **唯一**可能跳变的地方；切开之后每一小块内两者都是常数，闭式解才成立。
  std::vector<double> cuts{lo, hi};
  for (const Geometry & segment : geometries) {
    if (segment.s0_m > lo && segment.s0_m < hi) {
      cuts.push_back(segment.s0_m);
    }
  }
  for (const LaneSection & section : lane_sections) {
    if (section.s0_m > lo && section.s0_m < hi) {
      cuts.push_back(section.s0_m);
    }
  }
  std::sort(cuts.begin(), cuts.end());

  double total_m = 0.0;
  for (std::size_t i = 0; i + 1 < cuts.size(); ++i) {
    const double piece_m = cuts[i + 1] - cuts[i];
    if (piece_m <= kEps) {
      continue;  // 切点重合（例如车道段边界正好压在几何段边界上）
    }
    // 在小块中点取值：中点严格落在块内部，不会因为浮点误差取到相邻块的 k 或 t。
    const double mid_m = 0.5 * (cuts[i] + cuts[i + 1]);
    require_constant_width(lane_section_at(mid_m), id, lane_id);
    const double t_m = lane_offset_at(lane_id, mid_m);
    const double curvature = geometry_at(mid_m).curvature_inv_m;
    const double scale = 1.0 - t_m * curvature;
    if (scale <= 0.0) {
      // t·k ≥ 1 意味着车道中心落到了曲率中心上或更外侧，等距偏移曲线在此处
      // 退化（长度为 0 甚至反向）。这不是数值问题，是地图本身画错了：
      // 转弯半径小于等于车道到参考线的距离。
      throw std::invalid_argument(
        "道路 " + std::to_string(id) + " 的车道 " + std::to_string(lane_id) + " 在 s≈" +
        std::to_string(mid_m) + " 处退化：t=" + std::to_string(t_m) +
        "，曲率=" + std::to_string(curvature) + "，1−t·k=" + std::to_string(scale) + " ≤ 0");
    }
    total_m += piece_m * scale;
  }
  return total_m;
}

const Road & RoadMap::road(int road_id) const
{
  const auto it = roads.find(road_id);
  if (it == roads.end()) {
    throw std::out_of_range("地图里没有 id=" + std::to_string(road_id) + " 的道路");
  }
  return it->second;
}

const Junction & RoadMap::junction(int junction_id) const
{
  const auto it = junctions.find(junction_id);
  if (it == junctions.end()) {
    throw std::out_of_range("地图里没有 id=" + std::to_string(junction_id) + " 的路口");
  }
  return it->second;
}

}  // namespace ads_map
