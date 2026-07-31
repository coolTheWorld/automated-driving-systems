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

#include <cmath>
#include <stdexcept>
#include <string>

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

Pose2D Road::reference_pose_at(double s_m) const
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
      // 夹到 [0, length]：s 恰好等于道路长度时应当落在最后一段的终点，
      // 而不是因为末位浮点误差多走出去一点点。
      double ds = s_m - it->s0_m;
      ds = std::fmax(0.0, std::fmin(ds, it->length_m));
      return it->pose_at(ds);
    }
  }
  throw std::out_of_range("道路 " + std::to_string(id) + " 的第一段几何未从 s=0 开始");
}

Pose2D Road::lane_center_pose_at(int lane_id, double s_m) const
{
  const Pose2D ref = reference_pose_at(s_m);
  if (lane_id == 0) {
    // 中心车道就是参考线本身，宽度恒为 0。
    return ref;
  }

  const LaneSection & section = lane_section_at(s_m);
  const double ds = s_m - section.s0_m;

  // 从中心向外逐条累加：内侧车道算整宽，本车道算半宽。
  //
  // 为什么不直接写 sign·(|id| − 0.5)·width —— 那个公式只在**所有车道等宽**时
  // 成立。本项目当前的地图确实等宽，但把这个巧合固化进代码，等到哪天加了
  // 一条展宽的右转专用道，车道中心会静默偏掉半个车道宽而没有任何报错。
  double offset_m = 0.0;
  const int step = (lane_id > 0) ? 1 : -1;
  for (int id = step;; id += step) {
    const Lane * lane = section.find_lane(id);
    if (lane == nullptr) {
      throw std::invalid_argument(
        "道路 " + std::to_string(this->id) + " 在 s=" + std::to_string(s_m) + " 处没有车道 " +
        std::to_string(id) + "（求车道 " + std::to_string(lane_id) +
        " 的中心线时需要累加它的宽度）");
    }
    const double width = lane->width_at(ds);
    offset_m += (id == lane_id) ? width * 0.5 : width;
    if (id == lane_id) {
      break;
    }
  }
  if (lane_id < 0) {
    offset_m = -offset_m;  // t 以参考线左侧为正，右侧车道取负
  }

  // 沿参考线法向平移。法向 = 航向逆时针转 90°，即指向左侧。
  const double normal_rad = ref.heading_rad + M_PI_2;
  return Pose2D{
    ref.x_m + offset_m * std::cos(normal_rad), ref.y_m + offset_m * std::sin(normal_rad),
    ref.heading_rad};
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
