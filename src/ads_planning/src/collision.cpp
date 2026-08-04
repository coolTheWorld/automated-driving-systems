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

#include "ads_planning/collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ads_planning
{

namespace
{

/// @brief 矩形在给定单位轴上的投影"半径"。
///
/// 中心到最远角点在轴上的投影长度。矩形的两个半轴是
/// `(length/2)·u` 和 `(width/2)·v`（u 沿 heading，v 是它的左法向），
/// 于是半径 = (length/2)·|u·n| + (width/2)·|v·n|。
/// 取绝对值是因为无论半轴朝哪边，最远的那个角总在同侧。
double projection_radius(const Rectangle & rectangle, double axis_x, double axis_y)
{
  const double cos_heading = std::cos(rectangle.heading_rad);
  const double sin_heading = std::sin(rectangle.heading_rad);
  // u = (cos, sin)：长边方向；v = (−sin, cos)：宽边方向（左法向）。
  return 0.5 * rectangle.length_m * std::abs(cos_heading * axis_x + sin_heading * axis_y) +
         0.5 * rectangle.width_m * std::abs(-sin_heading * axis_x + cos_heading * axis_y);
}

/// @brief 在一条轴上，两个矩形投影区间的**间隙**。正数 = 该轴把它们分开了。
double gap_on_axis(const Rectangle & a, const Rectangle & b, double axis_x, double axis_y)
{
  const double center_delta =
    std::abs((b.center_x_m - a.center_x_m) * axis_x + (b.center_y_m - a.center_y_m) * axis_y);
  return center_delta - projection_radius(a, axis_x, axis_y) - projection_radius(b, axis_x, axis_y);
}

/// @brief 点到线段的最小距离。
double point_to_segment_distance(
  double point_x, double point_y, double from_x, double from_y, double to_x, double to_y)
{
  const double segment_x = to_x - from_x;
  const double segment_y = to_y - from_y;
  const double length_squared = segment_x * segment_x + segment_y * segment_y;

  // 退化线段（零长边，来自零宽/零长的矩形）：整条线段缩成起点，直接算点到点。
  // 不抛异常 —— 退化矩形是 SPEC §8 点名要覆盖的输入之一，不能崩也不能给随机结果。
  double ratio = 0.0;
  if (length_squared > 0.0) {
    ratio = ((point_x - from_x) * segment_x + (point_y - from_y) * segment_y) / length_squared;
    ratio = std::clamp(ratio, 0.0, 1.0);
  }
  const double foot_x = from_x + ratio * segment_x;
  const double foot_y = from_y + ratio * segment_y;
  return std::hypot(point_x - foot_x, point_y - foot_y);
}

/// @brief 遍历 `points` 的每个顶点到 `polygon` 的每条边，取最小距离。
double min_vertex_to_edge_distance(
  const std::array<std::array<double, 2>, 4> & points,
  const std::array<std::array<double, 2>, 4> & polygon)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & point : points) {
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      const auto & from = polygon[i];
      const auto & to = polygon[(i + 1) % polygon.size()];
      best = std::min(
        best, point_to_segment_distance(point[0], point[1], from[0], from[1], to[0], to[1]));
    }
  }
  return best;
}

}  // namespace

std::array<std::array<double, 2>, 4> corners_of(const Rectangle & rectangle)
{
  const double cos_heading = std::cos(rectangle.heading_rad);
  const double sin_heading = std::sin(rectangle.heading_rad);
  const double half_length = 0.5 * rectangle.length_m;
  const double half_width = 0.5 * rectangle.width_m;

  // 车体坐标下的四个角，按象限顺序：左前(+,+)、右前(+,−)、右后(−,−)、左后(−,+)。
  // 顺序必须是**绕行**的（相邻两个是一条边），否则下面 (i, i+1) 取边会取到对角线。
  const std::array<std::array<double, 2>, 4> local{
    {{half_length, half_width},
     {half_length, -half_width},
     {-half_length, -half_width},
     {-half_length, half_width}}};

  std::array<std::array<double, 2>, 4> out{};
  for (std::size_t i = 0; i < local.size(); ++i) {
    out[i][0] = rectangle.center_x_m + local[i][0] * cos_heading - local[i][1] * sin_heading;
    out[i][1] = rectangle.center_y_m + local[i][0] * sin_heading + local[i][1] * cos_heading;
  }
  return out;
}

bool overlaps(const Rectangle & a, const Rectangle & b)
{
  const double cos_a = std::cos(a.heading_rad);
  const double sin_a = std::sin(a.heading_rad);
  const double cos_b = std::cos(b.heading_rad);
  const double sin_b = std::sin(b.heading_rad);

  // 只需四条轴：各自的长边方向与宽边方向。矩形的对边平行，法向重复，所以不是八条。
  const std::array<std::array<double, 2>, 4> axes{
    {{cos_a, sin_a}, {-sin_a, cos_a}, {cos_b, sin_b}, {-sin_b, cos_b}}};

  for (const auto & axis : axes) {
    // **严格大于 0 才算分开** ⟹ 恰好相切（间隙 = 0）判为相交。
    // 闭矩形共享边界点在数学上确实相交；安全语义上"恰好贴上"也必须算撞上。
    // 写成 `>= 0` 就会翻过来，而随机测试几乎采不到恰好相切的位形 ——
    // 所以有一条专门的相切用例钉着它。
    if (gap_on_axis(a, b, axis[0], axis[1]) > 0.0) {
      return false;
    }
  }
  return true;
}

double distance_m(const Rectangle & a, const Rectangle & b)
{
  if (overlaps(a, b)) {
    return 0.0;
  }

  const auto corners_a = corners_of(a);
  const auto corners_b = corners_of(b);

  // 两个方向都要算：最近点对中**至少有一个**在顶点上，但不一定在同一个矩形的顶点上。
  // 只算单向的话，"A 的边贴着 B 的顶点"这种位形会被漏掉，给出偏大的距离 ——
  // 而距离偏大意味着**把不安全的候选判成安全的**，方向恰好是错的那一边。
  return std::min(
    min_vertex_to_edge_distance(corners_a, corners_b),
    min_vertex_to_edge_distance(corners_b, corners_a));
}

}  // namespace ads_planning
