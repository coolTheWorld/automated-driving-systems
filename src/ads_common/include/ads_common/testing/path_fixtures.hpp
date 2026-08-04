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

#ifndef ADS_COMMON__TESTING__PATH_FIXTURES_HPP_
#define ADS_COMMON__TESTING__PATH_FIXTURES_HPP_

// =============================================================================
//  测试用的路径构造器 —— 直线与圆弧
//
//  只造这两种形状是有意的：它们的弧长、曲率、切向**都能手推**，
//  所以判据不依赖任何实现细节。真实地图上的路也只有这两种几何
//  （OpenDRIVE 的 line + arc，spiral 本项目显式不支持）。
//
//  ⚠️ 全部用**等分**而不是"起点 + i × 步长"。后者在 长度/步长 恰好是整数时，
//     浮点上可能多算一步，导致最后两个点重合 —— P1 踩过一次（CLAUDE.md 陷阱表），
//     症状是 RViz 里完全看不出来，而 TrackedPath 会直接抛异常。
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "ads_common/angles.hpp"
#include "ads_common/reference_line.hpp"

namespace ads_common_test
{

/// 沿 +x 的直线，从 (0, y_m) 起，长 length_m，间距约 spacing_m。
inline std::vector<ads_common::Pose2D> MakeStraightAlongX(
  double length_m, double y_m, double spacing_m)
{
  const int segments = std::max(1, static_cast<int>(std::lround(length_m / spacing_m)));
  std::vector<ads_common::Pose2D> poses;
  poses.reserve(static_cast<std::size_t>(segments) + 1);
  for (int i = 0; i <= segments; ++i) {
    poses.push_back({length_m * i / segments, y_m, 0.0});
  }
  return poses;
}

/// 逆时针（左转）圆弧。圆心 (cx, cy)，半径 R，参数角从 start 扫过 sweep。
/// 点 = C + R·(cos φ, sin φ)，行驶切向 = φ + π/2，曲率 = +1/R。
///
/// @param skip_first 拼接到前一段后面时置 true，跳过与前一段末点重合的首点 ——
///                   TrackedPath 对重合点是**抛异常**而不是静默跳过。
inline std::vector<ads_common::Pose2D> MakeLeftArc(
  double radius_m, double center_x_m, double center_y_m, double start_rad, double sweep_rad,
  double spacing_m, bool skip_first)
{
  const int segments = std::max(1, static_cast<int>(std::lround(radius_m * sweep_rad / spacing_m)));
  std::vector<ads_common::Pose2D> poses;
  for (int i = skip_first ? 1 : 0; i <= segments; ++i) {
    const double phi_rad = start_rad + sweep_rad * i / segments;
    poses.push_back(
      {center_x_m + radius_m * std::cos(phi_rad), center_y_m + radius_m * std::sin(phi_rad),
       // 归一化：真实路径来自 ROS 四元数，值域必然是 (−π, π]。
       // 不归一化的话跨 ±π 的那一段就测不到该测的东西。
       ads_common::normalize_angle(phi_rad + M_PI_2)});
  }
  return poses;
}

/// 顺时针（右转）圆弧：参数角**递减**，行驶切向 = φ − π/2，曲率 = **−1/R**。
///
/// ⚠️ 单独提供一个右转构造器，是因为「所有测试路径都是左转」是个真实的覆盖漏洞：
///    曲率取负号的分支（比如漏了 std::abs）在全左转的用例里**一条都不红**，
///    而地图上左右转各占一半。
inline std::vector<ads_common::Pose2D> MakeRightArc(
  double radius_m, double center_x_m, double center_y_m, double start_rad, double sweep_rad,
  double spacing_m, bool skip_first)
{
  const int segments = std::max(1, static_cast<int>(std::lround(radius_m * sweep_rad / spacing_m)));
  std::vector<ads_common::Pose2D> poses;
  for (int i = skip_first ? 1 : 0; i <= segments; ++i) {
    const double phi_rad = start_rad - sweep_rad * i / segments;
    poses.push_back(
      {center_x_m + radius_m * std::cos(phi_rad), center_y_m + radius_m * std::sin(phi_rad),
       ads_common::normalize_angle(phi_rad - M_PI_2)});
  }
  return poses;
}

/// 多圈圆，圆心在原点，起点 (R, 0) 朝 +y。
/// 闭环用例要跑十几秒，一圈才 50 m，不给足圈数车会开出路径末端。
inline std::vector<ads_common::Pose2D> MakeCircleLaps(
  double radius_m, double laps, double spacing_m)
{
  return MakeLeftArc(radius_m, 0.0, 0.0, 0.0, 2.0 * M_PI * laps, spacing_m, false);
}

/// 从已有路径的末点沿其切向接一段直线。用来拼「弯道 → 直路」。
inline void AppendStraight(
  std::vector<ads_common::Pose2D> * poses, double length_m, double spacing_m)
{
  const ads_common::Pose2D tail = poses->back();
  const int segments = std::max(1, static_cast<int>(std::lround(length_m / spacing_m)));
  for (int i = 1; i <= segments; ++i) {
    const double advance_m = length_m * i / segments;
    poses->push_back(
      {tail.x_m + advance_m * std::cos(tail.heading_rad),
       tail.y_m + advance_m * std::sin(tail.heading_rad), tail.heading_rad});
  }
}

/// 直路 → 左转弯：曲率在一点之内从 0 跳到 1/R，是控制器能遇到的**最硬**的输入。
inline std::vector<ads_common::Pose2D> MakeStraightThenLeftArc(
  double straight_length_m, double radius_m, double sweep_rad, double spacing_m)
{
  std::vector<ads_common::Pose2D> poses = MakeStraightAlongX(straight_length_m, 0.0, spacing_m);
  // 圆心在直路末端的正左方（左转），起始参数角 −π/2 对应切向 0（= 直路方向）。
  const std::vector<ads_common::Pose2D> arc =
    MakeLeftArc(radius_m, straight_length_m, radius_m, -M_PI_2, sweep_rad, spacing_m, true);
  poses.insert(poses.end(), arc.begin(), arc.end());
  return poses;
}

}  // namespace ads_common_test

#endif  // ADS_COMMON__TESTING__PATH_FIXTURES_HPP_
