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

#include "ads_control/speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "numeric_checks.hpp"

namespace ads_control
{

namespace
{

using internal::RequireFinitePositive;

constexpr char kSpeedProfileParams[] = "SpeedProfileParams";

/// 曲率小于这个值就当直线处理，单位 1/m。
///
/// 取 1e-6（半径 1000 km）。存在的理由不是"数值保护"那么虚 ——
/// 曲率来自朝向的中心差分，直路上它是**浮点噪声级的非零值**（1e-17 量级）。
/// 不设阈值的话 √(a_lat/|κ|) 会算出 1e8 m/s，虽然随后被 min(v_cruise, ·) 吃掉，
/// 但中间会出现一个巨大的数，一旦哪天有人在这中间加一步就会炸。
/// 调大到 1e-3（半径 1 km）也没关系：那个半径下的限速是 38 m/s，早被巡航值盖住。
constexpr double kStraightCurvatureInvM = 1e-6;

}  // namespace

SpeedProfile::SpeedProfile(const TrackedPath & path, const SpeedProfileParams & params)
{
  RequireFinitePositive(params.cruise_speed_mps, kSpeedProfileParams, "cruise_speed_mps");
  RequireFinitePositive(
    params.max_lateral_accel_mps2, kSpeedProfileParams, "max_lateral_accel_mps2");
  RequireFinitePositive(params.max_accel_mps2, kSpeedProfileParams, "max_accel_mps2");
  RequireFinitePositive(params.max_decel_mps2, kSpeedProfileParams, "max_decel_mps2");

  const std::vector<PathPoint> & points = path.points();
  const std::size_t count = points.size();

  arc_lengths_m_.resize(count);
  speeds_mps_.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    arc_lengths_m_[i] = points[i].s_m;
  }

  // ---------------------------------------------------------------------------
  //  ① 曲率限速：a_lat = v²·|κ| ≤ a_lat_max
  // ---------------------------------------------------------------------------
  // 这不是"精细化"。本地图上定速 20 km/h 过 R=8 的路口转弯车道，
  // 横向加速度是 3.86 m/s² —— **紧急变道的量级**，园区接驳车上站着的人扶不住。
  for (std::size_t i = 0; i < count; ++i) {
    const double curvature_inv_m = std::abs(points[i].curvature_inv_m);
    if (curvature_inv_m < kStraightCurvatureInvM) {
      speeds_mps_[i] = params.cruise_speed_mps;
    } else {
      speeds_mps_[i] = std::min(
        params.cruise_speed_mps, std::sqrt(params.max_lateral_accel_mps2 / curvature_inv_m));
    }
  }

  // ---------------------------------------------------------------------------
  //  ② 终点归零 + 后向扫描（保证「减得下来」）
  // ---------------------------------------------------------------------------
  // 依据是匀加速公式 v₂² = v₁² + 2·a·Δs。
  //
  // 终点必须是 0。**先归零再扫描** —— 反过来的话后向扫描会从一个非零的
  // 终点速度往回传，剖面看着完全正常，只是车不会在终点停下来。
  speeds_mps_.back() = 0.0;
  for (std::size_t i = count - 1; i-- > 0;) {
    const double delta_s_m = arc_lengths_m_[i + 1] - arc_lengths_m_[i];
    const double reachable_mps =
      std::sqrt(speeds_mps_[i + 1] * speeds_mps_[i + 1] + 2.0 * params.max_decel_mps2 * delta_s_m);
    speeds_mps_[i] = std::min(speeds_mps_[i], reachable_mps);
  }

  // ---------------------------------------------------------------------------
  //  ③ 前向扫描（保证「加得上去」）
  // ---------------------------------------------------------------------------
  // ⚠️ 这里**曾经**写着「顺序不能反：前向扫描调低某点后，后向扫描算出的
  //    减速起点就失效了」。**那句话是错的**，2026-08-02 用 20000 组随机剖面
  //    实测：两种顺序的结果**逐点完全相同**（最大差异 0.0），
  //    且对调之后两条约束一条都不违反。
  //
  //    原因：两个扫描都**只会把速度调低**，而且各自恰好调到对方约束的安全侧。
  //    前向把 v[i] 降到 v[i−1]²+2a_a·Δs 时，后向要的
  //    v[i−1]² ≤ v[i]²+2a_d·Δs 变成 v[i−1]² ≤ v[i−1]²+2(a_a+a_d)·Δs，恒成立。
  //    反方向同理。
  //
  //    **真正必须成立的顺序**是：曲率限速与终点归零要在两个扫描**之前**完成。
  //    而且这个论证**依赖"只下调"** —— 将来若加入「这一点至少要跑多快」
  //    这类下界约束（比如为了不挡住后车），论证立刻失效，那时才真的要讲顺序、
  //    甚至要迭代到收敛。由 test_speed_profile 的 ScanOrderDoesNotMatter 盯着：
  //    加了下界那条用例会红。
  for (std::size_t i = 1; i < count; ++i) {
    const double delta_s_m = arc_lengths_m_[i] - arc_lengths_m_[i - 1];
    const double reachable_mps =
      std::sqrt(speeds_mps_[i - 1] * speeds_mps_[i - 1] + 2.0 * params.max_accel_mps2 * delta_s_m);
    speeds_mps_[i] = std::min(speeds_mps_[i], reachable_mps);
  }
}

double SpeedProfile::speed_at(const PathProjection & projection) const
{
  return speed_at(projection.index, projection.ratio);
}

double SpeedProfile::speed_at(std::size_t index, double ratio) const
{
  if (index + 1 >= speeds_mps_.size()) {
    throw std::out_of_range(
      "SpeedProfile::speed_at: 线段索引 " + std::to_string(index) + " 超出剖面范围（共 " +
      std::to_string(speeds_mps_.size()) + " 点）。剖面和路径对不上了 —— 换路径时必须同时换剖面。");
  }
  // 线性插值。剖面本身是按 v² 的约束构造的，所以对 v 线性插值并不精确满足约束，
  // 误差是 O(Δs²) 且**偏保守**（弦在弧下方）。0.5 m 点距下量级是 mm/s，忽略。
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  return speeds_mps_[index] + clamped_ratio * (speeds_mps_[index + 1] - speeds_mps_[index]);
}

}  // namespace ads_control
