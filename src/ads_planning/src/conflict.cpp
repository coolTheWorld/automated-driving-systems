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

#include "ads_planning/conflict.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "ads_common/numeric_checks.hpp"

namespace ads_planning
{

using ads_common::Pose2D;
using ads_common::ReferenceLine;

namespace
{

/// 段平均速度低于它就当「到不了」，m/s。
///
/// 不是数值美容：停车剖面末段的速度是**精确的 0**，0.05 只拦它前面
/// 数值上趋零的尾巴。取控制侧「停住」判据同一个量级（goal_residual 0.05），
/// 两处对「静止」的定义一致。
constexpr double kMinAvgSpeedMps = 0.05;

constexpr double kInf = std::numeric_limits<double>::infinity();

}  // namespace

std::vector<double> annotate_times(
  const std::vector<double> & arc_lengths_m, const std::vector<double> & speeds_mps)
{
  if (arc_lengths_m.size() != speeds_mps.size()) {
    throw std::invalid_argument("annotate_times: 弧长与速度数组长度不同");
  }
  if (arc_lengths_m.size() < 2) {
    throw std::invalid_argument("annotate_times: 至少需要 2 个点");
  }

  std::vector<double> times_s(arc_lengths_m.size(), 0.0);
  for (std::size_t i = 0; i + 1 < arc_lengths_m.size(); ++i) {
    const double delta_s_m = arc_lengths_m[i + 1] - arc_lengths_m[i];
    const double v_avg_mps = 0.5 * (speeds_mps[i] + speeds_mps[i + 1]);
    // 恒加速度段内 Δs = v_avg·Δt 是恒等式（behavior.md §1）。
    // +∞ 一旦出现就向后传染 —— 停车点之后的所有点都到不了，语义正确。
    if (v_avg_mps < kMinAvgSpeedMps || std::isinf(times_s[i])) {
      times_s[i + 1] = kInf;
    } else {
      times_s[i + 1] = times_s[i] + delta_s_m / v_avg_mps;
    }
  }
  return times_s;
}

double time_at(
  const std::vector<double> & arc_lengths_m, const std::vector<double> & speeds_mps,
  const std::vector<double> & times_s, double query_s_m)
{
  if (query_s_m <= arc_lengths_m.front()) {
    return times_s.front();
  }
  if (query_s_m >= arc_lengths_m.back()) {
    return times_s.back();
  }
  // 二分找段。剖面点数 ~60，线性扫也行，但这个函数每周期被冲突窗调 O(条数) 次，
  // 二分是顺手的事。
  const auto it = std::upper_bound(arc_lengths_m.begin(), arc_lengths_m.end(), query_s_m);
  const std::size_t i = static_cast<std::size_t>(it - arc_lengths_m.begin()) - 1;

  if (std::isinf(times_s[i])) {
    return kInf;
  }
  const double delta_s_m = query_s_m - arc_lengths_m[i];
  const double seg_s_m = arc_lengths_m[i + 1] - arc_lengths_m[i];
  const double v0 = speeds_mps[i];
  const double v1 = speeds_mps[i + 1];
  // 段内恒加速度 ⟹ v(s) = √(v0² + 2a·Δs)，a = (v1² − v0²)/(2·段长)。
  // t = t_i + Δs / v_avg(0→s)，v_avg = (v0 + v(s))/2 —— 闭式，不是线性插值。
  const double v_sq = v0 * v0 + (v1 * v1 - v0 * v0) * (delta_s_m / seg_s_m);
  const double v_here = std::sqrt(std::max(0.0, v_sq));
  const double v_avg = 0.5 * (v0 + v_here);
  if (v_avg < kMinAvgSpeedMps) {
    return kInf;
  }
  return times_s[i] + delta_s_m / v_avg;
}

std::optional<FollowConflict> find_follow_conflict(
  const ReferenceLine & line, double ego_s_m, const std::vector<TargetBox> & targets,
  const BehaviorParams & params)
{
  std::optional<FollowConflict> nearest;
  for (const TargetBox & target : targets) {
    const auto projection = line.project(Pose2D{target.center_x_m, target.center_y_m, 0.0});
    // 横向判**近边**：中心恰在走廊边上的目标，半个身子已经在道里了。
    const double lateral_near_m = std::abs(projection.lateral_error_m) - 0.5 * target.width_m;
    if (lateral_near_m > params.corridor_half_m) {
      continue;
    }
    // 近边弧长：目标沿车道摆放时精确；斜置目标偏保守（近边只会更近）—— 可接受。
    const double near_edge_s_m = projection.s_m - 0.5 * target.length_m;
    // 只看 ego 前方。**用近边比不用近边保守**：车尾已越过 ego 的目标不算前车。
    if (near_edge_s_m <= ego_s_m) {
      continue;
    }
    if (!nearest.has_value() || near_edge_s_m < nearest->near_edge_s_m) {
      nearest = FollowConflict{target.id, near_edge_s_m};
    }
  }
  return nearest;
}

std::vector<CrossingConflict> find_crossing_conflicts(
  const ReferenceLine & line, double ego_s_m, const std::vector<double> & arc_lengths_m,
  const std::vector<double> & speeds_mps, const std::vector<double> & times_s,
  const std::vector<PredictionHypothesis> & hypotheses, const BehaviorParams & params)
{
  std::vector<CrossingConflict> conflicts;
  for (const PredictionHypothesis & hypothesis : hypotheses) {
    // 逐点投影，聚成时空包络。假设的点本来就按 t 递增，包络取 min/max 即可。
    bool any = false;
    CrossingConflict window{hypothesis.obstacle_id, kInf, -kInf, kInf, -kInf};
    for (const PredictedPoint & point : hypothesis.points) {
      const auto projection = line.project(Pose2D{point.x_m, point.y_m, 0.0});
      const double inflated_half_m = params.corridor_half_m + 2.0 * point.sigma_cross_m;
      if (std::abs(projection.lateral_error_m) > inflated_half_m) {
        continue;
      }
      // ego 身后的重叠不构成前方冲突。
      if (projection.s_m <= ego_s_m) {
        continue;
      }
      any = true;
      window.s_lo_m = std::min(window.s_lo_m, projection.s_m);
      window.s_hi_m = std::max(window.s_hi_m, projection.s_m);
      window.t_lo_s = std::min(window.t_lo_s, point.t_s);
      window.t_hi_s = std::max(window.t_hi_s, point.t_s);
    }
    if (!any) {
      continue;
    }
    // 与 ego 的时间重叠（behavior.md §2.2）。t_ego 来自无约束标注 ——
    // 最早到达时间，保守方向。ego 到不了（+∞）则永不冲突。
    //
    // ⚠️ **标注的 t=0 在剖面起点，预测的 t=0 在「现在」（ego 所在处）。**
    //    两个时间轴差一个 t(ego_s) —— ego 不在剖面起点时（全线参考线剖面）
    //    不减掉它整个窗口判定就平移错位。L1 首轮实测踩中：ego 在 s=10、
    //    目标在 s=60，到达时间被算成 10.8 而不是 9.0。
    const double ego_now_s = time_at(arc_lengths_m, speeds_mps, times_s, ego_s_m);
    if (std::isinf(ego_now_s)) {
      // ego 位置本身落在「到不了」的段里（已被停车剖面钉住）：哪儿都去不了。
      continue;
    }
    const double ego_at_lo_s =
      time_at(arc_lengths_m, speeds_mps, times_s, window.s_lo_m) - ego_now_s;
    const double ego_at_hi_s =
      time_at(arc_lengths_m, speeds_mps, times_s, window.s_hi_m) - ego_now_s;
    const bool overlaps = (window.t_lo_s - params.time_margin_s <= ego_at_hi_s) &&
                          (ego_at_lo_s <= window.t_hi_s + params.time_margin_s);
    if (overlaps) {
      conflicts.push_back(window);
    }
  }
  return conflicts;
}

}  // namespace ads_planning
