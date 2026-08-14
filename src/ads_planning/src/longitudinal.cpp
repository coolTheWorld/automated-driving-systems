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

#include "ads_planning/longitudinal.hpp"

#include <algorithm>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ads_common/numeric_checks.hpp"
#include "ads_planning/behavior_tree.hpp"

namespace ads_planning
{

using ads_common::ReferenceLine;
using ads_common::RequireFiniteNonNegative;
using ads_common::RequireFinitePositive;

LongitudinalConstraint merge_constraints(const std::vector<LongitudinalConstraint> & constraints)
{
  LongitudinalConstraint merged;
  for (const LongitudinalConstraint & constraint : constraints) {
    if (constraint.stop_at_s_m.has_value()) {
      // **min，不是覆盖**。CP-P7-A ③ 的注入验红钉着这一行。
      merged.stop_at_s_m = merged.stop_at_s_m.has_value()
                             ? std::min(*merged.stop_at_s_m, *constraint.stop_at_s_m)
                             : constraint.stop_at_s_m;
    }
    merged.caps.insert(merged.caps.end(), constraint.caps.begin(), constraint.caps.end());
  }
  return merged;
}

const char * BehaviorStateName(BehaviorState state)
{
  switch (state) {
    case BehaviorState::kCruise:
      return "CRUISE";
    case BehaviorState::kFollow:
      return "FOLLOW";
    case BehaviorState::kYield:
      return "YIELD";
    case BehaviorState::kRedLight:
      return "RED_LIGHT";
  }
  return "UNKNOWN";
}

BehaviorArbiter::BehaviorArbiter(const BehaviorParams & params, int release_cycles)
: params_(params), release_cycles_(release_cycles)
{
  RequireFinitePositive(params.corridor_half_m, "BehaviorParams", "corridor_half_m");
  // blocking_half 只要求有限：max_lateral_offset 大于车半宽+间距时它合法为负
  // （那时只有横跨中心两侧的目标才算阻挡）。
  ads_common::RequireFinite(params.blocking_half_m, "BehaviorParams", "blocking_half_m");
  RequireFinitePositive(params.sigma_inflation_cap_m, "BehaviorParams", "sigma_inflation_cap_m");
  RequireFinitePositive(params.stand_off_m, "BehaviorParams", "stand_off_m");
  RequireFinitePositive(params.yield_margin_m, "BehaviorParams", "yield_margin_m");
  RequireFiniteNonNegative(params.time_margin_s, "BehaviorParams", "time_margin_s");
  RequireFinitePositive(params.front_offset_m, "BehaviorParams", "front_offset_m");
  if (release_cycles < 1) {
    throw std::invalid_argument("BehaviorArbiter: release_cycles 必须 ≥ 1");
  }
}

BehaviorArbiter::Decision BehaviorArbiter::decide(
  const ReferenceLine & line, double ego_s_m, const std::vector<TargetBox> & targets,
  const std::vector<PredictionHypothesis> & hypotheses, const std::vector<double> & arc_lengths_m,
  const std::vector<double> & speeds_mps, std::optional<double> red_light_stop_s_m)
{
  Decision decision;

  // ---- ① 冲突事实（conflict.cpp，纯几何）------------------------------------
  decision.follow = find_follow_conflict(line, ego_s_m, targets, params_);

  // 类别裁决（behavior.md §2.3）：已按 FOLLOW 处理的目标不再进横穿判定。
  // 一个目标只进一类 —— 两类同时捕获时约束等价（min 兜底），但标签会
  // FOLLOW/YIELD 交替，⑨ 判据会红。
  std::vector<PredictionHypothesis> crossing_hypotheses;
  crossing_hypotheses.reserve(hypotheses.size());
  for (const PredictionHypothesis & hypothesis : hypotheses) {
    if (decision.follow.has_value() && hypothesis.obstacle_id == decision.follow->id) {
      continue;
    }
    crossing_hypotheses.push_back(hypothesis);
  }
  const std::vector<double> times_s = annotate_times(arc_lengths_m, speeds_mps);
  decision.crossings = find_crossing_conflicts(
    line, ego_s_m, arc_lengths_m, speeds_mps, times_s, crossing_hypotheses, params_);

  // ---- ② 事实 → 约束，树外合成（红线：不可关，不经过树）---------------------
  std::vector<LongitudinalConstraint> constraints;
  if (decision.follow.has_value()) {
    LongitudinalConstraint follow_constraint;
    follow_constraint.stop_at_s_m =
      decision.follow->near_edge_s_m - params_.front_offset_m - params_.stand_off_m;
    constraints.push_back(std::move(follow_constraint));
  }
  for (const CrossingConflict & crossing : decision.crossings) {
    LongitudinalConstraint yield_constraint;
    yield_constraint.stop_at_s_m =
      crossing.s_lo_m - params_.front_offset_m - params_.yield_margin_m;
    constraints.push_back(std::move(yield_constraint));
  }
  // 红灯（S06 最小闭环）：停止线 − 车头偏置 − 边距。边距 1.0 把车头停进
  // SPEC「停止线前 0–2 m」判据带的中央。已越过停车点（stop_at ≤ ego_s）时
  // 约束照给 —— plan() 的截断路径会给单点零速（与行为停车语义一致），
  // 「冲进路口后急停」还是「硬闯」是个安全裁决，不在这里做，先停住再说。
  if (red_light_stop_s_m.has_value()) {
    LongitudinalConstraint red_constraint;
    red_constraint.stop_at_s_m =
      *red_light_stop_s_m - params_.front_offset_m - params_.red_light_margin_m;
    constraints.push_back(std::move(red_constraint));
  }
  decision.constraint = merge_constraints(constraints);

  // ---- ③ 行为树挑标签（只挑标签；显式树，无 XML）----------------------------
  BehaviorState raw = BehaviorState::kCruise;
  Fallback tree(collect(
    // 红灯分支放最前：标签优先级 = 谁的停车点语义最"制度性"。
    // 只挑标签 —— 红灯约束在上面②，树关不掉它（红线三条之一）。
    std::make_unique<Sequence>(collect(
      std::make_unique<Condition>([&] { return red_light_stop_s_m.has_value(); }),
      Action::always([&] { raw = BehaviorState::kRedLight; }))),
    std::make_unique<Sequence>(collect(
      std::make_unique<Condition>([&] { return decision.follow.has_value(); }),
      Action::always([&] { raw = BehaviorState::kFollow; }))),
    std::make_unique<Sequence>(collect(
      std::make_unique<Condition>([&] { return !decision.crossings.empty(); }),
      Action::always([&] { raw = BehaviorState::kYield; }))),
    Action::always([&] { raw = BehaviorState::kCruise; })));
  tree.tick();

  // ---- ④ 标签滞回（只在释放/换挡方向；进入立即）-----------------------------
  if (raw == published_) {
    pending_count_ = 0;
  } else if (published_ == BehaviorState::kCruise) {
    // CRUISE → 活动状态：立即。晚一拍 = 晚刹 0.1 s，安全方向不等待。
    published_ = raw;
    pending_count_ = 0;
  } else {
    // 其余变化（→CRUISE、F↔Y）：新标签连续 release_cycles 周期稳定才切。
    // 约束不经过这里 —— 冲突消失的那个周期约束就已经没了，车已在恢复；
    // 滞回防的是**标签**抖动（诊断可读性 + CP-P7-B ⑨）。
    if (raw == pending_) {
      ++pending_count_;
    } else {
      pending_ = raw;
      pending_count_ = 1;
    }
    if (pending_count_ >= release_cycles_) {
      published_ = raw;
      pending_count_ = 0;
    }
  }
  decision.state = published_;
  return decision;
}

}  // namespace ads_planning
