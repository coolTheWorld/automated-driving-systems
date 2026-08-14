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

#ifndef ADS_PLANNING__LONGITUDINAL_HPP_
#define ADS_PLANNING__LONGITUDINAL_HPP_

// =============================================================================
//  longitudinal —— 纵向约束与行为仲裁（P7-S2）
//
//  推导见 docs/modules/behavior.md §3–§4。**改这里先读它。**
//
//  职责链：conflict 出「事实」→ 本文件把事实换算成「约束」并**在树外**
//  取最保守合成（merge，CP-P7-A ③ 的注入验红钉着 min）；
//  行为树只挑一个**状态标签**给诊断和滞回用 —— 标签不参与安全裁决，
//  它晚一拍、抖一下都不影响约束（这正是滞回可以只作用在标签上的原因）。
// =============================================================================

#include <optional>
#include <vector>

#include "ads_common/reference_line.hpp"
#include "ads_planning/conflict.hpp"

namespace ads_planning
{

/// @brief 一段逐点限速，参考线弧长系。
struct SpeedCap
{
  double s_from_m{0.0};
  double s_to_m{0.0};
  double v_cap_mps{0.0};
};

/// @brief 行为层交给 plan() 的纵向约束。
struct LongitudinalConstraint
{
  /// 后轴该停住的弧长（参考线系）。空 = 不停。
  std::optional<double> stop_at_s_m;
  /// 逐点限速段集合。v1 行为层暂不发（机制就位，S4 实测后按需启用）。
  std::vector<SpeedCap> caps;
};

/// @brief 最保守合成：停车点取 min，限速段取并集（消费方逐点取 min）。
///
/// min 是安全的充分条件：停在更早的点 ⟹ 不越过任何一个约束点。
/// ⚠️ CP-P7-A ③ 用注入钉着这条：改成 max 或「后写覆盖」，两个目标时
///    会撞上更近的那个 —— 而单目标场景全绿。
LongitudinalConstraint merge_constraints(const std::vector<LongitudinalConstraint> & constraints);

/// @brief 行为状态标签。给诊断与滞回用，**不参与约束裁决**。
enum class BehaviorState
{
  kCruise,
  kFollow,
  kYield,
  // S06 红灯（P8，决策四最小闭环）。标签仅供诊断 —— 约束照旧树外合成。
  kRedLight,
};

/// @brief 状态名（诊断字符串）。
const char * BehaviorStateName(BehaviorState state);

/// @brief 行为仲裁器：组装树、做滞回、出约束。planning_node 每周期调一次。
///
/// 滞回只在**释放方向**（behavior.md §3）：
///   进入 FOLLOW/YIELD（从 CRUISE）：立即 —— 安全方向不等待；
///   其余任何标签变化（F→Y、Y→F、→CRUISE）：新标签连续 release_cycles
///   个周期稳定才切换。约束本身每周期重算重合成，**不经过滞回**。
class BehaviorArbiter
{
public:
  /// @param params         行为参数（见 conflict.hpp / behavior.md §5）。
  /// @param release_cycles 释放滞回周期数。0.5 s / 10 Hz = 5。
  /// @throw std::invalid_argument 参数非法（各字段正值检查）。
  BehaviorArbiter(const BehaviorParams & params, int release_cycles);

  /// @brief 一次仲裁的全部输出。
  struct Decision
  {
    /// 滞回后的状态标签（诊断用）。
    BehaviorState state{BehaviorState::kCruise};
    /// 合成后的纵向约束（安全用，未经滞回）。
    LongitudinalConstraint constraint;
    /// 本周期的原始冲突（诊断用）。
    std::optional<FollowConflict> follow;
    std::vector<CrossingConflict> crossings;
  };

  /// @brief 每规划周期调一次。
  ///
  /// @param line       参考线。
  /// @param ego_s_m    ego 后轴当前弧长。
  /// @param targets    感知目标快照。
  /// @param hypotheses 预测假设（**全部**给进来 —— 类别裁决在里面做：
  ///                   已按 FOLLOW 处理的目标，其假设不再进横穿判定，
  ///                   behavior.md §2.3）。
  /// @param arc_lengths_m / speeds_mps 无约束剖面（时间标注的输入）。
  /// @param red_light_stop_s_m 红灯停止线的参考线弧长（米）；无红灯传 nullopt。
  ///                   由 planning_node 从 /traffic_light/state 换算：
  ///                   state ≠ GREEN ⟹ 传入（YELLOW/UNKNOWN 都按红处理 ——
  ///                   「不知道灯色」在语义上不允许通过）；GREEN/无灯 ⟹ nullopt。
  Decision decide(
    const ads_common::ReferenceLine & line, double ego_s_m, const std::vector<TargetBox> & targets,
    const std::vector<PredictionHypothesis> & hypotheses, const std::vector<double> & arc_lengths_m,
    const std::vector<double> & speeds_mps,
    std::optional<double> red_light_stop_s_m = std::nullopt);

private:
  BehaviorParams params_;
  int release_cycles_;

  BehaviorState published_{BehaviorState::kCruise};
  BehaviorState pending_{BehaviorState::kCruise};
  int pending_count_{0};
};

}  // namespace ads_planning

#endif  // ADS_PLANNING__LONGITUDINAL_HPP_
