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

// =============================================================================
//  约束合成 + 行为仲裁的 L1 测试（CP-P7-A ③⑤⑦ + ② 的滞回半边）
//
//  ⑤ 的判据是**解析**的（behavior.md §2.1）：恒速前车 v_L 下，
//  ego 沿停车剖面 v(s)=√(2a(s_stop−s)) 跟随，稳态真值间距收敛到
//      stand_off + v_L²/(2·a_dec)
//  仿真只是把离散映射跑到不动点，判据来自物理不来自代码。
//
//  ## 故障注入实测（2026-08-13，写完立刻做的）
//
//  | 注入 | 结果 |
//  |---|---|
//  | merge_constraints 的 min 改成「后写覆盖」
//    | **红** 1 例：MergeTakesTheMostConservative（两个约束顺序敌对排列，
//    覆盖必挑错一个） |
//  | 滞回去掉（任何标签变化立即生效）
//    | **红** 1 例：EnterImmediatelyReleaseNeedsStability（闪断第 1 拍
//    标签就抖出去了） |
// =============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

#include "ads_common/reference_line.hpp"
#include "ads_planning/conflict.hpp"
#include "ads_planning/longitudinal.hpp"

namespace
{

using ads_common::Pose2D;
using ads_common::ReferenceLine;
using ads_planning::BehaviorArbiter;
using ads_planning::BehaviorParams;
using ads_planning::BehaviorState;
using ads_planning::LongitudinalConstraint;
using ads_planning::merge_constraints;
using ads_planning::PredictionHypothesis;
using ads_planning::SpeedCap;
using ads_planning::TargetBox;

BehaviorParams MakeParams()
{
  BehaviorParams params;
  params.corridor_half_m = 1.75;
  params.blocking_half_m = 0.55;  // = 0.9 + 0.5 − 0.85（推导量，见 conflict.hpp）
  params.stand_off_m = 4.0;
  params.yield_margin_m = 4.0;  // S4 实测从 2.0 上调：让行点也要停在盲区外
  params.time_margin_s = 1.0;
  params.front_offset_m = 3.55;
  return params;
}

ReferenceLine MakeStraightLine(double length_m, double spacing_m)
{
  std::vector<Pose2D> poses;
  for (double x = 0.0; x <= length_m + 1e-9; x += spacing_m) {
    poses.push_back({x, 0.0, 0.0});
  }
  return ReferenceLine(std::move(poses));
}

/// 恒速剖面三件套（弧长/速度）。
void MakeConstantProfile(
  double length_m, double speed_mps, std::vector<double> * s, std::vector<double> * v)
{
  for (double x = 0.0; x <= length_m + 1e-9; x += 0.5) {
    s->push_back(x);
    v->push_back(speed_mps);
  }
}

// -----------------------------------------------------------------------------
//  ③ 合成取最保守
// -----------------------------------------------------------------------------

TEST(MergeConstraints, MergeTakesTheMostConservative)
{
  // 两个方向都排一遍：**覆盖式合成**（谁后来听谁的）在其中一个排列下必然挑错。
  LongitudinalConstraint near;
  near.stop_at_s_m = 6.0;
  LongitudinalConstraint far;
  far.stop_at_s_m = 10.0;

  EXPECT_NEAR(*merge_constraints({near, far}).stop_at_s_m, 6.0, 1e-12);
  EXPECT_NEAR(*merge_constraints({far, near}).stop_at_s_m, 6.0, 1e-12);

  // caps 取并集（消费方逐点 min），一个都不许丢。
  LongitudinalConstraint with_cap;
  with_cap.caps.push_back(SpeedCap{5.0, 15.0, 2.0});
  const auto merged = merge_constraints({near, with_cap});
  EXPECT_NEAR(*merged.stop_at_s_m, 6.0, 1e-12);
  ASSERT_EQ(merged.caps.size(), 1u);
  EXPECT_NEAR(merged.caps[0].v_cap_mps, 2.0, 1e-12);
}

TEST(MergeConstraints, EmptyInputMeansNoConstraint)
{
  const auto merged = merge_constraints({});
  EXPECT_FALSE(merged.stop_at_s_m.has_value());
  EXPECT_TRUE(merged.caps.empty());
}

// -----------------------------------------------------------------------------
//  ⑤ 跟停稳态收敛到解析间距
// -----------------------------------------------------------------------------

TEST(BehaviorArbiter, FollowConvergesToAnalyticGap)
{
  // 离散映射：每周期 T 前车推进 v_L·T，仲裁给出新 stop_at，
  // ego 精确跟随停车剖面 v(s) = √(2·a·(stop − s))（取本周期 stop）。
  // 稳态间距解析值 = stand_off + v_L²/(2a)（behavior.md §2.1）。
  const auto params = MakeParams();
  const auto line = MakeStraightLine(400.0, 0.5);
  BehaviorArbiter arbiter(params, 5);

  const double kCycleS = 0.1;
  const double kLeadSpeed = 3.0;
  const double kDecel = 3.0;
  const double kLength = 4.4;

  double lead_center = 40.0;
  double ego_s = 0.0;
  double ego_v = 5.556;
  std::vector<double> s, v;
  MakeConstantProfile(400.0, 5.556, &s, &v);

  for (int k = 0; k < 1200; ++k) {
    const std::vector<TargetBox> targets{{21u, lead_center, 0.0, kLength, 1.8}};
    const auto decision = arbiter.decide(line, ego_s, targets, {}, s, v);
    ASSERT_TRUE(decision.constraint.stop_at_s_m.has_value());
    const double stop = *decision.constraint.stop_at_s_m;
    // 剖面速度（对当前停车点），叠加巡航上限。
    const double profile_v =
      stop > ego_s ? std::min(5.556, std::sqrt(2.0 * kDecel * (stop - ego_s))) : 0.0;
    ego_v = profile_v;
    ego_s += ego_v * kCycleS;
    lead_center += kLeadSpeed * kCycleS;
  }

  // 真值间距 = 前车近边 − ego 车头面。
  const double gap = (lead_center - kLength / 2.0) - (ego_s + params.front_offset_m);
  const double analytic = params.stand_off_m + kLeadSpeed * kLeadSpeed / (2.0 * kDecel);
  std::cout << "[⑤] 稳态间距实测 " << gap << " m，解析 " << analytic << " m\n";
  EXPECT_NEAR(gap, analytic, 0.02) << "0.02 = 半个周期的相对位移（离散映射的极限环幅度）";
  // 顺带钉住它落在 CP-P7-B ② 的判据带里。
  EXPECT_GE(gap, 4.0);
  EXPECT_LE(gap, 10.0);
}

TEST(BehaviorArbiter, StaticLeadStopsAtStandOff)
{
  // ⑥ 的约束半边：静止前车 → 停车点 = 近边 − front_offset − stand_off，
  // 停稳后真值间距恰好 = stand_off。
  const auto params = MakeParams();
  const auto line = MakeStraightLine(100.0, 0.5);
  BehaviorArbiter arbiter(params, 5);
  std::vector<double> s, v;
  MakeConstantProfile(100.0, 5.556, &s, &v);

  const std::vector<TargetBox> targets{{22u, 60.0, 0.0, 4.4, 1.8}};
  const auto decision = arbiter.decide(line, 10.0, targets, {}, s, v);
  EXPECT_EQ(decision.state, BehaviorState::kFollow);
  ASSERT_TRUE(decision.constraint.stop_at_s_m.has_value());
  EXPECT_NEAR(*decision.constraint.stop_at_s_m, 60.0 - 2.2 - 3.55 - 4.0, 1e-9);
}

// -----------------------------------------------------------------------------
//  ⑦ 椭圆只作用横穿类
// -----------------------------------------------------------------------------

TEST(BehaviorArbiter, FollowIgnoresPredictionEllipse)
{
  // 同一个 id 既有感知框（走廊内）又有带巨大 σ 的 STATIC 预测。
  // 跟停的 stop_at 必须**只**由感知近边决定 —— σ 再大也不许把停车点推远。
  // （σ=6 时 2σ 膨胀 12 m：若跟停吃了椭圆，stop_at 至少往回退 12 m。）
  const auto params = MakeParams();
  const auto line = MakeStraightLine(100.0, 0.5);
  std::vector<double> s, v;
  MakeConstantProfile(100.0, 5.556, &s, &v);

  PredictionHypothesis inflated;
  inflated.obstacle_id = 23u;
  for (double t = 0.0; t <= 3.0 + 1e-9; t += 0.25) {
    inflated.points.push_back({t, 60.0, 0.0, 6.0});
  }
  const std::vector<TargetBox> targets{{23u, 60.0, 0.0, 4.4, 1.8}};

  BehaviorArbiter arbiter(params, 5);
  const auto decision = arbiter.decide(line, 10.0, targets, {inflated}, s, v);
  EXPECT_EQ(decision.state, BehaviorState::kFollow);
  EXPECT_TRUE(decision.crossings.empty()) << "已按 FOLLOW 处理的目标不再进横穿判定（§2.3）";
  ASSERT_TRUE(decision.constraint.stop_at_s_m.has_value());
  EXPECT_NEAR(*decision.constraint.stop_at_s_m, 60.0 - 2.2 - 3.55 - 4.0, 1e-9)
    << "σ=6 的椭圆不许影响跟停点";
}

// -----------------------------------------------------------------------------
//  ② 滞回（标签侧；短路语义在 test_behavior_tree）
// -----------------------------------------------------------------------------

TEST(BehaviorArbiter, EnterImmediatelyReleaseNeedsStability)
{
  const auto params = MakeParams();
  const auto line = MakeStraightLine(100.0, 0.5);
  const int kRelease = 5;
  BehaviorArbiter arbiter(params, kRelease);
  std::vector<double> s, v;
  MakeConstantProfile(100.0, 5.556, &s, &v);

  const std::vector<TargetBox> lead{{24u, 60.0, 0.0, 4.4, 1.8}};
  const std::vector<TargetBox> none{};

  // 进入：第一拍就是 FOLLOW（安全方向不等待）。
  EXPECT_EQ(arbiter.decide(line, 10.0, lead, {}, s, v).state, BehaviorState::kFollow);

  // 闪断 4 拍（< release_cycles）：标签必须钉在 FOLLOW —— 走廊边缘抖动不许把
  // 状态机抖出去（CP-P7-B ⑨ 的机理）。约束照常消失（不经过滞回）。
  for (int k = 0; k < kRelease - 1; ++k) {
    const auto decision = arbiter.decide(line, 10.0, none, {}, s, v);
    EXPECT_EQ(decision.state, BehaviorState::kFollow) << "闪断第 " << k + 1 << " 拍";
    EXPECT_FALSE(decision.constraint.stop_at_s_m.has_value()) << "约束不滞回，车已在恢复";
  }
  // 冲突回来：计数清零，标签仍 FOLLOW。
  EXPECT_EQ(arbiter.decide(line, 10.0, lead, {}, s, v).state, BehaviorState::kFollow);
  // 连续 5 拍干净才释放。
  for (int k = 0; k < kRelease - 1; ++k) {
    EXPECT_EQ(arbiter.decide(line, 10.0, none, {}, s, v).state, BehaviorState::kFollow);
  }
  EXPECT_EQ(arbiter.decide(line, 10.0, none, {}, s, v).state, BehaviorState::kCruise);
}

TEST(BehaviorArbiter, YieldLabelForCrossingOnly)
{
  // 走廊外的目标 + 会横穿的预测 ⟹ YIELD 标签 + 让行停车点。
  const auto params = MakeParams();
  const auto line = MakeStraightLine(100.0, 0.5);
  BehaviorArbiter arbiter(params, 5);
  std::vector<double> s, v;
  MakeConstantProfile(100.0, 5.556, &s, &v);

  // 慢速横穿（0.75 m/s）：入廊 t=4.0、出廊 8.67，ego（恒速 5.556）到 s=60 是
  // t=9.0 —— 落在窗口 +τ 内（9 ≤ 8.5+1）。行人快到 1.5 m/s 的话 4.25 就穿完了，
  // ego 根本追不上冲突 —— 时序不重叠的横穿**本来就不该让**，那是另一个用例的事。
  PredictionHypothesis crossing;
  crossing.obstacle_id = 25u;
  for (double t = 0.0; t <= 9.0 + 1e-9; t += 0.25) {
    crossing.points.push_back({t, 60.0, -4.75 + 0.75 * t, 0.0});
  }
  // 感知框在路肩（走廊外）——不构成 FOLLOW。
  const std::vector<TargetBox> targets{{25u, 60.0, -4.75, 0.4, 0.4}};
  const auto decision = arbiter.decide(line, 10.0, targets, {crossing}, s, v);
  EXPECT_EQ(decision.state, BehaviorState::kYield);
  EXPECT_FALSE(decision.follow.has_value());
  ASSERT_EQ(decision.crossings.size(), 1u);
  ASSERT_TRUE(decision.constraint.stop_at_s_m.has_value());
  EXPECT_NEAR(*decision.constraint.stop_at_s_m, 60.0 - 3.55 - 4.0, 1e-9)
    << "让行停车点 = 冲突区入口 − front_offset − yield_margin";
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
