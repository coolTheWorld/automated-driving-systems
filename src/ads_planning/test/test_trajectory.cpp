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
//  L1：轨迹装配 + 停车剖面（P3-S4）
//
//  本文件验的是 P3 的**第二个交付能力** —— 绕不过去时停住。
//  它不是异常分支：SPEC §2 第 3 条原文就是「安全减速**或**绕行」，
//  而 planning.md §6 的不等式说明「绕不过去」在本项目的车道宽下**很容易发生**。
//
//  两条最要紧的：
//    - 停车点必须按**车体外廓**算，不是轨迹点。车头比后轴前伸 3.55 m，
//      拿轨迹点算的话车会一头顶上去才停。
//    - 停车轨迹**保持当前横向位置**，不回中心线。明知前方受阻还主动横移，
//      是往刚刚被判定为不可行的那一侧走。
// =============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "ads_common/reference_line.hpp"
#include "ads_common/testing/path_fixtures.hpp"
#include "ads_planning/trajectory.hpp"

namespace
{

using ads_common::ReferenceLine;
using ads_common_test::MakeStraightAlongX;
using ads_planning::FrenetState;
using ads_planning::plan;
using ads_planning::PlanParams;
using ads_planning::PlanResult;
using ads_planning::PlanStatus;
using ads_planning::Rectangle;

constexpr double kSampleStepM = 0.5;
constexpr double kCruiseSpeedMps = 5.556;  // 20 km/h
constexpr double kMaxDecelMps2 = 3.0;

/// 与 config/planning_params.yaml + vehicle_params.yaml 一致的一组参数。
/// ⚠️ L1 不读 YAML（会把毫秒级测试拖慢并给它一个文件依赖），这里是手抄的一份。
PlanParams MakeParams()
{
  PlanParams params;
  params.lattice.max_lateral_offset_m = 0.85;
  params.lattice.lateral_offset_step_m = 0.2;
  params.lattice.min_horizon_m = 10.0;
  params.lattice.max_horizon_m = 30.0;
  params.lattice.horizon_step_m = 10.0;
  params.lattice.resample_step_m = kSampleStepM;
  params.lattice.safety_margin_m = 0.5;
  params.lattice.vehicle_length_m = 4.4;
  params.lattice.vehicle_width_m = 1.8;
  params.lattice.rear_overhang_m = 0.85;
  params.lattice.weight_offset = 1.0;
  params.lattice.weight_curvature = 5.0;
  params.lattice.weight_clearance = 0.1;
  params.lattice.weight_consistency = 0.5;

  params.speed.cruise_speed_mps = kCruiseSpeedMps;
  params.speed.max_lateral_accel_mps2 = 1.15;  // P3-S5 从 1.5 降下来，见 planning_params.yaml
  params.speed.max_accel_mps2 = 1.5;
  params.speed.max_decel_mps2 = kMaxDecelMps2;

  params.stop_margin_m = 1.0;
  return params;
}

ReferenceLine MakeStraightLine(double length_m = 80.0)
{
  return ReferenceLine(MakeStraightAlongX(length_m, 0.0, kSampleStepM));
}

FrenetState AtOrigin() { return FrenetState{0.0, 0.0, 0.0, 0.0}; }

Rectangle BoxAt(double x_m, double lateral_m, double size_m = 0.5)
{
  return Rectangle{x_m, lateral_m, 0.0, size_m, size_m};
}

// ---------------------------------------------------------------------------
//  正常路径：几何 + 速度装配
// ---------------------------------------------------------------------------

TEST(Trajectory, NoObstaclesCruisesAlongTheCenterLine)
{
  const PlanResult result = plan(MakeStraightLine(), AtOrigin(), {}, MakeParams());

  ASSERT_EQ(result.status, PlanStatus::kOk);
  EXPECT_EQ(result.lateral_offset_m, 0.0);
  ASSERT_FALSE(result.points.empty());

  for (const auto & point : result.points) {
    EXPECT_NEAR(point.y_m, 0.0, 1e-12);
    // 直线中心线上没有曲率限速，也没到终点（前视 30 m < 路长 80 m），
    // 所以全程就是巡航速度。
    EXPECT_NEAR(point.speed_mps, kCruiseSpeedMps, 1e-9) << "s = " << point.s_m;
    EXPECT_NEAR(point.accel_mps2, 0.0, 1e-9) << "s = " << point.s_m;
  }
}

TEST(Trajectory, ArcLengthIsCumulativeChordNotIndexTimesStep)
{
  // ⚠️ 弧长必须按**实际弦长**累加。用"序号 × 采样步长"在弯道上偏得很厉害
  //    （P2 实测：本地图弯道外侧实际点距比标称大 14.58%），
  //    后果是曲率限速偏小 6.58% —— 车明显开得慢，且没有任何一层报错。
  const PlanResult result = plan(MakeStraightLine(), AtOrigin(), {}, MakeParams());
  ASSERT_EQ(result.status, PlanStatus::kOk);

  EXPECT_EQ(result.points.front().s_m, 0.0);
  for (std::size_t i = 1; i < result.points.size(); ++i) {
    const double chord_m = std::hypot(
      result.points[i].x_m - result.points[i - 1].x_m,
      result.points[i].y_m - result.points[i - 1].y_m);
    EXPECT_NEAR(result.points[i].s_m - result.points[i - 1].s_m, chord_m, 1e-12) << "第 " << i;
  }
}

TEST(Trajectory, AvoidanceTrajectoryIsSpeedLimitedByItsOwnCurvatureNotTheCenterLines)
{
  // ⚠️ **这是 S4 的 4.2**：速度剖面必须算在**选中的候选**上，不是参考线上。
  //    绕行轨迹的曲率比中心线大，限速因此更低 —— 而这个"更低"是**算出来的**。
  //    留在控制侧算的话它拿到的永远是中心线，减速纯属巧合。
  //
  //    把 a_lat_max 调得很小，让曲率限速真正咬住（默认 1.5 时这段机动太缓，
  //    限速算出来高于巡航速度，看不出差别 —— 那样这条用例就什么都没验）。
  PlanParams params = MakeParams();
  params.speed.max_lateral_accel_mps2 = 0.05;

  const ReferenceLine line = MakeStraightLine();
  const PlanResult straight = plan(line, AtOrigin(), {}, params);
  const PlanResult avoiding = plan(line, AtOrigin(), {BoxAt(25.0, -1.2)}, params);

  ASSERT_EQ(straight.status, PlanStatus::kOk);
  ASSERT_EQ(avoiding.status, PlanStatus::kOk);
  ASSERT_GT(avoiding.lateral_offset_m, 0.0) << "没有绕行，这条用例就不成立";

  const auto min_speed = [](const PlanResult & r) {
    double lowest = r.points.front().speed_mps;
    for (const auto & p : r.points) {
      lowest = std::min(lowest, p.speed_mps);
    }
    return lowest;
  };

  // 中心线是直线 ⟹ 曲率 0 ⟹ 无论 a_lat 多小都不限速。
  EXPECT_NEAR(min_speed(straight), kCruiseSpeedMps, 1e-9);
  // 绕行轨迹有曲率 ⟹ 被限速。
  EXPECT_LT(min_speed(avoiding), kCruiseSpeedMps - 0.1);
}

TEST(Trajectory, AccelerationIsHalfTheSpatialDerivativeOfSpeedSquared)
{
  // a_ff = ½·d(v²)/ds。剖面存的是 v²（不是 v），正是为了让这一项是**闭式**的。
  // 不给它的后果实测过：纯 P 速度环跟踪斜坡的稳态误差 = 斜率/K_p = 3.0 m/s。
  // ⚠️ 起点必须离路径末端**近于前视距离**，否则窗口是被前视截的、末点不归零，
  //    整条轨迹就是匀速，这条用例什么都验不到（初稿就是这么错的）。
  //    路长 40 m、前视 30 m、起点 s=25 ⟹ 剩余 15 m < 30 m ⟹ 末点就是终点。
  const ReferenceLine line = MakeStraightLine(40.0);
  const FrenetState near_end{25.0, 0.0, 0.0, 0.0};
  const PlanResult result = plan(line, near_end, {}, MakeParams());
  ASSERT_EQ(result.status, PlanStatus::kOk);

  bool saw_deceleration = false;
  for (std::size_t i = 0; i + 1 < result.points.size(); ++i) {
    const double ds_m = result.points[i + 1].s_m - result.points[i].s_m;
    ASSERT_GT(ds_m, 1e-9);
    const double v0 = result.points[i].speed_mps;
    const double v1 = result.points[i + 1].speed_mps;
    EXPECT_NEAR(result.points[i].accel_mps2, 0.5 * (v1 * v1 - v0 * v0) / ds_m, 1e-9) << "第 " << i;
    if (result.points[i].accel_mps2 < -0.1) {
      saw_deceleration = true;
    }
  }
  EXPECT_TRUE(saw_deceleration) << "全程没有减速段，这条用例其实没验到什么";
  EXPECT_NEAR(result.points.back().speed_mps, 0.0, 1e-9) << "路径末端必须停住";
}

// ---------------------------------------------------------------------------
//  停车路径（planning.md §6 决策四）—— P3 的第二个交付能力
// ---------------------------------------------------------------------------

TEST(Trajectory, BlockedLaneProducesAStoppingTrajectoryInsteadOfTheLeastBadCandidate)
{
  // 障碍物在车道正中（o_l = +0.25 > −0.55）⟹ 几何上无解。
  // 一个"尽力而为"的实现会返回最优候选，而那条是撞上去的 ——
  // 下游 ads_control 没有碰撞概念，会老老实实跟着开过去。
  const PlanResult result = plan(MakeStraightLine(), AtOrigin(), {BoxAt(25.0, 0.0)}, MakeParams());

  EXPECT_EQ(result.status, PlanStatus::kStopping);
  EXPECT_EQ(result.blocked_count, result.candidate_count);
  ASSERT_GE(result.points.size(), 2u);
  EXPECT_NEAR(result.points.back().speed_mps, 0.0, 1e-9) << "停车轨迹的末点速度必须是 0";
}

TEST(Trajectory, StopPointKeepsTheRequiredClearanceMeasuredOnTheVehicleBody)
{
  // ⚠️ **S4 的 4.3 判据。**
  //    停车点必须按**车体外廓**算，不是轨迹点 —— 车头比后轴前伸
  //    length/2 + (length/2 − rear_overhang) = 2.2 + 1.35 = 3.55 m。
  //    拿轨迹点算的话，车会一头顶上去才停，而"停车点距障碍物 25 m"
  //    这种日志读起来完全正常。
  const PlanParams params = MakeParams();
  const PlanResult result = plan(MakeStraightLine(), AtOrigin(), {BoxAt(25.0, 0.0)}, params);

  ASSERT_EQ(result.status, PlanStatus::kStopping);
  // 车体外廓到障碍物的实际间距 ≥ 侧向判据 + 停车裕度。
  EXPECT_GE(result.stop_clearance_m, params.lattice.safety_margin_m);
  EXPECT_GE(result.stop_clearance_m, params.stop_margin_m - 1e-9)
    << "停车裕度没生效：stop_margin_m = " << params.stop_margin_m;
  // 但也不能保守到"离得老远就停"——那在窄路上会寸步难行。
  EXPECT_LT(result.stop_clearance_m, params.stop_margin_m + 2.0);
}

TEST(Trajectory, StoppingTrajectoryHoldsTheCurrentLateralOffsetInsteadOfReturningToCenter)
{
  // 明知前方受阻还主动横移，是往刚刚被判定为不可行的那一侧走。
  // 所以停车轨迹保持当前横向位置。
  const FrenetState offset_start{0.0, 0.6, 0.0, 0.0};
  const PlanResult result =
    plan(MakeStraightLine(), offset_start, {BoxAt(25.0, 0.6)}, MakeParams());

  ASSERT_EQ(result.status, PlanStatus::kStopping);
  EXPECT_NEAR(result.lateral_offset_m, 0.6, 1e-12);
  for (const auto & point : result.points) {
    EXPECT_NEAR(point.y_m, 0.6, 1e-6) << "s = " << point.s_m;
  }
}

TEST(Trajectory, StoppingDecelerationNeverExceedsTheConfiguredLimit)
{
  // 停车剖面不另写减速逻辑，而是把几何截断后跑同一个 SpeedProfile ——
  // 它本来就在末点强制 v = 0 并向后扫描。这条用例确认那条复用真的生效：
  // 减速度不超过 max_decel。写成 +max_decel（符号搞反）这里立刻红。
  const PlanResult result = plan(MakeStraightLine(), AtOrigin(), {BoxAt(25.0, 0.0)}, MakeParams());
  ASSERT_EQ(result.status, PlanStatus::kStopping);

  for (std::size_t i = 0; i + 1 < result.points.size(); ++i) {
    EXPECT_GE(result.points[i].accel_mps2, -kMaxDecelMps2 - 1e-9) << "第 " << i << " 段减速超限";
    // 速度必须单调不增（一路减到 0），不能中途又加回去。
    EXPECT_LE(result.points[i + 1].speed_mps, result.points[i].speed_mps + 1e-9) << "第 " << i;
  }
}

TEST(Trajectory, EmergencyStopWhenTheVehicleIsAlreadyTooClose)
{
  // 车已经贴到障碍物上了 —— 本不该发生（上一周期就该停住），
  // 但真发生时要给一条**明确的零速轨迹**，而不是空数组。
  // 空数组会走进下游"没有轨迹"的降级分支，那条路径的语义是"规划器挂了"，
  // 与"规划器说停"必须分得开。
  const PlanResult result = plan(MakeStraightLine(), AtOrigin(), {BoxAt(3.0, 0.0)}, MakeParams());

  EXPECT_EQ(result.status, PlanStatus::kStopping);
  ASSERT_FALSE(result.points.empty());
  for (const auto & point : result.points) {
    EXPECT_EQ(point.speed_mps, 0.0);
  }
}

// ---------------------------------------------------------------------------
//  状态与回环
// ---------------------------------------------------------------------------

TEST(Trajectory, ReportsRouteExhaustedNearTheEndOfTheReferenceLine)
{
  // 「路走到头了」与「被挡住了」需要**不同的**下游动作
  // （前者是任务完成，后者要停车让行），所以状态必须分开。
  const ReferenceLine line = MakeStraightLine();
  const FrenetState near_end{line.length_m() - 0.2, 0.0, 0.0, 0.0};
  const PlanResult result = plan(line, near_end, {}, MakeParams());

  EXPECT_EQ(result.status, PlanStatus::kRouteExhausted);
  EXPECT_TRUE(result.points.empty());
}

TEST(Trajectory, LateralOffsetIsFedBackSoTheNextCycleStaysConsistent)
{
  // 不把 lateral_offset_m 传回去的话，每周期都是"首次规划"，
  // 一致性项永远为 0，车会在候选之间跳。这条用例确认那个回环量确实被暴露出来。
  const ReferenceLine line = MakeStraightLine();
  const std::vector<Rectangle> obstacles{BoxAt(25.0, -1.2)};

  const PlanResult first = plan(line, AtOrigin(), obstacles, MakeParams());
  ASSERT_EQ(first.status, PlanStatus::kOk);
  ASSERT_GT(first.lateral_offset_m, 0.0);

  // 把上一周期的结果喂回去，同样的输入应给出同样的选择（不抖）。
  const PlanResult second = plan(line, AtOrigin(), obstacles, MakeParams(), first.lateral_offset_m);
  ASSERT_EQ(second.status, PlanStatus::kOk);
  EXPECT_EQ(second.lateral_offset_m, first.lateral_offset_m);
}

TEST(Trajectory, ThrowsOnNegativeStopMargin)
{
  PlanParams bad = MakeParams();
  bad.stop_margin_m = -0.5;
  EXPECT_THROW(plan(MakeStraightLine(), AtOrigin(), {}, bad), std::invalid_argument);
}

}  // namespace

// =============================================================================
//  行为约束注入（P7-S3）—— stop_at 走停车剖面机制、caps 走逐点上限
//
//  ## 故障注入实测（2026-08-13，写完立刻做的）
//
//  | 注入 | 结果 |
//  |---|---|
//  | kStopping 分支不取行为/静态停车点的更早者（去掉行为截断）
//    | **红** 1 例：BehaviorStopEarlierThanStaticObstacleWins |
// =============================================================================

TEST(BehaviorConstraint, StopTruncatesAtTheRequestedPoint)
{
  const ReferenceLine line = MakeStraightLine();
  ads_planning::LongitudinalConstraint constraint;
  constraint.stop_at_s_m = 15.0;
  const PlanResult result = plan(line, AtOrigin(), {}, MakeParams(), std::nullopt, &constraint);

  ASSERT_EQ(result.status, PlanStatus::kOk);
  ASSERT_GE(result.points.size(), 2u);
  // 末点落在停车点上（向下取到采样格），且速度为 0。
  EXPECT_LE(result.points.back().s_m, 15.0 + 1e-9);
  EXPECT_GT(result.points.back().s_m, 15.0 - kSampleStepM - 1e-9);
  EXPECT_NEAR(result.points.back().speed_mps, 0.0, 1e-12);
  // 减速段是标准剖面：停车点前 5 m 处 v = √(2·a_dec·5)。
  for (const auto & point : result.points) {
    if (std::abs(point.s_m - (result.points.back().s_m - 5.0)) < 1e-9) {
      EXPECT_NEAR(point.speed_mps, std::sqrt(2.0 * kMaxDecelMps2 * 5.0), 1e-9);
    }
  }
}

TEST(BehaviorConstraint, StopBeyondTheWindowChangesNothing)
{
  // 停车点在 30 m 窗口之外：轨迹必须与无约束时**逐点严格相等** ——
  // 行为层只收紧不放宽，「没约束到」与「没有约束」得是同一件事。
  const ReferenceLine line = MakeStraightLine();
  const PlanResult baseline = plan(line, AtOrigin(), {}, MakeParams());

  ads_planning::LongitudinalConstraint constraint;
  constraint.stop_at_s_m = 100.0;
  const PlanResult constrained =
    plan(line, AtOrigin(), {}, MakeParams(), std::nullopt, &constraint);

  ASSERT_EQ(constrained.points.size(), baseline.points.size());
  for (std::size_t i = 0; i < baseline.points.size(); ++i) {
    EXPECT_EQ(constrained.points[i].speed_mps, baseline.points[i].speed_mps) << "第 " << i << " 点";
    EXPECT_EQ(constrained.points[i].s_m, baseline.points[i].s_m) << "第 " << i << " 点";
  }
}

TEST(BehaviorConstraint, StopEarlierThanStaticObstacleWins)
{
  // 车道被堵（kStopping 路径）+ 行为停车点更早：取更早者。
  // 反向注入（去掉 kStopping 分支里的行为截断）时本用例红。
  const ReferenceLine line = MakeStraightLine();
  const std::vector<Rectangle> obstacles{BoxAt(20.0, 0.85), BoxAt(20.0, 0.0), BoxAt(20.0, -0.85)};
  const PlanResult blocked = plan(line, AtOrigin(), obstacles, MakeParams());
  ASSERT_EQ(blocked.status, PlanStatus::kStopping);
  const double static_stop_s = blocked.points.back().s_m;

  ads_planning::LongitudinalConstraint constraint;
  constraint.stop_at_s_m = 8.0;
  ASSERT_LT(8.0, static_stop_s) << "前提：行为停车点确实更早，否则本用例什么都没测";
  const PlanResult both =
    plan(line, AtOrigin(), obstacles, MakeParams(), std::nullopt, &constraint);
  ASSERT_EQ(both.status, PlanStatus::kStopping);
  EXPECT_LE(both.points.back().s_m, 8.0 + 1e-9);
  EXPECT_NEAR(both.points.back().speed_mps, 0.0, 1e-12);
}

TEST(BehaviorConstraint, StopBehindEgoDegradesToSinglePointZero)
{
  // ego 已越过行为停车点（本不该发生）：单点零速，下游按 NO_PATH 刹停保持。
  const ReferenceLine line = MakeStraightLine();
  ads_planning::LongitudinalConstraint constraint;
  constraint.stop_at_s_m = -1.0;
  const PlanResult result = plan(line, AtOrigin(), {}, MakeParams(), std::nullopt, &constraint);
  ASSERT_EQ(result.points.size(), 1u);
  EXPECT_NEAR(result.points[0].speed_mps, 0.0, 1e-12);
}

TEST(BehaviorConstraint, CapsFlowThroughToTheProfile)
{
  // s ∈ [10, 20]（参考线系）压到 2.0：窗口内该段处处 ≤ 2.0，
  // 且两端按扫描斜坡过渡（斜坡本身在 test_speed_profile 里已闭式对账，
  // 这里只验「参考线系 → 候选系」的映射没有错位）。
  const ReferenceLine line = MakeStraightLine();
  ads_planning::LongitudinalConstraint constraint;
  constraint.caps.push_back(ads_planning::SpeedCap{10.0, 20.0, 2.0});
  const PlanResult result = plan(line, AtOrigin(), {}, MakeParams(), std::nullopt, &constraint);

  ASSERT_EQ(result.status, PlanStatus::kOk);
  bool saw_capped = false;
  for (const auto & point : result.points) {
    if (point.s_m >= 10.0 && point.s_m <= 20.0) {
      EXPECT_LE(point.speed_mps, 2.0 + 1e-9) << "s = " << point.s_m;
      saw_capped = true;
    }
  }
  EXPECT_TRUE(saw_capped) << "窗口没覆盖到限速段 —— 用例本身没激励";
}
