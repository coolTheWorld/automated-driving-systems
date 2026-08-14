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
//  L1：横向 lattice（P3-S3）
//
//  最重要的两条在最后一节：**planning.md §6 那个可行性不等式的两侧**。
//  它们不是"功能测试"，是把 ODD 边界钉进代码里 ——
//    贴边障碍物（o_l = −0.95 ≤ −0.55）⟹ 必须绕得过去；
//    车道中心障碍物（o_l = +0.25 > −0.55）⟹ **几何上无解**，必须报不可行。
//  第二条尤其要紧：一个"尽力而为"的实现会在这里输出一条撞上去的轨迹，
//  而下游 ads_control 没有碰撞概念，会老老实实跟着开过去。
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "ads_common/reference_line.hpp"
#include "ads_common/testing/path_fixtures.hpp"
#include "ads_planning/lattice.hpp"

namespace
{

using ads_common::ReferenceLine;
using ads_common_test::MakeLeftArc;
using ads_common_test::MakeStraightAlongX;
using ads_planning::Candidate;
using ads_planning::FrenetState;
using ads_planning::LatticeParams;
using ads_planning::LatticeResult;
using ads_planning::LatticeStatus;
using ads_planning::plan_lateral;
using ads_planning::Rectangle;

constexpr double kSampleStepM = 0.5;

/// 与 `config/planning_params.yaml` 和 `config/vehicle_params.yaml` 一致的一组参数。
///
/// ⚠️ **L1 不读 YAML**，所以这里是手抄的一份。改了 YAML 要回来改这里 ——
///    真正的一致性防线在 S5 的 `planning_node` 参数断言，不在这一层。
///    这是与 `ads_control` 相同的做法，理由也一样：让 L1 去解析 YAML
///    会把它从毫秒级拖慢，并给它一个本不该有的文件依赖。
LatticeParams MakeParams()
{
  LatticeParams params;
  // 车道半宽 1.75 − 半车宽 0.90 = 0.85（planning.md §6）
  params.max_lateral_offset_m = 0.85;
  params.lateral_offset_step_m = 0.2;  // ⟹ 9 条横向候选：−0.8 … 0 … +0.8
  params.min_horizon_m = 10.0;
  params.max_horizon_m = 30.0;
  params.horizon_step_m = 10.0;  // ⟹ 3 条纵向候选：10 / 20 / 30
  params.resample_step_m = kSampleStepM;
  params.safety_margin_m = 0.5;  // SPEC §8 场景 S04
  params.safety_floor_m = 0.5;  // 基线用例单级形态（floor == margin）；两级见专门用例
  // 运动学准入 = tan(max_steer 0.6)/wheelbase 2.7 × 转向余量 0.8（P8-S2d）
  params.max_curvature_inv_m = 0.2027;
  params.vehicle_length_m = 4.4;
  params.vehicle_width_m = 1.8;
  params.rear_overhang_m = 0.85;
  params.weight_offset = 1.0;
  params.weight_curvature = 5.0;
  params.weight_clearance = 0.1;
  params.weight_consistency = 0.5;  // **必须 < weight_offset**，见下面的收敛用例
  return params;
}

ReferenceLine MakeStraightLine(double length_m = 60.0)
{
  return ReferenceLine(MakeStraightAlongX(length_m, 0.0, kSampleStepM));
}

/// 起点：车正好在中心线上、朝向与参考线一致。
FrenetState AtOrigin() { return FrenetState{0.0, 0.0, 0.0, 0.0}; }

/// 边长 `size_m` 的正方形障碍物，中心在 (x, d)。参考线沿 +x，所以 d 就是 y。
Rectangle BoxAt(double x_m, double lateral_m, double size_m = 0.5)
{
  return Rectangle{x_m, lateral_m, 0.0, size_m, size_m};
}

// ---------------------------------------------------------------------------
//  基本行为
// ---------------------------------------------------------------------------

TEST(Lattice, NoObstaclesSelectsTheCenterLineExactly)
{
  // planning.md 的回归判据：P3 必须是**叠加**不是替换。
  // 没有障碍物时轨迹必须与中心线**精确**重合，不是"接近"——
  // 差一点点就意味着 P3 一上线，P2 已验收的 CP-P2-B 全部要重新标定。
  const ReferenceLine line = MakeStraightLine();
  const LatticeResult result = plan_lateral(line, AtOrigin(), {}, MakeParams());

  ASSERT_EQ(result.status, LatticeStatus::kOk);
  EXPECT_EQ(result.best.target_offset_m, 0.0);
  EXPECT_EQ(result.blocked_count, 0u);
  EXPECT_EQ(result.candidate_count, 9u * 3u);

  for (const auto & point : result.best.points) {
    EXPECT_NEAR(point.y_m, 0.0, 1e-12) << "x = " << point.x_m;
    EXPECT_NEAR(point.heading_rad, 0.0, 1e-12);
    EXPECT_NEAR(point.curvature_inv_m, 0.0, 1e-12);
  }
  // 无障碍物 ⟹ 最小间距是 +∞，而不是 0 或某个大数。
  // 这样 `weight/∞ = 0`，"没有障碍物"与"障碍物无穷远"在代价上等价。
  EXPECT_TRUE(std::isinf(result.best.min_clearance_m));
}

TEST(Lattice, TrajectoryIsEquallySpacedWithNoDuplicateEndpoint)
{
  // ⚠️ P1 踩过：`ceil(span/step)` + 末点夹到端点 ⟹ **最后两个点重合**，
  //    RViz 里完全看不出来，下游按弧长参数化时除以零。改成等分就没有这个问题。
  const ReferenceLine line = MakeStraightLine();
  const LatticeResult result = plan_lateral(line, AtOrigin(), {}, MakeParams());
  ASSERT_EQ(result.status, LatticeStatus::kOk);

  const auto & points = result.best.points;
  ASSERT_EQ(points.size(), 61u);  // 30 m / 0.5 m + 1

  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double spacing_m =
      std::hypot(points[i + 1].x_m - points[i].x_m, points[i + 1].y_m - points[i].y_m);
    EXPECT_NEAR(spacing_m, kSampleStepM, 1e-9) << "第 " << i << " 段";
    EXPECT_GT(spacing_m, 1e-6) << "第 " << i << " 段退化成零长";
  }
}

TEST(Lattice, ObstacleBehindTheVehicleDoesNotBlockAnything)
{
  // 障碍物在身后 ⟹ 与前视轨迹无关，照走中心线。
  // 这条守的是"把整条参考线上的障碍物都算进来"这类过度保守的实现。
  const ReferenceLine line = MakeStraightLine();
  const LatticeResult result = plan_lateral(line, AtOrigin(), {BoxAt(-10.0, 0.0)}, MakeParams());

  ASSERT_EQ(result.status, LatticeStatus::kOk);
  EXPECT_EQ(result.best.target_offset_m, 0.0);
  EXPECT_EQ(result.blocked_count, 0u);
}

TEST(Lattice, WorksOnACurvedReferenceLine)
{
  // 弯道上 σ = 1 − dκ ≠ 1，Frenet→笛卡尔那条路径才真正被走到。
  // 直线上 σ ≡ 1，很多错误在直线用例里全绿（S2 的注入④就是这样）。
  const ReferenceLine line(MakeLeftArc(20.0, 0.0, 0.0, 0.0, M_PI_2, kSampleStepM, false));
  LatticeParams params = MakeParams();
  params.max_horizon_m = 20.0;  // 圆弧总长约 31 m

  const LatticeResult result = plan_lateral(line, AtOrigin(), {}, params);
  ASSERT_EQ(result.status, LatticeStatus::kOk);
  EXPECT_EQ(result.best.target_offset_m, 0.0);

  // d ≡ 0 的候选，曲率必须等于参考线自身的曲率（κ/(1−0·κ) = κ）。
  for (const auto & point : result.best.points) {
    EXPECT_NEAR(point.curvature_inv_m, 1.0 / 20.0, 2e-3);
  }
}

TEST(Lattice, CloseObstacleForcesAShortManeuverAndTheOffsetIsHeldAfterwards)
{
  // 这条同时验两件事，而它们是同一个设计的两半：
  //
  //  ① **`S` 必须被采样。** 障碍物近在 12 m，`S = 30` 的候选在那儿只走到
  //     0.21·d_T ≈ 0.17 m，根本绕不开；只有 `S = 10` 的能及时到位。
  //     固定 `S` 的实现在这里只能报不可行 —— 而障碍物明明躲得开。
  //
  //  ② **机动做完之后要保持 `d_T`，不能继续外推五次式、更不能跳回 0。**
  //     所有候选都在同样长度（30 m）上评估，所以 `S = 10` 的候选有 20 m 的"保持段"。
  //     外推的话五次式在 3·S 处已经发散到几百米；跳回 0 的话轨迹在 `s = S`
  //     处有一个 0.8 m 的**位置跳变**，而下游控制会照着这个跳变打方向。
  //     两种错误都不会让碰撞检查变红 —— 只有逐段量点距才看得出来。
  const ReferenceLine line = MakeStraightLine();
  const LatticeResult result = plan_lateral(line, AtOrigin(), {BoxAt(12.0, -1.2)}, MakeParams());

  ASSERT_EQ(result.status, LatticeStatus::kOk);
  EXPECT_GT(result.best.target_offset_m, 0.0);
  // 选中的必须是**短机动**，否则说明 S 的采样没起作用。
  EXPECT_LT(result.best.maneuver_span_m, MakeParams().max_horizon_m);

  const auto & points = result.best.points;
  ASSERT_EQ(points.size(), 61u) << "评估长度必须与 S 无关，始终是完整前视距离";

  // 逐段点距：机动段略大于步长（含横向位移），保持段恰好等于步长。
  // 跳变会让某一段变成 √(0.5² + 0.8²) ≈ 0.94，外推会让末段大到离谱。
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double spacing_m =
      std::hypot(points[i + 1].x_m - points[i].x_m, points[i + 1].y_m - points[i].y_m);
    EXPECT_NEAR(spacing_m, kSampleStepM, 0.02) << "第 " << i << " 段出现跳变或外推";
  }

  // 保持段：`s > S` 之后横向偏移必须恒等于 d_T，且曲率回到 0（直线参考线上）。
  for (const auto & point : points) {
    if (point.x_m > result.best.maneuver_span_m + 1e-9) {
      EXPECT_NEAR(point.y_m, result.best.target_offset_m, 1e-9) << "x = " << point.x_m;
      EXPECT_NEAR(point.curvature_inv_m, 0.0, 1e-9) << "x = " << point.x_m;
    }
  }
}

// ---------------------------------------------------------------------------
//  代价函数：每一项都要能被单独观察到
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
//  两级准入（P8-S6，用户拍板）：margin 选新轨迹，floor 保底延续候选。
//  场景数值复刻 CARLA 死锁：ego 带偏移 0.8 在锥旁（锥 −0.95），延续候选的
//  间距 = (0.8 − 0.9) − (−0.95 + 0.25) = 0.6 —— 落在 (floor 0.5, margin 0.7)。
// ---------------------------------------------------------------------------
TEST(Lattice, TwoTierAdmissionLetsContinuationFinishTheManeuver)
{
  const ReferenceLine line = MakeStraightLine();
  LatticeParams params = MakeParams();
  params.safety_margin_m = 0.7;
  params.safety_floor_m = 0.5;
  const FrenetState beside_cone{0.0, 0.8, 0.0, 0.0};
  const std::vector<Rectangle> obstacles{BoxAt(15.0, -0.95)};

  const LatticeResult result = plan_lateral(line, beside_cone, obstacles, params);
  ASSERT_EQ(result.status, LatticeStatus::kOk)
    << "延续候选（间距 0.6 ≥ floor 0.5）被淘汰了 —— 两级准入没生效，死锁回归";
  EXPECT_NEAR(result.best.target_offset_m, 0.8, 1e-12)
    << "幸存的不该是别的候选：所有新机动都会把间距压到 margin 之下";
  EXPECT_GT(result.best.min_clearance_m, params.safety_floor_m);
  EXPECT_LT(result.best.min_clearance_m, params.safety_margin_m);
}

TEST(Lattice, CollapsedTiersReproduceTheDeadlock)
{
  // floor == margin ⟹ 单级退化。同一场景必须全灭 —— 这是把 CARLA 实测的
  // 死锁形态（27/27 blocked、急刹停锥旁 103 s）钉进用例：谁把两级拆了，
  // 这条不红上一条红，两条一起说清楚「为什么要有 floor」。
  const ReferenceLine line = MakeStraightLine();
  LatticeParams params = MakeParams();
  params.safety_margin_m = 0.7;
  params.safety_floor_m = 0.7;
  const FrenetState beside_cone{0.0, 0.8, 0.0, 0.0};
  const std::vector<Rectangle> obstacles{BoxAt(15.0, -0.95)};

  const LatticeResult result = plan_lateral(line, beside_cone, obstacles, params);
  EXPECT_EQ(result.status, LatticeStatus::kAllCandidatesBlocked);
}

TEST(Lattice, ConsistencyTermPullsTowardThePreviousChoice)
{
  const ReferenceLine line = MakeStraightLine();

  // 一致性权重关掉时，无障碍物就该一步回到中心线 —— 哪怕上一周期选了 0.6。
  LatticeParams params = MakeParams();
  params.weight_consistency = 0.0;
  const LatticeResult without = plan_lateral(line, AtOrigin(), {}, params, 0.6);
  ASSERT_EQ(without.status, LatticeStatus::kOk);
  EXPECT_EQ(without.best.target_offset_m, 0.0);

  // 权重拉大之后，同样的输入必须黏在上一周期的选择上。
  // 代价 = 1.0·d² + 100·(d − 0.6)²，连续最优在 0.594 ⟹ 采样点里 0.6 胜出。
  params.weight_consistency = 100.0;
  const LatticeResult with = plan_lateral(line, AtOrigin(), {}, params, 0.6);
  ASSERT_EQ(with.status, LatticeStatus::kOk);
  EXPECT_NEAR(with.best.target_offset_m, 0.6, 1e-12);
}

TEST(Lattice, ReturnsAllTheWayToTheCenterLineOnlyIfConsistencyWeightIsBelowOffsetWeight)
{
  // ⚠️ 这条把 planning_params.yaml 里那个**可推导的界**钉进代码。
  //
  //    连续最优在 d* = ρ·p，ρ = w_c/(w_o + w_c) < 1，所以偏移每周期都在缩。
  //    但采样是离散的：当 |d* − p| < 步长/2 时它会**停在 p 不动**，
  //    代入得「停住 ⟺ w_c > w_o」。
  //
  //    超界的症状极隐蔽：**车绕过障碍物之后永久停在某个偏移上**，
  //    全程不报错、轨迹平滑、也不压线 —— 只是从此再也不走车道中心了。
  const ReferenceLine line = MakeStraightLine();

  // ① 生产配置（w_c = 0.5 < w_o = 1.0）：必须一路收敛到 0，且每周期单调靠近。
  LatticeParams params = MakeParams();
  ASSERT_LT(params.weight_consistency, params.weight_offset) << "生产配置已越界";
  double previous_m = 0.8;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const LatticeResult result = plan_lateral(line, AtOrigin(), {}, params, previous_m);
    ASSERT_EQ(result.status, LatticeStatus::kOk);
    EXPECT_LE(std::abs(result.best.target_offset_m), std::abs(previous_m) + 1e-12)
      << "第 " << cycle << " 周期反而偏得更远了";
    previous_m = result.best.target_offset_m;
  }
  EXPECT_EQ(previous_m, 0.0) << "十个周期还没回到中心线";

  // ② 越界配置（w_c = 2.0 > w_o = 1.0）：会**永久卡在**某个非零偏移上。
  //    留这一半是为了证明上面那一半不是碰巧过的 —— 判据真的在那个界上翻转。
  params.weight_consistency = 2.0;
  previous_m = 0.8;
  for (int cycle = 0; cycle < 10; ++cycle) {
    previous_m = plan_lateral(line, AtOrigin(), {}, params, previous_m).best.target_offset_m;
  }
  EXPECT_NE(previous_m, 0.0) << "越界配置本该卡住，却回到了中心线 —— 那个界算错了";
}

TEST(Lattice, FirstCycleDoesNotTreatZeroAsThePreviousChoice)
{
  // 首次规划（nullopt）时一致性项必须**恒为 0**，而不是拿 0 当"上一次"。
  // 后者会在车本来就偏着的情况下凭空引入一个回中心线的偏好 ——
  // 而那个偏好本该由 weight_offset 单独表达，混进来就没法分别调了。
  const ReferenceLine line = MakeStraightLine();
  LatticeParams params = MakeParams();
  params.weight_offset = 0.0;  // 关掉"贴中心线"，只留一致性项
  params.weight_consistency = 100.0;
  params.weight_curvature = 0.0;

  // 车已经偏在 +0.8 上。首次规划时所有候选代价全为 0 ⟹ 第一个（−0.8）胜出。
  // 关键不是"选了哪个"，而是**它没有偏向 0** —— 若把 nullopt 当成 0，
  // 一致性项会把 0 拉成唯一最优。
  const FrenetState offset_start{0.0, 0.8, 0.0, 0.0};
  const LatticeResult result = plan_lateral(line, offset_start, {}, params, std::nullopt);
  ASSERT_EQ(result.status, LatticeStatus::kOk);
  EXPECT_NE(result.best.target_offset_m, 0.0);
}

TEST(Lattice, ClearanceWeightPushesTheTrajectoryFurtherFromTheObstacle)
{
  // 间距**权重**只在可行候选之间做偏好；它不能、也不该替代准入条件。
  // 这条用例证明它确实起作用：权重拉大后选中的候选离障碍物更远。
  const ReferenceLine line = MakeStraightLine();
  const std::vector<Rectangle> obstacles{BoxAt(25.0, -1.2)};

  LatticeParams params = MakeParams();
  const LatticeResult cheap = plan_lateral(line, AtOrigin(), obstacles, params);
  ASSERT_EQ(cheap.status, LatticeStatus::kOk);

  params.weight_clearance = 100.0;
  const LatticeResult expensive = plan_lateral(line, AtOrigin(), obstacles, params);
  ASSERT_EQ(expensive.status, LatticeStatus::kOk);

  EXPECT_GT(expensive.best.min_clearance_m, cheap.best.min_clearance_m);
  EXPECT_GT(expensive.best.target_offset_m, cheap.best.target_offset_m);
}

// ---------------------------------------------------------------------------
//  planning.md §6 的可行性不等式 —— 本文件最重要的两条
// ---------------------------------------------------------------------------

TEST(Lattice, EdgeObstacleIsAvoidedOnTheOppositeSideWithTheRequiredMargin)
{
  // 障碍物贴右边：0.5×0.5，中心 d = −1.2 ⟹ 左缘 o_l = −0.95 ≤ −0.55 ⟹ **可绕**。
  // 可行区间 d ∈ [o_l + g + w/2, W/2 − w/2] = [0.45, 0.85]，有 0.4 m 实打实的裕度。
  const ReferenceLine line = MakeStraightLine();
  const LatticeResult result = plan_lateral(line, AtOrigin(), {BoxAt(25.0, -1.2)}, MakeParams());

  ASSERT_EQ(result.status, LatticeStatus::kOk);
  // 往**左**绕（远离障碍物），不是往右。符号写反的话这里立刻红。
  EXPECT_GT(result.best.target_offset_m, 0.0);
  // 落在可行区间里。
  EXPECT_GE(result.best.target_offset_m, 0.45);
  EXPECT_LE(result.best.target_offset_m, 0.85);
  // **判据本身**：全程侧向间距 ≥ 0.5 m（SPEC §8 S04）。
  EXPECT_GE(result.best.min_clearance_m, MakeParams().safety_margin_m);
  // 往右的那些候选必须被淘汰掉，否则说明碰撞检查根本没生效。
  EXPECT_GT(result.blocked_count, 0u);
}

TEST(Lattice, CenterLaneObstacleIsGeometricallyInfeasibleAndReportedAsSuch)
{
  // ⚠️ **本文件最重要的一条。**
  //    同样的 0.5×0.5 障碍物挪到车道正中（d = 0）⟹ 左缘 o_l = +0.25 > −0.55。
  //    从左绕要 d ≥ 0.75 + 0.9 = 1.65，从右绕要 d ≤ −1.65，
  //    而车道内不压线的上限只有 0.85 —— **几何上无解**。
  //
  //    一个"尽力而为"的实现会在这里返回"最不糟的那条"，而那条是撞上去的；
  //    下游 ads_control 没有碰撞概念，会老老实实跟着开过去。
  //    正确行为是明确报不可行，由上层转入停车（planning.md §6 决策四）。
  const ReferenceLine line = MakeStraightLine();
  const LatticeResult result = plan_lateral(line, AtOrigin(), {BoxAt(25.0, 0.0)}, MakeParams());

  EXPECT_EQ(result.status, LatticeStatus::kAllCandidatesBlocked);
  // 「27 条候选全被淘汰」和「一条候选都没生成」是完全不同的故障，
  // diagnostics 必须分得开 —— 所以两个计数都要对。
  EXPECT_EQ(result.candidate_count, 27u);
  EXPECT_EQ(result.blocked_count, 27u);
}

TEST(Lattice, WideObstacleThatFitsNowhereIsAlsoInfeasible)
{
  // 另一侧的边界：障碍物窄一点但更靠中间，仍然过不去。
  // 用两条不同的位形，是因为「恰好卡在不等式边界上」和「离边界很远」
  // 在实现里走的是同一条路径，但只测一条的话，边界算错半个车宽也发现不了。
  const ReferenceLine line = MakeStraightLine();
  const LatticeResult result =
    plan_lateral(line, AtOrigin(), {BoxAt(25.0, -0.4, 1.6)}, MakeParams());
  EXPECT_EQ(result.status, LatticeStatus::kAllCandidatesBlocked);
}

// ---------------------------------------------------------------------------
//  几何细节与守卫
// ---------------------------------------------------------------------------

TEST(Lattice, CollisionUsesTheBodyCenterNotTheRearAxle)
{
  // ⚠️ 轨迹点是**后轴**位姿，碰撞检查要的是车体几何中心 ——
  //    两者差 length/2 − rear_overhang = 2.2 − 0.85 = 1.35 m。
  //    漏掉这一步，碰撞检查整体沿车头方向偏 1.35 m，而轨迹、代价、日志全部正常。
  //
  //    构造：把候选压成只有一条（中心线），障碍物放在轨迹末点正前方 3.0 m。
  //      加了偏移 ⟹ 车头在末点前 1.35 + 2.2 = 3.55 m，够到障碍物近端 2.75 m ⟹ 拦下
  //      漏了偏移 ⟹ 车头只到 2.2 m，间距 0.55 > 0.5 ⟹ 放行
  //    所以本条用例的"通过"与"不通过"之间只隔着那 1.35 m。
  LatticeParams params = MakeParams();
  params.max_lateral_offset_m = 0.05;  // < step ⟹ 只剩 d_T = 0 这一条横向候选
  params.min_horizon_m = 30.0;
  params.max_horizon_m = 30.0;

  const ReferenceLine line = MakeStraightLine();
  const LatticeResult result = plan_lateral(line, AtOrigin(), {BoxAt(33.0, 0.0)}, params);

  ASSERT_EQ(result.candidate_count, 1u) << "构造失败：候选不止一条，这条用例就不成立了";
  EXPECT_EQ(result.status, LatticeStatus::kAllCandidatesBlocked);
}

TEST(Lattice, EndOfRouteWithLateralTransientHoldsTheOffsetInsteadOfStopping)
{
  // P8-S2d 的核心用例，P7-S4 junction 实测的最小复现：出弯残余 0.5 m 横向
  // 偏差 + 剩 1.2 m 跑道。修复前：网格候选全是「在 1 m 跨度上强行收敛」的
  // 五次式，峰值曲率 ≈ 2.0（R = 0.5 m，车开不出来）—— 它们要么被旁边障碍物
  // 挡下（现场：全候选淘汰 → 车在离 goal 1.16 m 处 NO_PATH 停保持），
  // 要么被选中后让车蠕行。修复 = 两件事咬合：
  //   ① 运动学准入门把开不出来的网格候选全部淘汰（密采样五次式峰值曲率 ——
  //      输出点距 0.5 m 在短机动上恰好采在 |d″| 零点，峰值藏在点之间）；
  //   ② 短跑道保持候选（不做新机动，保持当前偏移走完）补上唯一合理的选项。
  //
  // 故障注入实测（2026-08-13，写完立刻做的）：
  //   去掉保持候选 ② → 本用例红（kAllCandidatesBlocked —— 门是对的，
  //                     那些机动确实做不出来，缺的是合理选项）；
  //   去掉曲率门 ①（上限调到 1e9）→ 本用例红（d_T=0 的爆曲率候选胜出，
  //                     target_offset 断言失败 —— 修复前的蠕行/被挡形态）。
  const ReferenceLine line = MakeStraightLine(100.0);
  const FrenetState near_end{line.length_m() - 1.2, 0.5, 0.0, 0.0};
  const LatticeResult result = plan_lateral(line, near_end, {}, MakeParams());

  ASSERT_EQ(result.status, LatticeStatus::kOk) << "末端短跑道 + 横向暂态必须仍有可行轨迹";
  // 胜出的必须是保持候选：目标 ≈ 当前偏移（不是网格上的强行收敛值）。
  EXPECT_NEAR(result.best.target_offset_m, 0.5, 1e-9);
  // 网格候选（Δd ≥ 0.1）在 1.2 m 跨度上峰值 |d″| ≥ 0.4，全部过不了运动学门。
  EXPECT_GE(result.curvature_blocked_count, 8u)
    << "曲率门没有拦住短跨度强行收敛的候选 —— 它们是车开不出来的";
  // 保持候选的几何必须是平的：任何一点的曲率都远小于门限
  // （修复前被选中的候选峰值 ≈ 2.0，藏在输出采样点之间）。
  for (const auto & pt : result.best.points) {
    EXPECT_LT(std::abs(pt.curvature_inv_m), 0.01);
  }
}

TEST(Lattice, HeadingTransientOnShortRunwayStillYieldsATrackableTrajectory)
{
  // P8-S2d 第二刀（junction 实测第二回暴露的）：**d′ 暂态**版本的末端边界。
  // 车离终点 1.5 m、带 0.2 的横向速度（航向差 11°—— Stanley 弯道出口的
  // 常态量级）。保持候选如果用五次式（把 d′ 压回零再稳住），峰值
  // |d″| ≈ 0.53 > 门限 0.20 —— **连保持候选都被运动学门淘汰**，
  // 全候选出局 → kStopping：实测车在离 goal 1.47 m 处停保持，
  // 而车道笔直、前方空无一物。
  //
  // 修法：保持候选改**二次衰减**剖面（d″ = −d′₀/S 常值，消掉 d′ 的
  // 最小峰值曲率剖面）：κ ≈ d′₀/S = 0.13 < 门限，终点漂到自然落点
  // d₀ + d′₀·S/2。
  //
  // 故障注入实测（2026-08-13）：保持候选退回五次式几何 → 本用例红
  // （kStopping —— 二次衰减是它能存活的唯一原因）。
  const ReferenceLine line = MakeStraightLine(100.0);
  const FrenetState near_end{line.length_m() - 1.5, -0.2, 0.2, 0.0};
  const LatticeResult result = plan_lateral(line, near_end, {}, MakeParams());

  ASSERT_EQ(result.status, LatticeStatus::kOk)
    << "带航向暂态的末端短跑道必须仍有可行轨迹（二次衰减保持候选）";
  // 自然落点 = −0.2 + 0.2·1.5/2 = −0.05。
  EXPECT_NEAR(result.best.target_offset_m, -0.05, 1e-9);
  // 几何曲率必须在运动学门之内（车开得出来）。
  for (const auto & pt : result.best.points) {
    EXPECT_LT(std::abs(pt.curvature_inv_m), 0.2027);
  }
}

TEST(Lattice, RunwayPermittingPartialConvergenceStillConverges)
{
  // 保持候选**不能**抢走还够跑道的场景：剩 5 m 时 Δd=0.3 的温和收敛
  // （峰值 |d″| ≈ 5.77×0.3/25 ≈ 0.07 < 门限）仍然可行且代价更低 ——
  // 末端处理是「跑道允许多少收敛多少」，不是「一到末端就放弃回中心」。
  const ReferenceLine line = MakeStraightLine(100.0);
  const FrenetState approaching{line.length_m() - 5.0, 0.5, 0.0, 0.0};
  const LatticeResult result = plan_lateral(line, approaching, {}, MakeParams());

  ASSERT_EQ(result.status, LatticeStatus::kOk);
  EXPECT_LT(std::abs(result.best.target_offset_m), 0.5 - 1e-9) << "跑道还够温和收敛时不该原地保持";
}

TEST(Lattice, ReportsHorizonTooShortNearTheEndOfTheRoute)
{
  // 「路走到头了」和「全被挡住了」需要**不同的**下游动作
  // （前者是任务完成，后者要停车让行），所以状态必须分开。
  const ReferenceLine line = MakeStraightLine();
  const FrenetState near_end{line.length_m() - 0.2, 0.0, 0.0, 0.0};
  const LatticeResult result = plan_lateral(line, near_end, {}, MakeParams());

  EXPECT_EQ(result.status, LatticeStatus::kHorizonTooShort);
  EXPECT_EQ(result.candidate_count, 0u);
}

TEST(Lattice, TruncatesTheHorizonAtTheEndOfTheReferenceLineInsteadOfThrowing)
{
  // 剩余 15 m < max_horizon 30 m：应当照常规划，只是轨迹短一些。
  // 让 ReferenceLine::at() 的越界异常穿透上去是错的 —— "路走到头了"是策略，
  // 该由这一层决定。
  const ReferenceLine line = MakeStraightLine();
  const FrenetState near_end{line.length_m() - 15.0, 0.0, 0.0, 0.0};
  const LatticeResult result = plan_lateral(line, near_end, {}, MakeParams());

  ASSERT_EQ(result.status, LatticeStatus::kOk);
  EXPECT_EQ(result.best.points.size(), 31u);  // 15 m / 0.5 m + 1
  EXPECT_NEAR(result.best.points.back().x_m, line.length_m(), 1e-9);
}

TEST(Lattice, ThrowsOnInvalidParameters)
{
  const ReferenceLine line = MakeStraightLine();
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  LatticeParams bad = MakeParams();
  bad.resample_step_m = 0.0;
  EXPECT_THROW(plan_lateral(line, AtOrigin(), {}, bad), std::invalid_argument);

  bad = MakeParams();
  bad.safety_margin_m = -0.1;
  EXPECT_THROW(plan_lateral(line, AtOrigin(), {}, bad), std::invalid_argument);

  bad = MakeParams();
  bad.safety_floor_m = 0.0;  // floor 与 margin 同规矩：不许为 0（准入不可关）
  EXPECT_THROW(plan_lateral(line, AtOrigin(), {}, bad), std::invalid_argument);

  bad = MakeParams();
  bad.safety_floor_m = bad.safety_margin_m + 0.1;  // 保底高于选择线 = 配置错误
  EXPECT_THROW(plan_lateral(line, AtOrigin(), {}, bad), std::invalid_argument);

  bad = MakeParams();
  bad.min_horizon_m = 40.0;  // > max
  EXPECT_THROW(plan_lateral(line, AtOrigin(), {}, bad), std::invalid_argument);

  bad = MakeParams();
  bad.weight_offset = kNaN;
  EXPECT_THROW(plan_lateral(line, AtOrigin(), {}, bad), std::invalid_argument);

  // 把「后轴到几何中心的距离」误填进 rear_overhang 是个真实会犯的错，
  // 而它的后果（碰撞检查整体偏移）完全不报错 —— 所以显式拦一道。
  bad = MakeParams();
  bad.rear_overhang_m = 3.0;  // > 半车长 2.2
  EXPECT_THROW(plan_lateral(line, AtOrigin(), {}, bad), std::invalid_argument);
}

TEST(Lattice, ThrowsOnNonFiniteObstacleOrStartState)
{
  // ⚠️ 不判的话不会崩，只会让每一次距离比较都返回 false ——
  //    表现为「所有候选都没碰撞」，车直接开过去。
  //    **不报错、只给一个看起来能用的结果**，本仓库已因这个模式吃过多次亏。
  const ReferenceLine line = MakeStraightLine();
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();

  EXPECT_THROW(
    plan_lateral(line, AtOrigin(), {BoxAt(kNaN, 0.0)}, MakeParams()), std::invalid_argument);
  EXPECT_THROW(
    plan_lateral(line, AtOrigin(), {BoxAt(25.0, kInf)}, MakeParams()), std::invalid_argument);
  EXPECT_THROW(
    plan_lateral(line, AtOrigin(), {Rectangle{25.0, 0.0, 0.0, -1.0, 0.5}}, MakeParams()),
    std::invalid_argument);
  EXPECT_THROW(
    plan_lateral(line, FrenetState{0.0, kNaN, 0.0, 0.0}, {}, MakeParams()), std::invalid_argument);
}

TEST(Lattice, ZeroSizeObstacleIsAcceptedRatherThanRejected)
{
  // 零尺寸是 SPEC §8 点名的退化情形，碰撞检查能正确处理（当成一个点），
  // 所以**不抛异常** —— 与负尺寸有本质区别。
  const ReferenceLine line = MakeStraightLine();
  EXPECT_NO_THROW(plan_lateral(line, AtOrigin(), {BoxAt(25.0, -1.2, 0.0)}, MakeParams()));
}

}  // namespace

// ---------------------------------------------------------------------------
//  ⚠️ margin=0 后门（2026-08-12 复检堵上的）
// ---------------------------------------------------------------------------
TEST(Lattice, RejectsZeroSafetyMarginOutright)
{
  // ⚠️ 这不是普通的参数校验用例。margin=0 曾经是**合法取值**（require_non_negative
  //    放行），而 distance_m() 对重叠的 OBB 返回 0.0 ⟹ 准入判据 `0.0 < 0.0`
  //    恒为假 ⟹ **撞上去的候选不被淘汰** —— 一个伪装成阈值的碰撞检查关闭开关，
  //    SPEC §11 明令禁止的东西。
  //
  //    防线有两道：① 这里钉死的 require_positive；② 准入处与 margin 无关的
  //    `min_clearance_m <= 0.0` 硬性淘汰。② 在 ① 存在时**不可达**（任何正
  //    margin 都先拦住重叠），无法用常规用例覆盖 —— 它防的是 ① 被改掉的未来。
  //    注入实测（2026-08-12）：把 ① revert 回 non_negative ⟹ 本用例红。
  const ReferenceLine line = MakeStraightLine();
  LatticeParams params = MakeParams();
  params.safety_margin_m = 0.0;
  EXPECT_THROW(ads_planning::plan_lateral(line, AtOrigin(), {}, params), std::invalid_argument)
    << "margin=0 被放行了 —— 碰撞准入可以被配置关掉（SPEC §11 禁止）";
}
