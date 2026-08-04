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
//  速度剖面的 L1 测试
//
//  剖面是**纯几何量**，所以每一条判据都能写成闭式解，不需要仿真：
//    * 曲率限速     v = √(a_lat/|κ|)
//    * 制动距离     d = (v₁² − v₂²) / (2·a)
//  用闭式解当判据的好处是**改了实现也不用改判据** —— 判据来自物理，不来自代码。
//
//  另有两条判据是**物理不变量**而不是数值：
//    * 处处 v²·|κ| ≤ a_lat_max        （不会有哪一点过弯太快）
//    * 处处满足前后向的匀加速关系      （处处减得下来、加得上去）
//  它们不依赖任何一条具体路径，所以拿一条混合路径跑一遍就能覆盖全部形状。
// =============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

#include "ads_common/reference_line.hpp"
#include "ads_common/testing/path_fixtures.hpp"
#include "ads_planning/speed_profile.hpp"

namespace
{

using ads_common::PathPoint;
using ads_common::Pose2D;
using ads_common::ReferenceLine;
using ads_common_test::AppendStraight;
using ads_common_test::MakeLeftArc;
using ads_common_test::MakeRightArc;
using ads_common_test::MakeStraightAlongX;
using ads_common_test::MakeStraightThenLeftArc;
using ads_planning::SpeedProfile;
using ads_planning::SpeedProfileParams;

// config/vehicle_params.yaml 与 control_params.yaml 的**手抄副本**（同 test_stanley.cpp
// 的说明：L1 不读 YAML，改了 YAML 要回来改这里，真正的防线在 S4 的一致性断言）。
constexpr double kCruiseSpeedMps = 5.556;  // 20 km/h
constexpr double kMaxLateralAccelMps2 = 1.5;
constexpr double kMaxAccelMps2 = 1.5;
constexpr double kMaxDecelMps2 = 3.0;

// 地图上的三种半径（config/campus_map.yaml）。
constexpr double kTurnRadiusM = 8.0;  // 路口转弯车道
constexpr double kCornerOuterRadiusM = 13.75;

SpeedProfileParams DefaultParams()
{
  return SpeedProfileParams{kCruiseSpeedMps, kMaxLateralAccelMps2, kMaxAccelMps2, kMaxDecelMps2};
}

/// 曲率限速的闭式解。
double CurvatureLimitMps(double radius_m)
{
  return std::min(kCruiseSpeedMps, std::sqrt(kMaxLateralAccelMps2 * radius_m));
}

/// 匀加速公式反解出的距离：从 v_from 变到 v_to 需要多长。
double DistanceForSpeedChangeM(double from_mps, double to_mps, double accel_mps2)
{
  return std::abs(from_mps * from_mps - to_mps * to_mps) / (2.0 * accel_mps2);
}

/// 一条混合路径：直路 20 m → R=8 左转 90° → 直路 20 m。
/// 三种几何、两处曲率跳变，够覆盖全部约束。
std::vector<Pose2D> MakeMixedPath(double spacing_m)
{
  std::vector<Pose2D> poses = MakeStraightThenLeftArc(20.0, kTurnRadiusM, M_PI_2, spacing_m);
  AppendStraight(&poses, 20.0, spacing_m);
  return poses;
}

}  // namespace

// =============================================================================
//  参数校验
// =============================================================================

TEST(SpeedProfileParamsCheck, RejectNonPositiveOrNonFiniteValues)
{
  const ReferenceLine path(MakeStraightAlongX(20.0, 0.0, 0.5));
  const std::vector<double> bad_values{0.0, -1.0, std::nan(""), HUGE_VAL};
  for (const double bad : bad_values) {
    SpeedProfileParams p = DefaultParams();
    p.cruise_speed_mps = bad;
    EXPECT_THROW(SpeedProfile(path, p), std::invalid_argument) << "cruise_speed_mps = " << bad;

    p = DefaultParams();
    p.max_lateral_accel_mps2 = bad;
    EXPECT_THROW(SpeedProfile(path, p), std::invalid_argument)
      << "max_lateral_accel_mps2 = " << bad;

    p = DefaultParams();
    p.max_accel_mps2 = bad;
    EXPECT_THROW(SpeedProfile(path, p), std::invalid_argument) << "max_accel_mps2 = " << bad;

    p = DefaultParams();
    p.max_decel_mps2 = bad;
    EXPECT_THROW(SpeedProfile(path, p), std::invalid_argument) << "max_decel_mps2 = " << bad;
  }
}

// =============================================================================
//  ① 曲率限速（任务 3.1）
// =============================================================================

TEST(CurvatureLimit, MatchesTheClosedFormOnEachRadiusOnTheMap)
{
  // R=8（路口转弯车道）：√(1.5×8) = 3.4641 m/s = 12.5 km/h。
  // 定速 20 km/h 过这个弯，横向加速度是 3.86 m/s² —— 紧急变道的量级。
  // R=13.75（环线四角外侧）：√(1.5×13.75) = 4.5415 m/s，仍低于巡航值。
  //
  // ⚠️ **判据不能卡到 1e-9**，而偏差的来源是可以精确写出来的：
  //    弧长按**弦长**累加（ReferenceLine 的做法），弦比弧短一个因子 (1 − Δφ²/24)，
  //    于是中心差分算出的曲率**偏大**同一个因子：κ̂ = (1/R)(1 + Δφ²/24)，Δφ = h/R。
  //    限速 ∝ 1/√κ，所以偏**小** (1 − Δφ²/48)：
  //      R=8,     h=0.5 → Δφ=0.0625  → 偏小 2.82e-4 m/s
  //      R=13.75, h=0.5 → Δφ=0.03636 → 偏小 1.25e-4 m/s
  //    实测就是这两个数（对到三位有效数字），所以判据放 1e-3 并**打印实测值**。
  //
  //    方向是**保守**的（限速偏小 = 过弯更慢），这一点单独断言 ——
  //    哪天它变成偏大，就是弧长或曲率的符号/公式被改坏了。
  //
  // 圆弧取 40 m 长：终点减速只影响末尾 2 m（3.4641 → 0 按 −3.0），中段才是纯曲率限速。
  struct Case
  {
    double radius_m;
    double expected_mps;
  };
  const Case cases[] = {
    {kTurnRadiusM, 3.4641016151377544}, {kCornerOuterRadiusM, 4.541475531146237}};

  for (const Case & c : cases) {
    const ReferenceLine path(MakeLeftArc(c.radius_m, 0.0, 0.0, 0.0, 40.0 / c.radius_m, 0.5, false));
    const SpeedProfile profile(path, DefaultParams());
    const double measured_mps = profile.speeds_mps()[20];

    ASSERT_NEAR(c.expected_mps, CurvatureLimitMps(c.radius_m), 1e-12) << "闭式解自己先要自洽";
    std::cout << "  [剖面] R=" << c.radius_m << " 限速 " << measured_mps << " m/s（闭式解 "
              << c.expected_mps << "，偏差 " << (c.expected_mps - measured_mps) << "）\n";

    EXPECT_NEAR(measured_mps, c.expected_mps, 1e-3);
    EXPECT_LE(measured_mps, c.expected_mps) << "离散化必须让限速偏保守，偏大说明公式被改坏了";
  }
}

TEST(CurvatureLimit, RightTurnsAreLimitedIdenticallyToLeftTurns)
{
  // 限速只关**曲率的绝对值**，左右转必须给出同一条剖面。
  //
  // 这条用例是故障注入时补上的：在此之前**所有**测试路径都是左转，
  // 而漏掉 std::abs 的实现会在右转（κ < 0）上算出 √(负数) = NaN ——
  // 全左转的用例里一条都不红，而地图上左右转各占一半。
  const ReferenceLine left(MakeLeftArc(kTurnRadiusM, 0.0, 0.0, 0.0, 40.0 / 8.0, 0.5, false));
  const ReferenceLine right(MakeRightArc(kTurnRadiusM, 0.0, 0.0, 0.0, 40.0 / 8.0, 0.5, false));
  const SpeedProfile left_profile(left, DefaultParams());
  const SpeedProfile right_profile(right, DefaultParams());

  ASSERT_LT(right.points()[20].curvature_inv_m, 0.0)
    << "右转的曲率应当为负，否则这条用例没测到东西";
  ASSERT_EQ(left_profile.speeds_mps().size(), right_profile.speeds_mps().size());

  double worst_mps = 0.0;
  for (std::size_t i = 0; i < left_profile.speeds_mps().size(); ++i) {
    ASSERT_TRUE(std::isfinite(right_profile.speeds_mps()[i])) << "第 " << i << " 点是非有限值";
    worst_mps =
      std::max(worst_mps, std::abs(left_profile.speeds_mps()[i] - right_profile.speeds_mps()[i]));
  }
  std::cout << "  [剖面] 左转 vs 右转剖面最大差异 " << worst_mps << " m/s\n";
  EXPECT_LT(worst_mps, 1e-12);
}

TEST(CurvatureLimit, AGentleEnoughCurveIsCappedByCruiseNotByTheLateralLimit)
{
  // R=100 → √150 = 12.2 m/s，远高于巡航 5.556 → 应当取巡航值。
  // 没有这条 min 的话，缓弯上剖面会给出一个超过 ODD 上限的速度。
  const ReferenceLine path(MakeLeftArc(100.0, 0.0, 0.0, 0.0, 40.0 / 100.0, 0.5, false));
  const SpeedProfile profile(path, DefaultParams());
  EXPECT_NEAR(profile.speeds_mps()[20], kCruiseSpeedMps, 1e-9);
}

TEST(CurvatureLimit, AStraightPathIsNotSlowedByFloatingPointCurvatureNoise)
{
  // 直路上曲率是朝向中心差分的**浮点噪声**（1e-17 量级），不是严格的 0。
  // 不设"当直线"阈值的话 √(a_lat/|κ|) 会算出 1e8 m/s ——
  // 虽然随后被 min(v_cruise, ·) 吃掉，但中间那个巨大的数是个隐患。
  const ReferenceLine path(MakeStraightAlongX(60.0, 0.0, 0.5));
  const SpeedProfile profile(path, DefaultParams());
  for (std::size_t i = 0; i < 50; ++i) {  // 末尾留给制动区
    EXPECT_NEAR(profile.speeds_mps()[i], kCruiseSpeedMps, 1e-9) << "第 " << i << " 点";
  }
}

TEST(CurvatureLimit, LateralAccelerationNeverExceedsTheLimitAnywhere)
{
  // **物理不变量**，不依赖具体路径形状：处处 v²·|κ| ≤ a_lat_max。
  // 上面那几条查的是"某一点等于闭式解"，这一条查的是"没有任何一点超"。
  const ReferenceLine path(MakeMixedPath(0.5));
  const SpeedProfile profile(path, DefaultParams());

  double worst_mps2 = 0.0;
  for (std::size_t i = 0; i < path.points().size(); ++i) {
    const double speed_mps = profile.speeds_mps()[i];
    worst_mps2 =
      std::max(worst_mps2, speed_mps * speed_mps * std::abs(path.points()[i].curvature_inv_m));
  }
  std::cout << "  [剖面] 全程最大横向加速度 " << worst_mps2 << " m/s²（上限 "
            << kMaxLateralAccelMps2 << "）\n";
  EXPECT_LE(worst_mps2, kMaxLateralAccelMps2 + 1e-9);
}

// =============================================================================
//  ② 终点归零 + 后向扫描（任务 3.2）
// =============================================================================

TEST(Scans, TheProfileEndsAtExactlyZero)
{
  // 不是"接近 0"，是**恰好 0**。留一点余速的话车会以那个速度滑过终点，
  // 而下游没有任何一层会因此报错。
  const ReferenceLine path(MakeMixedPath(0.5));
  const SpeedProfile profile(path, DefaultParams());
  EXPECT_EQ(profile.speeds_mps().back(), 0.0);
}

TEST(Scans, BrakingToTheGoalFollowsTheClosedForm)
{
  // 终点前的减速段应当严格是 v(s) = √(2·a_dec·(L − s))。
  // 5.556 → 0 按 −3.0 需要 5.145 m —— 也就是最后约 10 个路径点。
  const ReferenceLine path(MakeStraightAlongX(60.0, 0.0, 0.5));
  const SpeedProfile profile(path, DefaultParams());

  const double braking_distance_m = DistanceForSpeedChangeM(kCruiseSpeedMps, 0.0, kMaxDecelMps2);
  EXPECT_NEAR(braking_distance_m, 5.144856, 1e-6);

  double worst_deviation_mps = 0.0;
  for (std::size_t i = 0; i < path.points().size(); ++i) {
    const double remaining_m = path.length_m() - path.points()[i].s_m;
    if (remaining_m > braking_distance_m) {
      continue;  // 还没进制动区
    }
    const double expected_mps = std::sqrt(2.0 * kMaxDecelMps2 * remaining_m);
    worst_deviation_mps =
      std::max(worst_deviation_mps, std::abs(profile.speeds_mps()[i] - expected_mps));
  }
  std::cout << "  [剖面] 终点制动段与闭式解最大偏差 " << worst_deviation_mps * 1e3 << " mm/s\n";
  EXPECT_LT(worst_deviation_mps, 1e-9);
}

TEST(Scans, BrakingForACurveStartsBeforeTheCurveNotInsideIt)
{
  // 这是后向扫描存在的全部理由：曲率限速只说"弯里该多快"，
  // **没说"什么时候开始减速"**。少了它，车会开到弯道入口才发现要减速，
  // 而那时已经晚了 3.14 m —— 症状是过弯时横向加速度超标，
  // 但剖面本身看着完全正常（每一点都满足曲率限速）。
  const ReferenceLine path(MakeStraightThenLeftArc(30.0, kTurnRadiusM, M_PI_2, 0.5));
  const SpeedProfile profile(path, DefaultParams());

  // ⚠️ 参考点是**第一个曲率已经满值的点**，不是几何上的直弯衔接处（s = 30 m）。
  //    衔接点的曲率是中心差分算出来的，一侧是直路一侧是圆弧，只有满值的**一半**
  //    （实测 0.0625 而不是 0.125），所以它的限速是 4.899 而不是 3.464。
  //    拿 s=30 当参考点会把答案系统性地少算半个采样步 —— 初稿就是这么错的。
  std::size_t entry = 0;
  while (std::abs(path.points()[entry].curvature_inv_m) < 0.99 / kTurnRadiusM) {
    ++entry;
  }
  const double entry_s_m = path.points()[entry].s_m;
  const double entry_speed_mps = profile.speeds_mps()[entry];

  // 入弯前每一点都应当**精确**落在匀减速曲线上（或被巡航值盖住）：
  //     v(s) = min(v_cruise, √(v_entry² + 2·a_dec·(s_entry − s)))
  // 这比"减速起点在哪"强得多 —— 它把整条减速段逐点钉住。
  double worst_deviation_mps = 0.0;
  for (std::size_t i = 0; i <= entry; ++i) {
    const double back_m = entry_s_m - path.points()[i].s_m;
    const double expected_mps = std::min(
      kCruiseSpeedMps, std::sqrt(entry_speed_mps * entry_speed_mps + 2.0 * kMaxDecelMps2 * back_m));
    worst_deviation_mps =
      std::max(worst_deviation_mps, std::abs(profile.speeds_mps()[i] - expected_mps));
  }
  EXPECT_LT(worst_deviation_mps, 1e-9);

  // 再看"提前量"本身，和闭式解对一下量级。
  const double expected_lead_m =
    DistanceForSpeedChangeM(kCruiseSpeedMps, entry_speed_mps, kMaxDecelMps2);
  EXPECT_NEAR(
    DistanceForSpeedChangeM(kCruiseSpeedMps, CurvatureLimitMps(kTurnRadiusM), kMaxDecelMps2),
    3.144856, 1e-6);

  std::size_t first_slow = 0;
  while (profile.speeds_mps()[first_slow] > kCruiseSpeedMps - 1e-9) {
    ++first_slow;
  }
  const double lead_m = entry_s_m - path.points()[first_slow].s_m;
  std::cout << "  [剖面] 入弯前 " << lead_m << " m 开始减速（闭式解 " << expected_lead_m
            << " m，逐点最大偏差 " << worst_deviation_mps * 1e3 << " mm/s）\n";
  // 减速起点只能落在采样点上，所以判据是**一个采样步长**。
  EXPECT_LE(lead_m, expected_lead_m);
  EXPECT_GT(lead_m, expected_lead_m - 0.5);
}

TEST(Scans, TheDecelerationConstraintHoldsEverywhere)
{
  // **物理不变量**：处处 v[i]² ≤ v[i+1]² + 2·a_dec·Δs，即"从这一点减得到下一点"。
  const ReferenceLine path(MakeMixedPath(0.5));
  const SpeedProfile profile(path, DefaultParams());
  const std::vector<PathPoint> & points = path.points();

  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double delta_s_m = points[i + 1].s_m - points[i].s_m;
    const double v_here = profile.speeds_mps()[i];
    const double v_next = profile.speeds_mps()[i + 1];
    EXPECT_LE(v_here * v_here, v_next * v_next + 2.0 * kMaxDecelMps2 * delta_s_m + 1e-9)
      << "第 " << i << " 点减不下来";
  }
}

// =============================================================================
//  ③ 前向扫描（任务 3.3）
// =============================================================================

TEST(Scans, AcceleratingOutOfACurveTakesTheClosedFormDistance)
{
  // 出弯回到巡航速度要 6.29 m（3.4641 → 5.556 按 +1.5）。
  // 少了前向扫描，剖面会在出弯那一点**瞬间**跳回巡航速度 ——
  // 速度环于是要求一个远超 +1.5 的加速度，被限幅后车实际跟不上，
  // 表现为"出弯后速度一直低于剖面"，而人会去查 PID。
  std::vector<Pose2D> poses = MakeLeftArc(kTurnRadiusM, 0.0, 0.0, 0.0, M_PI_2, 0.5, false);
  AppendStraight(&poses, 40.0, 0.5);
  const ReferenceLine path(poses);
  const SpeedProfile profile(path, DefaultParams());

  EXPECT_NEAR(
    DistanceForSpeedChangeM(CurvatureLimitMps(kTurnRadiusM), kCruiseSpeedMps, kMaxAccelMps2),
    6.289712, 1e-6);

  // 参考点同上：**最后一个曲率仍是满值的点**，不是几何上的弯直衔接处。
  std::size_t exit = path.points().size() - 1;
  while (std::abs(path.points()[exit].curvature_inv_m) < 0.99 / kTurnRadiusM) {
    --exit;
  }
  const double exit_s_m = path.points()[exit].s_m;
  const double exit_speed_mps = profile.speeds_mps()[exit];
  const double expected_run_m =
    DistanceForSpeedChangeM(exit_speed_mps, kCruiseSpeedMps, kMaxAccelMps2);

  // 出弯后每一点都应当精确落在匀加速曲线上，直到被巡航值或终点减速接管。
  // 终点制动区（末尾 5.145 m）排除掉，那一段归后向扫描管。
  double worst_deviation_mps = 0.0;
  const double braking_start_s_m =
    path.length_m() - DistanceForSpeedChangeM(kCruiseSpeedMps, 0.0, kMaxDecelMps2);
  for (std::size_t i = exit; i < path.points().size(); ++i) {
    if (path.points()[i].s_m > braking_start_s_m) {
      break;
    }
    const double run_m = path.points()[i].s_m - exit_s_m;
    const double expected_mps = std::min(
      kCruiseSpeedMps, std::sqrt(exit_speed_mps * exit_speed_mps + 2.0 * kMaxAccelMps2 * run_m));
    worst_deviation_mps =
      std::max(worst_deviation_mps, std::abs(profile.speeds_mps()[i] - expected_mps));
  }
  EXPECT_LT(worst_deviation_mps, 1e-9);

  std::size_t index = exit;
  while (profile.speeds_mps()[index] < kCruiseSpeedMps - 1e-9) {
    ++index;
  }
  const double run_m = path.points()[index].s_m - exit_s_m;
  std::cout << "  [剖面] 出弯后 " << run_m << " m 回到巡航（闭式解 " << expected_run_m
            << " m，逐点最大偏差 " << worst_deviation_mps * 1e3 << " mm/s）\n";
  EXPECT_GE(run_m, expected_run_m);
  EXPECT_LT(run_m, expected_run_m + 0.5);
}

TEST(Scans, TheAccelerationConstraintHoldsEverywhere)
{
  // **物理不变量**：处处 v[i+1]² ≤ v[i]² + 2·a_acc·Δs，即"从上一点加得上来"。
  const ReferenceLine path(MakeMixedPath(0.5));
  const SpeedProfile profile(path, DefaultParams());
  const std::vector<PathPoint> & points = path.points();

  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double delta_s_m = points[i + 1].s_m - points[i].s_m;
    const double v_here = profile.speeds_mps()[i];
    const double v_next = profile.speeds_mps()[i + 1];
    EXPECT_LE(v_next * v_next, v_here * v_here + 2.0 * kMaxAccelMps2 * delta_s_m + 1e-9)
      << "第 " << i << " 点加不上去";
  }
}

TEST(Scans, ScanOrderDoesNotMatter)
{
  // ⚠️ `control.md` §4.2 和 `plan.md` 原本都写着**「顺序不能反」**。
  //    2026-08-02 用 20000 组随机剖面实测：**两种顺序逐点完全相同**（最大差 0.0）。
  //    机理：两个扫描都**只下调**，且各自恰好调到对方约束的安全侧 ——
  //      前向把 v[i] 降到 v[i−1]²+2a_a·Δs 时，后向要的 v[i−1]² ≤ v[i]²+2a_d·Δs
  //      变成 v[i−1]² ≤ v[i−1]²+2(a_a+a_d)·Δs，恒成立；反方向同理。
  //
  //    这条用例把结论钉住，**并且是一条给未来的告警**：这个论证依赖"只下调"。
  //    将来若加入「这一点至少要跑多快」这类**下界**约束（比如为了不挡后车），
  //    论证立刻失效，那时就真的要讲顺序甚至迭代到收敛 —— 而这条用例会先红。
  const ReferenceLine path(MakeMixedPath(0.5));
  const SpeedProfile profile(path, DefaultParams());
  const std::vector<PathPoint> & points = path.points();
  const std::size_t count = points.size();

  // 在测试里独立实现**对调**的顺序：曲率限速 + 终点归零 → 前向 → 后向。
  std::vector<double> other(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double curvature_inv_m = std::abs(points[i].curvature_inv_m);
    other[i] = (curvature_inv_m < 1e-6)
                 ? kCruiseSpeedMps
                 : std::min(kCruiseSpeedMps, std::sqrt(kMaxLateralAccelMps2 / curvature_inv_m));
  }
  other.back() = 0.0;
  for (std::size_t i = 1; i < count; ++i) {  // 先前向
    const double delta_s_m = points[i].s_m - points[i - 1].s_m;
    other[i] =
      std::min(other[i], std::sqrt(other[i - 1] * other[i - 1] + 2.0 * kMaxAccelMps2 * delta_s_m));
  }
  for (std::size_t i = count - 1; i-- > 0;) {  // 再后向
    const double delta_s_m = points[i + 1].s_m - points[i].s_m;
    other[i] =
      std::min(other[i], std::sqrt(other[i + 1] * other[i + 1] + 2.0 * kMaxDecelMps2 * delta_s_m));
  }

  double worst_mps = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    worst_mps = std::max(worst_mps, std::abs(other[i] - profile.speeds_mps()[i]));
  }
  std::cout << "  [剖面] 两种扫描顺序的最大逐点差异 " << worst_mps << " m/s\n";
  EXPECT_LT(worst_mps, 1e-12);
}

// =============================================================================
//  ④ 剖面自身的加速度（给速度环做前馈）
// =============================================================================

TEST(Feedforward, MatchesTheDecelerationLimitOnTheTerminalRamp)
{
  // 终点制动段上剖面满足 v² = 2·a_dec·剩余距离，对 s 求导得 v·dv/ds = −a_dec。
  // 所以**前馈在整条制动段上恒等于 −a_dec**，这是闭式解，不是近似。
  //
  // 它存在的理由：纯 P 跟踪斜坡的稳态误差 = 斜率/K_p = 3.0/1.0 = 3.0 m/s。
  // S4 首测就是这么冲过终点 4.26 m 的。
  const ReferenceLine path(MakeStraightAlongX(60.0, 0.0, 0.5));
  const SpeedProfile profile(path, DefaultParams());
  const double braking_distance_m = DistanceForSpeedChangeM(kCruiseSpeedMps, 0.0, kMaxDecelMps2);

  double worst_deviation_mps2 = 0.0;
  int checked = 0;
  for (std::size_t i = 0; i + 1 < path.points().size(); ++i) {
    const double remaining_m = path.length_m() - path.points()[i].s_m;
    // 只查制动段内部：更靠前的段还被巡航值盖着，那里前馈本来就该是 0。
    if (remaining_m > braking_distance_m - 0.5) {
      continue;
    }
    const double accel_mps2 = profile.target_accel_at(i, 0.0);
    worst_deviation_mps2 = std::max(worst_deviation_mps2, std::abs(accel_mps2 + kMaxDecelMps2));
    ++checked;
  }
  ASSERT_GT(checked, 5) << "制动段里没取到足够的点，这条用例什么都没测";
  std::cout << "  [前馈] 终点制动段 " << checked << " 个点，与 −a_dec 的最大偏差 "
            << worst_deviation_mps2 << " m/s²\n";
  // ⚠️ 判据卡到 1e-9 是**有意的**：剖面插的是 v²，而 v² = v₀² + 2a·Δs 在
  //    制动段上严格线性，所以 ½·d(v²)/ds **恰好**等于 −a_dec，是闭式解不是近似。
  //    初版剖面插的是 v，这里的偏差是 0.515 m/s²（末端最大）——
  //    正好落在最需要前馈准的地方，所以才改成插 v²。
  EXPECT_LT(worst_deviation_mps2, 1e-9);
}

TEST(Feedforward, IsZeroOnCruiseAndPositiveWhenAcceleratingOutOfACurve)
{
  // 巡航段速度恒定 → d(v²)/ds = 0 → 前馈为 0。给一个常值目标却发非零前馈，
  // 会让车持续加/减速而稳态误差永远消不掉。
  const ReferenceLine straight(MakeStraightAlongX(60.0, 0.0, 0.5));
  const SpeedProfile straight_profile(straight, DefaultParams());
  EXPECT_NEAR(straight_profile.target_accel_at(20, 0.0), 0.0, 1e-12);

  // 出弯加速段：前馈应当**恰好**等于 +a_acc（前向扫描卡住的那些段上是闭式解）。
  std::vector<Pose2D> poses = MakeLeftArc(kTurnRadiusM, 0.0, 0.0, 0.0, M_PI_2, 0.5, false);
  AppendStraight(&poses, 40.0, 0.5);
  const ReferenceLine path(poses);
  const SpeedProfile profile(path, DefaultParams());

  const double curve_exit_s_m = kTurnRadiusM * M_PI_2;
  std::size_t index = 0;
  while (path.points()[index].s_m < curve_exit_s_m + 1.0) {
    ++index;
  }
  std::cout << "  [前馈] 出弯加速段 a_ff = " << profile.target_accel_at(index, 0.0)
            << " m/s²（应为 +" << kMaxAccelMps2 << "）\n";
  EXPECT_NEAR(profile.target_accel_at(index, 0.0), kMaxAccelMps2, 1e-9);
}

TEST(Feedforward, RejectsAnIndexBeyondTheProfile)
{
  const ReferenceLine path(MakeStraightAlongX(20.0, 0.0, 0.5));
  const SpeedProfile profile(path, DefaultParams());
  const std::size_t last_segment = profile.speeds_mps().size() - 2;
  EXPECT_NO_THROW(profile.target_accel_at(last_segment, 1.0));
  EXPECT_THROW(profile.target_accel_at(last_segment + 1, 0.0), std::out_of_range);
}

// =============================================================================
//  查表
// =============================================================================

TEST(Lookup, InterpolatesInSpeedSquaredNotInSpeed)
{
  const ReferenceLine path(MakeStraightAlongX(60.0, 0.0, 0.5));
  const SpeedProfile profile(path, DefaultParams());

  // 挑一段速度确实在变的（制动区里），否则插值对不对看不出来。
  std::size_t index = path.points().size() - 5;
  const double lo = profile.speeds_mps()[index];
  const double hi = profile.speeds_mps()[index + 1];
  ASSERT_NE(lo, hi) << "选的这一段速度没变化，测不出插值";

  EXPECT_NEAR(profile.speed_at(index, 0.0), lo, 1e-12);
  EXPECT_NEAR(profile.speed_at(index, 1.0), hi, 1e-12);
  // 段内插的是 **v²**：剖面是用 v² = v₀² + 2a·Δs 构造的，插 v² 才精确复现
  // 那条 √ 曲线。插 v 的话前馈（= 曲线斜率）在末端偏 0.5 m/s²，见 Feedforward 那组。
  EXPECT_NEAR(
    profile.speed_at(index, 0.25), std::sqrt(lo * lo + 0.25 * (hi * hi - lo * lo)), 1e-12);
  EXPECT_GT(profile.speed_at(index, 0.5), std::min(lo, hi)) << "插值结果应落在两端之间";
}

TEST(Lookup, RejectsAnIndexBeyondTheProfile)
{
  // 索引对不上 = 剖面还是上一条路径的。抛异常而不是夹取，
  // 否则这个错误表现为"车速莫名其妙"而不是一条指名道姓的报错。
  const ReferenceLine path(MakeStraightAlongX(20.0, 0.0, 0.5));
  const SpeedProfile profile(path, DefaultParams());
  const std::size_t last_segment = profile.speeds_mps().size() - 2;

  EXPECT_NO_THROW(profile.speed_at(last_segment, 1.0));
  EXPECT_THROW(profile.speed_at(last_segment + 1, 0.0), std::out_of_range);
  EXPECT_THROW(profile.speed_at(999999, 0.0), std::out_of_range);
}

TEST(Lookup, TheProjectionOverloadAgreesWithTheIndexOverload)
{
  const ReferenceLine path(MakeMixedPath(0.5));
  const SpeedProfile profile(path, DefaultParams());

  // 拿一个真实的投影结果去查 —— 这正是 S4 的控制回调会做的事。
  const ads_common::PathProjection projection = path.project({12.3, 0.4, 0.0});
  EXPECT_NEAR(
    profile.speed_at(projection), profile.speed_at(projection.index, projection.ratio), 1e-15);
}
