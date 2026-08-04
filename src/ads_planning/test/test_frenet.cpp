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
//  L1：Frenet ↔ 笛卡尔双向变换（CP-P3-A）
//
//  判据分三层，越靠前越"硬"：
//    ① **闭式解** —— 圆弧上偏移曲线的曲率、半径都有精确表达式，直接比对；
//    ② **往返一致性** —— SPEC §8 L1 明确点名的那一条；
//    ③ **符号** —— d′ 的正负、法向的左右。这两个错了不会报错，只会让车往反方向绕。
//
//  ⚠️ 往返一致性在**折线**上达不到浮点级，这是表示法的性质不是 bug：
//     project() 的垂足是相对**弦**求的，而 lateral_error_m 沿的是**插值朝向**的法向，
//     两者差 ε = (ratio − 0.5)·Δφ，于是往返位置误差 = |d|·Δφ/2 = |d|·步长/(2R)。
//     所以直线用例判 1e-12（纯代数），圆弧用例判那个**闭式上界**并验它是紧的。
//     把圆弧也判成 1e-9 的话，唯一的结果是被迫去放宽判据。
// =============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "ads_common/angles.hpp"
#include "ads_common/reference_line.hpp"
#include "ads_common/testing/path_fixtures.hpp"
#include "ads_planning/frenet.hpp"

namespace
{

using ads_common::Pose2D;
using ads_common::ReferenceLine;
using ads_common_test::MakeLeftArc;
using ads_common_test::MakeRightArc;
using ads_common_test::MakeStraightAlongX;
using ads_planning::CartesianState;
using ads_planning::FrenetState;
using ads_planning::to_cartesian;
using ads_planning::to_frenet;

/// 参考线采样步长，与 map_node 的 `path_sample_step_m` 一致（0.5 m）。
/// 用真实值而不是随便取一个，是因为下面几条判据的**上界里含它**。
constexpr double kSampleStepM = 0.5;

// ---------------------------------------------------------------------------
//  ① 直线：纯代数，没有离散化误差，判到浮点级
// ---------------------------------------------------------------------------

TEST(FrenetStraightLine, ZeroOffsetRoundTripsToFloatingPointPrecision)
{
  const ReferenceLine line(MakeStraightAlongX(50.0, 0.0, kSampleStepM));

  for (double x_m = 1.0; x_m < 45.0; x_m += 3.7) {
    const Pose2D query{x_m, 0.0, 0.0};
    const FrenetState frenet = to_frenet(line, query);
    const CartesianState back = to_cartesian(line, frenet);

    EXPECT_NEAR(back.x_m, query.x_m, 1e-12) << "x = " << x_m;
    EXPECT_NEAR(back.y_m, query.y_m, 1e-12) << "x = " << x_m;
    EXPECT_NEAR(back.heading_rad, query.heading_rad, 1e-12) << "x = " << x_m;
    // 直线的曲率是 0，偏移曲线的也必须是 0（κ/σ = 0/1）。
    EXPECT_NEAR(back.curvature_inv_m, 0.0, 1e-12) << "x = " << x_m;
  }
}

TEST(FrenetStraightLine, LeftOffsetGoesLeftAndRightOffsetGoesRight)
{
  const ReferenceLine line(MakeStraightAlongX(50.0, 0.0, kSampleStepM));

  // 参考线朝 +x，左法向是 +y。d 左正 ⟹ d = +1.5 应当落在 y = +1.5。
  //
  // ⚠️ 这条用例守的是**法向取左还是取右**。取反的话所有绕行方向都会镜像，
  //    而轨迹本身依然平滑、长度也合理 —— RViz 里看不出任何异常，
  //    只有把车真开出去才会发现它往障碍物那一侧绕。
  const CartesianState left = to_cartesian(line, FrenetState{20.0, 1.5, 0.0, 0.0});
  EXPECT_NEAR(left.x_m, 20.0, 1e-12);
  EXPECT_NEAR(left.y_m, 1.5, 1e-12);

  const CartesianState right = to_cartesian(line, FrenetState{20.0, -1.5, 0.0, 0.0});
  EXPECT_NEAR(right.x_m, 20.0, 1e-12);
  EXPECT_NEAR(right.y_m, -1.5, 1e-12);
}

TEST(FrenetStraightLine, HeadingOffsetMatchesArctanOfLateralDerivative)
{
  const ReferenceLine line(MakeStraightAlongX(50.0, 0.0, kSampleStepM));

  // 直线上 κ = 0 ⟹ σ = 1 ⟹ Δψ = arctan(d′)。
  for (const double d_prime : {-0.5, -0.1, 0.0, 0.1, 0.5}) {
    const CartesianState out = to_cartesian(line, FrenetState{20.0, 0.8, d_prime, 0.0});
    EXPECT_NEAR(out.heading_rad, std::atan(d_prime), 1e-12) << "d′ = " << d_prime;
  }
}

// ---------------------------------------------------------------------------
//  ② 圆弧：与闭式解比对
// ---------------------------------------------------------------------------

/// 圆心在原点、半径 R 的**左转**圆弧。起点 (R, 0) 朝 +y，曲率 +1/R。
ReferenceLine MakeCenteredLeftArc(double radius_m, double sweep_rad)
{
  return ReferenceLine(MakeLeftArc(radius_m, 0.0, 0.0, 0.0, sweep_rad, kSampleStepM, false));
}

TEST(FrenetArc, ConstantOffsetCurvatureMatchesClosedForm)
{
  constexpr double kRadiusM = 20.0;
  const ReferenceLine line = MakeCenteredLeftArc(kRadiusM, M_PI_2);

  for (const double d_m : {-2.0, -0.85, 0.0, 0.85, 2.0}) {
    const double s_m = 10.0;
    const FrenetState state{s_m, d_m, 0.0, 0.0};
    const CartesianState out = to_cartesian(line, state);

    // 拿**参考线自己报的 κ** 去算期望值，而不是 1/R。
    // 这样这条用例只验「偏移公式对不对」，不掺进折线离散化对 κ 的影响 ——
    // 后者已经由 test_reference_line 单独守着。**一条用例只验一件事。**
    const double kappa = line.at(s_m).curvature_inv_m;
    const double expected = kappa / (1.0 - d_m * kappa);

    EXPECT_NEAR(out.curvature_inv_m, expected, 1e-12) << "d = " << d_m;
  }
}

TEST(FrenetArc, LeftOffsetOnLeftTurnShrinksRadiusByExactlyTheOffset)
{
  constexpr double kRadiusM = 20.0;
  constexpr double kOffsetM = 3.0;
  const ReferenceLine line = MakeCenteredLeftArc(kRadiusM, M_PI_2);

  // 左转圆的**左**侧是内侧（法向指向圆心），所以左偏 d 之后半径应是 R − d。
  // 这条与上一条是同一个 σ = 1 − dκ 的两个侧面：一个看曲率，一个看半径。
  // 分开写是因为**只看曲率的话，把 N 取反的错误抓不住**（κ/(1−dκ) 与 d 的符号有关，
  // 但如果同时把 d 和 N 都取反，曲率照样对，位置却镜像了）。
  const CartesianState out = to_cartesian(line, FrenetState{10.0, kOffsetM, 0.0, 0.0});
  const double radius_m = std::hypot(out.x_m, out.y_m);

  // 折线上的点本身就比真圆内缩一个弦高 step²/(8R)，所以这里的余量按它给。
  const double sagitta_m = kSampleStepM * kSampleStepM / (8.0 * kRadiusM);
  EXPECT_NEAR(radius_m, kRadiusM - kOffsetM, 3.0 * sagitta_m);
  // 方向性判据：必须是**变小**。写成 NEAR 的话，把 N 取反（半径 R + d）
  // 在容差足够大时也能过，所以再钉一条不等式。
  EXPECT_LT(radius_m, kRadiusM);
}

TEST(FrenetArc, RightTurnOffsetCurvatureMatchesClosedForm)
{
  // ⚠️ 单独测右转：本仓库有过「所有测试路径都是左转」的真实覆盖漏洞
  //    （见 path_fixtures.hpp 里 MakeRightArc 的说明）。
  //    σ = 1 − d·κ 里 κ 变号，漏了符号处理的实现在全左转用例里一条都不红。
  constexpr double kRadiusM = 20.0;
  const ReferenceLine line(MakeRightArc(kRadiusM, 0.0, 0.0, M_PI_2, M_PI_2, kSampleStepM, false));

  for (const double d_m : {-2.0, 0.0, 2.0}) {
    const double s_m = 10.0;
    const double kappa = line.at(s_m).curvature_inv_m;
    ASSERT_LT(kappa, 0.0) << "右转的曲率必须为负，夹具或参考线实现有问题";

    const CartesianState out = to_cartesian(line, FrenetState{s_m, d_m, 0.0, 0.0});
    EXPECT_NEAR(out.curvature_inv_m, kappa / (1.0 - d_m * kappa), 1e-12) << "d = " << d_m;
  }
}

TEST(FrenetArc, ArcLengthFactorAgreesWithMapLaneOffsetConvention)
{
  // ads_map 用同一个 σ = 1 − t·κ 算车道中心线长度：
  // R = 12 的弯上、横向偏移 t = −1.75（右车道）⟹ 有效半径 13.75 m。
  // 两个模块必须给出同一个数 —— 不然规划算出来的弧长和地图算出来的对不上，
  // 而两边各自都自洽。见 docs/modules/map_and_routing.md。
  constexpr double kRadiusM = 12.0;
  constexpr double kLaneOffsetM = -1.75;
  const ReferenceLine line = MakeCenteredLeftArc(kRadiusM, M_PI_2);

  const CartesianState out = to_cartesian(line, FrenetState{6.0, kLaneOffsetM, 0.0, 0.0});
  const double radius_m = std::hypot(out.x_m, out.y_m);

  const double sagitta_m = kSampleStepM * kSampleStepM / (8.0 * kRadiusM);
  EXPECT_NEAR(radius_m, 13.75, 3.0 * sagitta_m);
}

// ---------------------------------------------------------------------------
//  ③ 往返一致性（SPEC §8 L1 点名）
// ---------------------------------------------------------------------------

TEST(FrenetRoundTrip, ArcErrorStaysWithinTheDiscretizationBoundAndThatBoundIsTight)
{
  constexpr double kRadiusM = 8.0;  // 本项目地图的最小转弯半径 —— 最不利情形
  const ReferenceLine line = MakeCenteredLeftArc(kRadiusM, M_PI_2);

  // 闭式上界（推导见本文件头）：|d|·Δφ/2，其中 Δφ = 步长/R。
  const double delta_phi_rad = kSampleStepM / kRadiusM;
  constexpr double kOffsetM = 0.85;  // 车道内不压线的最大横向偏移（planning.md §6）
  const double bound_m = kOffsetM * delta_phi_rad / 2.0;

  double max_error_m = 0.0;
  // 密采样，确保取到 ratio ≈ 0 和 ≈ 1（误差最大处）以及 ratio ≈ 0.5（误差为 0 处）。
  for (double phi_rad = 0.05; phi_rad < M_PI_2 - 0.05; phi_rad += 0.00317) {
    // 在真圆上取一个左偏 kOffsetM 的点（左转圆的左侧是内侧，半径 R − d）。
    const double radius_m = kRadiusM - kOffsetM;
    const Pose2D query{
      radius_m * std::cos(phi_rad), radius_m * std::sin(phi_rad),
      ads_common::normalize_angle(phi_rad + M_PI_2)};

    const CartesianState back = to_cartesian(line, to_frenet(line, query));
    max_error_m = std::max(max_error_m, std::hypot(back.x_m - query.x_m, back.y_m - query.y_m));
  }

  // 打印实测值，不只断言通过 —— S2 的教训：只看绿灯的话，判据变脆了没人知道。
  std::cout << "  往返最大位置误差 = " << max_error_m * 1000.0
            << " mm，闭式上界 = " << bound_m * 1000.0 << " mm" << std::endl;

  EXPECT_LE(max_error_m, bound_m * 1.05) << "超过闭式上界，说明变换本身有问题";
  // **上界必须是紧的**：如果实测远小于上界，说明我把上界算错了（或者取样没覆盖到
  // 最不利的 ratio），那样这条用例其实什么都没验。
  EXPECT_GE(max_error_m, bound_m * 0.5) << "远小于上界 ⟹ 上界算错了或采样没覆盖最不利点";
}

TEST(FrenetRoundTrip, FrenetToCartesianToFrenetPreservesArcLengthAndOffsetOnStraightLine)
{
  const ReferenceLine line(MakeStraightAlongX(50.0, 0.0, kSampleStepM));

  for (const double d_m : {-1.5, -0.4, 0.0, 0.4, 1.5}) {
    const FrenetState original{22.5, d_m, 0.0, 0.0};
    const CartesianState cartesian = to_cartesian(line, original);
    const Pose2D pose{cartesian.x_m, cartesian.y_m, cartesian.heading_rad};
    const FrenetState back = to_frenet(line, pose);

    EXPECT_NEAR(back.s_m, original.s_m, 1e-12) << "d = " << d_m;
    EXPECT_NEAR(back.d_m, original.d_m, 1e-12) << "d = " << d_m;
    EXPECT_NEAR(back.d_prime, original.d_prime, 1e-12) << "d = " << d_m;
  }
}

// ---------------------------------------------------------------------------
//  ④ d′ 的符号 —— 错了不会报错，只会让车往反方向绕
// ---------------------------------------------------------------------------

TEST(FrenetLateralDerivative, HeadingLeftOfReferenceGivesPositiveDerivative)
{
  const ReferenceLine line(MakeStraightAlongX(50.0, 0.0, kSampleStepM));

  // 参考线朝 +x（θ = 0）。车头朝 +0.2 rad = 偏向参考线**左侧**，
  // 车正在往左跑 ⟹ d 在增大 ⟹ d′ > 0。
  const FrenetState state = to_frenet(line, Pose2D{20.0, 0.0, 0.2});
  EXPECT_GT(state.d_prime, 0.0);
  // 直线上 σ = 1，所以 d′ 应精确等于 tan(0.2)。
  EXPECT_NEAR(state.d_prime, std::tan(0.2), 1e-12);
}

TEST(FrenetLateralDerivative, HeadingRightOfReferenceGivesNegativeDerivative)
{
  const ReferenceLine line(MakeStraightAlongX(50.0, 0.0, kSampleStepM));

  const FrenetState state = to_frenet(line, Pose2D{20.0, 0.0, -0.2});
  EXPECT_LT(state.d_prime, 0.0);
  EXPECT_NEAR(state.d_prime, -std::tan(0.2), 1e-12);
}

TEST(FrenetLateralDerivative, MatchesMinusSigmaTimesTangentOfHeadingErrorOnArc)
{
  // 弯道上 σ ≠ 1，这条才能把「漏乘 σ」和「符号写反」区分开：
  // 直线上 σ = 1，漏乘它的实现在直线用例里全绿。
  constexpr double kRadiusM = 8.0;
  const ReferenceLine line = MakeCenteredLeftArc(kRadiusM, M_PI_2);

  const double phi_rad = 0.5;
  const double d_m = 0.85;
  const double radius_m = kRadiusM - d_m;
  const double yaw_offset_rad = 0.15;  // 车头相对切向再偏一点
  const Pose2D query{
    radius_m * std::cos(phi_rad), radius_m * std::sin(phi_rad),
    ads_common::normalize_angle(phi_rad + M_PI_2 + yaw_offset_rad)};

  const auto projection = line.project(query);
  const double sigma = 1.0 - projection.lateral_error_m * projection.curvature_inv_m;
  const double expected = -sigma * std::tan(projection.heading_error_rad);

  const FrenetState state = to_frenet(line, query);
  EXPECT_NEAR(state.d_prime, expected, 1e-12);
  // 车头额外左偏 ⟹ 正在离开参考线往左 ⟹ d′ > 0。
  EXPECT_GT(state.d_prime, 0.0);
}

TEST(FrenetLateralDerivative, SecondDerivativeIsAlwaysZeroByDesign)
{
  // 不是"还没实现"，是**有意不估**（头文件里有理由）。
  // 钉一条用例是为了：将来有人改成非零估计时，这条会红，从而被迫回去读那个理由。
  const ReferenceLine line(MakeStraightAlongX(50.0, 0.0, kSampleStepM));
  EXPECT_EQ(to_frenet(line, Pose2D{20.0, 0.7, 0.3}).d_double_prime, 0.0);
}

// ---------------------------------------------------------------------------
//  ⑤ 异常：不许静默给出一个"看起来能用的数"
// ---------------------------------------------------------------------------

TEST(FrenetGuards, ThrowsAtCuspSingularityWhereOffsetReachesTheCurvatureCenter)
{
  constexpr double kRadiusM = 5.0;
  const ReferenceLine line = MakeCenteredLeftArc(kRadiusM, M_PI_2);

  // d = R 正好是曲率中心（σ = 0），d > R 已经翻到另一侧（σ < 0）。
  EXPECT_THROW(to_cartesian(line, FrenetState{2.0, kRadiusM, 0.0, 0.0}), std::domain_error);
  EXPECT_THROW(to_cartesian(line, FrenetState{2.0, kRadiusM + 1.0, 0.0, 0.0}), std::domain_error);
  // 而正常范围内不许抛。
  EXPECT_NO_THROW(to_cartesian(line, FrenetState{2.0, 0.85, 0.0, 0.0}));
}

TEST(FrenetGuards, ThrowsWhenHeadingErrorExceedsTheLimit)
{
  const ReferenceLine line(MakeStraightAlongX(50.0, 0.0, kSampleStepM));

  // kMaxHeadingErrorRad = π/3 ≈ 1.047
  EXPECT_THROW(to_frenet(line, Pose2D{20.0, 0.0, 1.2}), std::domain_error);
  EXPECT_THROW(to_frenet(line, Pose2D{20.0, 0.0, -1.2}), std::domain_error);
  EXPECT_NO_THROW(to_frenet(line, Pose2D{20.0, 0.0, 0.9}));
}

TEST(FrenetGuards, ThrowsWhenArcLengthIsOutOfRangeOrNonFinite)
{
  const ReferenceLine line(MakeStraightAlongX(50.0, 0.0, kSampleStepM));
  const double length_m = line.length_m();

  EXPECT_THROW(to_cartesian(line, FrenetState{length_m + 1.0, 0.0, 0.0, 0.0}), std::out_of_range);
  EXPECT_THROW(to_cartesian(line, FrenetState{-1.0, 0.0, 0.0, 0.0}), std::out_of_range);
  EXPECT_THROW(to_cartesian(line, FrenetState{std::nan(""), 0.0, 0.0, 0.0}), std::out_of_range);
  EXPECT_THROW(
    to_cartesian(line, FrenetState{std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0}),
    std::out_of_range);
  // 端点本身必须能取（浮点噪声容差内），否则规划器把前视距离截到 length_m() 就会炸。
  EXPECT_NO_THROW(to_cartesian(line, FrenetState{length_m, 0.0, 0.0, 0.0}));
  EXPECT_NO_THROW(to_cartesian(line, FrenetState{0.0, 0.0, 0.0, 0.0}));
}

}  // namespace
