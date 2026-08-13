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
//  恒速/静态外推与不确定椭圆的 L1 判据（CP-P6-A ①⑥⑧）
//
//  全部对**闭式解**：模型本身是解析的，任何数值偏差都是 bug 不是精度问题。
//
//  ## 故障注入实测（2026-08-12，跑完回填 —— 第二行的预写就是错的，如实改）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 恒速方向改取 yaw（180° 二义的轴向） | **红 1 条**：`HeadingComesFromVelocityNotYaw` |
//  | 椭圆横向项去掉 ½ 因子 | **红 1 条**：`EllipseGrowthMatchesTheLaw`。
//  |   | 预写猜的是 2 条 —— 静态椭圆走自己的起步律，不经过被注入的
//  |   | 横向项。**预写注入表又一次想当然** |
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>

#include "ads_prediction/motion_model.hpp"

namespace
{

using ads_prediction::ModelKind;
using ads_prediction::MotionModelParams;
using ads_prediction::PredictConstantVelocity;
using ads_prediction::PredictedPath;
using ads_prediction::PredictStatic;
using ads_prediction::TargetSnapshot;

TargetSnapshot MakeTarget(double vx, double vy)
{
  TargetSnapshot target;
  target.id = 7;
  target.position = {10.0, -5.0};
  target.velocity = {vx, vy};
  target.yaw_rad = 0.3;
  target.length_m = 4.4;
  target.width_m = 1.8;
  target.net_displacement_1s_m = std::hypot(vx, vy);
  return target;
}

}  // namespace

// ---------------------------------------------------------------------------
//  CP-P6-A ①：恒速外推 vs 闭式解
// ---------------------------------------------------------------------------
TEST(MotionModel, ConstantVelocityMatchesClosedForm)
{
  MotionModelParams params;
  const TargetSnapshot target = MakeTarget(3.0, -1.0);
  const PredictedPath path = PredictConstantVelocity(target, params, 0.8);

  ASSERT_EQ(path.points.size(), 16U);  // 0..3.0 s 步长 0.2 → 15 段 16 点
  ASSERT_EQ(path.model, ModelKind::kConstantVelocity);
  double worst = 0.0;
  for (const auto & point : path.points) {
    const Eigen::Vector2d expected = target.position + target.velocity * point.t_s;
    worst = std::max(worst, (point.position - expected).norm());
    EXPECT_NEAR(point.speed_mps, target.velocity.norm(), 1e-12);
  }
  printf("[          ] 恒速 vs 闭式解最大偏差 %.3g m\n", worst);
  EXPECT_LT(worst, 1e-9) << "恒速外推不是解析的 —— 桩函数级正确性都没有";
  EXPECT_NEAR(path.points.back().t_s, params.horizon_s, 1e-12) << "末点不在视界上";
}

// ---------------------------------------------------------------------------
//  方向永远取速度矢量，不取 yaw（180° 二义那条规则）
// ---------------------------------------------------------------------------
TEST(MotionModel, HeadingComesFromVelocityNotYaw)
{
  MotionModelParams params;
  // 速度朝 −x（π），yaw 却是 0.3 —— 轴向二义时 yaw 与运动方向可以差 180°。
  TargetSnapshot target = MakeTarget(-4.0, 0.0);
  target.yaw_rad = 0.3;
  target.heading_resolved = false;
  const PredictedPath path = PredictConstantVelocity(target, params, 0.8);
  printf("[          ] 速度朝 π、yaw=0.3 → 预测朝向 %.3f（应为 ±π）\n", path.points[1].heading_rad);
  EXPECT_NEAR(std::fabs(path.points[1].heading_rad), M_PI, 1e-9)
    << "预测方向取了 yaw —— 50% 概率给出一条逆行轨迹（lshape_fit.hpp 的警告成真）";
  // 位置也必须朝 −x 走，而不是朝 yaw 的方向。
  EXPECT_LT(path.points.back().position.x(), target.position.x());
}

// ---------------------------------------------------------------------------
//  CP-P6-A ⑥：静态预测原地不动
// ---------------------------------------------------------------------------
TEST(MotionModel, StaticPredictionStaysPut)
{
  MotionModelParams params;
  // 静态预测不看速度（选择器判静止后才进来）——故意塞个非零速度，
  // 输出必须仍在原地：**"判它静止"与"它的速度字段"是两回事**。
  const TargetSnapshot target = MakeTarget(0.3, 0.1);
  const PredictedPath path = PredictStatic(target, params);
  for (const auto & point : path.points) {
    EXPECT_NEAR((point.position - target.position).norm(), 0.0, 1e-12)
      << "静态预测在 t=" << point.t_s << " 动了";
    EXPECT_NEAR(point.speed_mps, 0.0, 1e-12);
  }
  EXPECT_NEAR(path.points.back().heading_rad, target.yaw_rad, 1e-12)
    << "静态预测的朝向应沿用输入 yaw（只摆盒子）";
}

// ---------------------------------------------------------------------------
//  CP-P6-A ⑧：椭圆增长律逐点对账（闭式）
// ---------------------------------------------------------------------------
TEST(MotionModel, EllipseGrowthMatchesTheLaw)
{
  MotionModelParams params;
  const double a_lat = params.pedestrian_lateral_accel_mps2;
  const TargetSnapshot target = MakeTarget(1.2, 0.0);
  const PredictedPath path = PredictConstantVelocity(target, params, a_lat);
  for (const auto & point : path.points) {
    const double t = point.t_s;
    EXPECT_NEAR(point.sigma_along_m, params.sigma_pos0_m + params.sigma_speed_mps * t, 1e-12)
      << "σ_along 不符合 σ0 + σv·t（t=" << t << "）";
    EXPECT_NEAR(point.sigma_cross_m, params.sigma_pos0_m + 0.5 * a_lat * t * t, 1e-12)
      << "σ_cross 不符合 σ0 + a·t²/2（t=" << t << "）";
  }
  // 行人判据的物理锚点：t=3 时 2σ_cross = 2·(0.2 + 0.5·0.5·9) = 4.9 m ——
  // 行人 90° 变向 3 s 的最坏横移 3.6 m 落在 2σ 内（覆盖率判据的依据），
  // 而没有大到把整条路盖住（CP-P6-B ⑤ 的半轴上限卡的就是它）。
  const double sigma_cross_3s = path.points.back().sigma_cross_m;
  printf(
    "[          ] t=3 s 的 σ_cross = %.3f（2σ = %.2f，最坏横移 3.6）\n", sigma_cross_3s,
    2.0 * sigma_cross_3s);
  EXPECT_GT(2.0 * sigma_cross_3s, 3.6) << "椭圆连行人 90° 变向都盖不住 —— 假自信";
}

TEST(MotionModel, StaticEllipseGrowsIsotropically)
{
  MotionModelParams params;
  const TargetSnapshot target = MakeTarget(0.0, 0.0);
  const PredictedPath path = PredictStatic(target, params);
  for (const auto & point : path.points) {
    const double expected =
      params.sigma_pos0_m + 0.5 * params.static_start_accel_mps2 * point.t_s * point.t_s;
    EXPECT_NEAR(point.sigma_along_m, expected, 1e-12);
    EXPECT_NEAR(point.sigma_cross_m, expected, 1e-12)
      << "静止目标起步方向未知，两个半轴必须同律增长";
  }
}

// ---------------------------------------------------------------------------
//  坏输入直接拒绝（RequireFinite 的家规）
// ---------------------------------------------------------------------------
TEST(MotionModel, RejectsNonFiniteInput)
{
  MotionModelParams params;
  TargetSnapshot target = MakeTarget(1.0, 0.0);
  target.position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PredictConstantVelocity(target, params, 0.8), std::invalid_argument);
}
