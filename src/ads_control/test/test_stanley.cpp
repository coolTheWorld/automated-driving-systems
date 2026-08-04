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
//  横向 Stanley 的 L1 测试 —— 含 CP-P2-A 的闭环收敛判据
//
//  分三层，各验各的：
//    ① 静态：前轴换算、控制律逐点手算、限幅。不动车。
//    ② 夹具自检：运动学自行车模型本身对不对（定转角 → 半径 L/tan δ 的圆）。
//       **夹具没验过就拿去验控制器，等于用一把没校准的尺子量东西。**
//    ③ 闭环：控制器 + 夹具 + 真实的 ReferenceLine 跑起来，看误差怎么收敛。
//
//  ⚠️ 判据的期望值来自 P2-S1.1 的**独立数值仿真**（Python 前向欧拉闭环），
//     与 docs/modules/control.md 的几何闭式解是两套方法。这与 P1 的做法一致：
//     test_routing 的期望值也来自一个穷举脚本，而不是再写一遍 Dijkstra。
//     **两套方法都错到一块去的概率，比一套方法自己验自己低得多。**
//
//  ⚠️ 闭环全程**定速**（加速度恒为 0）。纵向是 S3 的事，混进来的话
//     一条用例红了分不清是横向还是纵向的锅。
// =============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <optional>
#include <vector>

#include "ads_common/angles.hpp"
#include "ads_common/reference_line.hpp"
#include "ads_common/testing/path_fixtures.hpp"
#include "ads_control/stanley.hpp"
#include "kinematic_bicycle.hpp"

namespace
{

using ads_common::PathProjection;
using ads_common::Pose2D;
using ads_common::ReferenceLine;
using ads_common_test::KinematicBicycle;
using ads_control::StanleyController;
using ads_control::StanleyParams;
// 路径构造器搬到了 path_fixtures.hpp —— test_speed_profile 也要用同一批形状，
// 各写一份的话两边对"什么叫 R=8 的弯"就可能悄悄产生分歧。
using ads_common_test::MakeCircleLaps;
using ads_common_test::MakeLeftArc;
using ads_common_test::MakeStraightAlongX;
using ads_common_test::MakeStraightThenLeftArc;

// -----------------------------------------------------------------------------
//  常量
// -----------------------------------------------------------------------------
// ⚠️ 这几个数是 config/vehicle_params.yaml 的**手抄副本**。
//    L1 是纯 C++、不读 YAML（读了就要拉 yaml-cpp，而且测试要读文件、不再是毫秒级），
//    所以这里必然是一份复制。**改了 YAML 要回来改这里** ——
//    这正是本仓库到处在防的那类"两处各写一份"，但在 L1 这一层没有更好的办法。
//    真正的防线在 S4：节点从 YAML 读，届时可以在 launch 层加一致性断言。
//    在此之前，这条缺口是**已知且被记下来的**，不是被忽略的。
constexpr double kWheelbaseM = 2.700;
constexpr double kMaxSteerAngleRad = 0.600;
constexpr double kMaxSteerRateRadS = 0.500;
constexpr double kCruiseSpeedMps = 5.556;

// 控制器自己的参数（S4 会放进 config/control_params.yaml）。
constexpr double kGainInvS = 1.0;
constexpr double kSoftSpeedMps = 0.5;
constexpr double kControlDtS = 0.02;  // 50 Hz

// 地图上最急的弯：路口转弯车道 R = 8 m（config/campus_map.yaml 的 turn_radius_m）。
constexpr double kTurnRadiusM = 8.0;
// 曲率限速：v = √(a_lat_max / |κ|) = √(1.5 × 8) = 3.4641 m/s。S3 会实现它，
// 这里直接用那个值，因为 CP-P2-A 要验的是"按限速过弯时跟得住"。
constexpr double kMaxLateralAccelMps2 = 1.5;

StanleyParams DefaultParams()
{
  return StanleyParams{kGainInvS, kSoftSpeedMps, kMaxSteerAngleRad, kMaxSteerRateRadS};
}

double ArcSpeedMps() { return std::sqrt(kMaxLateralAccelMps2 * kTurnRadiusM); }

// -----------------------------------------------------------------------------
//  闭环仿真
// -----------------------------------------------------------------------------

struct LoopSample
{
  double time_s{0.0};
  double lateral_error_m{0.0};
  double heading_error_rad{0.0};
  double steering_rad{0.0};
  double path_progress_m{0.0};
};

struct LoopOptions
{
  double duration_s{5.0};
  double dt_s{kControlDtS};

  /// 第一拍的最近点提示。多圈圆路径上**必须**给（通常是 0）：
  /// 各圈几何完全重合，全局搜索的最小值在浮点意义上是任意一圈 ——
  /// 用例会因为编译器换了个 cos 实现而随机换圈，症状是"偶发失败"。
  /// 这也对应真实用法：S4 收到新路径时把 hint 重置。
  std::optional<std::size_t> initial_hint{std::nullopt};

  /// **反例开关**：给控制律额外叠加曲率前馈 δ_ff = arcsin(L·κ)。
  /// 只有 CurvatureFeedforwardBreaksTheSteadyState 用它，产品代码里没有这条路径。
  bool curvature_feedforward{false};

  /// **反例开关**：把误差量在**后轴**而不是前轴上（= 漏掉前轴换算）。
  /// 只有 UsingTheRearAxleDegradesTrackingBadly 用它。
  bool project_the_rear_axle{false};
};

/// 跑一段定速闭环，返回逐拍的误差记录。
///
/// 每拍的顺序与真实节点一致：取位姿 → **换算到前轴** → 投影 → 控制律 → 限幅 → 驱动车辆。
std::vector<LoopSample> RunClosedLoop(
  const ReferenceLine & path, KinematicBicycle vehicle, const StanleyParams & params,
  const LoopOptions & options)
{
  StanleyController controller(params);
  controller.reset(0.0);  // 方向盘回正起步，与 vehicle_cmd_bridge 的初值一致

  std::optional<std::size_t> hint = options.initial_hint;
  std::vector<LoopSample> samples;
  const int steps = static_cast<int>(std::lround(options.duration_s / options.dt_s));
  samples.reserve(static_cast<std::size_t>(steps) + 1);

  for (int i = 0; i <= steps; ++i) {
    // ⚠️ 前轴，不是后轴。漏掉这一步就是 control.md §3.2 那个经典错误。
    //    UsingTheRearAxleDegradesTrackingBadly 用 project_the_rear_axle 把它复现出来。
    const Pose2D query =
      options.project_the_rear_axle ? vehicle.rear_axle_pose() : vehicle.front_axle_pose();
    const PathProjection projection = path.project(query, hint);
    hint = projection.index;

    double heading_error_rad = projection.heading_error_rad;
    if (options.curvature_feedforward) {
      // 代数恒等式：δ = δ_ff + ψ − arctan(u) ≡ (ψ + δ_ff) − arctan(u)。
      // 所以把前馈并进航向误差，就能**原样复用产品代码的限幅**，
      // 而不必在测试里再抄一遍限幅逻辑（抄一遍的话，反例红了还得先排除
      // "是不是我的限幅抄错了"）。
      heading_error_rad +=
        std::asin(std::clamp(vehicle.wheelbase_m * projection.curvature_inv_m, -1.0, 1.0));
    }

    const double steering_rad = controller.update(
      heading_error_rad, projection.lateral_error_m, vehicle.speed_mps, options.dt_s);

    samples.push_back(
      {i * options.dt_s, projection.lateral_error_m, projection.heading_error_rad, steering_rad,
       projection.s_m});

    vehicle.step(steering_rad, 0.0, options.dt_s);  // 定速：加速度恒 0
  }
  return samples;
}

/// 最后一次 |e| ≥ tolerance 的时刻。此后一直在容差内，所以这就是"收敛时间"。
/// 全程未超出返回 0；末尾仍超出则返回接近 duration 的值（= 没收敛）。
double SettlingTimeS(const std::vector<LoopSample> & samples, double tolerance_m)
{
  double settled_at_s = 0.0;
  for (const LoopSample & sample : samples) {
    if (std::abs(sample.lateral_error_m) >= tolerance_m) {
      settled_at_s = sample.time_s;
    }
  }
  return settled_at_s;
}

double PeakAbsLateralErrorM(const std::vector<LoopSample> & samples)
{
  double peak_m = 0.0;
  for (const LoopSample & sample : samples) {
    peak_m = std::max(peak_m, std::abs(sample.lateral_error_m));
  }
  return peak_m;
}

/// 稳态：`from_time_s` 之后的 |e| 最大值。取最大而不是平均 ——
/// 平均会把一个还在缓慢发散的过程算成"稳态很小"。
double SteadyStateAbsLateralErrorM(const std::vector<LoopSample> & samples, double from_time_s)
{
  double worst_m = 0.0;
  for (const LoopSample & sample : samples) {
    if (sample.time_s >= from_time_s) {
      worst_m = std::max(worst_m, std::abs(sample.lateral_error_m));
    }
  }
  return worst_m;
}

/// 三点定圆的半径：R = a·b·c / (4·面积)。
///
/// 全部用**差分向量**算，绝对坐标只在相减时出现一次 —— 半径 7.5 m、步长 0.07 m
/// 这种量级悬殊的情形下，直接解圆心方程会因为两个近乎相等的大数相减而掉精度。
double CircumRadiusM(const Pose2D & p0, const Pose2D & p1, const Pose2D & p2)
{
  const double ax_m = p1.x_m - p0.x_m;
  const double ay_m = p1.y_m - p0.y_m;
  const double bx_m = p2.x_m - p1.x_m;
  const double by_m = p2.y_m - p1.y_m;
  const double cx_m = p2.x_m - p0.x_m;
  const double cy_m = p2.y_m - p0.y_m;
  const double twice_area_m2 = std::abs(ax_m * cy_m - ay_m * cx_m);
  return std::hypot(ax_m, ay_m) * std::hypot(bx_m, by_m) * std::hypot(cx_m, cy_m) /
         (2.0 * twice_area_m2);
}

/// 全程实际下发过的最大转向角速度，rad/s。用来判「速率限幅到底有没有起作用」——
/// 比拿横向误差去反推可靠得多，因为横向误差里还混着离散化残差等好几项。
double MaxSteeringRateRadS(const std::vector<LoopSample> & samples, double dt_s)
{
  double worst_rad_s = 0.0;
  for (std::size_t i = 1; i < samples.size(); ++i) {
    worst_rad_s =
      std::max(worst_rad_s, std::abs(samples[i].steering_rad - samples[i - 1].steering_rad) / dt_s);
  }
  return worst_rad_s;
}

/// 超调：误差穿到另一侧的最大幅度，占初始误差的比例。没穿过去就是 0。
double OvershootRatio(const std::vector<LoopSample> & samples)
{
  const double initial_m = samples.front().lateral_error_m;
  if (initial_m == 0.0) {
    return 0.0;
  }
  const double initial_sign = (initial_m > 0.0) ? 1.0 : -1.0;
  double worst_opposite_m = 0.0;
  for (const LoopSample & sample : samples) {
    worst_opposite_m = std::max(worst_opposite_m, -sample.lateral_error_m * initial_sign);
  }
  return worst_opposite_m / std::abs(initial_m);
}

}  // namespace

// =============================================================================
//  ① 静态：前轴换算（任务 2.1）
// =============================================================================

TEST(FrontAxle, IsOneWheelbaseAheadAlongTheVehicleHeading)
{
  // 朝 +x：纯 x 偏移。这一条是"看着对"的那种，单独有它挡不住任何错误。
  const Pose2D east = ads_control::front_axle_pose({10.0, 20.0, 0.0}, kWheelbaseM);
  EXPECT_NEAR(east.x_m, 10.0 + kWheelbaseM, 1e-12);
  EXPECT_NEAR(east.y_m, 20.0, 1e-12);

  // 朝 +y：偏移整个跑到 y 上去。**这一条才有鉴别力** ——
  // 把换算写成 `x += L`（忘了乘 cos）的实现在上面那条里是绿的，在这里必红。
  const Pose2D north = ads_control::front_axle_pose({10.0, 20.0, M_PI_2}, kWheelbaseM);
  EXPECT_NEAR(north.x_m, 10.0, 1e-12);
  EXPECT_NEAR(north.y_m, 20.0 + kWheelbaseM, 1e-12);

  // 任意角，含跨 ±π 的朝向。
  constexpr double kHeadingRad = 2.9;
  const Pose2D oblique = ads_control::front_axle_pose({-3.0, 4.5, kHeadingRad}, kWheelbaseM);
  EXPECT_NEAR(oblique.x_m, -3.0 + kWheelbaseM * std::cos(kHeadingRad), 1e-12);
  EXPECT_NEAR(oblique.y_m, 4.5 + kWheelbaseM * std::sin(kHeadingRad), 1e-12);

  // 朝向不变：车是刚体，前轴不会自己转。
  EXPECT_NEAR(oblique.heading_rad, kHeadingRad, 1e-12);
}

TEST(FrontAxle, RejectsAZeroOrNonFiniteWheelbase)
{
  // 轴距 0 = 前轴等于后轴 = **静默退化成 Stanley 最经典的实现错误**。
  // 抛异常是为了让"配置没读到"在启动时就喊出来，而不是变成弯道上的持续外偏。
  EXPECT_THROW(ads_control::front_axle_pose({0.0, 0.0, 0.0}, 0.0), std::invalid_argument);
  EXPECT_THROW(ads_control::front_axle_pose({0.0, 0.0, 0.0}, -2.7), std::invalid_argument);
  EXPECT_THROW(ads_control::front_axle_pose({0.0, 0.0, 0.0}, std::nan("")), std::invalid_argument);
}

TEST(FrontAxle, RejectsANonFinitePose)
{
  EXPECT_THROW(
    ads_control::front_axle_pose({std::nan(""), 0.0, 0.0}, kWheelbaseM), std::invalid_argument);
  EXPECT_THROW(
    ads_control::front_axle_pose({0.0, HUGE_VAL, 0.0}, kWheelbaseM), std::invalid_argument);
  EXPECT_THROW(
    ads_control::front_axle_pose({0.0, 0.0, std::nan("")}, kWheelbaseM), std::invalid_argument);
}

// =============================================================================
//  ① 静态：控制律（任务 2.2）
// =============================================================================

TEST(ControlLaw, MatchesTheHandComputedValue)
{
  // ψ = 0.1 rad，e = +0.4 m（车在路径左侧），v = 5.556 m/s
  //   u = k_e·e / (k_soft + v) = 0.4 / 6.056       = 0.06605019815059446
  //   arctan(u)                                     = 0.06595439796074151
  //   δ = ψ − arctan(u) = 0.1 − 0.06595439796074151 = 0.034045602039258493
  // 期望值由外部 Python 算出后写死在这里 —— 用同一个表达式再算一遍等于什么都没验。
  const double delta_rad =
    StanleyController::raw_steering_rad(0.1, 0.4, kCruiseSpeedMps, DefaultParams());
  EXPECT_NEAR(delta_rad, 0.034045602039258493, 1e-12);
}

TEST(ControlLaw, SteersRightWhenTheVehicleIsLeftOfThePath)
{
  // 车在路径**左**侧 1 m、车头与路径平行 → 必须向**右**打（δ < 0）。
  // 符号写反就是正反馈，车会一头冲出路径 —— 这是本节里唯一藏不住的错误。
  const double delta_rad =
    StanleyController::raw_steering_rad(0.0, 1.0, kCruiseSpeedMps, DefaultParams());
  EXPECT_LT(delta_rad, 0.0);
  EXPECT_NEAR(delta_rad, -0.1636487856745889, 1e-12);
}

TEST(ControlLaw, SteersLeftWhenTheVehicleHeadingIsRightOfThePath)
{
  // 车在路径上（e = 0）、车头偏右 30°（ψ = +0.5236）→ 必须向**左**打，
  // 且此时横向项恰好为 0，输出就是 ψ 本身。
  const double heading_error_rad = 30.0 * M_PI / 180.0;
  const double delta_rad =
    StanleyController::raw_steering_rad(heading_error_rad, 0.0, kCruiseSpeedMps, DefaultParams());
  EXPECT_NEAR(delta_rad, heading_error_rad, 1e-15);
}

TEST(ControlLaw, ZeroSpeedDoesNotBlowUp)
{
  // k_soft 存在的唯一理由。没有它这里是 atan(1/0) → ±π/2，**停车时方向盘打死**，
  // 而起步和停车是每次运行都必然经过的工况。
  const double delta_rad = StanleyController::raw_steering_rad(0.0, 1.0, 0.0, DefaultParams());
  EXPECT_TRUE(std::isfinite(delta_rad));
  EXPECT_NEAR(delta_rad, -1.1071487177940904, 1e-12);  // −atan(1.0 / 0.5)
}

TEST(ControlLaw, NegativeSpeedIsTreatedAsZeroRatherThanFlippingTheSign)
{
  // 若不夹取，v = −3 会让分母 0.5 − 3 = −2.5 < 0，横向项**整体变号** ——
  // 那是正反馈。/odom 在停车时抖出负速度很常见，所以这条不是纸上谈兵。
  const double reverse_rad = StanleyController::raw_steering_rad(0.0, 1.0, -3.0, DefaultParams());
  const double standstill_rad = StanleyController::raw_steering_rad(0.0, 1.0, 0.0, DefaultParams());
  EXPECT_NEAR(reverse_rad, standstill_rad, 1e-15);
  EXPECT_LT(reverse_rad, 0.0);  // 仍然是"向右修正"，没有变号
}

TEST(Params, RejectNonPositiveOrNonFiniteValues)
{
  const std::vector<double> bad_values{0.0, -1.0, std::nan(""), HUGE_VAL};
  for (const double bad : bad_values) {
    StanleyParams p = DefaultParams();
    p.gain_inv_s = bad;
    EXPECT_THROW(StanleyController{p}, std::invalid_argument) << "gain_inv_s = " << bad;

    p = DefaultParams();
    p.soft_speed_mps = bad;
    EXPECT_THROW(StanleyController{p}, std::invalid_argument) << "soft_speed_mps = " << bad;

    p = DefaultParams();
    p.max_steer_angle_rad = bad;
    EXPECT_THROW(StanleyController{p}, std::invalid_argument) << "max_steer_angle_rad = " << bad;

    p = DefaultParams();
    p.max_steer_rate_rad_s = bad;
    EXPECT_THROW(StanleyController{p}, std::invalid_argument) << "max_steer_rate_rad_s = " << bad;
  }
}

// =============================================================================
//  ① 静态：限幅（任务 2.3）
// =============================================================================

TEST(Limits, NeverExceedsTheSteeringAngleLimit)
{
  StanleyController controller(DefaultParams());
  // 巨大的横向误差 → 控制律要求接近 −π/2，远超机械极限 0.600。
  for (int i = 0; i < 500; ++i) {
    const double delta_rad = controller.update(0.0, 1000.0, kCruiseSpeedMps, kControlDtS);
    EXPECT_LE(std::abs(delta_rad), kMaxSteerAngleRad + 1e-12) << "第 " << i << " 拍";
  }
  // 足够久之后应当**贴在**极限上，而不是停在半路。
  EXPECT_NEAR(controller.steering_rad(), -kMaxSteerAngleRad, 1e-12);
}

TEST(Limits, NeverExceedsTheSteeringRateLimit)
{
  StanleyController controller(DefaultParams());
  double previous_rad = controller.steering_rad();
  const double max_step_rad = kMaxSteerRateRadS * kControlDtS;  // 0.010 rad/拍

  // 第一拍就给一个"打死"指令：限幅前是 −π/2 量级，限幅后每拍只许动 0.01。
  for (int i = 0; i < 200; ++i) {
    const double delta_rad = controller.update(0.0, 1000.0, kCruiseSpeedMps, kControlDtS);
    EXPECT_LE(std::abs(delta_rad - previous_rad), max_step_rad + 1e-12) << "第 " << i << " 拍";
    previous_rad = delta_rad;
  }

  // 零到满舵要 0.600 / 0.500 = 1.2 s = 60 拍。
  // ⚠️ 这个数**只在阶跃工况下有意义**。入弯不是阶跃（δ≈ψ 而 ψ 以 v/R 缓增），
  //    所以别拿 1.2 s 去估入弯暂态 —— 详见 TheSteeringRateLimitOnlyBinds… 那条。
  StanleyController fresh(DefaultParams());
  for (int i = 0; i < 60; ++i) {
    fresh.update(0.0, 1000.0, kCruiseSpeedMps, kControlDtS);
  }
  EXPECT_NEAR(fresh.steering_rad(), -kMaxSteerAngleRad, 1e-12);
}

TEST(Limits, ZeroTimeStepHoldsTheSteering)
{
  // dt = 0 不是错误：速率限幅允许的变化量恰好是 0。这从公式里自然掉出来，
  // 不需要特判 —— 而"需要特判"的实现往往在 dt=0 时直接除零。
  StanleyController controller(DefaultParams());
  controller.update(0.0, 1000.0, kCruiseSpeedMps, kControlDtS);
  const double held_rad = controller.steering_rad();
  EXPECT_NEAR(controller.update(0.0, 1000.0, kCruiseSpeedMps, 0.0), held_rad, 1e-15);
}

TEST(Limits, RejectsNegativeTimeStep)
{
  // dt < 0 意味着时钟倒流（本仓库见过：两套仿真并存时 TF 刷 jump back in time）。
  // 当成正常值会让速率限幅方向反过来。
  StanleyController controller(DefaultParams());
  EXPECT_THROW(controller.update(0.0, 1.0, kCruiseSpeedMps, -0.02), std::invalid_argument);
}

TEST(Limits, NonFiniteInputThrowsAndLeavesTheStateUnpoisoned)
{
  // **这条比"会抛异常"更重要**：转角是**持久状态**，一拍的 NaN 会让控制器
  // 永久输出 NaN（NaN 参与任何运算仍是 NaN），之后输入恢复正常也回不来。
  // 现场表现是"车自己停了"（下游 isfinite 挡下 + 看门狗刹停），
  // 于是所有人去查控制器 —— 而错在上游。
  StanleyController controller(DefaultParams());
  controller.update(0.1, 0.2, kCruiseSpeedMps, kControlDtS);
  const double before_rad = controller.steering_rad();

  EXPECT_THROW(
    controller.update(std::nan(""), 0.2, kCruiseSpeedMps, kControlDtS), std::invalid_argument);
  EXPECT_THROW(
    controller.update(0.1, std::nan(""), kCruiseSpeedMps, kControlDtS), std::invalid_argument);
  EXPECT_THROW(controller.update(0.1, 0.2, HUGE_VAL, kControlDtS), std::invalid_argument);

  EXPECT_TRUE(std::isfinite(controller.steering_rad()));
  EXPECT_NEAR(controller.steering_rad(), before_rad, 1e-15);
}

TEST(Limits, ResetClampsToTheAngleLimit)
{
  StanleyController controller(DefaultParams());
  controller.reset(10.0);
  EXPECT_NEAR(controller.steering_rad(), kMaxSteerAngleRad, 1e-12);
  controller.reset();
  EXPECT_NEAR(controller.steering_rad(), 0.0, 1e-15);
  EXPECT_THROW(controller.reset(std::nan("")), std::invalid_argument);
}

// =============================================================================
//  ② 夹具自检：运动学自行车模型（任务 2.4）
// =============================================================================

TEST(BicycleModel, ConstantSteeringDrivesACircleOfRadiusWheelbaseOverTanDelta)
{
  // δ = arcsin(L/R_f) 时，后轴走半径 R_r = √(R_f² − L²) 的圆。
  // 这两个式子是 control.md §3.4 的几何基础 —— 夹具对不上它们的话，
  // 后面所有闭环判据都是在拿一把没校准的尺子量东西。
  const double steering_rad = std::asin(kWheelbaseM / kTurnRadiusM);
  const double expected_radius_m =
    std::sqrt(kTurnRadiusM * kTurnRadiusM - kWheelbaseM * kWheelbaseM);

  KinematicBicycle vehicle;
  vehicle.wheelbase_m = kWheelbaseM;
  vehicle.x_m = expected_radius_m;
  vehicle.y_m = 0.0;
  vehicle.heading_rad = M_PI_2;
  vehicle.speed_mps = ArcSpeedMps();

  // ⚠️ **不要**用「到假定圆心的距离」当判据，这一条是实测踩出来的。
  //
  //    前向欧拉画的是**内接正多边形**：顶点确实都落在一个圆上（半径
  //    R_r·(1 + (ωΔt)²/24)，比 R_r 大 26 µm），但那个圆的**圆心不在**
  //    "起点 + R_r × 左法向"上。原因是多边形的边与外接圆在顶点处的切线
  //    差了半个转角 Δφ/2 —— 而本用例是让车沿**切向**起步的，
  //    于是圆心整体偏移 R_r·sin(Δφ/2) = 34.64 mm（实测 34.67 mm，对得上）。
  //
  //    那 34.6 mm 是**初始条件的产物，不是模型误差**。判据只要换成
  //    "局部三点定出来的曲率半径"就与初始条件无关了 —— 这也正是
  //    "轨迹是一个半径 L/tan δ 的圆"这句话的直接翻译。
  std::vector<Pose2D> track;
  track.reserve(1001);
  track.push_back(vehicle.rear_axle_pose());
  for (int i = 0; i < 1000; ++i) {
    vehicle.step(steering_rad, 0.0, kControlDtS);
    track.push_back(vehicle.rear_axle_pose());
  }

  double worst_deviation_m = 0.0;
  for (std::size_t i = 0; i + 2 < track.size(); ++i) {
    const double radius_m = CircumRadiusM(track[i], track[i + 1], track[i + 2]);
    worst_deviation_m = std::max(worst_deviation_m, std::abs(radius_m - expected_radius_m));
  }
  std::cout << "  [夹具] 后轴圆半径 " << expected_radius_m << " m，999 组三点定圆最大偏差 "
            << worst_deviation_m * 1e3 << " mm\n";

  // 理论偏差 R_r·(ωΔt)²/24 = 26 µm，判据 1 mm 留了约 38 倍余量。
  EXPECT_LT(worst_deviation_m, 1e-3);
}

TEST(BicycleModel, ClosesTheLoopAfterExactlyOneRevolution)
{
  const double steering_rad = std::asin(kWheelbaseM / kTurnRadiusM);
  const double radius_m = std::sqrt(kTurnRadiusM * kTurnRadiusM - kWheelbaseM * kWheelbaseM);
  const double speed_mps = ArcSpeedMps();

  // ⚠️ 步长是**推导出来的**，不是 0.02：前向欧拉画的是正多边形，
  //    只有当"步数 × 每步转角"恰好等于 2π 时多边形才闭合。
  //    用 0.02 的话残留的是一个**角度余数**（多边形没画完最后一条边），
  //    那是采样问题，不是模型误差 —— 把它算进闭合误差等于测错了东西。
  constexpr int kSteps = 2000;
  const double yaw_rate_rad_s = speed_mps / radius_m;
  const double dt_s = 2.0 * M_PI / (kSteps * yaw_rate_rad_s);

  KinematicBicycle vehicle;
  vehicle.wheelbase_m = kWheelbaseM;
  vehicle.x_m = radius_m;
  vehicle.heading_rad = M_PI_2;
  vehicle.speed_mps = speed_mps;
  const double start_x_m = vehicle.x_m;
  const double start_y_m = vehicle.y_m;

  for (int i = 0; i < kSteps; ++i) {
    vehicle.step(steering_rad, 0.0, dt_s);
  }
  const double closure_m = std::hypot(vehicle.x_m - start_x_m, vehicle.y_m - start_y_m);
  std::cout << "  [夹具] 整圈闭合误差 " << closure_m * 1e3 << " mm\n";
  EXPECT_LT(closure_m, 1e-3);
}

TEST(BicycleModel, ZeroSteeringGoesStraightAndPositiveSteeringTurnsLeft)
{
  KinematicBicycle straight;
  straight.wheelbase_m = kWheelbaseM;
  straight.speed_mps = kCruiseSpeedMps;
  for (int i = 0; i < 100; ++i) {
    straight.step(0.0, 0.0, kControlDtS);
  }
  EXPECT_NEAR(straight.y_m, 0.0, 1e-12);
  EXPECT_NEAR(straight.heading_rad, 0.0, 1e-12);
  EXPECT_NEAR(straight.x_m, kCruiseSpeedMps * 100 * kControlDtS, 1e-9);

  // δ > 0 = 左转 = 朝向增大 = y 增大。符号约定与 REP-103 一致（逆时针为正）。
  KinematicBicycle left;
  left.wheelbase_m = kWheelbaseM;
  left.speed_mps = kCruiseSpeedMps;
  for (int i = 0; i < 100; ++i) {
    left.step(0.2, 0.0, kControlDtS);
  }
  EXPECT_GT(left.heading_rad, 0.0);
  EXPECT_GT(left.y_m, 0.0);
}

// =============================================================================
//  ③ 闭环：CP-P2-A（任务 2.5）
// =============================================================================

TEST(ClosedLoop, ConvergesFromOneMetreLateralOffset)
{
  // 场景 1：直线路径，初始 |e| = 1.0 m，v = 5.556 m/s（巡航）。
  // 判据：5 s 内收到 |e| < 0.05 m，超调 < 5%。
  // 理论 ln(20)/k_e = 3.00 s；S1.1 的 Python 闭环给 3.42 s、超调 0.00%。
  const ReferenceLine path(MakeStraightAlongX(60.0, 0.0, 0.5));

  KinematicBicycle vehicle;
  vehicle.wheelbase_m = kWheelbaseM;
  vehicle.y_m = 1.0;  // 前轴与后轴同在 y = +1（车头朝 +x），所以初始 e = +1.0
  vehicle.speed_mps = kCruiseSpeedMps;

  LoopOptions options;
  options.duration_s = 5.0;
  const std::vector<LoopSample> samples = RunClosedLoop(path, vehicle, DefaultParams(), options);

  ASSERT_NEAR(samples.front().lateral_error_m, 1.0, 1e-9);
  const double settling_s = SettlingTimeS(samples, 0.05);
  const double overshoot = OvershootRatio(samples);
  std::cout << "  [CP-P2-A ①] 收敛时间 " << settling_s << " s，超调 " << overshoot * 100.0
            << " %\n";

  EXPECT_LT(settling_s, 5.0);
  EXPECT_LT(overshoot, 0.05);
}

TEST(ClosedLoop, ConvergesFromAThirtyDegreeHeadingError)
{
  // 场景 2：直线路径，初始 ψ = 30°、e = 0（**前轴**在路径上），v = 5.556。
  // 判据：5 s 内 |e| < 0.05 m。S1.1 给 2.64 s、峰值 0.986 m。
  const ReferenceLine path(MakeStraightAlongX(60.0, 0.0, 0.5));

  constexpr double kHeadingRad = 30.0 * M_PI / 180.0;
  KinematicBicycle vehicle;
  vehicle.wheelbase_m = kWheelbaseM;
  // 后轴压低一个 L·sin30°，好让**前轴**恰好落在路径上 —— e 是在前轴处量的。
  vehicle.x_m = 0.0;
  vehicle.y_m = -kWheelbaseM * std::sin(kHeadingRad);
  vehicle.heading_rad = kHeadingRad;
  vehicle.speed_mps = kCruiseSpeedMps;

  LoopOptions options;
  options.duration_s = 5.0;
  const std::vector<LoopSample> samples = RunClosedLoop(path, vehicle, DefaultParams(), options);

  ASSERT_NEAR(samples.front().lateral_error_m, 0.0, 1e-9);
  const double settling_s = SettlingTimeS(samples, 0.05);
  std::cout << "  [CP-P2-A ②] 收敛时间 " << settling_s << " s，峰值 |e| "
            << PeakAbsLateralErrorM(samples) << " m\n";

  EXPECT_LT(settling_s, 5.0);
}

TEST(ClosedLoop, TracksAConstantCurvatureArcWithoutSteadyStateError)
{
  // 场景 3：R = 8 m 圆弧，v = 3.4641 m/s（按 a_lat = 1.5 限速）。
  // 判据：**稳态** |e| < 0.05 m。
  //
  // ⚠️ 这条判据来自 control.md §3.4 的推导，而推导**推翻了初稿**：
  //    初稿写的是「Stanley 在定曲率上有固有稳态误差，判据放 0.10 m」。
  //    实际 ψ ≡ δ 是几何恒等式，代回控制律直接得 e_ss = 0。
  //    若不推导就写代码，这条会以「稳态差 10 cm、判据刚好卡在边缘」的形态存在，
  //    而人会把它当成 Stanley 的固有特性接受下来。
  //
  // 路径用 0.1 m 采样：让折线本身的误差（矢高 0.16 mm）远小于要测的量，
  // 这样测出来的就是**控制律**的残差而不是采样的残差。真实采样另有一条用例。
  const ReferenceLine path(MakeCircleLaps(kTurnRadiusM, 3.0, 0.1));

  KinematicBicycle vehicle;
  vehicle.wheelbase_m = kWheelbaseM;
  vehicle.x_m = kTurnRadiusM;  // 后轴放在路径圆上，前轴因此在圆外 0.44 m
  vehicle.heading_rad = M_PI_2;
  vehicle.speed_mps = ArcSpeedMps();

  LoopOptions options;
  options.duration_s = 20.0;
  options.initial_hint = 0;  // 多圈路径必须给，理由见 LoopOptions
  const std::vector<LoopSample> samples = RunClosedLoop(path, vehicle, DefaultParams(), options);

  const double steady_state_m = SteadyStateAbsLateralErrorM(samples, 15.0);
  const double steady_psi_rad = samples.back().heading_error_rad;
  const double analytic_psi_rad = std::asin(kWheelbaseM / kTurnRadiusM);
  std::cout << "  [CP-P2-A ③] 稳态 |e| " << steady_state_m << " m，稳态 ψ " << steady_psi_rad
            << " rad（解析 " << analytic_psi_rad << "）\n";

  EXPECT_LT(steady_state_m, 0.05);

  // §3.4 的核心断言：稳态航向误差**就是**稳态转角，这不是巧合而是恒等式。
  // 那 19.72° 是内轮差（后轴内偏 0.469 m），不是误差 ——
  // RViz 里过路口能明显看到车"斜着"过弯，将来一定有人来问。
  EXPECT_NEAR(steady_psi_rad, analytic_psi_rad, 0.02);
}

TEST(ClosedLoop, TheSteadyStateResidualIsProportionalToTheTimeStep)
{
  // 上一条测出来的稳态不是 0 而是厘米级。这条用例回答「那到底是误差还是残差」：
  // **与 dt 严格成正比 = 离散化残差**，外推 dt → 0 就是解析的 0。
  //
  // 这个区分是有实际后果的：想把它压小要**提高控制频率**，调增益动不了它。
  // 没有这条用例的话，下一个人会花一天去调 k_e。
  const ReferenceLine path(MakeCircleLaps(kTurnRadiusM, 3.0, 0.1));

  const std::vector<double> steps_s{0.04, 0.02, 0.01};
  std::vector<double> residuals_m;
  for (const double dt_s : steps_s) {
    KinematicBicycle vehicle;
    vehicle.wheelbase_m = kWheelbaseM;
    vehicle.x_m = kTurnRadiusM;
    vehicle.heading_rad = M_PI_2;
    vehicle.speed_mps = ArcSpeedMps();

    LoopOptions options;
    options.duration_s = 20.0;
    options.dt_s = dt_s;
    options.initial_hint = 0;
    const std::vector<LoopSample> samples = RunClosedLoop(path, vehicle, DefaultParams(), options);
    residuals_m.push_back(SteadyStateAbsLateralErrorM(samples, 15.0));
  }

  std::cout << "  [残差 ∝ dt] ";
  for (std::size_t i = 0; i < steps_s.size(); ++i) {
    std::cout << "dt=" << steps_s[i] << " → " << residuals_m[i]
              << " m（e/dt = " << residuals_m[i] / steps_s[i] << "）  ";
  }
  std::cout << "\n";

  // 比值在 4 倍的 dt 范围内应当基本不变。判据放 10%：
  // 太松就测不出"其实是个常数误差"，太紧会被欧拉的高阶项碰到。
  const double ratio_coarse = residuals_m.front() / steps_s.front();
  const double ratio_fine = residuals_m.back() / steps_s.back();
  EXPECT_NEAR(ratio_fine / ratio_coarse, 1.0, 0.10);
}

TEST(ClosedLoop, EnteringACurveFromAStraightHasASmallTransient)
{
  // 场景 4：直路 → R = 8 m 圆弧，v = 3.4641，车从直路上完美跟踪进入。
  // 判据：**入弯峰值** |e| < 0.10 m。
  //
  // ⚠️ 初稿估的是 0.19 m，按「ψ 要一个时间常数才建立，这期间车走直线」算的 ——
  //    **那个估计错在 ψ 没有滞后**：路径切向以 v/R 的速率转走，ψ 立刻同步增长，
  //    不经过任何一阶环节。S1.1 的 Python 闭环给 0.0166 m。
  //    但那次仿真**没有转向速率限幅**，而真控制器有（0.5 rad/s，
  //    从 0 打到入弯所需的 0.344 rad 要 0.69 s）—— 差值就是这条用例要测出来的东西。
  const ReferenceLine path(MakeStraightThenLeftArc(10.0, kTurnRadiusM, M_PI, 0.1));

  KinematicBicycle vehicle;
  vehicle.wheelbase_m = kWheelbaseM;
  vehicle.speed_mps = ArcSpeedMps();  // 起点 (0,0)、朝 +x，前轴在 (2.7,0)，e = 0

  LoopOptions options;
  options.duration_s = 8.0;
  const std::vector<LoopSample> samples = RunClosedLoop(path, vehicle, DefaultParams(), options);

  ASSERT_NEAR(samples.front().lateral_error_m, 0.0, 1e-9);
  // 车不能开出路径末端，否则量到的是"投影被夹到端点"的假误差。
  ASSERT_LT(samples.back().path_progress_m, path.length_m() - 1.0);

  const double peak_m = PeakAbsLateralErrorM(samples);
  std::cout << "  [CP-P2-A ④] 入弯峰值 |e| " << peak_m << " m，末段稳态 "
            << SteadyStateAbsLateralErrorM(samples, 6.0) << " m\n";

  EXPECT_LT(peak_m, 0.10);
}

TEST(ClosedLoop, TheSteeringRateLimitOnlyBindsAboveRadiusTimesRateLimit)
{
  // 上一条用例里入弯峰值只有 1.6 cm，而 plan.md 原本担心的是
  // 「0.5 rad/s 意味着零到满舵 1.2 s，这个相位滞后可能把稳定的增益变成震荡的」。
  // 那个担心**在本地图的几何上不成立**，原因可以精确写下来：
  //
  //   入弯时 e ≈ 0，于是 δ ≈ ψ；而 ψ 以路径切向的转速 **v/R** 增长。
  //   所以入弯真正需要的转向速率就是 v/R，**不是**"1.2 s 打到满舵"那种阶跃。
  //   限幅开始起作用的临界车速因此是   v* = R · max_steer_rate = 8 × 0.5 = 4.0 m/s。
  //
  //   按 a_lat_max = 1.5 限速得 3.464 m/s，在临界值**以下 13%** —— 限幅根本没碰到。
  //
  // ⚠️ 交给 S3 的一条约束：a_lat_max 调到 2.0 时 √(2.0×8) = **恰好 4.0 m/s**，
  //    正好顶在临界值上。也就是说"横向加速度上限"这个看起来只关舒适性的参数，
  //    与转向速率限幅是**耦合**的。本用例把这个耦合钉住。
  //
  // 判据用**实际下发过的最大转向角速度**，不用横向误差 ——
  // 巡航速度下横向误差里混着更大的离散化残差（a_lat = 3.86 m/s²），
  // 拿它反推限幅有没有起作用会被那一项淹没（实测过：0.0426 vs 0.0466 m，
  // 差 9%，且**方向与直觉相反**，根本不能作为判据）。
  const ReferenceLine path(MakeStraightThenLeftArc(10.0, kTurnRadiusM, 1.5 * M_PI, 0.1));

  const double binding_speed_mps = kTurnRadiusM * kMaxSteerRateRadS;
  EXPECT_LT(ArcSpeedMps(), binding_speed_mps) << "曲率限速后的车速应当低于临界车速";

  struct Run
  {
    double max_steering_rate_rad_s;
    double peak_lateral_error_m;
  };
  // 车速越高走得越远；时长按"前轴不越过路径末端"反推，并留足余量 ——
  // 弯道上前轴走的是 R=8 的弧、后轴走 R=7.53 的弧，前轴的弧长多 6.2%，
  // 只按车速×时长估会不够（这一项漏掉过一次，量到的是被夹到端点的假误差）。
  auto run_at = [&path](double speed_mps, double max_steer_rate_rad_s) {
    StanleyParams params = DefaultParams();
    params.max_steer_rate_rad_s = max_steer_rate_rad_s;

    KinematicBicycle vehicle;
    vehicle.wheelbase_m = kWheelbaseM;
    vehicle.speed_mps = speed_mps;
    LoopOptions options;
    options.duration_s = (path.length_m() - 12.0) / speed_mps;
    const std::vector<LoopSample> samples = RunClosedLoop(path, vehicle, params, options);
    EXPECT_LT(samples.back().path_progress_m, path.length_m() - 1.0);
    return Run{MaxSteeringRateRadS(samples, options.dt_s), PeakAbsLateralErrorM(samples)};
  };

  // 「放开限幅」= 把上限设成一个绝不会被碰到的值，于是量到的就是**控制律需要的**速率。
  constexpr double kNoRateLimitRadS = 50.0;
  const Run limited_free = run_at(ArcSpeedMps(), kNoRateLimitRadS);
  const Run limited_real = run_at(ArcSpeedMps(), kMaxSteerRateRadS);
  const Run cruise_free = run_at(kCruiseSpeedMps, kNoRateLimitRadS);
  const Run cruise_real = run_at(kCruiseSpeedMps, kMaxSteerRateRadS);

  std::cout << "  [速率限幅] 临界车速 " << binding_speed_mps << " m/s\n"
            << "            v=" << ArcSpeedMps() << "：需要 "
            << limited_free.max_steering_rate_rad_s << " rad/s（未超限），峰值 |e| "
            << limited_real.peak_lateral_error_m << " m\n"
            << "            v=" << kCruiseSpeedMps << "：需要 "
            << cruise_free.max_steering_rate_rad_s << " rad/s（**超限**），峰值 |e| "
            << cruise_real.peak_lateral_error_m << " m\n";

  // ① 曲率限速后，控制律需要的转向速率本来就低于限幅 → 限幅不起作用。
  EXPECT_LT(limited_free.max_steering_rate_rad_s, kMaxSteerRateRadS);
  // ② 定速巡航冲进同一个弯，需要的速率超出限幅 → 限幅会起作用。
  EXPECT_GT(cruise_free.max_steering_rate_rad_s, kMaxSteerRateRadS);
  // ③ 而限幅一旦启用就**真的**卡住了输出（这才证明它接进了闭环，不是摆设）。
  EXPECT_LE(cruise_real.max_steering_rate_rad_s, kMaxSteerRateRadS + 1e-12);
  // ④ 即便超限，峰值仍在 CP-P2-A 的判据内。**这不是判据，是把余量记下来**：
  //    a_lat_max 调到 2.0 顶在临界值上时，这里会先红。
  EXPECT_LT(cruise_real.peak_lateral_error_m, 0.10);
}

TEST(ClosedLoop, RealisticPathSamplingBarelyChangesTheSteadyState)
{
  // 上面的圆弧用例用 0.1 m 采样，为的是量控制律本身。真实路径是 map_node 发的
  // **0.5 m**（参考线弧长；弯道外侧实际点距还要大 14.58%）。
  // 这条用例回答 S4 真正关心的问题：换成真实采样后差多少。
  //
  // 折线内接于圆，矢高 R(1−cos(Δφ/2)) = 3.9 mm，所以预期是"稍微变大一点"。
  // 如果哪天这个差值突然变大，八成是投影退化成了"取最近采样点"。
  const ReferenceLine fine_path(MakeCircleLaps(kTurnRadiusM, 3.0, 0.1));
  const ReferenceLine coarse_path(MakeCircleLaps(kTurnRadiusM, 3.0, 0.5));

  KinematicBicycle vehicle;
  vehicle.wheelbase_m = kWheelbaseM;
  vehicle.x_m = kTurnRadiusM;
  vehicle.heading_rad = M_PI_2;
  vehicle.speed_mps = ArcSpeedMps();

  LoopOptions options;
  options.duration_s = 20.0;
  options.initial_hint = 0;

  const double fine_m =
    SteadyStateAbsLateralErrorM(RunClosedLoop(fine_path, vehicle, DefaultParams(), options), 15.0);
  const double coarse_m = SteadyStateAbsLateralErrorM(
    RunClosedLoop(coarse_path, vehicle, DefaultParams(), options), 15.0);
  std::cout << "  [采样影响] 0.1 m 采样 " << fine_m << " m，0.5 m 采样 " << coarse_m << " m，差 "
            << (coarse_m - fine_m) * 1e3 << " mm\n";

  EXPECT_LT(coarse_m, 0.05);           // 真实采样下判据照样成立
  EXPECT_LT(coarse_m - fine_m, 0.01);  // 采样带来的额外偏差 < 1 cm
}

// =============================================================================
//  ③ 闭环反例：漏掉前轴换算（任务 2.1 的另一半）
// =============================================================================

TEST(ClosedLoop, UsingTheRearAxleDegradesTrackingBadly)
{
  // FrontAxle.IsOneWheelbaseAhead 只验了那个**函数**算得对不对。
  // 但真正会出事的是：S4 的节点忘了调它，直接把 base_link 的位姿投上去。
  // 那种错误在"函数正确"的前提下照样发生，所以必须在**闭环**里也钉一条。
  //
  // （这条用例是故障注入时补上的：把 front_axle_pose() 改成直接返回后轴，
  //   当时只有那一条静态用例变红，所有闭环用例全绿 —— 因为闭环用的是夹具
  //   自己的换算。**测试套件对这个错误当时是没有鉴别力的。**）
  //
  // 解析预期（定曲率、后轴投影）：后轴走的是同心圆，车身朝向与该点的路径切向
  // 恰好平行，于是 ψ ≡ 0，航向项整个消失。转弯全靠横向项撑：
  //     arctan(k_e·|e|/(k_soft+v)) = arctan(L/R_rear)，R_rear ≈ R + |e|
  //     ⟹ |e| ≈ (k_soft+v)·L/(k_e·(R+|e|)) ≈ 3.9641×2.7/9.17 ≈ 1.17 m
  // 且符号为**负** = 路径右侧 = 弯道**外**侧 —— 正是文档里说的"持续外偏"。
  const ReferenceLine path(MakeCircleLaps(kTurnRadiusM, 4.0, 0.1));

  KinematicBicycle vehicle;
  vehicle.wheelbase_m = kWheelbaseM;
  vehicle.x_m = kTurnRadiusM;
  vehicle.heading_rad = M_PI_2;
  vehicle.speed_mps = ArcSpeedMps();

  LoopOptions options;
  options.duration_s = 25.0;
  options.initial_hint = 0;
  options.project_the_rear_axle = true;
  const std::vector<LoopSample> samples = RunClosedLoop(path, vehicle, DefaultParams(), options);

  const double steady_state_m = SteadyStateAbsLateralErrorM(samples, 20.0);
  std::cout << "  [反例] 用后轴代替前轴，弯道稳态 |e| " << steady_state_m << " m（符号 "
            << samples.back().lateral_error_m << "，负 = 外偏），稳态 ψ "
            << samples.back().heading_error_rad << " rad\n";

  // ① 必须远超判据 —— 证明定曲率那条用例挡得住这个错误。
  EXPECT_GT(steady_state_m, 0.5);
  // ② 方向必须是**外**偏（左转弯 → e 为负 = 路径右侧 = 弯道外侧）。
  //    方向对上了才说明我们理解的机理是对的，而不是碰巧红了。
  EXPECT_LT(samples.back().lateral_error_m, 0.0);
  // ③ 航向误差应当塌到 0 附近 —— 这就是"航向项整个消失"的直接证据。
  EXPECT_NEAR(samples.back().heading_error_rad, 0.0, 0.02);
}

// =============================================================================
//  ③ 闭环反例：不要加曲率前馈（任务 2.6）
// =============================================================================

TEST(ClosedLoop, CurvatureFeedforwardBreaksTheSteadyState)
{
  // 「曲率都算出来了，顺手加个 δ_ff = arcsin(L·κ) 让它入弯更快」——
  // 一个看起来毫无争议的改进。§3.4 已证明航向项**就是**那个前馈，
  // 再加一份就是双倍补偿：
  //     δ = δ_ff + ψ − arctan(u)，且 ψ ≡ δ  ⟹  arctan(u) = δ_ff
  //     ⟹  e_ss = (k_soft + v)·tan(δ_ff)/k_e = 3.9641 × 0.3585 = 1.4213 m
  // 车道越界线才 0.85 m —— **车直接开到路面外面去了**。
  //
  // 这条用例存在的意义不是"测前馈"，而是**证明上面那条定曲率用例有鉴别力**：
  // 该改动在直路上毫无影响、所有直线用例全绿、代码审查也挡不住
  // （那行代码看起来完全正确）。挡住它的只有一条定曲率稳态用例。
  const ReferenceLine path(MakeCircleLaps(kTurnRadiusM, 5.0, 0.1));

  KinematicBicycle vehicle;
  vehicle.wheelbase_m = kWheelbaseM;
  vehicle.x_m = kTurnRadiusM;
  vehicle.heading_rad = M_PI_2;
  vehicle.speed_mps = ArcSpeedMps();

  LoopOptions options;
  options.duration_s = 30.0;
  options.initial_hint = 0;
  options.curvature_feedforward = true;
  const std::vector<LoopSample> samples = RunClosedLoop(path, vehicle, DefaultParams(), options);

  const double steady_state_m = SteadyStateAbsLateralErrorM(samples, 25.0);
  const double analytic_m =
    (kSoftSpeedMps + ArcSpeedMps()) * std::tan(std::asin(kWheelbaseM / kTurnRadiusM)) / kGainInvS;
  std::cout << "  [反例] 加了曲率前馈后稳态 |e| " << steady_state_m << " m（解析 " << analytic_m
            << " m，判据线 0.05 m）\n";

  // ① 必须**远远**超过判据 —— 否则说明那条判据分辨不出这个错误。
  EXPECT_GT(steady_state_m, 0.5);
  // ② 且要落在解析值附近，证明我们理解的机理是对的，而不是碰巧红了。
  EXPECT_NEAR(steady_state_m, analytic_m, 0.15 * analytic_m);
}
