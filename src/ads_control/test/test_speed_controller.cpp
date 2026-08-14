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
//  速度环的 L1 测试
//
//  被控对象有两个版本，各测各的：
//
//    ① **理想积分器** `v̇ = a`。用来验 control.md §4.4 那条反直觉的结论：
//       纯 P 对常值目标就没有稳态误差，且闭环时间常数恰为 1/K_p。
//
//    ② **带 bridge 抗饱和的对象**（§4.5）。`gazebo_bridge` 把加速度积到
//       速度设定值上，并且**不许设定值超前实测速度 1 m/s 以上**。
//       这一段是非线性的，L1 不建模的话，Gazebo 上会看到一个 L1 完全没预测到的
//       起步迟滞，而人会去怀疑 PID 增益。
//
//  ⚠️ 抗饱和有两处，别混：一处在 **bridge**（设定值不许超前实测），
//     一处在**本控制器**（输出饱和时冻结积分）。前者防的是"顶着墙一路积到限速"，
//     后者防的是"松开限幅后车猛窜"。两者都要有，谁也替不了谁。
// =============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "ads_control/speed_controller.hpp"

namespace
{

using ads_control::SpeedController;
using ads_control::SpeedControllerParams;

// config/vehicle_params.yaml 的手抄副本（同 test_stanley.cpp 的说明）。
constexpr double kCruiseSpeedMps = 5.556;
constexpr double kMaxSpeedMps = 8.333;
constexpr double kMaxAccelMps2 = 1.5;
constexpr double kMaxDecelMps2 = 3.0;
/// gazebo_bridge 的 `setpoint_lead_mps_`：速度设定值最多超前**实测**速度这么多。
constexpr double kSetpointLeadMps = 1.0;

// control_params.yaml（S4 新建）。
constexpr double kProportionalGainInvS = 1.0;
constexpr double kIntegralGainInvS2 = 0.0;  // §4.4：初值就是 0，而且它是对的
constexpr double kControlDtS = 0.02;        // 50 Hz

SpeedControllerParams DefaultParams()
{
  return SpeedControllerParams{
    kProportionalGainInvS, kIntegralGainInvS2, kMaxAccelMps2, kMaxDecelMps2};
}

/// 理想积分器对象：`v̇ = a`。这是 §4.4 分析的那个对象。
struct IdealIntegratorPlant
{
  double speed_mps{0.0};
  void step(double accel_mps2, double dt_s) { speed_mps += accel_mps2 * dt_s; }
};

/// `gazebo_bridge` 的速度设定值链路（`vehicle_cmd_bridge_node.cpp` 的 on_timer）。
///
/// 逐行对着源码写的，包括那两处限幅的**顺序**和**边界**：
///   * 加速度先按 **emergency_decel(−5.0)** 截断，不是 max_decel —— bridge 模拟的是
///     "这辆车能做到什么"，不是"谁被允许这么做"。
///   * 设定值下限固定为 0（VehicleCmd 没有挡位字段，负加速度只能理解为减速）。
struct BridgeSpeedPlant
{
  double setpoint_mps{0.0};
  double measured_mps{0.0};
  /// false = 车轮打滑 / 顶着墙 / 上陡坡：实测速度跟不上设定值。
  /// 这正是 bridge 那段抗饱和当初要防的工况。
  bool vehicle_follows_setpoint{true};
  /// 上一拍"超前上限"有没有真的卡住设定值。
  bool lead_cap_was_binding{false};

  void step(double accel_cmd_mps2, double dt_s)
  {
    const double accel_mps2 = std::clamp(accel_cmd_mps2, -5.0, kMaxAccelMps2);
    setpoint_mps += accel_mps2 * dt_s;

    const double upper_mps = std::min(kMaxSpeedMps, measured_mps + kSetpointLeadMps);
    lead_cap_was_binding = setpoint_mps > upper_mps;
    setpoint_mps = std::clamp(setpoint_mps, 0.0, upper_mps);

    if (vehicle_follows_setpoint) {
      measured_mps = setpoint_mps;
    }
  }
};

/// **反例用**：没有抗饱和的朴素 PI。只在 WithoutAntiWindup… 那条用例里出现。
struct NaivePi
{
  double integral_ms{0.0};
  double update(double target_mps, double measured_mps, double dt_s)
  {
    const double error_mps = target_mps - measured_mps;
    integral_ms += error_mps * dt_s;  // 无条件积分 —— 问题就在这一行
    return std::clamp(
      kProportionalGainInvS * error_mps + 0.5 * integral_ms, -kMaxDecelMps2, kMaxAccelMps2);
  }
};

}  // namespace

// =============================================================================
//  参数与入参校验
// =============================================================================

TEST(SpeedControllerParamsCheck, RejectsNonPositiveGainsAndLimits)
{
  const std::vector<double> bad_values{0.0, -1.0, std::nan(""), HUGE_VAL};
  for (const double bad : bad_values) {
    SpeedControllerParams p = DefaultParams();
    p.proportional_gain_inv_s = bad;
    EXPECT_THROW(SpeedController{p}, std::invalid_argument) << "proportional_gain_inv_s = " << bad;

    p = DefaultParams();
    p.max_accel_mps2 = bad;
    EXPECT_THROW(SpeedController{p}, std::invalid_argument) << "max_accel_mps2 = " << bad;

    p = DefaultParams();
    p.max_decel_mps2 = bad;
    EXPECT_THROW(SpeedController{p}, std::invalid_argument) << "max_decel_mps2 = " << bad;
  }
}

TEST(SpeedControllerParamsCheck, AcceptsZeroIntegralGainButRejectsNegativeOrNonFinite)
{
  // K_i = 0 是**推荐初值**（§4.4）。把它也拦掉的话，文档里那条建议就成了
  // 一个跑不起来的建议 —— 所以它单独用"非负"而不是"正"来校验。
  SpeedControllerParams zero = DefaultParams();
  zero.integral_gain_inv_s2 = 0.0;
  EXPECT_NO_THROW(SpeedController{zero});

  for (const double bad : {-0.1, std::nan(""), HUGE_VAL}) {
    SpeedControllerParams p = DefaultParams();
    p.integral_gain_inv_s2 = bad;
    EXPECT_THROW(SpeedController{p}, std::invalid_argument) << "integral_gain_inv_s2 = " << bad;
  }
}

TEST(SpeedControllerInputs, NonFiniteInputThrowsAndLeavesTheIntegralUnpoisoned)
{
  // 积分是**持久状态**：一拍的 NaN 会让控制器永久输出 NaN，
  // 之后输入恢复正常也回不来。现场表现是「车自己停了」，而错在上游。
  SpeedControllerParams params = DefaultParams();
  params.integral_gain_inv_s2 = 0.5;
  SpeedController controller(params);
  controller.update(2.0, 0.0, 1.0, kControlDtS);
  const double before_ms = controller.integral_ms();

  EXPECT_THROW(controller.update(std::nan(""), 0.0, 1.0, kControlDtS), std::invalid_argument);
  EXPECT_THROW(controller.update(2.0, 0.0, HUGE_VAL, kControlDtS), std::invalid_argument);
  EXPECT_THROW(controller.update(2.0, 0.0, 1.0, std::nan("")), std::invalid_argument);
  EXPECT_THROW(controller.update(2.0, 0.0, 1.0, -kControlDtS), std::invalid_argument);

  EXPECT_TRUE(std::isfinite(controller.integral_ms()));
  EXPECT_NEAR(controller.integral_ms(), before_ms, 1e-15);
}

TEST(SpeedControllerInputs, ZeroTimeStepDoesNotAdvanceTheIntegral)
{
  SpeedControllerParams params = DefaultParams();
  params.integral_gain_inv_s2 = 0.5;
  SpeedController controller(params);
  controller.update(2.0, 0.0, 1.0, kControlDtS);
  const double before_ms = controller.integral_ms();
  controller.update(2.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(controller.integral_ms(), before_ms, 1e-15);
}

TEST(SpeedControllerInputs, ResetClearsTheIntegral)
{
  SpeedControllerParams params = DefaultParams();
  params.integral_gain_inv_s2 = 0.5;
  SpeedController controller(params);
  for (int i = 0; i < 50; ++i) {
    controller.update(2.0, 0.0, 1.0, kControlDtS);
  }
  ASSERT_GT(controller.integral_ms(), 0.0);
  controller.reset();
  EXPECT_EQ(controller.integral_ms(), 0.0);
}

// =============================================================================
//  ① 理想积分器对象：纯 P 就够（任务 3.4）
// =============================================================================

TEST(IdealPlant, PureProportionalHasNoSteadyStateError)
{
  // §4.4 的核心结论，而且它**与直觉相反**：「P 控制器总有稳态误差」说的是
  // 比例型对象（加热器那种）。这里的对象自带一个积分（a = v̇），
  // 误差趋零时 a 趋零而 v 保持住 —— 所以 K_i 在这里几乎无事可做。
  SpeedController controller(DefaultParams());  // K_i = 0
  IdealIntegratorPlant plant;

  for (int i = 0; i < 1000; ++i) {  // 20 s
    plant.step(controller.update(kCruiseSpeedMps, 0.0, plant.speed_mps, kControlDtS), kControlDtS);
  }
  const double residual_mps = std::abs(kCruiseSpeedMps - plant.speed_mps);
  std::cout << "  [纵向] 纯 P（K_i=0）20 s 后稳态误差 " << residual_mps * 1e3 << " mm/s\n";
  EXPECT_LT(residual_mps, 1e-6);
}

TEST(IdealPlant, TheClosedLoopTimeConstantIsOneOverKp)
{
  // 闭环 v̇ = K_p·(v_ref − v)，一阶滞后，时间常数 1/K_p。
  // K_p = 1.0 → 1 s 之后误差应剩 e⁻¹ = 36.8%。
  //
  // 阶跃取 1.0 m/s 而不是巡航速度：5.556 的误差会让 P 项要求 5.556 m/s²，
  // 一上来就撞加速度限幅，**测到的就不是时间常数而是限幅了**。
  constexpr double kStepMps = 1.0;
  SpeedController controller(DefaultParams());
  IdealIntegratorPlant plant;

  const int steps_for_one_tau =
    static_cast<int>(std::lround(1.0 / kProportionalGainInvS / kControlDtS));
  for (int i = 0; i < steps_for_one_tau; ++i) {
    const double accel_mps2 = controller.update(kStepMps, 0.0, plant.speed_mps, kControlDtS);
    ASSERT_LE(std::abs(accel_mps2), kMaxAccelMps2) << "阶跃选大了，撞到限幅就测不到时间常数";
    plant.step(accel_mps2, kControlDtS);
  }
  const double remaining_ratio = (kStepMps - plant.speed_mps) / kStepMps;
  std::cout << "  [纵向] 一个时间常数后剩余误差 " << remaining_ratio * 100.0 << " %（理论 "
            << std::exp(-1.0) * 100.0 << " %）\n";
  // 前向欧拉的离散版是 (1 − K_p·dt)^(1/(K_p·dt)) = 0.98^50 = 0.3642，
  // 连续解是 0.3679，差 1%。判据放 3%。
  EXPECT_NEAR(remaining_ratio, std::exp(-1.0), 0.03);
}

TEST(IdealPlant, OutputIsClampedToTheComfortLimitsNotThePhysicalOnes)
{
  SpeedController controller(DefaultParams());

  // 一个巨大的正误差 → 顶到 +1.5，不会更高。
  EXPECT_NEAR(controller.update(100.0, 0.0, 0.0, kControlDtS), kMaxAccelMps2, 1e-12);

  // 一个巨大的负误差 → 顶到 **−3.0**，**不是** bridge 允许的 −5.0。
  // −5.0 是车辆的物理能力，只有安全模块可以下发；常规控制器自己不得越过 −3.0。
  // 写反的话不会有任何一层拦下来 —— bridge 按 −5.0 截断，正好放行。
  controller.reset();
  EXPECT_NEAR(controller.update(0.0, 0.0, 100.0, kControlDtS), -kMaxDecelMps2, 1e-12);
}

// =============================================================================
//  ① 之二：加速度前馈 —— 斜坡目标（S4 补）
// =============================================================================

TEST(IdealPlant, PureProportionalCannotTrackARampAndFeedforwardFixesIt)
{
  // §4.4 那条「纯 P 没有稳态误差」只对**常值**目标成立。速度剖面不是常值：
  // 入弯前和终点前都是按 √(2a·Δs) 下降的，换算成时间就是一条斜率 −a_dec 的斜坡。
  //
  // 一阶系统跟踪斜坡的稳态误差 = 斜率 / K_p = 3.0 / 1.0 = **3.0 m/s**。
  // S4 闭环实测就是这么冲过终点 4.26 m 的，而同一个原因还把最大横向加速度
  // 顶到 2.113（入弯超速 0.85 m/s）。
  //
  // 这条用例把「有没有前馈」这一个因素单独摘出来：同一条斜坡跑两遍。
  // ⚠️ 斜率取 −1.0 而不是真实的 a_dec = −3.0，是为了让斜坡**跑得够久**：
  //    稳态误差要 3 个时间常数（3/K_p = 3 s）才建立起来，而 −3.0 的斜坡
  //    从巡航 5.556 降到 0 只有 1.85 s —— 那时误差才走到理论值的 78%
  //    （实测 2.29 / 3.0），拿它当"稳态"是测错了东西。
  //
  //    这件事本身值得记一笔：**真实终点的减速段短到误差根本来不及收敛**，
  //    而且 v_ref = √(2a·剩余) 在末端的**时间**斜率是发散的
  //    （dv_ref/dt = −a·v/v_ref，v_ref → 0），所以 S4 实测的末端误差
  //    4.36 m/s 比"稳态"的 3.0 还大 —— 不矛盾，是同一件事的更极端版本。
  constexpr double kRampSlopeMps2 = -1.0;
  constexpr double kStartSpeedMps = 8.0;

  auto run = [](bool use_feedforward) {
    SpeedController controller(DefaultParams());
    IdealIntegratorPlant plant;
    plant.speed_mps = kStartSpeedMps;
    double reference_mps = kStartSpeedMps;
    double worst_error_mps = 0.0;
    // 跑 5 s：斜坡从 8.0 降到 3.0，全程不触加速度限幅（|a| ≤ 3.0）。
    for (int i = 0; i < 250; ++i) {
      reference_mps += kRampSlopeMps2 * kControlDtS;
      const double accel_mps2 = controller.update(
        reference_mps, use_feedforward ? kRampSlopeMps2 : 0.0, plant.speed_mps, kControlDtS);
      plant.step(accel_mps2, kControlDtS);
      if (i > 200) {  // 前 4 s（4 个时间常数）是暂态，只看之后的稳态
        worst_error_mps = std::max(worst_error_mps, std::abs(reference_mps - plant.speed_mps));
      }
    }
    return worst_error_mps;
  };

  const double without_ff_mps = run(false);
  const double with_ff_mps = run(true);
  std::cout << "  [前馈] 斜坡稳态误差：无前馈 " << without_ff_mps << " m/s（理论 "
            << std::abs(kRampSlopeMps2) / kProportionalGainInvS << "），有前馈 " << with_ff_mps
            << " m/s\n";

  // ① 没有前馈时，误差必须逼近理论值 斜率/K_p —— 否则这条用例证明不了机理。
  EXPECT_NEAR(without_ff_mps, std::abs(kRampSlopeMps2) / kProportionalGainInvS, 0.1);
  // ② 有前馈时误差被整个消掉（只剩离散化残渣）。
  EXPECT_LT(with_ff_mps, 0.05);
}

// =============================================================================
//  ② 抗饱和（任务 3.4 的另一半）
// =============================================================================

TEST(IdealPlant, ACarStoppedInsideABrakingRampCanStillRestart)
{
  // P8-S2d 实测（junction，让行后重启）：车停在别人轨迹的制动带里
  // （离末端 0.8 m，剖面 v_ref = 2.19、前馈 = −3.0）。前馈若原样用
  // 剖面存的 v_ref·dv/ds，输出恒为 −3.0 + K_p·2.19 = **−0.81** ——
  // 目标速度 2.19 却永远起不了步，车在离 goal 1.15 m 处冻住，
  // TRACKING、指令流全正常，没有任何报错。
  //
  // 前馈的物理量是 a = v·dv/ds（**实际** v）。v = 0 时它就是 0 ——
  // 负前馈按 v/v_ref 衰减后，P 项得以把车启动。
  //
  // 故障注入实测（2026-08-13）：去掉衰减（前馈原样透传）→ 本用例红
  // （输出 −0.81 < 0）。
  SpeedController controller(DefaultParams());
  const double accel_mps2 = controller.update(2.19, -3.0, 0.0, kControlDtS);
  EXPECT_GT(accel_mps2, 0.5) << "停着的车收到正目标速度必须给出正加速度";
}

TEST(IdealPlant, BrakingFeedforwardIsUntouchedWhenTrackingTheRamp)
{
  // 收窄条款的另一半：车跟得上剖面（v = v_ref）时衰减比值 = 1，
  // 前馈必须**原样**通过 —— 已验收的制动行为（CP-P2-B/P3-B 的停车剖面）
  // 一个数都不许变。
  SpeedController controller(DefaultParams());
  const double accel_mps2 = controller.update(2.19, -1.2, 2.19, kControlDtS);
  EXPECT_NEAR(accel_mps2, -1.2, 1e-12);  // 误差 0 ⟹ 纯前馈
}

TEST(AntiWindup, TheIntegralStopsGrowingOnceTheOutputSaturates)
{
  // 增益特意选成"先线性、后饱和"：K_p=0.2、K_i=0.5、目标 1.0 m/s，车不动。
  //   起初 raw = 0.2×1.0 = 0.2，没饱和 → 正常积分；
  //   积分涨到 I = (1.5−0.2)/0.5 = 2.6 m 时 raw 触到 +1.5 → 此后冻结。
  // 直接用 K_p=1、目标 5.556 的话一上来就饱和，积分从头到尾是 0，
  // **这条用例就什么都没测到**。
  SpeedControllerParams params = DefaultParams();
  params.proportional_gain_inv_s = 0.2;
  params.integral_gain_inv_s2 = 0.5;
  SpeedController controller(params);

  BridgeSpeedPlant plant;
  plant.vehicle_follows_setpoint = false;  // 顶着墙：实测速度恒为 0

  for (int i = 0; i < 500; ++i) {  // 10 s
    plant.step(controller.update(1.0, 0.0, plant.measured_mps, kControlDtS), kControlDtS);
  }
  const double naive_integral_ms = 1.0 * 10.0;  // 无条件积分会累到 ∫1.0 dt = 10 m
  const double expected_freeze_ms = (kMaxAccelMps2 - 0.2 * 1.0) / 0.5;
  std::cout << "  [抗饱和] 10 s 顶着墙后积分 " << controller.integral_ms() << " m（冻结点 "
            << expected_freeze_ms << "，无抗饱和会到 " << naive_integral_ms << "）\n";

  EXPECT_NEAR(controller.integral_ms(), expected_freeze_ms, 0.05);
  EXPECT_LT(controller.integral_ms(), 0.5 * naive_integral_ms);
}

TEST(AntiWindup, TheIntegralResumesOnceTheOutputLeavesSaturation)
{
  // 冻结不能是**永久**的：误差反向之后积分必须能重新动起来，否则控制器
  // 会带着一坨陈旧的积分继续跑。
  //
  // ⚠️ **这条用例区分不了"条件积分"和"饱和就完全冻结"** —— 故障注入实测：
  //    改成完全冻结，28 个用例一条都不红。原因见 speed_controller.cpp 里的证明：
  //    条件积分保证 K_i·I ≤ a_max，于是"误差反向却仍处于饱和"这个状态
  //    **从干净初值出发根本到不了**，两种写法在可达状态上完全等价。
  //    记在这里是为了**不让下一个人以为这个区别有测试在保护**。
  SpeedControllerParams params = DefaultParams();
  params.proportional_gain_inv_s = 0.2;
  params.integral_gain_inv_s2 = 0.5;
  SpeedController controller(params);

  for (int i = 0; i < 500; ++i) {  // 先积到饱和并冻结
    controller.update(1.0, 0.0, 0.0, kControlDtS);
  }
  const double frozen_ms = controller.integral_ms();
  ASSERT_GT(frozen_ms, 2.0);

  // 目标突然低于实测（比如进了弯道，剖面要求减速）→ 误差变号。
  controller.update(0.0, 0.0, 1.0, kControlDtS);
  EXPECT_LT(controller.integral_ms(), frozen_ms) << "反向误差被一起冻住了，退饱和会滞后";
}

TEST(AntiWindup, WithoutAntiWindupTheReleaseOvershootIsMuchWorse)
{
  // **反例**：证明上面那两条用例有鉴别力。
  // 场景是真实的：车顶着阻力（或起步瞬间轮胎打滑）10 s，然后阻力消失。
  // 没有抗饱和的话，积分在这 10 s 里一路涨到 10 m，
  // 松开之后控制器要先把这坨积分"卸掉"才肯松油门 —— 症状是**车猛窜**，
  // 而人的第一反应是"K_p 调大了"。
  constexpr double kTargetMps = 1.0;

  auto run = [](bool use_anti_windup) {
    SpeedControllerParams params = DefaultParams();
    params.proportional_gain_inv_s = kProportionalGainInvS;
    params.integral_gain_inv_s2 = 0.5;
    SpeedController controller(params);
    NaivePi naive;
    BridgeSpeedPlant plant;
    plant.vehicle_follows_setpoint = false;

    double peak_mps = 0.0;
    for (int i = 0; i < 1500; ++i) {  // 30 s
      if (i == 500) {                 // 10 s 后阻力消失
        plant.vehicle_follows_setpoint = true;
      }
      const double accel_mps2 =
        use_anti_windup ? controller.update(kTargetMps, 0.0, plant.measured_mps, kControlDtS)
                        : naive.update(kTargetMps, plant.measured_mps, kControlDtS);
      plant.step(accel_mps2, kControlDtS);
      if (i > 500) {
        peak_mps = std::max(peak_mps, plant.measured_mps);
      }
    }
    return peak_mps;
  };

  const double guarded_peak_mps = run(true);
  const double naive_peak_mps = run(false);
  std::cout << "  [抗饱和反例] 松开后峰值速度：带抗饱和 " << guarded_peak_mps << " m/s，朴素积分 "
            << naive_peak_mps << " m/s（目标 " << kTargetMps << "）\n";

  EXPECT_GT(naive_peak_mps, 2.0 * kTargetMps) << "反例本身没冲出去，这条用例证明不了什么";
  EXPECT_LT(guarded_peak_mps, 1.5 * kTargetMps);
}

// =============================================================================
//  ③ bridge 的设定值超前上限（§4.5）
// =============================================================================

TEST(BridgePlant, TheSetpointLeadCapOnlyBindsWhenTheVehicleCannotFollow)
{
  // §4.5 说「以 +1.5 m/s² 加速时，积出 1.0 m/s 的超前量需要 0.667 s，
  // 常规巡航不触发，起步瞬间会」。**这条用例把"会"的条件说清楚**：
  // 那 0.667 s 的算法隐含假设**实测速度一直是 0**，也就是车根本没动。
  // 车正常跟随时设定值与实测同步推进，超前量恒为一拍的 a·dt = 0.03 m/s，
  // **上限一次都碰不到**。
  {
    SpeedController controller(DefaultParams());
    BridgeSpeedPlant plant;  // 正常跟随
    bool ever_bound = false;
    for (int i = 0; i < 500; ++i) {  // 10 s，从 0 加到巡航
      plant.step(
        controller.update(kCruiseSpeedMps, 0.0, plant.measured_mps, kControlDtS), kControlDtS);
      ever_bound = ever_bound || plant.lead_cap_was_binding;
    }
    EXPECT_FALSE(ever_bound) << "车能跟上时不该碰到超前上限";
    EXPECT_NEAR(plant.measured_mps, kCruiseSpeedMps, 1e-3);
  }

  // 车完全动不了时，设定值以 +1.5 积到 1.0 就被卡住 —— 恰好 0.667 s。
  {
    SpeedController controller(DefaultParams());
    BridgeSpeedPlant plant;
    plant.vehicle_follows_setpoint = false;
    double bound_at_s = -1.0;
    for (int i = 0; i < 500; ++i) {
      plant.step(
        controller.update(kCruiseSpeedMps, 0.0, plant.measured_mps, kControlDtS), kControlDtS);
      if (plant.lead_cap_was_binding && bound_at_s < 0.0) {
        bound_at_s = (i + 1) * kControlDtS;
      }
    }
    std::cout << "  [bridge] 车动不了时，超前上限在 " << bound_at_s << " s 卡住（闭式解 "
              << kSetpointLeadMps / kMaxAccelMps2 << " s）\n";
    EXPECT_NEAR(bound_at_s, kSetpointLeadMps / kMaxAccelMps2, 2.0 * kControlDtS);
    // 而且设定值确实被摁在 1.0 —— 这就是 bridge 那段抗饱和的全部作用。
    EXPECT_NEAR(plant.setpoint_mps, kSetpointLeadMps, 1e-9);
  }
}
