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

#ifndef ADS_CONTROL__SPEED_CONTROLLER_HPP_
#define ADS_CONTROL__SPEED_CONTROLLER_HPP_

// =============================================================================
//  纵向：速度环 —— 纯 C++17，**不依赖 ROS**
//
//      a = K_p·(v_ref − v) + K_i·∫(v_ref − v)dt      然后限幅到 [−a_dec, +a_acc]
//
//  完整推导见 docs/modules/control.md §4.4–§4.5。三条反直觉的：
//
//  1. **纯 P 对常值目标速度就没有稳态误差。** 因为被控对象本身是个积分器
//     （控制器输出加速度，而 a = v̇）。闭环 `v̇ = K_p·(v_ref − v)` 是一阶滞后，
//     时间常数 `1/K_p`。这和"P 控制器总有稳态误差"的直觉相反 ——
//     那条直觉说的是**比例型**对象（如加热器），不是积分型。
//
//  2. **所以 `K_i` 初值是 0，而且它是对的。** 积分项补偿的是**持续扰动**
//     （长坡、恒定阻力、执行器零偏），本项目地图是平的。给一个大 `K_i`
//     的后果是超调加震荡，而且是那种"看起来还需要更多调参"的震荡。
//     **只有实测出稳态误差才往上加，并且要说得出那个误差来自哪个物理量。**
//     「加点积分试试」是本领域最常见的一种无依据调参。
//
//  3. **即便 `K_i = 0` 也要把抗饱和写进去。** 因为它一旦被调成非零，
//     没有抗饱和的积分器会在限幅期间一路累积，症状是**松开限幅后车猛窜**，
//     而那时人只会觉得"增益调大了"。结构上留好比事后补便宜得多。
//
//  ⚠️ 本层**只做速度环**，没有加速度内环。SPEC §3.2 写的是双闭环 ——
//     这是 2026-08-01 拍板的取舍：当前链路上加速度内环没有可闭合的对象
//     （`gazebo_bridge` 把加速度积分成速度设定值，那一段几乎是理想积分器，
//     内环等于在补偿一个恒等式）。**不是"实现漏了"**。
//     等有了真车或 CARLA 的执行机构模型再补。SPEC §3.2 的脚注由 S5.3 补上。
// =============================================================================

namespace ads_control
{

/// @brief 速度环的全部参数。与本包其余参数结构一样**故意不给默认值**。
struct SpeedControllerParams
{
  /// 比例增益 `K_p`，量纲 **1/s**。闭环时间常数就是 `1/K_p`。
  ///
  /// 初值 1.0 → 63% 收敛用 1 s，95% 用 3 s。
  /// 调大（→ 2.0）→ 速度跟得紧，但更容易顶到 bridge 的抗饱和（见 §4.5）
  /// 和加速度限幅；调小（→ 0.5）→ 起步和终点减速拖沓。范围 0.5–2.0。
  double proportional_gain_inv_s;

  /// 积分增益 `K_i`，量纲 **1/s²**（乘的是速度误差的时间积分，量纲 m）。
  ///
  /// **初值 0.0，允许为零** —— 见文件头第 2 条。
  double integral_gain_inv_s2;

  /// 加速度上限，m/s²，**正数**。来自 `vehicle_params.yaml` 的 `limits.max_accel_mps2`（1.5）。
  double max_accel_mps2;

  /// 减速度上限，m/s²，**正数**（输出下限是它的相反数）。
  /// 来自 `vehicle_params.yaml` 的 `limits.max_decel_mps2`（3.0）。
  ///
  /// ⚠️ **不是 `emergency_decel_mps2`（5.0）。** 那是车辆物理能力，
  ///    只有安全模块可以下发。`gazebo_bridge` 按 5.0 截断是因为它模拟的是
  ///    "这辆车能做到什么"；**常规控制器自己不得越过 3.0**。
  double max_decel_mps2;
};

/// @brief 速度环 PI 控制器（`K_d` 有意不提供，见下）。
///
/// **有状态**：误差积分。换路径、重新接管、长时间停控之后必须 `reset()`。
///
/// @note **没有 `K_d`。** 对象已经是一阶，微分项唯一的作用是放大 `/odom`
///       的速度噪声。不提供这个参数，比提供一个"建议填 0"的参数更清楚 ——
///       后者迟早有人去调它。
class SpeedController
{
public:
  /// @throw std::invalid_argument `proportional_gain_inv_s` / `max_accel_mps2` /
  ///        `max_decel_mps2` 非有限或非正；`integral_gain_inv_s2` 非有限或为负。
  explicit SpeedController(const SpeedControllerParams & params);

  /// @brief 走一拍。
  ///
  /// @param target_speed_mps   期望速度（查 `SpeedProfile` 得到），m/s。
  /// @param measured_speed_mps 实测速度（来自 `/odom`），m/s。
  /// @param dt_s               距上一拍的时间间隔，s，**必须用节点时钟**（SPEC §3.3）。
  /// @return 加速度指令，m/s²，已限幅到 `[−max_decel, +max_accel]`。
  /// @throw std::invalid_argument 任一入参非有限；或 `dt_s < 0`。
  ///
  /// @note **抗饱和用的是条件积分**：输出饱和、且误差还在往饱和方向推时，
  ///       这一拍**不积分**。
  ///
  ///       ⚠️ 实测补充：从干净初值出发，它与"饱和就完全冻结积分"**完全等价**
  ///       （故障注入验证过）。条件积分保证 `K_i·I ≤ a_max`，于是
  ///       "误差反向却仍处于饱和"这个状态根本到不了。留条件积分是为了将来
  ///       （非零 reset、运行期改限幅）仍然正确，**不是**因为现在有区别。
  ///
  /// @note 非有限入参一律抛异常。积分是**持久状态**，一拍的 NaN 会让控制器
  ///       永久输出 NaN。现场表现是「车自己停了」，而错在上游 ——
  ///       见 CLAUDE.md 陷阱表「用比较去拦非有限值」。
  double update(double target_speed_mps, double measured_speed_mps, double dt_s);

  /// @brief 清空积分状态。
  void reset() noexcept { integral_ms_ = 0.0; }

  /// @brief 当前的误差积分，单位 **m**（速度 × 时间）。主要给测试和诊断看。
  double integral_ms() const noexcept { return integral_ms_; }

  /// @brief 上一拍的输出，m/s²。
  double accel_mps2() const noexcept { return accel_mps2_; }

  const SpeedControllerParams & params() const noexcept { return params_; }

private:
  SpeedControllerParams params_;
  double integral_ms_{0.0};
  double accel_mps2_{0.0};
};

}  // namespace ads_control

#endif  // ADS_CONTROL__SPEED_CONTROLLER_HPP_
