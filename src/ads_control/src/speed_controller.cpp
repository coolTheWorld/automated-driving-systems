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

#include "ads_control/speed_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "numeric_checks.hpp"

namespace ads_control
{

namespace
{

using internal::RequireFinite;
using internal::RequireFiniteNonNegative;
using internal::RequireFinitePositive;

constexpr char kParams[] = "SpeedControllerParams";
constexpr char kController[] = "SpeedController";

}  // namespace

SpeedController::SpeedController(const SpeedControllerParams & params) : params_(params)
{
  RequireFinitePositive(params_.proportional_gain_inv_s, kParams, "proportional_gain_inv_s");
  // K_i 用 NonNegative 而不是 Positive：0 是**推荐初值**（见头文件第 2 条），
  // 拦掉它等于把文档里的建议变成一个跑不起来的建议。
  RequireFiniteNonNegative(params_.integral_gain_inv_s2, kParams, "integral_gain_inv_s2");
  RequireFinitePositive(params_.max_accel_mps2, kParams, "max_accel_mps2");
  RequireFinitePositive(params_.max_decel_mps2, kParams, "max_decel_mps2");
}

double SpeedController::update(double target_speed_mps, double measured_speed_mps, double dt_s)
{
  RequireFinite(target_speed_mps, kController, "target_speed_mps");
  RequireFinite(measured_speed_mps, kController, "measured_speed_mps");
  RequireFinite(dt_s, kController, "dt_s");
  if (dt_s < 0.0) {
    throw std::invalid_argument(
      "SpeedController: dt_s 为负（" + std::to_string(dt_s) +
      " s），时钟倒流了。当成正常值会让积分反向累积。");
  }

  const double error_mps = target_speed_mps - measured_speed_mps;

  // ---------------------------------------------------------------------------
  //  条件积分抗饱和
  // ---------------------------------------------------------------------------
  // 先按"假如积分了"算一遍，看输出会不会饱和；只有**不饱和**、
  // 或者**误差正在把输出拉回线性区**时，才真的把这一步积分记下来。
  //
  // 为什么不能无脑积分：加速度限幅期间误差一直存在，积分会一路涨。
  // 等误差终于变号时，控制器要先把那一大坨积分"卸掉"才会松油门 ——
  // 症状是**松开限幅后车猛窜**，而人的第一反应是"K_p 调大了"。
  //
  // ⚠️ 这里**曾经**写着"不能简单地'饱和就完全冻结积分'，那样反方向的误差也进不来、
  //    退饱和会滞后"。**从干净初值出发，这两种写法其实完全等价** ——
  //    故障注入实测：改成 `pushing_deeper = saturated_high || saturated_low`
  //    之后 28 个用例一条都不红。
  //
  //    证明（也解释了为什么它不是巧合）：条件积分保证 `K_i·I ≤ a_max` 恒成立。
  //    因为 I 最后一次增长时输出没饱和，即 `K_p·e + K_i·I ≤ a_max` 且那时 e > 0，
  //    所以 `K_i·I ≤ a_max − K_p·e ≤ a_max`。于是"误差为负却仍高位饱和"
  //    （需要 `K_i·I > a_max + K_p·|e|`）**永远到不了**。低位对称。
  //
  //    仍然保留条件积分，理由是它在上面那个前提被打破时才是对的：
  //    将来若加一个"把积分重置成某个非零值"的接口、或者运行期改限幅，
  //    完全冻结就会真的卡住。**但别以为现在有测试在保护这个区别 —— 没有。**
  const double candidate_integral_ms = integral_ms_ + error_mps * dt_s;
  const double raw_accel_mps2 = params_.proportional_gain_inv_s * error_mps +
                                params_.integral_gain_inv_s2 * candidate_integral_ms;

  accel_mps2_ = std::clamp(raw_accel_mps2, -params_.max_decel_mps2, params_.max_accel_mps2);

  const bool saturated_high = raw_accel_mps2 > accel_mps2_;
  const bool saturated_low = raw_accel_mps2 < accel_mps2_;
  const bool pushing_deeper =
    (saturated_high && error_mps > 0.0) || (saturated_low && error_mps < 0.0);
  if (!pushing_deeper) {
    integral_ms_ = candidate_integral_ms;
  }

  return accel_mps2_;
}

}  // namespace ads_control
