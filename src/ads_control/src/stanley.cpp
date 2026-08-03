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

#include "ads_control/stanley.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ads_control
{

namespace
{

/// 校验一个"必须是有限正数"的参数，失败时报出**名字**。
///
/// 不内联成 `if (!(x > 0)) throw` 的原因很实际：五个参数如果共用一句
/// 「参数非法」的错误信息，排查时还得回来读代码才知道是哪一项。
/// 启动期的一次字符串拼接换来的是"看日志就知道改哪一行 YAML"。
void RequireFinitePositive(double value, const char * name)
{
  // ⚠️ 必须先判 isfinite。NaN 参与任何比较都返回 false，所以 `value <= 0.0`
  //    对 NaN **恒为假** —— 只写那半句的话 NaN 会被原样放行。
  //    ±inf 也要拦：它能通过 `> 0` 但会让整条控制律失去意义。
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(
      std::string("StanleyParams::") + name + " 必须是有限正数，收到 " + std::to_string(value) +
      "。是不是配置里漏了这一项（聚合初始化会把漏掉的项填 0）？");
  }
}

/// 校验一个"必须有限"的运行期入参（正负都合法）。
void RequireFinite(double value, const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(
      std::string("StanleyController: ") + name + " 非有限（" + std::to_string(value) +
      "）。坏数据在上游，不要让它污染控制器的持久状态。");
  }
}

}  // namespace

Pose2D front_axle_pose(const Pose2D & rear_axle_pose, double wheelbase_m)
{
  if (!std::isfinite(wheelbase_m) || wheelbase_m <= 0.0) {
    throw std::invalid_argument(
      "front_axle_pose: 轴距必须是有限正数，收到 " + std::to_string(wheelbase_m) +
      " m。轴距为 0 会让前轴等于后轴，也就是静默退化成 Stanley 最经典的实现错误。");
  }
  RequireFinite(rear_axle_pose.x_m, "rear_axle_pose.x_m");
  RequireFinite(rear_axle_pose.y_m, "rear_axle_pose.y_m");
  RequireFinite(rear_axle_pose.heading_rad, "rear_axle_pose.heading_rad");

  // 刚体：前轴沿**车身**朝向前移一个轴距，朝向不变。
  // 注意这里用的是车身朝向 θ，不是"前轮指向" θ+δ —— 前轮能转，前轴不能。
  Pose2D front;
  front.x_m = rear_axle_pose.x_m + wheelbase_m * std::cos(rear_axle_pose.heading_rad);
  front.y_m = rear_axle_pose.y_m + wheelbase_m * std::sin(rear_axle_pose.heading_rad);
  front.heading_rad = rear_axle_pose.heading_rad;
  return front;
}

StanleyController::StanleyController(const StanleyParams & params) : params_(params)
{
  RequireFinitePositive(params_.gain_inv_s, "gain_inv_s");
  RequireFinitePositive(params_.soft_speed_mps, "soft_speed_mps");
  RequireFinitePositive(params_.max_steer_angle_rad, "max_steer_angle_rad");
  RequireFinitePositive(params_.max_steer_rate_rad_s, "max_steer_rate_rad_s");
}

double StanleyController::raw_steering_rad(
  double heading_error_rad, double lateral_error_m, double speed_mps, const StanleyParams & params)
{
  // 车速夹到非负：分母 `k_soft + v` 必须恒正。见头文件里关于倒车的说明 ——
  // 这是数值保护，不是倒车支持。
  const double speed_for_denominator_mps = std::max(speed_mps, 0.0);
  const double denominator_mps = params.soft_speed_mps + speed_for_denominator_mps;

  // 横向误差项。atan2 而不是 atan(x/y)：分母恒正时两者完全等价，
  // 但 atan2 不做除法，少一处"将来 k_soft 的校验被改坏就会炸"的地方。
  const double cross_track_term_rad =
    std::atan2(params.gain_inv_s * lateral_error_m, denominator_mps);

  // ⚠️ **减号**。教科书是加号，因为教科书的 e 右正；本项目 e 左正。
  //
  //    自洽性检查（改这一行时在脑子里跑一遍）：
  //      车在路径左侧、车头与路径平行 → e = +1, ψ = 0
  //        → δ = 0 − atan(正) < 0 → 向右打 ✓
  //      车在路径上、车头偏右 30°   → e = 0, ψ = +0.52
  //        → δ = +0.52 > 0 → 向左打 ✓
  return heading_error_rad - cross_track_term_rad;
}

double StanleyController::update(
  double heading_error_rad, double lateral_error_m, double speed_mps, double dt_s)
{
  RequireFinite(heading_error_rad, "heading_error_rad");
  RequireFinite(lateral_error_m, "lateral_error_m");
  RequireFinite(speed_mps, "speed_mps");
  RequireFinite(dt_s, "dt_s");
  if (dt_s < 0.0) {
    throw std::invalid_argument(
      "StanleyController: dt_s 为负（" + std::to_string(dt_s) +
      " s），时钟倒流了。本仓库见过一次：两套仿真并存时 TF 会刷 "
      "'Detected jump back in time'。当成正常值会让速率限幅方向反过来。");
  }

  const double raw_rad = raw_steering_rad(heading_error_rad, lateral_error_m, speed_mps, params_);

  // ---------------------------------------------------------------------------
  //  ① 转角限幅 → ② 转向速率限幅
  // ---------------------------------------------------------------------------
  // 先夹到机械极限得到一个**可达的**目标，再按速率往它走。
  //
  // ⚠️ 这里**曾经**写着"顺序不能反，反过来会浪费速率预算"。**那句话是错的** ——
  //    故障注入实测：把两步对调（先限速率、再夹角度），27 个用例**一条都不红**。
  //    原因是两个 clamp 都是单调的，复合之后恒等：
  //      raw > angle_max 时，两种写法都给 prev + min(step, angle_max − prev)。
  //    所以这只是**可读性**上的取舍，不是正确性要求。留下这个顺序是因为
  //    "算出可达目标 → 按速率走过去"读起来就是执行机构真实的样子。
  //
  // 这个顺序顺带给出一条一眼可见的不变量：结果落在 steering_rad_ 与 target 之间，
  // 两者都在 ±max_steer_angle_rad 内，所以结果也在。
  const double target_rad =
    std::clamp(raw_rad, -params_.max_steer_angle_rad, params_.max_steer_angle_rad);

  const double max_step_rad = params_.max_steer_rate_rad_s * dt_s;
  const double step_rad = std::clamp(target_rad - steering_rad_, -max_step_rad, max_step_rad);

  steering_rad_ += step_rad;
  return steering_rad_;
}

void StanleyController::reset(double steering_rad)
{
  RequireFinite(steering_rad, "reset(steering_rad)");
  // 夹到机械极限，维持"任何时候 |steering_rad_| ≤ max_steer_angle_rad"这条不变量。
  steering_rad_ =
    std::clamp(steering_rad, -params_.max_steer_angle_rad, params_.max_steer_angle_rad);
}

}  // namespace ads_control
