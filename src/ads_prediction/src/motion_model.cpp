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

#include "ads_prediction/motion_model.hpp"

#include <cmath>
#include <vector>

#include "ads_common/numeric_checks.hpp"

namespace ads_prediction
{

namespace
{

/// 采样时刻表：0, step, 2·step, …, horizon（含两端）。
/// ⚠️ 等分而不是「累加 + 夹取」—— 与 lane_sampling 的坑 ② 同族：
///    horizon/step 恰好整除时浮点 ceil 会多算一步、末尾出重合点。
std::vector<double> SampleTimes(const MotionModelParams & params)
{
  const int count = static_cast<int>(std::ceil(params.horizon_s / params.step_s - 1e-9));
  std::vector<double> times;
  times.reserve(static_cast<std::size_t>(count) + 1);
  for (int i = 0; i <= count; ++i) {
    times.push_back(params.horizon_s * static_cast<double>(i) / static_cast<double>(count));
  }
  return times;
}

void RequireFiniteInputs(const TargetSnapshot & target, const MotionModelParams & params)
{
  ads_common::RequireFinite(target.position.x(), "ads_prediction", "position.x");
  ads_common::RequireFinite(target.position.y(), "ads_prediction", "position.y");
  ads_common::RequireFinite(target.velocity.x(), "ads_prediction", "velocity.x");
  ads_common::RequireFinite(target.velocity.y(), "ads_prediction", "velocity.y");
  ads_common::RequireFinite(target.yaw_rad, "ads_prediction", "yaw_rad");
  ads_common::RequireFinitePositive(params.horizon_s, "MotionModelParams", "horizon_s");
  ads_common::RequireFinitePositive(params.step_s, "MotionModelParams", "step_s");
}

}  // namespace

PredictedPath PredictStatic(const TargetSnapshot & target, const MotionModelParams & params)
{
  RequireFiniteInputs(target, params);
  PredictedPath path;
  path.target_id = target.id;
  path.model = ModelKind::kStatic;
  for (const double t : SampleTimes(params)) {
    PredictedPoint point;
    point.t_s = t;
    point.position = target.position;
    point.heading_rad = target.yaw_rad;  // 只摆盒子；碰撞盒对 180° 对称，轴向足够
    point.speed_mps = 0.0;
    // 「随时可能起步、方向未知」：两个半轴同律增长。
    const double sigma = params.sigma_pos0_m + 0.5 * params.static_start_accel_mps2 * t * t;
    point.sigma_along_m = sigma;
    point.sigma_cross_m = sigma;
    path.points.push_back(point);
  }
  return path;
}

PredictedPath PredictConstantVelocity(
  const TargetSnapshot & target, const MotionModelParams & params, double lateral_accel_mps2)
{
  RequireFiniteInputs(target, params);
  ads_common::RequireFinite(lateral_accel_mps2, "PredictConstantVelocity", "lateral_accel_mps2");

  const double speed = target.velocity.norm();
  // ⚠️ 方向 = 速度方向，**不是 yaw**（头文件的那条规则）。
  //    速度为零时没有方向可言 —— 退回原地（选择器不该把零速目标送进来，
  //    但防御性地不给它编方向）。
  const double heading =
    speed > 1e-9 ? std::atan2(target.velocity.y(), target.velocity.x()) : target.yaw_rad;

  PredictedPath path;
  path.target_id = target.id;
  path.model = ModelKind::kConstantVelocity;
  for (const double t : SampleTimes(params)) {
    PredictedPoint point;
    point.t_s = t;
    point.position = target.position + target.velocity * t;
    point.heading_rad = heading;
    point.speed_mps = speed;
    point.sigma_along_m = params.sigma_pos0_m + params.sigma_speed_mps * t;
    point.sigma_cross_m = params.sigma_pos0_m + 0.5 * lateral_accel_mps2 * t * t;
    path.points.push_back(point);
  }
  return path;
}

}  // namespace ads_prediction
