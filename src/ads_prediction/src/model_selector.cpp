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

#include "ads_prediction/model_selector.hpp"

#include <cmath>

#include "ads_common/numeric_checks.hpp"

namespace ads_prediction
{

ModelKind SelectModel(const TargetSnapshot & target, const SelectorParams & params)
{
  ads_common::RequireFinite(target.velocity.x(), "SelectModel", "velocity.x");
  ads_common::RequireFinite(target.velocity.y(), "SelectModel", "velocity.y");
  const double speed = target.velocity.norm();

  // 1. 太慢：速度方向由噪声主导，不编轨迹（新航迹初速 0 自动落此档）。
  if (speed < params.min_dynamic_speed_mps) {
    return ModelKind::kStatic;
  }
  // 2. 物理上不可能：坏状态不许外推（S0 墙沿假航迹的教训）。
  if (speed > params.odd_max_speed_mps) {
    return ModelKind::kStatic;
  }
  // 2b. 尺寸上同样物理不可能：园区 ODD 里没有 5.5 m 以上的运动目标。
  //     建筑片段的滑移锚点有「真实」净位移，位移一致性闸拦不住 ——
  //     P7-S4 实测（行为层上线当天）肇事者清一色 6.0 m 档，见 hpp 注释。
  if (target.length_m > params.odd_max_length_m) {
    return ModelKind::kStatic;
  }
  // 3. 位移一致性：声称的速度必须有净位移背书（S1 体检：结构物航迹
  //    |v|>0.5 占 24.5%，全是簇形交替的原地摆）。没有历史 = 不知道 = 不编。
  if (!target.net_displacement_1s_m.has_value()) {
    return ModelKind::kStatic;
  }
  const double required_m = speed * params.displacement_window_s * params.min_displacement_ratio;
  if (*target.net_displacement_1s_m < required_m) {
    return ModelKind::kStatic;
  }
  // 4. 车辆尺度 → 车道跟随（归属不成立时调用方退恒速）。
  if (target.length_m >= params.min_vehicle_length_m) {
    return ModelKind::kLaneFollow;
  }
  // 5. 行人尺度的动目标 → 恒速 + 椭圆。
  return ModelKind::kConstantVelocity;
}

}  // namespace ads_prediction
