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

#ifndef ADS_PREDICTION__MODEL_SELECTOR_HPP_
#define ADS_PREDICTION__MODEL_SELECTOR_HPP_

// =============================================================================
//  运动模型选择（P6-1 决策五 + S1 体检的位移一致性闸）
//
//  ⚠️ **不用 classification**。感知的分类实测 25%/48% 判对且无判据在守 ——
//     选择只用三样有实测精度背书的量：速度（p95 误差 0.18）、
//     尺寸（近边误差 p95 0.13）、车道归属（由 nearest_lane + 门限判）。
//     TargetSnapshot 里根本没有 classification 字段，结构性防呆。
//
//  规则（顺序即优先级，推导见 docs/modules/prediction.md §5）：
//    1. |v| < min_dynamic          → 静态（速度方向由噪声主导；新航迹初速 0 也落此档）
//    2. |v| > odd_max              → 静态（物理上不可能的状态不许外推 ——
//                                     S0 的墙沿假航迹 11.9/15.5 m/s 教训）
//    3. 位移证据不足               → 静态（S1 实测：结构物航迹 |v|>0.5 占 24.5%，
//                                     簇形交替的假速度**没有净位移背书**）
//    4. 车辆尺度（length ≥ 阈值）  → 车道跟随（归属不成立时调用方退恒速）
//    5. 其余（行人尺度的动目标）    → 恒速 + 不确定椭圆
// =============================================================================

#include "ads_prediction/types.hpp"

namespace ads_prediction
{

/// @brief 选择器参数。每个值的物理依据见 prediction.md §5。
struct SelectorParams
{
  /// 动目标的最低速度 [m/s]。0.5 与 tracker 的 heading_min_speed_mps
  /// **数值对齐**（同一个物理论证：低于它速度方向由噪声主导）。
  /// ⚠️ 两处是独立声明的参数 —— tracker 那边改了这里不会自动跟上；
  ///    对齐的是**论证**不是变量。调小 → 把速度噪声当运动方向去外推。
  double min_dynamic_speed_mps{0.5};

  /// ODD 物理速度上限 [m/s]。8.33 = 30 km/h（SPEC §2）。
  /// 超过它的速度状态必然是坏的（园区里没有东西跑这么快）—— 不许外推。
  double odd_max_speed_mps{8.33};

  /// 位移一致性：净位移 ≥ 声称速度 × 窗长 × 本比值 才认这个速度。
  /// 0.5：真实匀速目标的净位移比 ≈ 1.0（误差后 ≥0.8），簇形交替的
  /// 假速度净位移比 ≈ 0（位置原地摆）—— 两类隔一倍以上。
  /// 窗长由 node 层的历史决定（约 1 s），见 TargetSnapshot 的字段注释。
  double min_displacement_ratio{0.5};

  /// 位移证据的窗长 [s]，用于把 net_displacement_1s_m 换算成速度背书。
  /// 必须与 node 层维护历史的窗一致（node 从同一份参数读）。
  double displacement_window_s{1.0};

  /// 车辆尺度的最短长度 [m]。2.5：行人/杆件 ≤0.5，车 ≥4（记忆值），
  /// 中间隔着一整倍 —— 用尺寸不用分类（尺寸是直接测量）。
  double min_vehicle_length_m{2.5};
};

/// @brief 选运动模型。
/// @param target 输入目标。
/// @param params 参数。
/// @return kStatic / kConstantVelocity / kLaneFollow（后者归属不成立时
///         由调用方退成恒速 —— 本函数只回答"该试哪个"）。
ModelKind SelectModel(const TargetSnapshot & target, const SelectorParams & params);

}  // namespace ads_prediction

#endif  // ADS_PREDICTION__MODEL_SELECTOR_HPP_
