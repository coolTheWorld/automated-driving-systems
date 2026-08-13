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

#ifndef ADS_PREDICTION__MOTION_MODEL_HPP_
#define ADS_PREDICTION__MOTION_MODEL_HPP_

// =============================================================================
//  恒速外推 + 静态预测 + 不确定椭圆（推导见 docs/modules/prediction.md §2–§3）
//
//  ⚠️ 恒速外推的**方向永远取速度矢量**，不取 yaw —— 一条规则同时杀掉两个
//     风险源：L-Shape 轴向的 180° 二义（猜错 = 预测逆行）和分类错误。
//     故障注入实测这条被用例守着（方向改取 yaw → 红）。
// =============================================================================

#include "ads_prediction/types.hpp"

namespace ads_prediction
{

/// @brief 视界与不确定椭圆的参数。每个值的物理依据见 prediction.md §3。
struct MotionModelParams
{
  /// 预测视界 [s]。SPEC §10 P6 行的「未来 3 s」。
  double horizon_s{3.0};

  /// 输出步长 [s]。0.2 s × 15 步 = 3 s；调小输出点数线性涨，精度不变
  /// （模型是解析的，点只是采样）。
  double step_s{0.2};

  /// 起始位置不确定性（1σ）[m]。= 感知横向位置误差 p95（0.28–0.29，CP-P5-B
  /// 实测）除以 1.65（p95 → 1σ，高斯）≈ 0.17，取 0.2 留噪。
  double sigma_pos0_m{0.2};

  /// 速度误差的传播率 [m/s]。= 感知速度误差 p95（0.11–0.18，CP-P5-B 实测）
  /// 除以 1.65 ≈ 0.1，取 0.15 覆盖弯道段（实测弯道复测轮 0.4–0.9 的 p95
  /// 主要来自过渡帧，稳态在 0.2 内）。沿运动方向：σ_along(t) = σ0 + σv·t。
  double sigma_speed_mps{0.15};

  /// 未建模横向机动的加速度包络 [m/s²]，σ_cross(t) = σ0 + a·t²/2。
  /// 行人取 0.5：1.2 m/s 步速下 0.5 m/s² 持续 2.4 s 就是 90° 变向 ——
  /// 覆盖「随时可能拐弯」而不至于 3 s 就把整条路盖住（t=3 时 σ_cross=2.45）。
  /// 调大 → 椭圆没有信息量（CP-P6-B ⑤ 的半轴上限卡它）；
  /// 调小 → 变向行人出椭圆（覆盖率判据卡它）。两头都有判据，不是自由旋钮。
  double pedestrian_lateral_accel_mps2{0.5};

  /// 恒速外推的车辆横向包络 [m/s²]。0.8 ≈ 园区车弯道侧向加速度的一半
  /// （campus 弯道设计值 1.56，见 dynamic_actors.yaml）：无车道信息时
  /// 按「可能在打方向但不会满舵」的中间假设。
  double vehicle_lateral_accel_mps2{0.8};

  /// 车道跟随的横向包络 [m/s²]。0.3：车道约束吸收了大部分横向自由度，
  /// 剩下的是车道内的摆动（S1 实测 curve_car 对中心线 max 0.21 m）。
  double lane_follow_lateral_accel_mps2{0.3};

  /// 静态预测的起步包络 [m/s²]。1.5 = 园区车最大纵向加速度：静止目标
  /// 「随时可能起步、方向未知」，两个半轴同律增长 σ(t) = σ0 + a·t²/2。
  double static_start_accel_mps2{1.5};
};

/// @brief 静态预测：不编运动轨迹，原地 + 椭圆按起步包络增长。
/// @param target 输入目标（其 velocity 被**忽略** —— 判定它静止是选择器的事，
///        本函数只负责"静止目标该输出什么"）。
/// @param params 参数。
/// @return 单条轨迹，全部点在原地，heading 沿用输入 yaw（只摆盒子）。
PredictedPath PredictStatic(const TargetSnapshot & target, const MotionModelParams & params);

/// @brief 恒速外推：position + velocity·t，方向 = 速度方向（**不是 yaw**）。
/// @param target 输入目标，|velocity| 应当 > 0（选择器保证；为 0 时输出原地）。
/// @param params 参数。
/// @param lateral_accel_mps2 横向包络（行人/车辆按选择器给的档位传入）。
/// @return 单条轨迹。
PredictedPath PredictConstantVelocity(
  const TargetSnapshot & target, const MotionModelParams & params, double lateral_accel_mps2);

}  // namespace ads_prediction

#endif  // ADS_PREDICTION__MOTION_MODEL_HPP_
