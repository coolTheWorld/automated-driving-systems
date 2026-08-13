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

#ifndef ADS_PREDICTION__TYPES_HPP_
#define ADS_PREDICTION__TYPES_HPP_

// =============================================================================
//  预测的输入/输出类型（纯 C++，零 ROS —— 消息翻译在 node 层）
//
//  ⚠️ TargetSnapshot **故意没有 classification 字段**（P6-1 决策五）：
//     感知的分类实测只有 25%/48% 判对，而且没有任何判据在守它 ——
//     按它选运动模型等于把已知的错误率直接乘进预测。模型选择只用
//     速度、尺寸、车道归属这三样**有实测精度背书**的量。
//     结构上拿不到，就不存在"哪天有人顺手用上"的可能。
// =============================================================================

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <vector>

namespace ads_prediction
{

/// @brief 预测输入：一帧里的一个目标（来自 /perception/obstacles 的翻译）。
struct TargetSnapshot
{
  std::uint32_t id{0};
  /// 包围盒中心，map 系 [m]。
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  /// 包围盒朝向 [rad]。⚠️ heading_resolved=false 时它只是**轴向**（[0,π)，
  /// 180° 二义）—— 预测的运动方向**永远不用它**，只用 velocity 的方向；
  /// 它唯一的用途是沿预测轨迹摆盒子（碰撞盒对 180° 对称，摆哪头都一样）。
  double yaw_rad{0.0};
  bool heading_resolved{false};
  /// 速度，map 系 [m/s]。
  Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
  double length_m{0.0};
  double width_m{0.0};
  double height_m{0.0};
  /// 过去约 1 s 内的**净位移** [m]，由 node 层从逐 id 位置历史算。
  /// 没有足够历史时为 nullopt（新目标）。
  ///
  /// ⚠️ 这是 S1 体检逼出来的（2026-08-12 实测）：结构物（墙沿/杆件）航迹
  ///    的 |v|>0.5 占 **24.5%**、p95=5.5 m/s —— 簇形交替让 KF 报出高速度而
  ///    位置原地摆。**声称的速度必须有净位移背书**，否则按静止处理。
  std::optional<double> net_displacement_1s_m{};
};

/// @brief 预测输出的一个点。
struct PredictedPoint
{
  /// 相对预测时刻的时间 [s]，从 0 起。
  double t_s{0.0};
  /// 位置，map 系 [m]。
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  /// 该点处的包围盒朝向 [rad]（运动方向；静态预测时沿用输入的 yaw）。
  double heading_rad{0.0};
  double speed_mps{0.0};
  /// 不确定椭圆半轴（1σ）：沿运动方向 / 垂直运动方向 [m]。增长律见
  /// docs/modules/prediction.md §3。
  double sigma_along_m{0.0};
  double sigma_cross_m{0.0};
};

/// @brief 选中的运动模型。
enum class ModelKind : std::uint8_t
{
  kStatic = 0,  ///< 不编运动轨迹：原地 + 椭圆增长
  kConstantVelocity = 1,
  kLaneFollow = 2,
};

/// @brief 一条预测轨迹（一个目标可有多条 = 路口多假设）。
struct PredictedPath
{
  std::uint32_t target_id{0};
  ModelKind model{ModelKind::kStatic};
  /// 本假设的概率，同一目标的各假设之和为 1。
  double probability{1.0};
  /// 按 t_s 递增；车道在视界内走到头且无后继时**如实截断**（点数变少）。
  std::vector<PredictedPoint> points{};
};

}  // namespace ads_prediction

#endif  // ADS_PREDICTION__TYPES_HPP_
