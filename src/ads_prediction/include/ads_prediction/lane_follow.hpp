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

#ifndef ADS_PREDICTION__LANE_FOLLOW_HPP_
#define ADS_PREDICTION__LANE_FOLLOW_HPP_

// =============================================================================
//  车道跟随预测（推导见 docs/modules/prediction.md §4）
//
//  骨架：nearest_lane 定归属 → 沿 successors 枚举车道链（路口分叉出多假设）
//  → sample_lane_centerline 采样成折线 → ReferenceLine 弧长参数化 →
//  按当前速率沿线外推，横向偏移线性衰减。
//
//  ⚠️ 三条边界，每条都有 L1 用例守着（CP-P6-A ③④⑦）：
//  ① 正编号车道逆 s 行驶 —— 翻转在 sample_lane_centerline 里做（一份逻辑），
//     这里只负责把 LaneNode 的 entry/exit 顺序原样传给它；
//  ② 车道走到头且无后继 ⟹ **如实截断**（点数变少），不外推也不抛异常；
//  ③ 归属不成立（横向太远 / 运动方向与车道切向不符）⟹ 返回**空**，
//     调用方退恒速 —— 对向车道里逆行的目标不假装能按车道预测。
// =============================================================================

#include <vector>

#include "ads_map/lane_graph.hpp"
#include "ads_prediction/motion_model.hpp"
#include "ads_prediction/types.hpp"

namespace ads_prediction
{

/// @brief 车道归属与链枚举的参数。依据见 prediction.md §4。
struct LaneFollowParams
{
  /// 归属的最大横向距离 [m]。2.0 < 半车道宽 1.75 + 感知横向误差 p95 0.29；
  /// 调大到 3.5 → 隔壁车道的目标也被归进来（预测直接错一条车道）。
  double match_max_lateral_m{2.0};

  /// 运动方向与车道切向的最大夹角 [rad]。30°：正常车道内行驶的朝向摆动
  /// ≪ 30°，而逆行是 180°、横穿是 90° —— 三类干净分开。
  /// 调大到 60° → 横穿车道的目标被按"沿车道"预测，方向错 90°。
  double match_max_heading_rad{0.5235987755982988};

  /// 中心线采样步长（参考线 s 上的）[m]。0.5：最急的车道弧 r = 12 − 1.75
  /// = 10.25 上，折线弦高 = step²/(8r) = 0.003 m —— 比 CP-P6-A ② 的
  /// 0.01 m 判据小一个量级（取 1.0 时弦高 0.012，判据直接被采样误差吃掉）。
  double sample_step_m{0.5};

  /// 初始横向偏移的衰减时间 [s]。车道保持的行为假设：目标在车道内的
  /// 偏移会在几秒内回到中心线。2.0 s 是 Stanley 类横向闭环的量级。
  /// 调成 0 → 预测瞬间跳上中心线（起点误差 = 当前偏移）；
  /// 调很大 → 恒定偏移平移整条预测（过弯时偏出车道）。
  double lateral_decay_s{2.0};

  /// 假设数上限。园区路口最多三岔，正常 ≤ 3；上限防病态图。
  int max_hypotheses{4};
};

/// @brief 车道跟随预测。
/// @param target 输入目标（速度应当 > 0，选择器保证）。
/// @param graph 车道图（决策二：静态先验，经 lib 读取）。
/// @param motion 视界/椭圆参数。
/// @param params 归属/链枚举参数。
/// @return 每个可达分支一条轨迹（概率均分）；**归属不成立时为空**，
///         调用方应退恒速外推。
std::vector<PredictedPath> PredictLaneFollow(
  const TargetSnapshot & target, const ads_map::LaneGraph & graph, const MotionModelParams & motion,
  const LaneFollowParams & params);

}  // namespace ads_prediction

#endif  // ADS_PREDICTION__LANE_FOLLOW_HPP_
