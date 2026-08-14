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

#ifndef ADS_PLANNING__TRAJECTORY_HPP_
#define ADS_PLANNING__TRAJECTORY_HPP_

// =============================================================================
//  轨迹装配：几何（lattice）+ 速度（剖面）→ 一条能直接发给控制的轨迹
//  纯 C++17，**不依赖 ROS**
//
//  本模块的**两个交付能力**在这里汇合（SPEC §2 第 3 条原文是
//  「安全减速**或**绕行」，两个都要）：
//
//    侧向留得下  ⟹  kOk       ：沿选中的候选走，速度由该候选自身的曲率决定
//    留不下      ⟹  kStopping ：沿当前横向位置直行，在障碍物前 stop_margin_m 停住
//
//  ⚠️ **停车剖面不另写一套减速逻辑。** 做法是把几何在停车点**截断**，
//     然后跑同一个 SpeedProfile —— 它本来就会在末点强制 v = 0 并向后扫描，
//     于是减速自动按 max_decel 展开。另写一份的话，两处对"怎么减速"的理解
//     迟早分家，而症状是**只在绕不过去的时候**才出现的刹车曲线不一致。
// =============================================================================

#include <cstddef>
#include <optional>
#include <vector>

#include "ads_planning/longitudinal.hpp"

#include "ads_common/reference_line.hpp"
#include "ads_planning/collision.hpp"
#include "ads_planning/frenet.hpp"
#include "ads_planning/lattice.hpp"
#include "ads_planning/speed_profile.hpp"

namespace ads_planning
{

/// @brief 轨迹上的一个点。字段与 `ads_msgs/TrajectoryPoint` 一一对应。
struct TrajectoryPoint
{
  /// 位置与朝向，`map` 系。**这是后轴中心（`base_link`）的期望位姿**，
  /// 不是车体几何中心 —— 与 `ads_map` 的路径、控制器的参考点保持一致。
  double x_m{0.0};
  double y_m{0.0};
  double heading_rad{0.0};

  /// 曲率，1/m，正 = 左转。
  ///
  /// @note **由生产方给出，下游不要重算。** 控制侧有 `ReferenceLine` 会算一遍，
  ///       两边各算就有两个真值；绕障段（五次多项式解析曲率 vs 三点差分）
  ///       差得比中心线上大得多。
  double curvature_inv_m{0.0};

  /// 沿**本条轨迹**的累计弧长，从 0 起，单位 m。不是参考线的 s。
  double s_m{0.0};

  /// 该点的目标速度，m/s。已含曲率限速、终点/障碍物停车、前后向扫描。
  double speed_mps{0.0};

  /// 该点起始的目标加速度，`a = ½·d(v²)/ds`，单位 m/s²。给速度环做**前馈**。
  ///
  /// @note **语义是「本段内恒定」**，不是"该点瞬时值"。因此下游在
  ///       段内插值时：速度取 `√(v₀² + ratio·(v₁²−v₀²))`（v² 线性），
  ///       加速度**直接取段起点的值**（分段恒定）。
  ///       这条是消息契约的一部分，写错了不会报错 —— 只会让前馈略偏。
  ///
  /// @note **不给它的后果是实测过的**：纯 P 速度环跟踪**斜坡**目标的稳态误差
  ///       = 斜率/K_p = 3.0/1.0 = 3.0 m/s，症状是"入弯偏快、终点冲过头"，
  ///       而稳态巡航时速度跟得很准（那时目标是常值）。见 control.md §4.4。
  double accel_mps2{0.0};
};

/// @brief 一次规划的结果状态。
enum class PlanStatus
{
  /// 找到了可行轨迹。`lateral_offset_m != 0` 表示正在绕行。
  kOk,
  /// **所有候选都被淘汰** ⟹ 沿当前横向位置直行并在障碍物前停住。
  /// 上层必须把它报进 diagnostics —— 否则"车停了"与"车挂了"分不开。
  kStopping,
  /// 参考线剩余长度不足以规划。通常意味着到终点了。
  kRouteExhausted,
};

/// @brief 一次规划的全部输出。
struct PlanResult
{
  PlanStatus status{PlanStatus::kRouteExhausted};
  std::vector<TrajectoryPoint> points;

  /// 选中的终点横向偏移，单位 m。**下一周期要把它当 `previous_target_offset_m` 传回来**
  /// （代价函数的一致性项），否则每周期都是"首次规划"，车会在候选之间跳。
  double lateral_offset_m{0.0};

  /// 诊断量：本次生成/淘汰了几条候选。
  /// 「27 条全被淘汰」和「一条都没生成」是完全不同的故障，必须分得开。
  std::size_t candidate_count{0};
  std::size_t blocked_count{0};
  /// 因运动学不可跟踪（峰值曲率超限）被淘汰的候选数（P8-S2d，见 lattice.hpp）。
  std::size_t curvature_blocked_count{0};

  /// `kStopping` 时：停车点距最近障碍物的纵向裕度，单位 m。用于验收判据。
  /// 其余状态下为 `+∞`。
  double stop_clearance_m{0.0};
};

/// @brief 规划参数的集合。
struct PlanParams
{
  LatticeParams lattice;
  SpeedProfileParams speed;

  /// 停车点到障碍物的额外纵向裕度，单位 m。
  ///
  /// 与 `lattice.safety_margin_m` 是**两回事**：后者是侧向擦过时的间距判据
  /// （SPEC §8 S04），前者是正面停住时车头留多远。
  /// 调大 → 停得更保守，但在窄路上更容易"还没到就停了"；
  /// 调小 → 停车点更靠前，定位误差或制动偏差一来就顶上去。
  double stop_margin_m{0.0};
};

/// @brief 规划一条带速度的轨迹。
///
/// @param line      参考线（`/route/path`）。
/// @param start     自车当前 Frenet 状态，来自 `to_frenet()`。
/// @param obstacles 障碍物矩形（`map` 系，几何中心）。
/// @param params    见 `PlanParams`。
/// @param previous_target_offset_m 上一周期的 `PlanResult::lateral_offset_m`。
/// @param constraint 行为层的纵向约束（P7-S3，可选）。
///        `stop_at_s_m` 是**参考线弧长系**的停车点（后轴），注入方式是把
///        选中候选的几何截断在那里 + terminal=0 —— 与静态障碍物的停车剖面
///        走**同一条代码路径**，不另写减速逻辑。停车点在静态停车点之后时
///        取更早者（min，与 merge 同一条保守原则）。
///        `caps` 映射到 `SpeedProfileParams::speed_caps_mps` 逐点取 min。
///        ⚠️ 候选自身弧长与参考线弧长差一个 `start.s_m` 平移；绕行候选的
///        横向过渡还会带来毫米级伸缩 —— 相对 stand_off（4 m）量级可忽略，
///        如实记下不假装精确。
/// @throw std::invalid_argument 参数或障碍物非法（由 `plan_lateral` 抛出）。
///
/// @note **不可行时不返回"最不糟的那条候选"**，而是转入 `kStopping`。
///       返回最优候选等于把碰撞检查放进一个可被绕过的分支：
///       下游 `ads_control` 没有碰撞概念，会老老实实跟着那条撞的轨迹开过去。
PlanResult plan(
  const ads_common::ReferenceLine & line, const FrenetState & start,
  const std::vector<Rectangle> & obstacles, const PlanParams & params,
  std::optional<double> previous_target_offset_m = std::nullopt,
  const LongitudinalConstraint * constraint = nullptr);

}  // namespace ads_planning

#endif  // ADS_PLANNING__TRAJECTORY_HPP_
