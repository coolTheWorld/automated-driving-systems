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

#ifndef ADS_PLANNING__LATTICE_HPP_
#define ADS_PLANNING__LATTICE_HPP_

// =============================================================================
//  横向 lattice：采样 → 碰撞筛选 → 代价排序 —— 纯 C++17，**不依赖 ROS**
//
//  推导、参数含义与调大调小的后果见 docs/modules/planning.md §4–§6。
//  这里只重复两条**做错了不会报错**的：
//
//  1. **安全间距是准入条件，不是代价项。** 不满足直接淘汰，不允许"够便宜就擦着过"。
//     放进代价里的话，`weight_clearance` 一调就能让车贴着障碍物过去，
//     而这是把安全逻辑放进一个可被权重绕过的分支 —— CLAUDE.md 明令禁止。
//
//  2. **全部候选都不可行时返回"不可行"，不返回"最不糟的那条"。**
//     代价函数已经能给每条候选打分了，"取分最高的"看起来天经地义 ——
//     但那意味着全撞的时候也输出一条撞的轨迹，而下游 ads_control 没有碰撞概念，
//     会老老实实跟着开过去。**不可行就要说不可行**，由上层接管去停车。
//
//  ⚠️ 「留不留得下」有一个可以事先算的不等式（planning.md §6）：
//     从左绕行可行 ⟺ 障碍物左缘 ≤ W/2 − w − g。本项目代入 = **−0.55 m**。
//     也就是说**随手把障碍物放在车道中间，在不压车道线的前提下几何上无解**。
//     这不是本层的 bug，是 ODD 的边界 —— 本层如实返回 kAllCandidatesBlocked。
// =============================================================================

#include <cstddef>
#include <optional>
#include <vector>

#include "ads_common/reference_line.hpp"
#include "ads_planning/collision.hpp"
#include "ads_planning/frenet.hpp"

namespace ads_planning
{

/// @brief 横向 lattice 的全部参数。
///
/// 全部来自 `config/planning_params.yaml` 与 `config/vehicle_params.yaml`，
/// **本层不含任何默认值**（SPEC §7「不允许硬编码魔数」）—— 给了默认值，
/// 上层忘记配置时就会静默用一组没人审过的数跑起来。
struct LatticeParams
{
  // ---- 采样网格 ----------------------------------------------------------
  /// 终点横向偏移 `|d_T|` 的上限，单位 m。
  ///
  /// 由调用方按「车道半宽 − 半车宽」算（本项目 1.75 − 0.90 = 0.85）。
  /// 调大 → 能绕更宽的障碍物，但会压车道线甚至越入对向（P3 的 ODD 不允许）；
  /// 调小 → 可绕行的障碍物变少，更多情形退化成停车。
  double max_lateral_offset_m{0.0};

  /// 候选之间的横向间隔，单位 m。
  ///
  /// 调大 → 候选变少、算得快，但**可能恰好跳过唯一可行的那条缝**
  ///        （§6 的可行区间可能只有 0.4 m 宽）；
  /// 调小 → 候选线性增多，单周期耗时跟着涨。
  double lateral_offset_step_m{0.0};

  /// 完成横向机动所用的纵向长度 `S`，采样区间 `[min, max]`，步长 `step`，单位 m。
  ///
  /// **为什么 `S` 也要采样，而不是固定一个值**：五次式要走到 `s = S` 才达到 `d_T`，
  /// 中途只走了一部分。固定 `S = 30` 时，`s = 10` 处只到 `0.21·d_T` ——
  /// **近处的障碍物根本绕不开**，而所有候选看起来都"在往那边走"。
  /// 让 `S` 一起采样，规划器才能在「缓而晚」和「急而早」之间选。
  ///
  /// `max` 调大 → 变道更缓更舒适，但候选整体更迟钝；
  /// `min` 调小 → 能应付更近的障碍物，但曲率变大 → 曲率限速把速度压低，
  ///              且转角速率可能顶到限幅（P2 实测的临界车速 `v* = R·rate`）；
  /// `step` 调大 → 候选变少算得快，但可能跳过唯一能及时绕开的那个 `S`。
  ///
  /// 下界的物理依据是制动距离：`S` 至少要大于「以当前速度发现障碍物后停下来的距离」。
  /// **本层不检查这一条** —— 它要知道车速，而车速属于纵向（S4），
  /// 混进横向层会让这一层没法脱离纵向单独测试。见 `plan_lateral` 的 note。
  double min_horizon_m{0.0};
  double max_horizon_m{0.0};
  double horizon_step_m{0.0};

  /// 输出轨迹的点距，单位 m。调大 → 下游插值误差大；调小 → 消息变大、耗时涨。
  double resample_step_m{0.0};

  // ---- 安全（准入条件，不是代价）------------------------------------------
  /// 车体外廓到障碍物外廓的最小允许间距，单位 m。
  ///
  /// **判据来自 SPEC §8 场景 S04（「侧向间距 > 0.5 m」），不在本模块发明。**
  /// 调大 → 更安全但可绕行的情形急剧变少（§6 的不等式里它是直接扣掉的一项）；
  /// 调小 → 传感器误差或定位漂移一来就变成实际碰撞。
  double safety_margin_m{0.0};

  // ---- 车体几何（取自 vehicle_params.yaml，不许另填）----------------------
  double vehicle_length_m{0.0};
  double vehicle_width_m{0.0};
  /// 后悬：车尾到后轴的距离，单位 m。
  ///
  /// @note **不要求调用方填「后轴到几何中心的距离」**，那是推导量
  ///       （= `length/2 − rear_overhang`，本项目 1.35 m），由本层自己算。
  ///       让人填推导量等于给一个填错就静默出事的机会：整个碰撞检查会
  ///       沿车头方向整体偏 1.35 m，而轨迹、代价、日志全都正常。
  double rear_overhang_m{0.0};

  // ---- 代价权重 ----------------------------------------------------------
  /// 贴中心线的倾向，乘 `d_T²`。
  /// 调大 → 绕行更晚更急；调小 → 无事也偏着走，白白占用对向余量。
  double weight_offset{0.0};

  /// 平滑度，乘 `∫κ²ds`。
  ///
  /// **这一项只有在 `S` 也被采样时才起作用** —— 若 `S` 固定，`∫κ²ds` 与 `d_T²`
  /// 都随 `|d_T|` 单调增，两者永远不会给出相反的排序，该项完全冗余。
  /// 有了 `S` 的采样，它才是「别为了早点绕开就猛打方向」这个偏好的载体。
  ///
  /// 调大 → 偏好缓而长的机动，但可能选一条"平滑地擦过障碍物"的（间距仍由准入条件兜底）；
  /// 调小 → 偏好急而短的机动，横向加速度可能顶到限值。
  double weight_curvature{0.0};

  /// 远离障碍物的倾向，乘 `1/最小间距`。无障碍物时最小间距为 +∞，该项自然为 0。
  /// 调大 → 离障碍物更远，但更容易顶到车道边界；
  /// 调小 → 贴着安全间距走，传感器误差一来就变成碰撞。
  double weight_clearance{0.0};

  /// 与上一周期的一致性，乘 `(d_T − 上周期 d_T)²`。
  /// 调大 → 抗抖动，但**过大会迟钝**：障碍物出现后半天不动；
  /// 调小 → 上下周期跳变，车在障碍物前左右摇摆。
  double weight_consistency{0.0};
};

/// @brief 一条候选轨迹。
struct Candidate
{
  /// 终点横向偏移 `d_T`，单位 m。
  double target_offset_m{0.0};

  /// 完成机动所用的纵向长度 `S`，单位 m。
  double maneuver_span_m{0.0};

  /// 重采样后的笛卡尔点（含解析曲率）。第一个点是自车当前投影处。
  ///
  /// @note **所有候选都在同样长度上采样**（= 可用前视距离），与各自的 `S` 无关：
  ///       `s > S` 之后保持 `d ≡ d_T`（导数为 0）。
  ///       否则短 `S` 的候选轨迹也短，碰撞检查覆盖的距离就少，
  ///       于是**它会显得更安全** —— 一个只看"有没有撞"的判据完全发现不了。
  ///
  /// @note **曲率由这里给出，下游不要重算。** 控制侧有 `ReferenceLine` 会算一遍，
  ///       两边各算就有两个真值；绕障轨迹上（五次多项式曲率 vs 三点差分）
  ///       差异比中心线上大得多。
  std::vector<CartesianState> points;

  /// 全程车体外廓到最近障碍物外廓的最小间距，单位 m。
  /// **无障碍物时是 `+∞`** —— 不是 0 也不是某个大数，因为"没有障碍物"和
  /// "障碍物很远"在代价上应当等价，而 `weight/∞ = 0` 天然给出这个语义。
  double min_clearance_m{0.0};

  /// 总代价，越小越好。
  double cost{0.0};
};

/// @brief `plan_lateral()` 的结果状态。
///
/// 用枚举而不是 `bool + 字符串`：调用方被迫把每种情形都处理掉，
/// 而「不可行」和「路走到头了」需要**不同的**下游动作
/// （前者停车让行，后者是任务完成）。
enum class LatticeStatus
{
  /// 找到了可行候选，`best` 有效。
  kOk,
  /// 采样到的候选**全部**被碰撞或安全间距淘汰。上层应转入停车（planning.md §6）。
  kAllCandidatesBlocked,
  /// 参考线剩余长度不足以采样。通常意味着到终点了。
  kHorizonTooShort,
};

/// @brief 一次横向规划的结果。
struct LatticeResult
{
  LatticeStatus status{LatticeStatus::kHorizonTooShort};
  /// 仅当 `status == kOk` 时有效。
  Candidate best;
  /// 本次一共生成了几条候选。
  std::size_t candidate_count{0};
  /// 其中因碰撞或间距不足被淘汰的有几条。**要报进 diagnostics** ——
  /// 「9 条候选全被淘汰」和「一条候选都没生成」是完全不同的故障。
  std::size_t blocked_count{0};
};

/// @brief 把一个**后轴中心**位姿换成车体外廓矩形（几何中心）。
///
/// 两者相差 `length/2 − rear_overhang`，本项目 = 2.2 − 0.85 = **1.35 m**。
///
/// @note 公开它是因为停车轨迹也要用同一套换算。**这个推导绝不能有两份** ——
///       漏掉或写错的症状是碰撞检查整体沿车头方向偏 1.35 m，
///       而轨迹、代价、日志全部正常：车会从障碍物"侧面擦过去"却报告安全。
Rectangle vehicle_body_at(const CartesianState & rear_axle_pose, const LatticeParams & params);

/// @brief 按给定的目标横向偏移生成一条候选的几何，**不做任何碰撞检查**。
///
/// @param line             参考线。
/// @param start            自车当前 Frenet 状态。
/// @param target_offset_m  终点横向偏移 `d_T`。
/// @param maneuver_span_m  完成机动的纵向长度 `S`，必须为正。
/// @param evaluation_span_m 输出轨迹的总长度，必须 ≥ 一个采样步长。
/// @param resample_step_m  输出点距。
/// @return 后轴中心的期望位姿序列（含解析曲率）。
/// @throw std::domain_error 途中出现尖点（`σ = 1 − dκ ≤ 0`）。
/// @throw std::out_of_range `evaluation_span_m` 超出参考线剩余长度。
///
/// @note 公开它是给**停车轨迹**用的：那时所有候选都已被淘汰，
///       但车仍然需要一条几何来沿着减速。若把这段逻辑在 trajectory.cpp 里
///       复制一份，两处对「`s > S` 之后保持 `d ≡ d_T`」这类约定的理解迟早分家 ——
///       而症状只在"绕不过去"的路径上出现，日常跑车完全正常。
std::vector<CartesianState> build_lateral_geometry(
  const ads_common::ReferenceLine & line, const FrenetState & start, double target_offset_m,
  double maneuver_span_m, double evaluation_span_m, double resample_step_m);

/// @brief 采样一组横向候选，筛掉不安全的，按代价选最优的一条。
///
/// @param line       参考线（`/route/path`）。
/// @param start      自车当前的 Frenet 状态，来自 `to_frenet()`。
/// @param obstacles  障碍物矩形（`map` 系，几何中心）。
/// @param params     见 `LatticeParams`。
/// @param previous_target_offset_m 上一周期选中的 `d_T`；`std::nullopt` 表示首次规划
///                   （此时一致性项恒为 0，而不是拿 0 当"上一次"—— 那会在车本来
///                   就偏着的情况下凭空引入一个回中心线的偏好）。
/// @return 见 `LatticeResult`。
/// @throw std::invalid_argument 参数或障碍物含非有限值、非正尺寸等。
///
/// @note **本层不检查「S 够不够刹停」。** 那要知道车速，而车速属于纵向（S4）。
///       这里只拒绝几何上不可能的（剩余参考线短于一个采样步长）。
///       把速度相关的判断混进横向层，会让这一层没法脱离纵向单独测试。
///
/// @note 复杂度 `O(横向候选 × 纵向候选 × 采样点数 × 障碍物数)`。本项目
///       9 × 3 × 61 × ~3 ≈ 5000 次矩形距离计算，仍在 50 ms 预算内。
///       **若要提速，先测再动** ——
///       P0a-S3 的教训：据"频率低 = 算力不够"砍掉一半激光雷达分辨率，
///       结果只快了 4%，因为病根是 QoS 不是 GPU。
LatticeResult plan_lateral(
  const ads_common::ReferenceLine & line, const FrenetState & start,
  const std::vector<Rectangle> & obstacles, const LatticeParams & params,
  std::optional<double> previous_target_offset_m = std::nullopt);

}  // namespace ads_planning

#endif  // ADS_PLANNING__LATTICE_HPP_
