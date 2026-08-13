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

#ifndef ADS_PLANNING__CONFLICT_HPP_
#define ADS_PLANNING__CONFLICT_HPP_

// =============================================================================
//  conflict —— 时空冲突计算（P7-S2，纯几何，零 ROS）
//
//  推导与参数依据见 docs/modules/behavior.md §1–§2。**改这里先读它。**
//
//  两类冲突**必须**分开（behavior.md §2 开头的理由，一句话版）：
//    前车停住后预测转 STATIC，椭圆走起步律（2σ(3s)=13.9 m）——
//    跟停逻辑吃椭圆的话，跟车间距被封死到 18 m 开外，跟车就不存在了。
//  所以：自车道类（FOLLOW）用**感知近边**、不看预测；
//        横穿类（YIELD）才用预测轨迹 + 2σ_cross 膨胀。
//
//  本文件不出「指令」，只出「事实」（冲突在哪、什么时候）。
//  事实 → 约束的换算在 longitudinal.hpp，行为标签在 behavior_tree 组装的树里。
// =============================================================================

#include <cstdint>
#include <optional>
#include <vector>

#include "ads_common/reference_line.hpp"

namespace ads_planning
{

/// @brief 行为层的全部参数。含义、依据与调大/调小的后果见 behavior.md §5。
///
/// 与 SpeedProfileParams 一样**故意不给默认值**：漏读配置时聚合初始化填 0，
/// 0 会被消费方指名报错，而不是悄悄用一个"看起来合理"的数开车。
struct BehaviorParams
{
  /// 横穿走廊半宽，m。取车道半宽 1.75。**只用于横穿类**（预测点落进
  /// 这个范围就算侵入我的车道）。
  ///
  /// ⚠️ FOLLOW **不用它**（S3 集成时实测改的）：P3 的贴边锥桶（近缘 −0.95）
  ///    落在 1.75 的走廊里，用走廊判前车会把「可绕」的东西当成跟停对象 ——
  ///    test_closed_loop_obstacle 当场变红。前车的判据是 blocking_half_m。
  double corridor_half_m;

  /// 自车道类（FOLLOW）的**阻挡**阈值，m。**推导量**（planning_node 从
  /// lattice 参数算，不单独配）：
  ///     B = 车半宽 + safety_margin − max_lateral_offset = 0.9 + 0.5 − 0.85 = 0.55
  /// 目标横向区间 [t_lo, t_hi] 满足 t_lo ≤ B 且 t_hi ≥ −B ⟺ 它与**每一个**
  /// 横向候选的外廓都间距不足 ⟺ 绕不过去 ⟺ 它是前车。
  /// 这与 planning.md §6 的可行性不等式是**同一个不等式**（推导见
  /// behavior.md §2.1）—— 规划器用它判「绕得过去吗」，行为层用它判
  /// 「这是不是前车」，两处共用一个几何真理。
  double blocking_half_m;
  /// 跟停时车头面到目标近边的站立间距，m。= 感知盲区 3 + 余量 1（判据 ② 下界）。
  double stand_off_m;
  /// 让行时车头面到冲突区入口的距离，m。冲突区不是实体，比 stand_off 小。
  double yield_margin_m;
  /// 时间窗余量 τ，s。吃掉预测发布延迟（新目标 ~1 s）与时间标注误差。
  double time_margin_s;
  /// 横穿膨胀带上限，m。inflated_half = corridor + min(2σ_cross, 本值)。
  ///
  /// P7-S4 实测逼出来的（6 s 视界的副作用）：CV 椭圆按 ½a·t² 长，6 s 尾端
  /// 2σ ≈ 18 m —— 行人**已经离开**走廊向南走，远期点照样「可能横回来」，
  /// 假让行拖着尾巴（⑨ 抖动源之一）。上限取一个车道宽 3.5：超过一个车道宽
  /// 的横向不确定不再构成让行理由 —— 连续运动的目标真要横穿时，
  /// 其**近期低 σ 预测点必然先进走廊**（连续性论证，不靠远期包络）。
  double sigma_inflation_cap_m;
  /// 后轴到车头面的距离，m。**推导量** length − rear_overhang = 3.55，
  /// 轨迹点语义是后轴（TrajectoryPoint.msg），停车点必须把车头长度让出来。
  double front_offset_m;
};

/// @brief 感知目标的快照（map 系 OBB 的行为层视图）。
///
/// ⚠️ 只带行为层要用的字段。速度、分类、朝向都**不在这里** ——
///    FOLLOW 判定刻意不看运动状态（STATIC 红线用例 ⑥ 的结构性满足），
///    横穿判定看的是**预测**不是快照。
struct TargetBox
{
  std::uint32_t id{0};
  double center_x_m{0.0};
  double center_y_m{0.0};
  /// 沿目标自身长轴的尺寸。近边近似 s_c − length/2（目标沿车道摆放时精确）。
  double length_m{0.0};
  /// 横向判定用近边：|d_c| − width/2 ≤ corridor_half。
  double width_m{0.0};
};

/// @brief 预测轨迹点（PredictedTrajectoryPoint 的 lib 侧镜像，零 ROS）。
struct PredictedPoint
{
  double t_s{0.0};
  double x_m{0.0};
  double y_m{0.0};
  /// 横向 1σ，m。膨胀量取 2σ（约 95% 包络）——只有横穿类消费它。
  double sigma_cross_m{0.0};
};

/// @brief 一条预测假设（一个目标可能有多条 —— 路口分叉的多假设）。
struct PredictionHypothesis
{
  std::uint32_t obstacle_id{0};
  std::vector<PredictedPoint> points;
};

/// @brief 自车道冲突：走廊里最近的那个目标。
struct FollowConflict
{
  std::uint32_t id{0};
  /// 目标近边在参考线上的弧长，m。
  double near_edge_s_m{0.0};
};

/// @brief 横穿冲突窗：某条假设与走廊的时空重叠包络。
struct CrossingConflict
{
  std::uint32_t id{0};
  double s_lo_m{0.0};
  double s_hi_m{0.0};
  double t_lo_s{0.0};
  double t_hi_s{0.0};
};

/// @brief ego 路径的时间标注：从「现在」到达各点的时刻，s。
///
/// 剖面分段恒加速度 ⟹ Δt = Δs / v_avg 是**恒等式不是近似**（behavior.md §1）。
/// v_avg < 0.05 m/s 的段取 +∞（到不了；停车点之后自然没有冲突）。
///
/// ⚠️ 喂**无约束**剖面：注入约束只会让 ego 更晚到，无约束时间是最早到达
///    时间，用它判冲突是保守方向（可能多让，不会少让）。
///
/// @param arc_lengths_m 逐点弧长，严格递增。
/// @param speeds_mps    同点位的剖面速度（无约束）。
/// @return 每点的到达时刻；首点为 0。
/// @throw std::invalid_argument 数组长度不同或少于 2 个点。
std::vector<double> annotate_times(
  const std::vector<double> & arc_lengths_m, const std::vector<double> & speeds_mps);

/// @brief 在时间标注上查任意弧长的到达时刻（段内闭式，不是线性插值）。
///
/// 段内 v(s) = √(v_i² + 2a·Δs)，t = t_i + Δs / ((v_i + v(s))/2) —— 精确。
/// s 超出末点按末点算；落在 +∞ 段返回 +∞。
double time_at(
  const std::vector<double> & arc_lengths_m, const std::vector<double> & speeds_mps,
  const std::vector<double> & times_s, double query_s_m);

/// @brief 自车道类冲突：**阻挡所有横向候选**、ego 前方最近的目标。
///
/// 判定只看**感知框**（不看预测、不看速度、不看分类）——
/// 「STATIC 前车必须触发跟停」由此结构性满足，不是一条特判。
/// 「阻挡」而不是「在走廊里」：可绕的贴边障碍物归 lattice 管（P3 的能力
/// 原样保留），绕不过去的才是前车（blocking_half_m 的推导见其注释）。
///
/// @param line     参考线（ego 的路径）。
/// @param ego_s_m  ego 后轴当前在参考线上的弧长。
/// @param targets  感知目标快照。
/// @param params   见 BehaviorParams。
/// @return 走廊内最近目标的近边；走廊里没有目标时为空。
std::optional<FollowConflict> find_follow_conflict(
  const ads_common::ReferenceLine & line, double ego_s_m, const std::vector<TargetBox> & targets,
  const BehaviorParams & params);

/// @brief 横穿类冲突：预测假设与走廊的时空重叠。
///
/// 对每条假设逐点投影，|d| ≤ corridor_half + 2σ_cross 的点聚成
/// [s_lo,s_hi]×[t_lo,t_hi] 包络；与 ego 时间窗重叠（含 τ 余量）才算冲突：
///     t_lo − τ ≤ t_ego(s_hi)  且  t_ego(s_lo) ≤ t_hi + τ
///
/// **多假设按任一冲突合成**（不概率加权、不过滤 —— P7 事实 11）。
///
/// @param line          参考线。
/// @param ego_s_m       ego 后轴当前弧长（只考虑前方的重叠段）。
/// @param arc_lengths_m / speeds_mps / times_s  §1 的无约束标注三件套。
/// @param hypotheses    预测假设（调用方负责剔除已按 FOLLOW 处理的目标 ——
///                      behavior.md §2.3：一个目标只进一类，先 FOLLOW）。
/// @param params        见 BehaviorParams。
/// @return 触发的冲突窗列表（未触发的重叠不返回）。
std::vector<CrossingConflict> find_crossing_conflicts(
  const ads_common::ReferenceLine & line, double ego_s_m, const std::vector<double> & arc_lengths_m,
  const std::vector<double> & speeds_mps, const std::vector<double> & times_s,
  const std::vector<PredictionHypothesis> & hypotheses, const BehaviorParams & params);

}  // namespace ads_planning

#endif  // ADS_PLANNING__CONFLICT_HPP_
