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

#ifndef ADS_PLANNING__FRENET_HPP_
#define ADS_PLANNING__FRENET_HPP_

// =============================================================================
//  Frenet ↔ 笛卡尔 双向变换 —— 纯 C++17，**不依赖 ROS**
//
//  完整推导见 docs/modules/planning.md §3。这里只重复三条
//  **做错了不会报错、只会给出一条看起来能开的轨迹**的：
//
//  1. `d′` 的符号是 **−σ·tan(heading_error)**，那个负号是推导出来的不是凑的。
//     `heading_error_rad` 的定义是 angle_diff(车头朝向, 路径切向) = θ_路径 − θ_车，
//     而 d′ 要的是**车相对参考线**的夹角 θ_车 − θ_路径，正好差一个负号。
//     写错的症状：车遇到障碍物往**错误的一侧**绕，被代价函数发现后又拉回来 ——
//     表现为在障碍物前左右摇摆，而每一层单看都"正常工作"。
//
//  2. 偏移曲线的曲率要**除以 σ = 1 − d·κ**，不是照抄参考线的 κ。
//     写错的症状：绕行段曲率偏小 → 曲率限速偏高 → 弯道横向加速度超标。
//     **直线段（κ=0）完全正确**，所以直线用例一条都不会红。
//
//  3. `σ ≤ 0` 必须**抛异常**。那是偏移曲线的尖点，(s, d) 不再一一对应，
//     再算下去每个数都是垃圾。本项目地图上 d·κ ≤ 0.106 永不触发 ——
//     但"不会发生"和"发生了会静默给错"是两回事，判它的代价是一行。
// =============================================================================

#include <cstddef>
#include <optional>

#include "ads_common/reference_line.hpp"

namespace ads_planning
{

/// @brief Frenet 坐标下的横向状态。
///
/// @note 全部导数是**对弧长 s 求的，不是对时间 t**。
///       这是「路径 lattice」而非「时空 lattice」的选择，对**静态**障碍物是正确的
///       （障碍物不动，"什么时候到"不影响"能不能过"），对动态障碍物**表达能力不够**
///       —— 那时正确解可能是"等它过去"（d 完全不变），路径 lattice 里没有这个候选。
///       见 docs/modules/planning.md §4.3 与 §9。
struct FrenetState
{
  /// 沿参考线的累计弧长，单位 m。
  double s_m{0.0};
  /// 横向偏移，单位 m，**左为正**（与 `PathProjection::lateral_error_m` 同一个量）。
  double d_m{0.0};
  /// `dd/ds`，**无量纲**（m/m）。等于 tan(轨迹与参考线的夹角) × σ。
  double d_prime{0.0};
  /// `d²d/ds²`，单位 1/m。
  double d_double_prime{0.0};
};

/// @brief 笛卡尔系（`map`，ENU）下的位姿 + 曲率。
struct CartesianState
{
  double x_m{0.0};
  double y_m{0.0};
  /// 轨迹在该点的切线方向，单位弧度。
  double heading_rad{0.0};
  /// 轨迹在该点的曲率，单位 1/m，**正 = 左转**（与参考线同一约定）。
  double curvature_inv_m{0.0};
};

/// @brief 笛卡尔 → Frenet。
///
/// @param line   参考线。
/// @param pose   查询位姿（`map` 系）。做规划时传的是**后轴中心**位姿 ——
///               与控制侧不同，控制侧传前轴（Stanley 的前轴换算），
///               而 lattice 的起点是车辆本体状态。
/// @param hint   上一拍的 `PathProjection::index`，`std::nullopt` 表示全局搜索。
/// @param window 局部搜索半窗口，单位点数。
/// @return Frenet 状态。**`d_double_prime` 恒为 0**，见下面的 note。
/// @throw std::domain_error 航向误差过大（`|heading_error| ≥ kMaxHeadingErrorRad`）
///        或 `σ ≤ kMinArcLengthFactor`。
///
/// @note **`d_double_prime` 恒返回 0，这是有意的。** 它对应横向的二阶信息，
///       仿真里能从转角反推、真车上要么没有要么很脏。取 0 的后果是重规划时
///       **轨迹的曲率有一个小台阶**（位置和朝向仍然连续）。本项目 10 Hz 重规划
///       + 低速，这个台阶被转向执行机构（τ = 0.294 s）平滑掉了。
///       **要改成非零估计，前提是先量出那个估计的噪声** —— 否则是拿脏数据换连续性。
///
/// @note `hint` 不是性能优化而是正确性要求，理由见 `ReferenceLine::project()`。
///       规划一拍 100 ms、车速 8.3 m/s ⟹ 走 0.83 m ≈ 1.5 个采样点，
///       默认 30 点的窗口有两个数量级余量。
FrenetState to_frenet(
  const ads_common::ReferenceLine & line, const ads_common::Pose2D & pose,
  std::optional<std::size_t> hint = std::nullopt,
  std::size_t window = ads_common::ReferenceLine::kDefaultSearchWindow);

/// @brief Frenet → 笛卡尔（等距偏移曲线 `p = r(s) + d·N(s)`）。
///
/// @param line  参考线。
/// @param state Frenet 状态。
/// @param reference_curvature_rate_1pm2 参考线的 `dκ/ds`，单位 1/m²。默认 0，见下。
/// @return 笛卡尔位姿 + 曲率。
/// @throw std::out_of_range `state.s_m` 超出参考线范围（由 `ReferenceLine::at()` 抛）。
/// @throw std::domain_error `σ = 1 − d·κ ≤ kMinArcLengthFactor`（尖点，见文件头第 3 条）。
///
/// @note **`dκ/ds` 默认取 0 是一个有依据的近似，不是偷懒。**
///       本项目的参考线来自 OpenDRIVE，几何段是**直线和定曲率圆弧**，
///       段内 `κ′ ≡ 0`，只在段与段的接缝处跳变。而折线上去差分那个跳变，
///       得到的是一个宽度等于采样步长的**尖峰**，比真值更糟。
///       所以默认 0，并把接口留出来 —— 将来参考线换成 spiral（缓和曲线）时
///       `κ′` 是段内的常数，那时传进来就是精确的。
///       近似的代价：该项在 `κ_p` 里的贡献是 `κ′·d·tan Δψ`，
///       本项目 `|d| ≤ 0.85`、`|tan Δψ| ≲ 0.2`，量级在 0.17·κ′ 以下。
CartesianState to_cartesian(
  const ads_common::ReferenceLine & line, const FrenetState & state,
  double reference_curvature_rate_1pm2 = 0.0);

/// 弧长因子 `σ = 1 − d·κ` 的下限。低于它视为尖点并抛异常。
///
/// 取 1e-3：`σ` 是无量纲的、正常工作区间在 0.9 附近（本项目 `d·κ ≤ 0.106`），
/// 1e-3 远小于任何有物理意义的取值，又远大于浮点噪声。
/// 调大 → 在还没真正退化时就拒绝，正常绕行可能被误判；
/// 调小 → 逼近奇异点时 `κ_p = κ/σ` 会放大到几百，下游转角限幅会兜住，
///        但那时轨迹已经没有物理意义了。
inline constexpr double kMinArcLengthFactor = 1e-3;

/// 航向误差的绝对值上限，单位 rad。超过它 `to_frenet` 抛异常。
///
/// 取 π/3（60°）。理由：`d′ = −σ·tan(heading_error)`，`tan` 在接近 π/2 时爆炸
/// （tan 60° = 1.73，tan 80° = 5.67，tan 89° = 57），而一个偏离参考线 60° 的车
/// 已经不能说是"在沿这条线走"了 —— 继续规划得到的 lattice 起点毫无意义。
/// 绕障时车相对中心线的正常夹角在 15° 量级，60° 留了 4 倍余量。
/// 调大 → 车掉头或严重偏离时仍然输出一条荒谬的轨迹；
/// 调小 → 正常绕行的大偏航瞬间被误判成故障，车会莫名其妙停下。
inline constexpr double kMaxHeadingErrorRad = 1.0471975511965976;  // π/3

}  // namespace ads_planning

#endif  // ADS_PLANNING__FRENET_HPP_
