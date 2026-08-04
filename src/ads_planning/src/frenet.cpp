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

#include "ads_planning/frenet.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "ads_common/angles.hpp"

namespace ads_planning
{

namespace
{

/// @brief 算弧长因子 σ = 1 − d·κ，并在退化时抛异常。
///
/// σ 有两个身份，理解其中任何一个都足以说明为什么它不能 ≤ 0：
///   ① **弧长比**：偏移曲线走过 Δs_offset = σ·Δs_ref。σ<0 意味着"参考线往前走、
///      偏移曲线往后退"，那是曲线折回来了（尖点）。
///   ② **雅可比行列式**：它是 (s,d) → (x,y) 这个变换的雅可比。行列式为 0
///      就是坐标退化，为负就是**翻面** —— 此时同一个 (x,y) 对应多个 (s,d)。
double arc_length_factor(double lateral_offset_m, double curvature_inv_m)
{
  const double factor = 1.0 - lateral_offset_m * curvature_inv_m;
  if (!(factor > kMinArcLengthFactor)) {
    // ⚠️ 写成 `!(factor > x)` 而不是 `factor <= x`，是为了让 **NaN 也被拦住**。
    //    NaN 参与任何比较都返回 false，所以 `factor <= x` 对 NaN 恒为假、原样放行；
    //    取反之后恒为真，NaN 就走进了这个分支。本仓库已因这一条吃过两次亏，
    //    见 CLAUDE.md 陷阱表「用比较去拦非有限值」。
    throw std::domain_error(
      "ads_planning: 弧长因子 σ = 1 − d·κ = " + std::to_string(factor) + " 已退化（d = " +
      std::to_string(lateral_offset_m) + ", κ = " + std::to_string(curvature_inv_m) +
      "）。这是偏移曲线的尖点，Frenet 坐标在此处不再一一对应。");
  }
  return factor;
}

}  // namespace

FrenetState to_frenet(
  const ads_common::ReferenceLine & line, const ads_common::Pose2D & pose,
  std::optional<std::size_t> hint, std::size_t window)
{
  const ads_common::PathProjection projection = line.project(pose, hint, window);

  // 航向误差过大时 tan() 会把 d′ 放大到毫无意义（tan 89° = 57）。
  // 与其输出一个荒谬的 lattice 起点，不如明确报错 —— 上游此时一定出了别的问题
  // （路由发错了、TF 反了、车真的掉头了），而那些问题在这里最容易被看见。
  if (!(std::abs(projection.heading_error_rad) < kMaxHeadingErrorRad)) {
    throw std::domain_error(
      "ads_planning: 航向误差 " + std::to_string(projection.heading_error_rad) + " rad 超过上限 " +
      std::to_string(kMaxHeadingErrorRad) + "，车已不能算是在沿参考线行驶");
  }

  const double sigma = arc_length_factor(projection.lateral_error_m, projection.curvature_inv_m);

  FrenetState state;
  state.s_m = projection.s_m;
  // lateral_error_m 的定义就是 Frenet 的 d（左正），同一个量，直接用。
  state.d_m = projection.lateral_error_m;

  // ---------------------------------------------------------------------------
  //  d′ = dd/ds —— 全模块最容易写错符号的一行
  // ---------------------------------------------------------------------------
  // 推导（planning.md §3.5）：
  //   dp/ds = σ·T + d′·N          （对偏移曲线求导，见 §3.2）
  //   ⟹ tan(轨迹与参考线夹角 Δψ) = d′ / σ
  //   ⟹ d′ = σ·tan(Δψ)
  //
  // 而 Δψ 是**车相对参考线**的夹角 = θ_车 − θ_路径，
  // `heading_error_rad` 的定义却是 angle_diff(车头, 路径) = θ_路径 − θ_车。
  // 两者互为相反数，所以这里有一个负号。
  //
  // **物理自检（改这一行务必手算一遍）**：车头偏向参考线**左侧**时，
  // 车正在往左跑 ⟹ d 增大 ⟹ d′ > 0。
  // 此时 θ_车 > θ_路径 ⟹ heading_error_rad < 0 ⟹ −tan(负数) > 0 ✓
  state.d_prime = -sigma * std::tan(projection.heading_error_rad);

  // d″ 恒 0，理由见头文件的 note（不是没实现，是有意不估）。
  state.d_double_prime = 0.0;

  return state;
}

CartesianState to_cartesian(
  const ads_common::ReferenceLine & line, const FrenetState & state,
  double reference_curvature_rate_1pm2)
{
  // at() 越界抛 std::out_of_range —— 有意不夹取，见 ReferenceLine::at() 的注释。
  const ads_common::PathPoint reference = line.at(state.s_m);

  const double kappa = reference.curvature_inv_m;
  const double sigma = arc_length_factor(state.d_m, kappa);

  const double sin_theta = std::sin(reference.heading_rad);
  const double cos_theta = std::cos(reference.heading_rad);

  CartesianState out;

  // ---------------------------------------------------------------------------
  //  位置：p = r(s) + d·N(s)，N 是**左**法向 (−sinθ, cosθ)
  // ---------------------------------------------------------------------------
  // 法向必须取左正，因为 d 是左正的。取成右法向的话，所有绕行方向都会反过来 ——
  // 而轨迹本身依然平滑、长度也合理，RViz 里看不出任何异常。
  out.x_m = reference.x_m - state.d_m * sin_theta;
  out.y_m = reference.y_m + state.d_m * cos_theta;

  // ---------------------------------------------------------------------------
  //  朝向：θ_p = θ + Δψ，Δψ = arctan(d′/σ)
  // ---------------------------------------------------------------------------
  // 用 atan2(d′, σ) 而不是 atan(d′/σ)：σ 已经保证 > kMinArcLengthFactor，
  // 但 atan2 省掉一次除法、且在 σ 很小时数值上更稳。
  // 因为 σ > 0，atan2 的结果落在 (−π/2, π/2)，与 atan 完全一致。
  const double delta_psi_rad = std::atan2(state.d_prime, sigma);
  out.heading_rad = ads_common::normalize_angle(reference.heading_rad + delta_psi_rad);

  // ---------------------------------------------------------------------------
  //  曲率（planning.md §3.3）
  // ---------------------------------------------------------------------------
  //           ⎡ (d″ + (κ′·d + κ·d′)·tanΔψ)·cos²Δψ        ⎤   cosΔψ
  //   κ_p  =  ⎢ ──────────────────────────────────  +  κ ⎥ · ──────
  //           ⎣                 σ                         ⎦     σ
  //
  // 退化自检（常偏移：d′ = d″ = 0 ⟹ Δψ = 0 ⟹ cosΔψ = 1）：
  //   κ_p = (0 + κ)·1/σ = κ/(1 − d·κ)   ← 熟知的等距偏移曲线曲率
  // 代数字：R=12 的圆（κ=1/12）向左偏 4 m，新半径应是 8 m ⟹ κ_p 应为 1/8。
  //   (1/12)/(1 − 4/12) = (1/12)/(2/3) = 1/8 ✓
  //
  // **这个 /σ 是第 2 条陷阱**：漏掉它在直线上（κ=0）完全正确，
  // 只在弯道的绕行段偏小，症状是曲率限速偏高、横向加速度超标。
  const double cos_delta_psi = std::cos(delta_psi_rad);
  const double tan_delta_psi = state.d_prime / sigma;
  const double bracket =
    state.d_double_prime +
    (reference_curvature_rate_1pm2 * state.d_m + kappa * state.d_prime) * tan_delta_psi;
  out.curvature_inv_m =
    (bracket * cos_delta_psi * cos_delta_psi / sigma + kappa) * cos_delta_psi / sigma;

  return out;
}

}  // namespace ads_planning
