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

#include "ads_planning/lattice.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "ads_common/numeric_checks.hpp"
#include "ads_planning/quintic.hpp"

namespace ads_planning
{

namespace
{

// 校验工具在 ads_common —— **不要在这里再写一份**。
// 三个薄封装只是为了省掉每处都写 kPlanLateral 这个上下文串。
constexpr char kPlanLateral[] = "plan_lateral";

void require_finite(double value, const char * name)
{
  ads_common::RequireFinite(value, kPlanLateral, name);
}

void require_positive(double value, const char * name)
{
  ads_common::RequireFinitePositive(value, kPlanLateral, name);
}

void require_non_negative(double value, const char * name)
{
  ads_common::RequireFiniteNonNegative(value, kPlanLateral, name);
}

/// @brief 入口处一次性校验全部参数与障碍物。
///
/// 校验放在这里而不是内层循环，是因为内层每周期跑上千次；
/// 而**放在这里而不是完全不做**，是因为脏数据（尤其 NaN）传进碰撞检查后
/// 不会崩，只会让每一次比较都返回 false —— 表现为「所有候选都没碰撞」，
/// 车直接开过去。**不报错、只给一个看起来能用的结果**，本仓库已因这个模式吃过多次亏。
void validate(const LatticeParams & params, const std::vector<Rectangle> & obstacles)
{
  require_positive(params.max_lateral_offset_m, "max_lateral_offset_m");
  require_positive(params.max_curvature_inv_m, "max_curvature_inv_m");
  require_positive(params.lateral_offset_step_m, "lateral_offset_step_m");
  require_positive(params.min_horizon_m, "min_horizon_m");
  require_positive(params.max_horizon_m, "max_horizon_m");
  require_positive(params.horizon_step_m, "horizon_step_m");
  if (params.min_horizon_m > params.max_horizon_m) {
    throw std::invalid_argument("plan_lateral: min_horizon_m 大于 max_horizon_m");
  }
  require_positive(params.resample_step_m, "resample_step_m");
  // ⚠️ margin 必须**严格为正**（2026-08-12 复检堵上的后门）：
  //    原来是 require_non_negative —— 而 distance_m() 对**重叠**的 OBB 返回 0.0，
  //    于是 margin=0 时准入判据 `0.0 < 0.0` 恒为假，**撞上去的候选不被淘汰**。
  //    这实质上是「可被配置关掉的碰撞检查」（SPEC §11 明令禁止），
  //    只是关闭开关伪装成了一个合法的阈值取值。margin=0（车体擦着障碍物过）
  //    不是任何 ODD 允许的行为，没有合法使用场景。
  require_positive(params.safety_margin_m, "safety_margin_m");
  // floor 同 margin 的理由必须严格为正（margin=0 后门那条注释同样适用）；
  // floor > margin 没有语义（保底线高于选择线 ⟹ 两级退化且更严，配置错误）。
  require_positive(params.safety_floor_m, "safety_floor_m");
  if (params.safety_floor_m > params.safety_margin_m) {
    throw std::invalid_argument("plan_lateral: safety_floor_m 大于 safety_margin_m");
  }
  require_positive(params.vehicle_length_m, "vehicle_length_m");
  require_positive(params.vehicle_width_m, "vehicle_width_m");
  require_non_negative(params.rear_overhang_m, "rear_overhang_m");
  require_non_negative(params.weight_offset, "weight_offset");
  require_non_negative(params.weight_curvature, "weight_curvature");
  require_non_negative(params.weight_clearance, "weight_clearance");
  require_non_negative(params.weight_consistency, "weight_consistency");

  if (params.rear_overhang_m > 0.5 * params.vehicle_length_m) {
    throw std::invalid_argument(
      "plan_lateral: rear_overhang_m 超过半车长，几何中心会跑到后轴后面 —— "
      "多半是把「后轴到中心的距离」填进来了，那是推导量，本层自己算");
  }

  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const std::string tag = "obstacles[" + std::to_string(i) + "].";
    require_finite(obstacles[i].center_x_m, (tag + "center_x_m").c_str());
    require_finite(obstacles[i].center_y_m, (tag + "center_y_m").c_str());
    require_finite(obstacles[i].heading_rad, (tag + "heading_rad").c_str());
    // 零尺寸**允许**（SPEC §8 点名的退化情形，碰撞检查能正确处理），负尺寸不允许。
    require_non_negative(obstacles[i].length_m, (tag + "length_m").c_str());
    require_non_negative(obstacles[i].width_m, (tag + "width_m").c_str());
  }
}

/// @brief 生成候选的终点横向偏移集合，**保证含 0 且左右对称**。
///
/// 用 `k·step`（k 取整数）而不是从 −max 累加到 +max：后者在浮点上未必落到 0，
/// 而「中心线」这条候选必须**精确**存在 —— 无障碍物时它要给出 |d| 恰好为 0
/// 的轨迹，差 1e-16 都会让「无障碍物 ⟹ 沿中心线」这条判据变成一个容差问题。
std::vector<double> sample_offsets(const LatticeParams & params)
{
  const auto steps =
    static_cast<int>(std::floor(params.max_lateral_offset_m / params.lateral_offset_step_m));
  std::vector<double> offsets;
  offsets.reserve(static_cast<std::size_t>(2 * steps + 1));
  for (int k = -steps; k <= steps; ++k) {
    offsets.push_back(k * params.lateral_offset_step_m);
  }
  return offsets;
}

/// @brief 生成机动长度 `S` 的候选集合，含 `min` 且不超过 `max` 与可用前视距离。
///
/// 与横向偏移不同，这里**不需要**"精确含某个值"，所以用简单累加即可；
/// 但必须保证至少有一个元素 —— 否则 min > 可用距离时候选集为空，
/// 结果会是 kAllCandidatesBlocked（"全被淘汰"），而真相是"一条都没生成"。
/// 那两件事在 diagnostics 里必须分得开。
std::vector<double> sample_horizons(const LatticeParams & params, double available_m)
{
  std::vector<double> horizons;
  const double upper_m = std::min(params.max_horizon_m, available_m);
  for (double span_m = params.min_horizon_m; span_m <= upper_m + 1e-9;
       span_m += params.horizon_step_m) {
    horizons.push_back(std::min(span_m, upper_m));
  }
  if (horizons.empty()) {
    // min_horizon 都超过可用距离：退而用可用距离本身，机动会比配置的更急，
    // 但**这是唯一还能绕的选择**，比直接放弃好。真正兜底的是安全间距的准入条件。
    horizons.push_back(available_m);
  }
  return horizons;
}

}  // namespace

Rectangle vehicle_body_at(const CartesianState & rear_axle_pose, const LatticeParams & params)
{
  // 轨迹点是**后轴中心**（`base_link`，Autoware 惯例），而碰撞检查要的是几何中心。
  // 两者相差 length/2 − rear_overhang，本项目 = 2.2 − 0.85 = 1.35 m。
  // 漏掉这一步的症状：碰撞检查整体沿车头方向偏 1.35 m，
  // 而轨迹、代价、日志全部正常 —— 车会从障碍物"侧面擦过去"却报告安全。
  const double rear_axle_to_center_m = 0.5 * params.vehicle_length_m - params.rear_overhang_m;
  Rectangle body;
  body.center_x_m =
    rear_axle_pose.x_m + rear_axle_to_center_m * std::cos(rear_axle_pose.heading_rad);
  body.center_y_m =
    rear_axle_pose.y_m + rear_axle_to_center_m * std::sin(rear_axle_pose.heading_rad);
  body.heading_rad = rear_axle_pose.heading_rad;
  body.length_m = params.vehicle_length_m;
  body.width_m = params.vehicle_width_m;
  return body;
}

std::vector<CartesianState> build_lateral_geometry(
  const ads_common::ReferenceLine & line, const FrenetState & start, double target_offset_m,
  double maneuver_span_m, double evaluation_span_m, double resample_step_m)
{
  // 终点一阶二阶导取 0：绕完要回到与车道中心线平行、曲率一致（planning.md §4.1）。
  const QuinticPolynomial lateral(
    start.d_m, start.d_prime, start.d_double_prime, target_offset_m, 0.0, 0.0, maneuver_span_m);

  // 等分而不是 ceil(span/step)：`span/step` 恰好是整数时，浮点上它可能是
  // 80.00000000000001，ceil 会多算一步，末点被夹到端点后**最后两个点重合**，
  // 下游按弧长参数化时除以零。P1 踩过一次，RViz 里完全看不出来。
  const auto segment_count =
    std::max(1, static_cast<int>(std::llround(evaluation_span_m / resample_step_m)));

  std::vector<CartesianState> points;
  points.reserve(static_cast<std::size_t>(segment_count) + 1);

  for (int i = 0; i <= segment_count; ++i) {
    const double along_m = evaluation_span_m * i / segment_count;

    FrenetState state;
    state.s_m = start.s_m + along_m;
    if (along_m <= maneuver_span_m) {
      state.d_m = lateral.value_at(along_m);
      state.d_prime = lateral.first_derivative_at(along_m);
      state.d_double_prime = lateral.second_derivative_at(along_m);
    } else {
      // 机动做完之后**保持**那个偏移，不要继续外推五次式 ——
      // 五次式在区间外发散得很快（1.5·S 处已偏出米级），
      // 而"变道完成后沿新位置直行"才是物理上该发生的事。
      state.d_m = target_offset_m;
      state.d_prime = 0.0;
      state.d_double_prime = 0.0;
    }

    points.push_back(to_cartesian(line, state));
  }
  return points;
}

namespace
{

/// @brief 候选五次式在机动段上的**峰值曲率**（密采样，1/m）。
///
/// ⚠️ 不能用输出几何点（0.5 m 点距）上的曲率求峰值：短机动上那些点恰好落在
/// |d″| 的零点（along = 0 与 S 是边界条件、0.5·S 是 S 形的拐点），峰值藏在
/// 采样点之间 —— 实测 rem=1.2、Δd=0.5 的候选峰值曲率 ≈ 2.0，而三个输出点上
/// 全部接近 0，速度剖面与代价因此完全看不见它。
///
/// 采样数 32：五次式的 |κ|(s) 在机动段内至多两个峰（d″ 是三次式），
/// 32 点的最坏漏采误差 ≪ 门限余量；再密只是白花时间。
///
/// @throw std::domain_error 尖点（σ = 1 − d·κ ≤ 0），调用方按"候选不可行"处理。
template <typename ProfileFn>
double peak_profile_curvature_inv_m(
  const ads_common::ReferenceLine & line, double start_s_m, double span_m, ProfileFn profile)
{
  constexpr int kSamples = 32;
  double peak = 0.0;
  for (int i = 0; i <= kSamples; ++i) {
    const double along_m = span_m * i / kSamples;
    FrenetState state = profile(along_m);
    state.s_m = start_s_m + along_m;
    peak = std::max(peak, std::abs(to_cartesian(line, state).curvature_inv_m));
  }
  return peak;
}

double peak_curvature_inv_m(
  const ads_common::ReferenceLine & line, const FrenetState & start, double target_offset_m,
  double maneuver_span_m)
{
  const QuinticPolynomial lateral(
    start.d_m, start.d_prime, start.d_double_prime, target_offset_m, 0.0, 0.0, maneuver_span_m);
  return peak_profile_curvature_inv_m(line, start.s_m, maneuver_span_m, [&](double along_m) {
    FrenetState state;
    state.d_m = lateral.value_at(along_m);
    state.d_prime = lateral.first_derivative_at(along_m);
    state.d_double_prime = lateral.second_derivative_at(along_m);
    return state;
  });
}

/// @brief 保持候选的横向剖面：**二次衰减**，不是五次式（P8-S2d 第二刀）。
///
/// 五次式版保持候选要在跑道内把 d′ 压回零再稳住，峰值 |d″| ≈ 5.25·d′/S ——
/// 实测 **d′ = 0.1（航向差 5.7°）在 1.5 m 跑道上就超运动学门**，而 Stanley
/// 在弯道出口 ±3–6° 摆动是常态：junction 末段全部候选（含保持）被门淘汰，
/// 车在离 goal 1.47 m 处 kStopping 停保持。
///
/// 二次衰减（d″ = −d′₀/S 常值）是消掉 d′ 的**最小峰值曲率**剖面：
/// κ ≈ d′₀/S，d′ = 0.3（17°）在 1.5 m 上也只有 0.2。代价是终点偏移
/// 漂到自然落点 d₀ + d′₀·S/2（≤ ~0.2 m），由调用方判是否仍在车道内。
FrenetState hold_profile_at(const FrenetState & start, double decay_span_m, double along_m)
{
  FrenetState state;
  if (along_m < decay_span_m) {
    const double ratio = along_m / decay_span_m;
    state.d_m = start.d_m + start.d_prime * along_m * (1.0 - 0.5 * ratio);
    state.d_prime = start.d_prime * (1.0 - ratio);
    state.d_double_prime = -start.d_prime / decay_span_m;
  } else {
    state.d_m = start.d_m + 0.5 * start.d_prime * decay_span_m;
    state.d_prime = 0.0;
    state.d_double_prime = 0.0;
  }
  return state;
}

/// @brief 按保持剖面生成整段几何（与 build_lateral_geometry 同样的等分采样）。
std::vector<CartesianState> build_hold_geometry(
  const ads_common::ReferenceLine & line, const FrenetState & start, double evaluation_span_m,
  double resample_step_m)
{
  const auto segment_count =
    std::max(1, static_cast<int>(std::llround(evaluation_span_m / resample_step_m)));
  std::vector<CartesianState> points;
  points.reserve(static_cast<std::size_t>(segment_count) + 1);
  for (int i = 0; i <= segment_count; ++i) {
    const double along_m = evaluation_span_m * i / segment_count;
    FrenetState state = hold_profile_at(start, evaluation_span_m, along_m);
    state.s_m = start.s_m + along_m;
    points.push_back(to_cartesian(line, state));
  }
  return points;
}

}  // namespace

LatticeResult plan_lateral(
  const ads_common::ReferenceLine & line, const FrenetState & start,
  const std::vector<Rectangle> & obstacles, const LatticeParams & params,
  std::optional<double> previous_target_offset_m)
{
  validate(params, obstacles);
  require_finite(start.s_m, "start.s_m");
  require_finite(start.d_m, "start.d_m");
  require_finite(start.d_prime, "start.d_prime");
  require_finite(start.d_double_prime, "start.d_double_prime");

  LatticeResult result;

  // ---------------------------------------------------------------------------
  //  前视距离：被参考线的剩余长度截断
  // ---------------------------------------------------------------------------
  // 主动截断而不是让 ReferenceLine::at() 抛异常 —— at() 越界抛是对的
  // （"弧长 s 处"根本不存在那样一个点），但"路走到头了"是**策略**，
  // 该由这一层决定，不该表现为一个异常穿透上去。
  const double remaining_m = line.length_m() - start.s_m;
  // **所有候选都在这同一段长度上采样与检查**，与各自的机动长度 S 无关。
  // 否则短 S 的候选轨迹也短，碰撞检查覆盖的距离就少 —— 于是它会**显得更安全**，
  // 而一个只问"有没有撞"的判据完全发现不了。
  const double evaluation_span_m = std::min(params.max_horizon_m, remaining_m);
  if (!(evaluation_span_m >= params.resample_step_m)) {
    result.status = LatticeStatus::kHorizonTooShort;
    return result;
  }

  const std::vector<double> offsets = sample_offsets(params);
  const std::vector<double> horizons = sample_horizons(params, evaluation_span_m);
  result.candidate_count = offsets.size() * horizons.size();

  double best_cost = std::numeric_limits<double>::infinity();
  bool found = false;

  const auto evaluate_candidate = [&](
                                    double target_offset_m, double maneuver_span_m,
                                    std::vector<CartesianState> prebuilt_points = {}) {
    // 两级准入的分档：目标偏移在当前偏移半个网格步之内 = 延续既成轨迹，
    // 准入用 floor；其余 = 新机动，用 margin。按**目标**分而不是按几何逐点
    // 分 —— 逐点混合会造出"前半段吃 floor 后半段吃 margin"的怪胎候选。
    const bool is_continuation =
      std::abs(target_offset_m - start.d_m) <= 0.5 * params.lateral_offset_step_m;
    const double admission_m = is_continuation ? params.safety_floor_m : params.safety_margin_m;
    Candidate candidate;
    candidate.target_offset_m = target_offset_m;
    candidate.maneuver_span_m = maneuver_span_m;

    // 尖点（σ = 1 − dκ ≤ 0）在本项目地图上到不了（d·κ ≤ 0.106），
    // 但换地图或放宽 max_lateral_offset_m 之后就可能。to_cartesian 会抛，
    // 这里**把它当成"这条候选不可行"而不是让异常穿透上去** ——
    // 一条候选的几何退化不该让整个规划周期失败。
    try {
      if (!prebuilt_points.empty()) {
        // 保持候选：几何与运动学门由调用处按自己的剖面做完了。
        candidate.points = std::move(prebuilt_points);
      } else {
        // ---- 运动学准入（P8-S2d）：车开不出来的候选直接淘汰 ----------------
        // 必须在几何构建之前密采样判（见 peak_curvature_inv_m 的说明 ——
        // 输出点距上的曲率在短机动上恰好全落在零点，峰值是看不见的）。
        if (
          peak_curvature_inv_m(line, start, target_offset_m, maneuver_span_m) >
          params.max_curvature_inv_m) {
          ++result.blocked_count;
          ++result.curvature_blocked_count;
          return;
        }
        candidate.points = build_lateral_geometry(
          line, start, target_offset_m, maneuver_span_m, evaluation_span_m, params.resample_step_m);
      }
    } catch (const std::domain_error &) {
      ++result.blocked_count;
      return;
    }

    double min_clearance_m = std::numeric_limits<double>::infinity();
    for (const CartesianState & pose : candidate.points) {
      const Rectangle body = vehicle_body_at(pose, params);
      for (const Rectangle & obstacle : obstacles) {
        min_clearance_m = std::min(min_clearance_m, distance_m(body, obstacle));
      }
    }

    // **准入条件，不是代价项**：不满足直接淘汰，不允许"够便宜就擦着过"。
    //
    // ⚠️ 重叠（<= 0）单独一道**与 margin 无关**的硬性淘汰：即使上面那条
    //    require_positive 哪天被谁改回去，撞上去的候选也过不了这里。
    //    碰撞检查不许有任何参数取值能把它关掉（SPEC §11），
    //    所以防线要有一道不吃参数的。
    if (min_clearance_m <= 0.0 || min_clearance_m < admission_m) {
      ++result.blocked_count;
      return;
    }

    candidate.min_clearance_m = min_clearance_m;

    // -------------------------------------------------------------------------
    //  代价
    // -------------------------------------------------------------------------
    // 曲率项用**候选自身**的弦长做一阶求积，而不是参考线的 Δs：
    // 两者差一个因子 √(σ² + d′²)，而这个因子**逐候选不同**（d 不同 ⟹ σ 不同），
    // 拿参考线的 Δs 会系统性地偏袒外侧候选。直接用输出点的弦长最省事也最一致 ——
    // 下游用这些点建 ReferenceLine 时算出来的弧长就是它。
    double curvature_integral = 0.0;
    for (std::size_t i = 0; i + 1 < candidate.points.size(); ++i) {
      const double chord_m = std::hypot(
        candidate.points[i + 1].x_m - candidate.points[i].x_m,
        candidate.points[i + 1].y_m - candidate.points[i].y_m);
      curvature_integral +=
        candidate.points[i].curvature_inv_m * candidate.points[i].curvature_inv_m * chord_m;
    }

    // 无障碍物时 min_clearance 是 +∞，`weight/∞ = 0` —— 于是"没有障碍物"与
    // "障碍物无穷远"在代价上等价，这正是想要的语义，不需要额外分支。
    const double clearance_cost = params.weight_clearance / min_clearance_m;

    // 首次规划时一致性项恒为 0，**不是拿 0 当"上一次"**：
    // 后者会在车本来就偏着的情况下凭空引入一个回中心线的偏好，
    // 而那个偏好本该由 weight_offset 单独表达。
    double consistency_cost = 0.0;
    if (previous_target_offset_m.has_value()) {
      const double delta = target_offset_m - previous_target_offset_m.value();
      consistency_cost = params.weight_consistency * delta * delta;
    }

    candidate.cost = params.weight_offset * target_offset_m * target_offset_m +
                     params.weight_curvature * curvature_integral + clearance_cost +
                     consistency_cost;

    if (candidate.cost < best_cost) {
      best_cost = candidate.cost;
      result.best = std::move(candidate);
      found = true;
    }
  };

  for (const double target_offset_m : offsets) {
    for (const double maneuver_span_m : horizons) {
      evaluate_candidate(target_offset_m, maneuver_span_m);
    }
  }

  // ---- 短跑道保持候选（P8-S2d）--------------------------------------------
  // 剩余长度装不下任何常规机动（S < min_horizon）时，网格候选全是「在
  // 不足 min_horizon 的跨度上强行收敛」—— Δd 只要 0.1，1 m 跨度上的峰值
  // |d″| ≈ 0.58 就已超运动学门限，于是**整张网格都会被上面那道门淘汰**。
  // 这不是门太严：那些机动车确实做不出来。缺的是「不做新机动，保持当前
  // 偏移走完最后几米」这个物理上唯一合理的选项 —— 补上它，末端永远至少
  // 有一条可跟踪的候选，除非连保持现状都被障碍物堵死（那时 kStopping
  // 才是正确语义）。P7-S4 junction 实测：没有它，出弯残余 0.5 m 偏差 +
  // 剩 1 m 跑道 → 全候选淘汰 → 车在离 goal 1.16 m 处 NO_PATH 停保持。
  //
  // 目标夹回 ±max_offset：暂态可能越界（起点状态不受网格约束），
  // 保持在界外会一路压着线走完 —— 夹回去是温和的（超出量通常只有厘米级）。
  if (evaluation_span_m < params.min_horizon_m) {
    ++result.candidate_count;
    // 自然落点 = d₀ + d′₀·S/2（二次衰减的终点，见 hold_profile_at 的说明）。
    const double natural_end_m = start.d_m + 0.5 * start.d_prime * evaluation_span_m;
    if (std::abs(natural_end_m) <= params.max_lateral_offset_m) {
      try {
        const double hold_peak = peak_profile_curvature_inv_m(
          line, start.s_m, evaluation_span_m,
          [&](double along_m) { return hold_profile_at(start, evaluation_span_m, along_m); });
        if (hold_peak > params.max_curvature_inv_m) {
          ++result.blocked_count;
          ++result.curvature_blocked_count;
        } else {
          evaluate_candidate(
            natural_end_m, evaluation_span_m,
            build_hold_geometry(line, start, evaluation_span_m, params.resample_step_m));
        }
      } catch (const std::domain_error &) {
        ++result.blocked_count;
      }
    } else {
      // 自然落点会出车道：退回五次式收敛到界内（d′ 大时可能被门淘汰 ——
      // 那时车正带着大横向速度冲向车道边缘，kStopping 停下是正确语义）。
      const double hold_offset_m =
        std::clamp(start.d_m, -params.max_lateral_offset_m, params.max_lateral_offset_m);
      evaluate_candidate(hold_offset_m, evaluation_span_m);
    }
  }

  // 全被淘汰时**明确返回不可行**，不退而求其次给"最不糟的一条"。
  // 那样等于把碰撞检查放进一个可被绕过的分支：下游 ads_control 没有碰撞概念，
  // 会老老实实跟着那条撞的轨迹开过去。
  result.status = found ? LatticeStatus::kOk : LatticeStatus::kAllCandidatesBlocked;
  return result;
}

}  // namespace ads_planning
