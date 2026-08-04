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

#include "ads_planning/quintic.hpp"

namespace ads_planning
{

namespace
{

void require_finite(double value, const char * name)
{
  // 单独判 isfinite：NaN 参与任何比较都返回 false，靠下面的 `> 0` 拦不住它，
  // 而且要拦 ±inf 不只是 NaN。见 CLAUDE.md 陷阱表。
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string("plan_lateral: ") + name + " 不是有限值");
  }
}

void require_positive(double value, const char * name)
{
  require_finite(value, name);
  if (value <= 0.0) {
    throw std::invalid_argument(
      std::string("plan_lateral: ") + name + " = " + std::to_string(value) + "，必须为正数");
  }
}

void require_non_negative(double value, const char * name)
{
  require_finite(value, name);
  if (value < 0.0) {
    throw std::invalid_argument(
      std::string("plan_lateral: ") + name + " = " + std::to_string(value) + "，不允许为负");
  }
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
  require_positive(params.lateral_offset_step_m, "lateral_offset_step_m");
  require_positive(params.min_horizon_m, "min_horizon_m");
  require_positive(params.max_horizon_m, "max_horizon_m");
  require_positive(params.horizon_step_m, "horizon_step_m");
  if (params.min_horizon_m > params.max_horizon_m) {
    throw std::invalid_argument("plan_lateral: min_horizon_m 大于 max_horizon_m");
  }
  require_positive(params.resample_step_m, "resample_step_m");
  require_non_negative(params.safety_margin_m, "safety_margin_m");
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

/// @brief 把一个后轴位姿换成车体外廓矩形。
///
/// 轨迹点是**后轴中心**（`base_link`，Autoware 惯例），而碰撞检查要的是几何中心。
/// 两者相差 `length/2 − rear_overhang`，本项目 = 2.2 − 0.85 = 1.35 m。
/// 漏掉这一步的症状：碰撞检查整体沿车头方向偏 1.35 m，
/// 而轨迹、代价、日志全部正常 —— 车会从障碍物"侧面擦过去"却报告安全。
Rectangle body_at(const CartesianState & rear_axle_pose, const LatticeParams & params)
{
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

  // 等分而不是 ceil(span/step)：`span/step` 恰好是整数时，浮点上它可能是
  // 80.00000000000001，ceil 会多算一步，末点被夹到端点后**最后两个点重合**，
  // 下游按弧长参数化时除以零。P1 踩过一次，RViz 里完全看不出来。
  const auto segment_count =
    std::max(1, static_cast<int>(std::llround(evaluation_span_m / params.resample_step_m)));

  const std::vector<double> offsets = sample_offsets(params);
  const std::vector<double> horizons = sample_horizons(params, evaluation_span_m);
  result.candidate_count = offsets.size() * horizons.size();

  double best_cost = std::numeric_limits<double>::infinity();
  bool found = false;

  for (const double target_offset_m : offsets) {
    for (const double maneuver_span_m : horizons) {
      // 终点一阶二阶导取 0：绕完要回到与车道中心线平行、曲率一致（planning.md §4.1）。
      const QuinticPolynomial lateral(
        start.d_m, start.d_prime, start.d_double_prime, target_offset_m, 0.0, 0.0, maneuver_span_m);

      Candidate candidate;
      candidate.target_offset_m = target_offset_m;
      candidate.maneuver_span_m = maneuver_span_m;
      candidate.points.reserve(static_cast<std::size_t>(segment_count) + 1);

      bool blocked = false;
      double min_clearance_m = std::numeric_limits<double>::infinity();

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

        // 尖点（σ = 1 − dκ ≤ 0）在本项目地图上到不了（d·κ ≤ 0.106），
        // 但换地图或放宽 max_lateral_offset_m 之后就可能。to_cartesian 会抛，
        // 这里**把它当成"这条候选不可行"而不是让异常穿透上去** ——
        // 一条候选的几何退化不该让整个规划周期失败。
        CartesianState pose;
        try {
          pose = to_cartesian(line, state);
        } catch (const std::domain_error &) {
          blocked = true;
          break;
        }

        const Rectangle body = body_at(pose, params);
        for (const Rectangle & obstacle : obstacles) {
          min_clearance_m = std::min(min_clearance_m, distance_m(body, obstacle));
        }
        // **准入条件，不是代价项**：不满足直接淘汰，不允许"够便宜就擦着过"。
        if (min_clearance_m < params.safety_margin_m) {
          blocked = true;
          break;
        }

        candidate.points.push_back(pose);
      }

      if (blocked) {
        ++result.blocked_count;
        continue;
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
    }
  }

  // 全被淘汰时**明确返回不可行**，不退而求其次给"最不糟的一条"。
  // 那样等于把碰撞检查放进一个可被绕过的分支：下游 ads_control 没有碰撞概念，
  // 会老老实实跟着那条撞的轨迹开过去。
  result.status = found ? LatticeStatus::kOk : LatticeStatus::kAllCandidatesBlocked;
  return result;
}

}  // namespace ads_planning
