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

#include "ads_planning/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "ads_common/numeric_checks.hpp"

namespace ads_planning
{

namespace
{

constexpr char kPlan[] = "ads_planning::plan";

/// @brief 给一串几何点标上速度与加速度。
///
/// 弧长按**实际弦长**累加，不是"序号 × 采样步长"。
/// 后者在弯道上偏得很厉害（P2 实测：本项目地图弯道外侧的实际点距比标称大 14.58%），
/// 而它的后果是曲率限速偏小 6.58% —— 车明显开得慢，且没有任何一层报错。
std::vector<TrajectoryPoint> annotate_with_speed(
  const std::vector<CartesianState> & geometry, SpeedProfileParams params,
  double terminal_speed_mps)
{
  params.terminal_speed_mps = terminal_speed_mps;
  const std::size_t count = geometry.size();

  std::vector<double> arc_lengths_m(count, 0.0);
  std::vector<double> curvatures_inv_m(count, 0.0);
  curvatures_inv_m[0] = geometry[0].curvature_inv_m;
  for (std::size_t i = 1; i < count; ++i) {
    arc_lengths_m[i] =
      arc_lengths_m[i - 1] +
      std::hypot(geometry[i].x_m - geometry[i - 1].x_m, geometry[i].y_m - geometry[i - 1].y_m);
    curvatures_inv_m[i] = geometry[i].curvature_inv_m;
  }

  const SpeedProfile profile(arc_lengths_m, curvatures_inv_m, params);

  std::vector<TrajectoryPoint> points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    TrajectoryPoint point;
    point.x_m = geometry[i].x_m;
    point.y_m = geometry[i].y_m;
    point.heading_rad = geometry[i].heading_rad;
    point.curvature_inv_m = geometry[i].curvature_inv_m;
    point.s_m = arc_lengths_m[i];
    point.speed_mps = profile.speeds_mps()[i];
    // 加速度的语义是「本段内恒定」，所以末点没有对应的段，取 0。
    // 末点的速度剖面本来就是终点（v = 0 或路径末端），不会有人在那儿要前馈。
    point.accel_mps2 = (i + 1 < count) ? profile.target_accel_at(i, 0.0) : 0.0;
    points.push_back(point);
  }
  return points;
}

/// @brief 单点、零速的紧急停车轨迹。
///
/// 用在「车已经贴到障碍物上了」这种本不该发生的情形：此时连一条能减速的
/// 几何都排不出来。
///
/// ⚠️ **契约修正（2026-08-12 复检）**：这里原来声称"单点让下游知道目标速度
///    是 0、与空数组的降级分支分得开"—— **那个区分不存在**：control_node 对
///    points.size() < 2 一律按「没有轨迹」丢弃，单点与空数组走的是**同一条**
///    NO_PATH 刹停分支。行为上这是安全的（两条路都刹停并保持），真正把
///    「规划器说停」与「规划器挂了」分开的是 **diagnostics**（本文件的调用方
///    会报"尖点/不可行"，见 plan_with_stop 的 ERROR/WARN 路径）——
///    区分活在诊断通道里，不在轨迹通道里。保留单点是因为它还携带停车点
///    位姿，将来若有消费者可以用；但**不要再把它当成一条能被跟踪的轨迹**。
std::vector<TrajectoryPoint> emergency_stop_at(const CartesianState & pose)
{
  TrajectoryPoint point;
  point.x_m = pose.x_m;
  point.y_m = pose.y_m;
  point.heading_rad = pose.heading_rad;
  point.curvature_inv_m = pose.curvature_inv_m;
  point.s_m = 0.0;
  point.speed_mps = 0.0;
  point.accel_mps2 = 0.0;
  return {point};
}

/// @brief 逐弦长累加（annotate_with_speed 的前半段单独拿出来，注入约束要先用）。
std::vector<double> chord_arc_lengths(const std::vector<CartesianState> & geometry)
{
  std::vector<double> arc_lengths_m(geometry.size(), 0.0);
  for (std::size_t i = 1; i < geometry.size(); ++i) {
    arc_lengths_m[i] =
      arc_lengths_m[i - 1] +
      std::hypot(geometry[i].x_m - geometry[i - 1].x_m, geometry[i].y_m - geometry[i - 1].y_m);
  }
  return arc_lengths_m;
}

/// @brief 把行为层的停车点（参考线弧长系）注入几何：截断在停车点处。
///
/// @return true = 截断发生（调用方必须把 terminal 设成 0）。
///
/// @note 停车点在起点之前/截断后不足 2 点 ⟹ 几何缩成**单点零速**
///       （emergency_stop_at 的语义）—— ego 已经越过了行为停车点，
///       这本不该发生（上一周期就该停住），但真发生时给明确的停车指示。
bool truncate_at_behavior_stop(std::vector<CartesianState> * geometry, double stop_rel_m)
{
  const std::vector<double> arc_lengths_m = chord_arc_lengths(*geometry);
  if (stop_rel_m <= 0.0) {
    geometry->resize(1);
    return true;
  }
  std::size_t stop_index = 0;
  for (std::size_t i = 0; i < geometry->size(); ++i) {
    if (arc_lengths_m[i] <= stop_rel_m) {
      stop_index = i;
    }
  }
  if (stop_index + 1 >= geometry->size()) {
    return false;  // 停车点在窗口之外：本窗口不用截，末点照常按前视处理。
  }
  geometry->resize(stop_index + 1);
  return true;
}

/// @brief 把行为层的逐段限速映射到几何点位上（参考线弧长系 → 候选弧长系）。
///
/// @return 与几何逐点对应的上限数组；没有任何段覆盖时为空（= 不注入）。
std::vector<double> map_caps_to_geometry(
  const std::vector<CartesianState> & geometry, double s_offset_m,
  const std::vector<SpeedCap> & caps, double cruise_speed_mps)
{
  if (caps.empty()) {
    return {};
  }
  const std::vector<double> arc_lengths_m = chord_arc_lengths(geometry);
  std::vector<double> per_point(geometry.size(), cruise_speed_mps);
  bool any = false;
  for (std::size_t i = 0; i < geometry.size(); ++i) {
    const double s_ref_m = s_offset_m + arc_lengths_m[i];
    for (const SpeedCap & cap : caps) {
      if (s_ref_m >= cap.s_from_m && s_ref_m <= cap.s_to_m) {
        per_point[i] = std::min(per_point[i], cap.v_cap_mps);
        any = true;
      }
    }
  }
  return any ? per_point : std::vector<double>{};
}

}  // namespace

PlanResult plan(
  const ads_common::ReferenceLine & line, const FrenetState & start,
  const std::vector<Rectangle> & obstacles, const PlanParams & params,
  std::optional<double> previous_target_offset_m, const LongitudinalConstraint * constraint)
{
  ads_common::RequireFiniteNonNegative(params.stop_margin_m, kPlan, "stop_margin_m");

  PlanResult result;
  result.stop_clearance_m = std::numeric_limits<double>::infinity();

  const LatticeResult lattice =
    plan_lateral(line, start, obstacles, params.lattice, previous_target_offset_m);
  result.candidate_count = lattice.candidate_count;
  result.blocked_count = lattice.blocked_count;

  if (lattice.status == LatticeStatus::kHorizonTooShort) {
    result.status = PlanStatus::kRouteExhausted;
    return result;
  }

  if (lattice.status == LatticeStatus::kOk) {
    result.status = PlanStatus::kOk;
    result.lateral_offset_m = lattice.best.target_offset_m;

    // ⚠️ **末点该不该归零，取决于窗口是被什么截断的。**
    //    被**前视距离**截断（路还长）⟹ 末点只是"看到这儿为止"，
    //    归零就等于宣称「30 m 后你会停住」，那是假的，后向扫描会把
    //    窗口后半段的速度全压低。
    //    被**路径末端**截断 ⟹ 那就是终点，必须归零。
    //
    //    判据用 `max_horizon < 剩余长度`：成立说明是前视截的，路还长。
    //    容差取一个采样步长 —— 差不到一个点的距离时按"到头了"算，
    //    保守方向（早停而不是冲过去）。
    const double remaining_m = line.length_m() - start.s_m;
    const bool limited_by_horizon =
      params.lattice.max_horizon_m < remaining_m - params.lattice.resample_step_m;

    // ---- 行为约束注入（P7-S3）----------------------------------------------
    // stop_at 走**同一条**「几何截断 + terminal=0」路径（见下面 kStopping 分支
    // 的说明），caps 走 SpeedProfileParams 的逐点上限。两者都只收紧不放宽。
    std::vector<CartesianState> geometry = lattice.best.points;
    SpeedProfileParams speed_params = params.speed;
    bool stopped_by_behavior = false;
    if (constraint != nullptr) {
      if (constraint->stop_at_s_m.has_value()) {
        stopped_by_behavior =
          truncate_at_behavior_stop(&geometry, *constraint->stop_at_s_m - start.s_m);
      }
      speed_params.speed_caps_mps =
        map_caps_to_geometry(geometry, start.s_m, constraint->caps, params.speed.cruise_speed_mps);
    }
    if (geometry.size() < 2) {
      // ego 已越过行为停车点（stop_rel ≤ 0）：单点零速，语义同 emergency_stop_at。
      result.points = emergency_stop_at(geometry.front());
      return result;
    }
    const double terminal_mps =
      (limited_by_horizon && !stopped_by_behavior) ? params.speed.cruise_speed_mps : 0.0;
    result.points = annotate_with_speed(geometry, speed_params, terminal_mps);
    return result;
  }

  // ---------------------------------------------------------------------------
  //  全部候选被淘汰 ⟹ 停车（planning.md §6 决策四）
  // ---------------------------------------------------------------------------
  result.status = PlanStatus::kStopping;
  // **保持当前横向位置**，不做新的横向机动。
  // 换成 d_T = 0（回中心线）看着更"规范"，但那是在明知前方受阻时又**主动横移**，
  // 而横移的那一侧刚刚才被判定为不可行 —— 等于往未知里走。
  result.lateral_offset_m = start.d_m;

  const double evaluation_span_m =
    std::min(params.lattice.max_horizon_m, line.length_m() - start.s_m);

  std::vector<CartesianState> geometry;
  try {
    geometry = build_lateral_geometry(
      line, start, start.d_m, params.lattice.min_horizon_m, evaluation_span_m,
      params.lattice.resample_step_m);
  } catch (const std::domain_error &) {
    // 连"保持当前偏移"都算不出几何（尖点）—— 只能原地停。
    result.points = emergency_stop_at(to_cartesian(line, FrenetState{start.s_m, start.d_m, 0, 0}));
    result.stop_clearance_m = 0.0;
    return result;
  }

  // ---------------------------------------------------------------------------
  //  找停车点：第一个"车体离障碍物近于安全间距"的位置，再往回退 stop_margin_m
  // ---------------------------------------------------------------------------
  // 注意这里用的是**车体外廓**而不是轨迹点本身 —— 车头比后轴前伸 3.55 m，
  // 拿轨迹点算的话车会一头顶上去才停。
  std::vector<double> arc_lengths_m(geometry.size(), 0.0);
  for (std::size_t i = 1; i < geometry.size(); ++i) {
    arc_lengths_m[i] =
      arc_lengths_m[i - 1] +
      std::hypot(geometry[i].x_m - geometry[i - 1].x_m, geometry[i].y_m - geometry[i - 1].y_m);
  }

  std::size_t first_blocked = geometry.size();
  for (std::size_t i = 0; i < geometry.size(); ++i) {
    const Rectangle body = vehicle_body_at(geometry[i], params.lattice);
    double clearance_m = std::numeric_limits<double>::infinity();
    for (const Rectangle & obstacle : obstacles) {
      clearance_m = std::min(clearance_m, distance_m(body, obstacle));
    }
    if (clearance_m < params.lattice.safety_margin_m) {
      first_blocked = i;
      break;
    }
  }

  if (first_blocked == 0) {
    // 当前位置就已经太近了。这**本不该发生**（上一周期就该停住），
    // 但真发生时要给一条明确的零速轨迹，而不是让下游去猜。
    result.points = emergency_stop_at(geometry[0]);
    result.stop_clearance_m = 0.0;
    return result;
  }

  // 退到「最后一个安全点再往回 stop_margin_m」。
  const double target_stop_s_m = arc_lengths_m[first_blocked - 1] - params.stop_margin_m;
  std::size_t stop_index = 0;
  for (std::size_t i = 0; i < first_blocked; ++i) {
    if (arc_lengths_m[i] <= target_stop_s_m) {
      stop_index = i;
    }
  }

  if (stop_index < 1) {
    result.points = emergency_stop_at(geometry[0]);
    result.stop_clearance_m = 0.0;
    return result;
  }

  // **把几何截断在停车点，然后跑同一个 SpeedProfile。**
  // 它本来就在末点强制 v = 0 并向后扫描，于是减速自动按 max_decel 展开 ——
  // 不需要另写一套减速逻辑，也就不会有两套对"怎么减速"的理解。
  geometry.resize(stop_index + 1);
  // 行为停车点比静态停车点更早时取更早者（min，与 merge 同一条保守原则）。
  if (constraint != nullptr && constraint->stop_at_s_m.has_value()) {
    truncate_at_behavior_stop(&geometry, *constraint->stop_at_s_m - start.s_m);
    if (geometry.size() < 2) {
      result.points = emergency_stop_at(geometry.front());
      result.stop_clearance_m = 0.0;
      return result;
    }
  }
  // 停车轨迹的末点就是停车点 ⟹ 终点速度必须是 0。
  result.points = annotate_with_speed(geometry, params.speed, 0.0);

  const Rectangle stop_body = vehicle_body_at(geometry.back(), params.lattice);
  double stop_clearance_m = std::numeric_limits<double>::infinity();
  for (const Rectangle & obstacle : obstacles) {
    stop_clearance_m = std::min(stop_clearance_m, distance_m(stop_body, obstacle));
  }
  result.stop_clearance_m = stop_clearance_m;

  return result;
}

}  // namespace ads_planning
