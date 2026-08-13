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

#include "ads_prediction/lane_follow.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "ads_common/angles.hpp"
#include "ads_common/reference_line.hpp"
#include "ads_map/lane_sampling.hpp"

namespace ads_prediction
{

namespace
{

/// 一条车道链：车道图节点下标序列。
using Chain = std::vector<std::size_t>;

/// 枚举从 start 出发、折线总长足够覆盖 need_m 的车道链。
///
/// 深度优先，遇到多后继就分叉；无后继时链在此**如实结束**（截断由上层做）。
/// 假设数封顶 max_hypotheses：园区路口最多三岔，正常一个视界内 ≤ 3 条；
/// 超限时**保留先枚举到的**并停止分叉 —— 上限只是防病态图的护栏，
/// 真触发说明地图不是本项目的园区。
void EnumerateChains(
  const ads_map::LaneGraph & graph, std::size_t node, double first_span_m, double need_m,
  int max_hypotheses, Chain * current, std::vector<Chain> * out)
{
  current->push_back(node);
  const ads_map::LaneNode & lane_node = graph.node(node);
  const double remaining_m = need_m - first_span_m;
  const bool enough = remaining_m <= 0.0;
  if (enough || lane_node.successors.empty()) {
    if (static_cast<int>(out->size()) < max_hypotheses) {
      out->push_back(*current);
    }
    current->pop_back();
    return;
  }
  for (const std::size_t next : lane_node.successors) {
    if (static_cast<int>(out->size()) >= max_hypotheses) {
      break;
    }
    const ads_map::LaneNode & next_node = graph.node(next);
    // 后继车道贡献的弧长按参考线 s 差近似（车道弧长与它差 ≤14%，
    // 这里只决定「链要延多长」，多采一点无害，folyline 的真实长度由
    // ReferenceLine 说了算）。
    const double next_span_m = std::fabs(next_node.exit_s_m - next_node.entry_s_m);
    EnumerateChains(graph, next, first_span_m + next_span_m, need_m, max_hypotheses, current, out);
  }
  current->pop_back();
}

}  // namespace

std::vector<PredictedPath> PredictLaneFollow(
  const TargetSnapshot & target, const ads_map::LaneGraph & graph, const MotionModelParams & motion,
  const LaneFollowParams & params)
{
  const double speed = target.velocity.norm();
  if (speed <= 1e-9) {
    return {};  // 没有速度就没有"沿车道行驶"可言，交回调用方
  }
  const double motion_heading = std::atan2(target.velocity.y(), target.velocity.x());

  // ---- 归属：位置 + 运动方向（不是 yaw —— 180° 二义那条规则）--------------
  const auto match = graph.nearest_lane(target.position.x(), target.position.y(), motion_heading);
  if (!match.has_value() || match->distance_m > params.match_max_lateral_m) {
    return {};
  }

  // ---- 枚举车道链并逐条外推 ------------------------------------------------
  const double need_m = speed * motion.horizon_s + 2.0 * params.sample_step_m;
  const std::size_t start = graph.index_of(match->lane);
  const ads_map::LaneNode & start_node = graph.node(start);
  // 第一段从当前 s 到出口：正编号车道 exit < entry，方向由采样函数处理。
  const double first_span_m = std::fabs(start_node.exit_s_m - match->s_m);

  std::vector<Chain> chains;
  Chain scratch;
  EnumerateChains(graph, start, first_span_m, need_m, params.max_hypotheses, &scratch, &chains);
  if (chains.empty()) {
    return {};
  }

  std::vector<PredictedPath> paths;
  for (const Chain & chain : chains) {
    // 链 → 折线。第一段从 match->s_m 起；后续车道整段，跳过与上一段
    // 出口重合的首点（几何连续性由 test_lane_graph 的
    // EveryEdgeIsGeometricallyContinuous 保证）。
    std::vector<ads_common::Pose2D> poses;
    for (std::size_t i = 0; i < chain.size(); ++i) {
      const ads_map::LaneNode & lane_node = graph.node(chain[i]);
      const double from_s = (i == 0) ? match->s_m : lane_node.entry_s_m;
      const std::vector<ads_map::Pose2D> samples = ads_map::sample_lane_centerline(
        graph.road_map(), lane_node.id, from_s, lane_node.exit_s_m, params.sample_step_m);
      for (std::size_t k = (i == 0 ? 0 : 1); k < samples.size(); ++k) {
        poses.push_back({samples[k].x_m, samples[k].y_m, samples[k].heading_rad});
      }
    }
    if (poses.size() < 2) {
      continue;  // 车道尽头无以成线（目标恰在末点、又无后继）
    }
    const ads_common::ReferenceLine line(std::move(poses));

    // 带符号的初始横向偏移 + 运动方向门限（对向车道里逆行 → 拒绝）。
    const ads_common::PathProjection projection =
      line.project({target.position.x(), target.position.y(), motion_heading});
    if (
      std::fabs(ads_common::angle_diff(motion_heading, projection.heading_rad)) >
      params.match_max_heading_rad) {
      return {};  // 方向不符不是"这条链不行"，是"车道跟随不适用"——整体退恒速
    }
    const double d0 = projection.lateral_error_m;

    PredictedPath path;
    path.target_id = target.id;
    path.model = ModelKind::kLaneFollow;
    const int count = static_cast<int>(std::ceil(motion.horizon_s / motion.step_s - 1e-9));
    for (int i = 0; i <= count; ++i) {
      const double t = motion.horizon_s * static_cast<double>(i) / static_cast<double>(count);
      const double s = projection.s_m + speed * t;
      if (s > line.length_m()) {
        break;  // 车道走到头且无后继：**如实截断**（CP-P6-A ④）
      }
      const ads_common::PathPoint on_line = line.at(s);
      // 横向偏移线性衰减：几秒内回到中心线（车道保持的行为假设）。
      const double decay = std::max(0.0, 1.0 - t / params.lateral_decay_s);
      const double offset = d0 * decay;
      PredictedPoint point;
      point.t_s = t;
      point.position = Eigen::Vector2d(
        on_line.x_m - offset * std::sin(on_line.heading_rad),
        on_line.y_m + offset * std::cos(on_line.heading_rad));
      point.heading_rad = on_line.heading_rad;
      point.speed_mps = speed;
      point.sigma_along_m = motion.sigma_pos0_m + motion.sigma_speed_mps * t;
      point.sigma_cross_m =
        motion.sigma_pos0_m + 0.5 * motion.lane_follow_lateral_accel_mps2 * t * t;
      path.points.push_back(point);
    }
    if (!path.points.empty()) {
      paths.push_back(std::move(path));
    }
  }

  // 概率均分（v1 没有意图信息，SPEC 升级路径才有意图分类器）。
  for (PredictedPath & path : paths) {
    path.probability = 1.0 / static_cast<double>(paths.size());
  }
  return paths;
}

}  // namespace ads_prediction
