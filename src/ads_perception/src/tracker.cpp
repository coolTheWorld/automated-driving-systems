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

#include "ads_perception/tracker.hpp"

// ⚠️ `inverse()` 在 **Eigen/LU** 模块里，只 include <Eigen/Core> 会编过但**链不上**
//    （报 undefined reference to MatrixBase<...>::inverse）。
//    Eigen 的头没有扩展名，cpplint 按后缀把它归成 C 系统头 ——
//    所以必须排在 C++ 标准库**之前**，见 CLAUDE.md 的 lint 陷阱表。
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "ads_common/angles.hpp"
#include "ads_common/numeric_checks.hpp"
#include "ads_perception/hungarian.hpp"

namespace ads_perception
{

Tracker::Tracker(const TrackerParams & params) : params_(params)
{
  ads_common::RequireFinitePositive(
    params_.process_accel_stddev_mps2, "TrackerParams", "process_accel_stddev_mps2");
  ads_common::RequireFinitePositive(
    params_.measurement_stddev_m, "TrackerParams", "measurement_stddev_m");
  ads_common::RequireFinitePositive(params_.gate_chi_square, "TrackerParams", "gate_chi_square");
  ads_common::RequireFinitePositive(
    params_.anchor_shift_max_m, "TrackerParams", "anchor_shift_max_m");
  ads_common::RequireFinitePositive(
    params_.coast_max_speed_mps, "TrackerParams", "coast_max_speed_mps");
  if (params_.confirm_hits <= 0 || params_.max_misses <= 0) {
    throw std::invalid_argument("TrackerParams: confirm_hits 与 max_misses 必须为正");
  }
}

void Tracker::Predict(double dt_s)
{
  for (Track & track : tracks_) {
    ++track.age_frames;
  }
  Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
  transition(0, 2) = dt_s;
  transition(1, 3) = dt_s;

  // 过程噪声：**加速度白噪声**模型（constant-acceleration noise）。
  //
  // 一次积分给速度、两次给位置，所以 Q 的四个块分别是
  //   位置-位置 dt⁴/4、位置-速度 dt³/2、速度-速度 dt²，再乘 σ_a²。
  // ⚠️ **不要把 Q 写成对角阵**（那是最常见的简化）：位置与速度的误差
  //    在恒速模型下是**强相关**的（速度错了，位置就跟着以 dt 的速率错开），
  //    对角 Q 会让滤波器低估这个相关性，表现为速度估计长期偏小 ——
  //    而位置看起来完全正常，因为观测每帧都在把它拉回来。
  const double sigma_squared =
    params_.process_accel_stddev_mps2 * params_.process_accel_stddev_mps2;
  const double dt2 = dt_s * dt_s;
  const double dt3 = dt2 * dt_s;
  const double dt4 = dt3 * dt_s;
  Eigen::Matrix4d process_noise = Eigen::Matrix4d::Zero();
  for (int axis = 0; axis < 2; ++axis) {
    const int p = axis;
    const int v = axis + 2;
    process_noise(p, p) = dt4 / 4.0;
    process_noise(p, v) = dt3 / 2.0;
    process_noise(v, p) = dt3 / 2.0;
    process_noise(v, v) = dt2;
  }
  process_noise *= sigma_squared;

  for (Track & track : tracks_) {
    track.state = transition * track.state;
    track.covariance = transition * track.covariance * transition.transpose() + process_noise;
  }
}

bool Tracker::AxesConsistent(double detection_yaw_rad, double track_yaw_rad) const
{
  // 轴向的等价类是 π 而不是 2π（`a` 与 `a + π` 是同一条轴），
  // 所以先取最短角差再折到 [0, π/2]。
  // ⚠️ 直接用 |a − b| 的话，一条 0.01 rad 的轴和一条 3.13 rad 的轴
  //    会被判成差 3.12 —— 而它们其实是同一条。
  double difference = std::abs(ads_common::angle_diff(detection_yaw_rad, track_yaw_rad));
  if (difference > M_PI / 2.0) {
    difference = M_PI - difference;
  }
  return difference <= params_.extent_memory_max_axis_diff_rad;
}

Detection Tracker::AlignedDetection(const Detection & detection, const Track & track) const
{
  // ⚠️ **L-Shape 的「长/宽」命名与目标本身无关。** 它把两个范围排序，
  //    较大的那个叫 length（见 lshape_fit.cpp 的末尾几行）。于是对向车
  //    正对时可见范围是 1.80 × 0.08，"长轴"指的其实是**车宽**，
  //    轴向相对目标真实的长轴整个翻了 90°。
  //
  //    一个 a × b 的盒子（轴向 ψ）与一个 b × a 的盒子（轴向 ψ+90°）
  //    **是同一个盒子**。所以这里不是"猜"，是把检测改写成与航迹同一种说法。
  //
  // ⚠️ 不做这一步的后果是具体的：目标由远及近、侧面刚露出来的那一帧轴向翻转，
  //    AxesConsistent 判否 ⟹ 既不补全也不重锚 ⟹ 新息里混着一整跳
  //    ⟹ 判不进卡方门限 ⟹ 另起一条航迹。实测 ID 在 17–22 m 反复切换。
  double difference = ads_common::angle_diff(detection.yaw_rad, track.yaw_rad);
  // 轴向的等价类是 π：先折到 (−π/2, π/2]。
  if (difference > M_PI / 2.0) {
    difference -= M_PI;
  } else if (difference <= -M_PI / 2.0) {
    difference += M_PI;
  }
  if (std::abs(difference) <= M_PI / 4.0) {
    return detection;  // 已经是同一种说法
  }
  // 更接近 90°：换个说法。**只改描述，盒子一点没动。**
  Detection aligned = detection;
  aligned.yaw_rad = detection.yaw_rad + (difference > 0.0 ? -M_PI / 2.0 : M_PI / 2.0);
  std::swap(aligned.length_m, aligned.width_m);
  return aligned;
}

Eigen::Vector2d Tracker::AnchorOffset(
  double yaw_rad, double deficit_long_m, double deficit_lat_m, const Eigen::Vector2d & box_center,
  const Eigen::Vector2d & sensor_position)
{
  if (deficit_long_m <= 0.0 && deficit_lat_m <= 0.0) {
    return Eigen::Vector2d::Zero();
  }
  const Eigen::Vector2d axis_long(std::cos(yaw_rad), std::sin(yaw_rad));
  const Eigen::Vector2d axis_lat(-axis_long.y(), axis_long.x());
  const Eigen::Vector2d to_sensor = sensor_position - box_center;

  // 缺口补在**背离传感器**的一侧。投影恰好为 0 时不补：那说明传感器在这条轴的
  // 垂直平分面上，两边同样可见，没有任何依据偏向某一侧 —— 硬猜一边的话，
  // 目标从这一侧转到那一侧时中心会跳，而那正是本节要消除的东西。
  auto side = [](double projection) { return (projection > 0.0) - (projection < 0.0); };
  return -axis_long * (0.5 * std::max(0.0, deficit_long_m) * side(to_sensor.dot(axis_long))) -
         axis_lat * (0.5 * std::max(0.0, deficit_lat_m) * side(to_sensor.dot(axis_lat)));
}

std::optional<Eigen::Vector2d> Tracker::VehiclePriorPush(
  const Detection & detection, const Track & track, const Eigen::Vector2d & sensor_position) const
{
  // ---- 车辆形状先验（P8-S2b 立，P9-S5c 改成沿车头方向补先验盒）------------
  // 正对驶来的车从头到尾只露尾面：记忆里没有「长」（实测停在 1.9，正对
  // 中心沿视线偏 1.3–2.1 m —— P6 台账那条 2.2），而轴向记忆是**横向**
  // （L-Shape 的长轴在正对时是车宽）—— 既有的沿轴补全在这个情形连方向
  // 都是错的。观测与记忆都不知道的量，只能由先验给：把观测框补成
  // 4.4 × 1.8 的先验盒，缺口补在**背离传感器**的一侧（AnchorOffset 的几何）。
  //
  // 三层门控，各挡一类误用：
  //   未锚定时 —— 速度像车（≥2.5）×正对/背对（速度与视线 |cos|≥0.8）×
  //   展宽像车（记忆长边 ≥1.4）×记忆还不知道全长（< 先验）×观测深度 < 1.0
  //   （正对形态的签名）：园区 ODD 里满足这些的只有车；车头方向取速度方向；
  //   已锚定后 —— **不再看速度、不再看深度**（永久，见 Track::vehicle_prior_anchored），
  //   车头方向取 prior_heading_rad，推量随侧面露出连续收缩；
  //   任何时候 —— 推量 ≤ anchor_shift_max（先验盒最大缺口 2.05 在内）。
  //
  // ⚠️ 旧版沿**视线**推、且只在深度 < 1.0 时推，两个后果都实测到了（P9-S5c
  //    bag 回放）：斜视 10–20° 时中心横向偏 0.35–0.7 m；侧面刚露出（深度
  //    1.0→1.8）那帧推量从 1.7 跳到 0，新息一跳 1.3 m 判不进门 —— 每次由远
  //    及近在 18–19 m 处稳定换一次 ID。沿车头方向按「沿/横」延展各自补缺
  //    之后，13.3→13.4→13.6→14.0 s 四帧的补全中心与真值差 0.1 m 以内、连续。
  if (track.is_structure) {
    return std::nullopt;
  }
  const Eigen::Vector2d line_of_sight = detection.position - sensor_position;
  const double range_m = line_of_sight.norm();
  if (range_m < 1e-6) {
    return std::nullopt;
  }
  double heading_rad = track.prior_heading_rad;
  if (!track.vehicle_prior_anchored) {
    const double depth_m = std::min(detection.length_m, detection.width_m);
    if (depth_m >= 1.0) {
      return std::nullopt;
    }
    const double speed_mps = track.velocity().norm();
    const double memory_long_edge_m = std::max(track.length_m, track.width_m);
    if (
      speed_mps < params_.vehicle_prior_min_speed_mps ||
      memory_long_edge_m < params_.vehicle_prior_min_width_m ||
      memory_long_edge_m >= params_.vehicle_prior_length_m) {
      return std::nullopt;
    }
    const double cos_angle = std::abs(track.velocity().dot(line_of_sight)) / (speed_mps * range_m);
    if (cos_angle < 0.8) {
      return std::nullopt;
    }
    heading_rad = std::atan2(track.velocity().y(), track.velocity().x());
  }
  // 观测框在车头方向上的「沿 / 横」延展（框轴向与车头方向可以差任意角）。
  const double relative_rad = detection.yaw_rad - heading_rad;
  const double along_m = detection.length_m * std::abs(std::cos(relative_rad)) +
                         detection.width_m * std::abs(std::sin(relative_rad));
  const double across_m = detection.length_m * std::abs(std::sin(relative_rad)) +
                          detection.width_m * std::abs(std::cos(relative_rad));
  const Eigen::Vector2d push = AnchorOffset(
    heading_rad, params_.vehicle_prior_length_m - along_m, params_.vehicle_prior_width_m - across_m,
    detection.position, sensor_position);
  const double push_m = push.norm();
  if (push_m <= 1e-9 || push_m > params_.anchor_shift_max_m) {
    return std::nullopt;
  }
  return push;
}

Eigen::Vector2d Tracker::CompletedCenter(
  const Detection & detection, const Track & track, const Eigen::Vector2d & sensor_position) const
{
  // 结构物：不补全。「同一个目标露出更多」的前提对建筑片段不成立 ——
  // 它的"尺寸差"来自可见面滑移，不是遮挡（见 odd_max_length_m 的注释）。
  if (track.is_structure) {
    return detection.position;
  }
  // 车辆形状先验（P8-S2b）：正对/背对情形沿视线补全，见 VehiclePriorPush。
  if (const auto push = VehiclePriorPush(detection, track, sensor_position)) {
    return detection.position + *push;
  }

  const Detection aligned = AlignedDetection(detection, track);
  if (!AxesConsistent(aligned.yaw_rad, track.yaw_rad)) {
    return detection.position;
  }
  // 观测**小于**记忆：看不见的那截藏在背离传感器的一侧，把中心补回去。
  const Eigen::Vector2d offset = AnchorOffset(
    aligned.yaw_rad, track.length_m - aligned.length_m, track.width_m - aligned.width_m,
    aligned.position, sensor_position);
  // ⚠️ 位移超出物理上限 ⟹ "同一个目标露出更多/被挡住一截"的前提不成立，
  //    这一半的补全退出、用原始中心 —— 让原始新息直面卡方门限。
  //    （重锚那一半在 TrackAnchorShift 里有同一道闸，各封各的位移。）
  //    理由与实测见 TrackerParams::anchor_shift_max_m。
  if (offset.norm() > params_.anchor_shift_max_m) {
    return detection.position;
  }
  return aligned.position + offset;
}

Eigen::Vector2d Tracker::TrackAnchorShift(
  const Detection & detection, const Track & track, const Eigen::Vector2d & sensor_position) const
{
  // 结构物：不重锚（理由同 CompletedCenter 的旁路）。
  if (track.is_structure) {
    return Eigen::Vector2d::Zero();
  }
  // 先验锚定的车：中心已经是 4.4 × 1.8 先验盒的几何中心，观测露出更多不是
  // 「此前按更小的盒子锚定」—— 不重锚。（P9-S5c：旧版在侧面刚露出那帧按记忆
  // 宽度增量把航迹横向再挪 0.46 m，与先验补全叠加，正是判不进门的另一半。）
  if (track.vehicle_prior_anchored) {
    return Eigen::Vector2d::Zero();
  }
  const Detection aligned = AlignedDetection(detection, track);
  if (!AxesConsistent(aligned.yaw_rad, track.yaw_rad)) {
    return Eigen::Vector2d::Zero();
  }
  // 观测**大于**记忆：目标露出了更多，说明航迹此前那个位置是按**更小的盒子**
  // 锚定的。近边没动，中心却因为盒子变长而往远处挪了半个增量 ——
  // 那不是运动，是**重新锚定**。不补的话卡尔曼会把它读成速度。
  //
  // ⚠️ 这一条与 CompletedCenter 是**同一件事的两半**，缺一个就只修一半：
  //    实测目标由远及近时长度在一帧内涨 0.87 m ⟹ 中心挪 0.44 m ⟹
  //    0.1 s 一帧 = **4.4 m/s 假速度**（实测峰值 4.641，真值 4.0）。
  //    只做 CompletedCenter 时速度判据在 0.967 / 1.109 之间抖（判据 1.0）。
  const Eigen::Vector2d offset = AnchorOffset(
    aligned.yaw_rad, aligned.length_m - track.length_m, aligned.width_m - track.width_m,
    aligned.position, sensor_position);
  // ⚠️ 与 CompletedCenter 同一道物理上限，各封各的位移：超限的那一半
  //    退出，没被补偿的尺寸差就原样留在新息里、直面卡方门限。
  //    墙沿碎片 0.03 → 6 m 的"重锚"位移 3 m 正是把 5.9 m 外另一个碎片
  //    拉进门限的虫洞。见 TrackerParams::anchor_shift_max_m。
  if (offset.norm() > params_.anchor_shift_max_m) {
    return Eigen::Vector2d::Zero();
  }
  return offset;
}

void Tracker::Associate(
  const std::vector<Detection> & detections, const Eigen::Vector2d & sensor_position,
  std::vector<int> * assignment)
{
  assignment->assign(tracks_.size(), -1);
  if (tracks_.empty() || detections.empty()) {
    return;
  }

  const double measurement_variance = params_.measurement_stddev_m * params_.measurement_stddev_m;

  std::vector<std::vector<double>> cost(
    tracks_.size(), std::vector<double>(detections.size(), kForbiddenCost));
  for (std::size_t t = 0; t < tracks_.size(); ++t) {
    // 新息协方差 S = H P Hᵀ + R。H 取位置两行，所以 HPHᵀ 就是 P 的左上 2×2。
    Eigen::Matrix2d innovation_covariance = tracks_[t].covariance.topLeftCorner<2, 2>();
    innovation_covariance(0, 0) += measurement_variance;
    innovation_covariance(1, 1) += measurement_variance;
    const Eigen::Matrix2d information = innovation_covariance.inverse();

    for (std::size_t d = 0; d < detections.size(); ++d) {
      // ⚠️ 门限必须用**补全后**的位置。航迹预测的是补全中心，而检测给的是
      //    可见部分的中心，两者相差可达一个半车长 —— 拿原始中心去判，
      //    目标一靠近就判不进门 ⟹ 关联失败 ⟹ 航迹重建 ⟹ **ID 跳变**。
      //    实测正是在 3.5 m 处连着跳了两次（感知长度 1.56 ↔ 4.41）。
      // ⚠️ 两边都要换算到**同一个锚点**再比：观测比记忆小就补全观测，
      //    观测比记忆大就重新锚定航迹。少做一半的话，目标由远及近露出侧面
      //    的那几帧新息里混着一个 1 m 量级的"重新锚定"量，会判不进门 ⟹ ID 跳变。
      const Eigen::Vector2d measurement =
        CompletedCenter(detections[d], tracks_[t], sensor_position);
      Eigen::Vector2d anchored =
        tracks_[t].position() + TrackAnchorShift(detections[d], tracks_[t], sensor_position);
      // 先验首次生效的那一帧：量测已按先验补全，航迹还锚在尾面 ——
      // 不给 anchored 加同一个 push 的话新息平白多 2 m，判不进门 ⟹ 断裂
      // （首版实测：一条正对车断成 6 条航迹）。
      if (!tracks_[t].vehicle_prior_anchored) {
        if (const auto push = VehiclePriorPush(detections[d], tracks_[t], sensor_position)) {
          anchored += *push;
        }
      }
      const Eigen::Vector2d innovation = measurement - anchored;
      const double mahalanobis = innovation.transpose() * information * innovation;
      // ⚠️ 门限用**卡方**。这里可以这么做，因为协方差是滤波器自己按已知的
      //    Q/R 递推出来的 —— 与 P4 那个未标定的 NDT 协方差不同
      //    （那里只能用固定阈值，见 docs/modules/localization.md §10.6）。
      //
      // ⚠️ 必须先判有限性再比较：NaN 参与任何比较都返回 false，
      //    于是 `mahalanobis <= gate` 对 NaN 恒为假 —— 恰好等于"拒绝"，
      //    但那是碰巧对了。显式判，免得哪天有人把条件反过来写。
      if (std::isfinite(mahalanobis) && mahalanobis <= params_.gate_chi_square) {
        cost[t][d] = mahalanobis;
      }
    }
  }

  *assignment = SolveAssignment(cost);
}

void Tracker::ApplyUpdate(
  const Detection & detection, const Eigen::Vector2d & sensor_position, Track * track)
{
  // ⚠️ 顺序要紧：补全与轴向判断都要用**更新前**的 track（它才有记忆里的尺寸
  //    与上一帧的轴向）。先把 yaw/尺寸写回去的话，补全量恒为零，整节失效
  //    —— 而且不会报错，只是安静地退回旧行为。
  const Detection aligned = AlignedDetection(detection, *track);
  const bool axes_consistent = AxesConsistent(aligned.yaw_rad, track->yaw_rad);
  const Eigen::Vector2d measurement = CompletedCenter(detection, *track, sensor_position);
  // 重新锚定：只挪**位置**，不动速度也不动协方差 —— 这是一次坐标改写，
  // 不是一次观测。改协方差等于宣称"我对位置更不确定了"，而事实相反。
  track->state.head<2>() += TrackAnchorShift(detection, *track, sensor_position);
  // 车辆先验的首次锚定：同一条「坐标改写不是观测」的规矩。此后永久
  // 按车补全（速度掉门控也不回跳 —— 回跳一次 = 一次 2 m 的假新息）。
  if (!track->vehicle_prior_anchored) {
    if (const auto push = VehiclePriorPush(detection, *track, sensor_position)) {
      track->state.head<2>() += *push;
      track->vehicle_prior_anchored = true;
    }
  }
  // 先验档的车头朝向跟速度走（停住后保持最近有效值，见 hpp）。
  if (track->vehicle_prior_anchored && track->velocity().norm() > 0.5) {
    track->prior_heading_rad = std::atan2(track->velocity().y(), track->velocity().x());
  }

  const double measurement_variance = params_.measurement_stddev_m * params_.measurement_stddev_m;

  Eigen::Matrix<double, 2, 4> observation = Eigen::Matrix<double, 2, 4>::Zero();
  observation(0, 0) = 1.0;
  observation(1, 1) = 1.0;

  Eigen::Matrix2d innovation_covariance = observation * track->covariance * observation.transpose();
  innovation_covariance(0, 0) += measurement_variance;
  innovation_covariance(1, 1) += measurement_variance;

  const Eigen::Matrix<double, 4, 2> gain =
    track->covariance * observation.transpose() * innovation_covariance.inverse();
  const Eigen::Vector2d innovation = measurement - track->position();
  track->state += gain * innovation;

  // Joseph 形式：(I−KH) P (I−KH)ᵀ + K R Kᵀ。
  // 结构上就是对称半正定的，代价是多一次 4×4 乘法 —— 与 ESKF 那边同一个理由。
  const Eigen::Matrix4d factor = Eigen::Matrix4d::Identity() - gain * observation;
  Eigen::Matrix2d measurement_noise = Eigen::Matrix2d::Identity() * measurement_variance;
  track->covariance =
    factor * track->covariance * factor.transpose() + gain * measurement_noise * gain.transpose();

  // ---- 结构物档（P8-S2b）----
  // 判定用**观测**长边的连续超限（不用记忆 —— 记忆是单调 max，真车一次
  // 瞬间并簇就永久污染；连续 5 帧把并簇滤掉）。一旦成立永久生效。
  const double observed_long_edge_m = std::max(detection.length_m, detection.width_m);
  track->big_observation_streak =
    observed_long_edge_m > params_.odd_max_length_m ? track->big_observation_streak + 1 : 0;
  if (track->big_observation_streak >= params_.structure_confirm_frames) {
    track->is_structure = true;
  }
  // ⚠️ 速度的冻结在**报告层**（Track::reported_velocity），不在状态层。
  //    两版失败史（都实测过）：确认后才清 state → 假速度全在确认窗前几帧
  //    积累并已发布（confirm_hits=3 早于结构确认 2 帧，画像原样 89 条）；
  //    单帧观测大框就冻 state → 滑移观测 vs 静止预测的新息累积到 ~1.3 m
  //    判不进门，一条墙断成两条航迹。矛盾的本质：可见面滑移是**真实的
  //    测量位移**，关联需要内部速度去跟它，但它不是目标的运动、不许出门。
  //    KF 内部照常积（关联健康），出口处按 reported_velocity 清零。

  track->yaw_rad = aligned.yaw_rad;
  // 最近观测尺寸（结构物的发布出口用它 —— 见 node 的说明）。
  track->last_observed_length_m = detection.length_m;
  track->last_observed_width_m = detection.width_m;
  if (axes_consistent && observed_long_edge_m <= params_.odd_max_length_m) {
    // 记住已观测到的最大值：**"我至少看到过这么大"是一个物理上单调的事实**，
    // 而"这一帧看到多少"随遮挡起伏。上限见 max_extent_m 的注释。
    //
    // ⚠️ **超 ODD 的观测帧不喂记忆**（P8-S2b 第四刀）：
    //    「看到过这么大」对真目标成立的前提是那一帧看到的**就是它** ——
    //    并簇/墙沿碎片的 6 m 帧不满足。放进去的后果实测过两种：
    //    真车被一次 2 帧并簇永久污染成 6.0 记忆；碎片航迹的记忆膨胀成
    //    6×6 **虚胖框**，几何上盖到 10 m 外的路面，行为层把它当成
    //    挡路的前车（CP-P7-B ⑨ 照印里那个稳定复现的幻影 FOLLOW）。
    //    记忆由此永远 ≤ odd_max —— ODD 内没有更大的**目标**。
    track->length_m = std::min(params_.max_extent_m, std::max(track->length_m, aligned.length_m));
    track->width_m = std::min(params_.max_extent_m, std::max(track->width_m, aligned.width_m));
    // ⚠️ 归一化成 length ≥ width，**必须做**：合并之后"宽"可能已经超过"长"
    //    （远处只看得见车头 ⟹ 记忆是 1.8 × 0.1，等侧面露出来"宽"长到 4.4）。
    //    不归一化的话 ClassifyBySize 拿 length = 1.8 去比 vehicle_min_length = 2.5
    //    ⟹ 一辆量得完全正确的车被判成 UNKNOWN。ResolveHeading 同样假定
    //    yaw 是长轴 —— 不换的话 180° 消歧会绕着**车宽**那条轴做，必然错。
    //    位置此前已经锚定好了，这里只是换个说法，盒子没动。
    if (track->width_m > track->length_m) {
      std::swap(track->length_m, track->width_m);
      track->yaw_rad = ads_common::normalize_angle(track->yaw_rad + M_PI / 2.0);
    }
  } else {
    // 换算之后仍然差 20° 以上 ⟹ 目标是**真的转了**（或者拟合坏了），
    // 记忆里的尺寸不再对应同一条轴。此时丢掉记忆重新开始，宁可退回旧行为。
    // ⚠️ 注意这里已经**不是**在处理 90° 翻转了 —— 那一档由 AlignedDetection
    //    化解掉了。留着这条分支是给"目标真的在转弯"用的。
    //
    // ⚠️ 存**原始检测**的尺寸与轴向，不是 aligned 的（2026-08-12 复检修复）：
    //    aligned 可能已被换算成"宽 ≥ 长"（它按**旧**轴向的约定改写），直接
    //    存回去会破坏「length ≥ width 且 yaw 是长轴」的不变式 —— 轴差落在
    //    (45°, 70°) 窗口时，一辆尺寸量得完全正确的车被 ClassifyBySize 判成
    //    UNKNOWN（length 1.7 < 2.5 且 width 4.3 > 2.8），ResolveHeading 还会
    //    绕短轴消歧、差 90° 却标 resolved。原始检测由 L-Shape 保证
    //    length ≥ width，天然满足不变式。钳位同样不能少。
    track->yaw_rad = detection.yaw_rad;
    track->length_m = std::min(params_.max_extent_m, detection.length_m);
    track->width_m = std::min(params_.max_extent_m, detection.width_m);
  }
  track->height_m = detection.height_m;
  ++track->hits;
  track->consecutive_misses = 0;
  track->occluded_misses = 0;
  track->last_observed_position = track->position();
  // 命中即解除"被取代"：能配上检测就说明它不是鬼影（或者鬼影判错了），
  // 恢复遮挡滑行的资格。
  track->superseded = false;
  if (track->hits >= params_.confirm_hits) {
    track->confirmed = true;
  }
  ResolveHeading(track);
}

void Tracker::SupersedeReanchoredTrack(const Track & newborn)
{
  // 兑现出生时记下的候选：新航迹刚确认，而它出生时旁边那条旧航迹**从那时起
  // 一直丢失** ⟹ 两者是同一个目标的前后两个框，旧的那条不许再按"被遮挡"滑行。
  // 三个条件缺一不可：
  //   · 旧航迹还活着且这一帧仍未命中（命中过的话 superseded 早被清掉/根本不该判）；
  //   · 旧航迹丢失的帧数 ≥ 新航迹的年龄（丢失从新航迹出生前就开始，中途没接上）；
  //   · 新航迹在 max_misses 内确认 —— 更晚确认的话旧航迹早该被普通未命中删掉，
  //     它还活着只能是**别的**遮挡者在给它续命，那是真遮挡，不归这条管。
  if (newborn.reanchor_of_id == 0 || newborn.age_frames >= params_.max_misses) {
    return;
  }
  for (Track & old : tracks_) {
    if (old.id != newborn.reanchor_of_id || old.id == newborn.id) {
      continue;
    }
    if (
      old.consecutive_misses + old.occluded_misses >= newborn.age_frames &&
      old.consecutive_misses + old.occluded_misses > 0) {
      old.superseded = true;
    }
    return;
  }
}

void Tracker::ResolveHeading(Track * track) const
{
  const Eigen::Vector2d velocity = track->velocity();
  const double speed = velocity.norm();
  if (!(speed >= params_.heading_min_speed_mps)) {
    // ⚠️ 太慢就**如实说不知道**，不要拿噪声去定朝向。
    //    静止目标的车头方向物理上无解 —— 猜错的概率 50%，
    //    而错了之后 P6 会预测出一条逆行轨迹（见 lshape_fit.hpp 的文件头）。
    track->heading_resolved = false;
    return;
  }

  // L-Shape 给的是**轴向**（[0, π)），车头是它或它 + π。
  // 用速度方向挑那个夹角 < 90° 的。
  const double velocity_heading = std::atan2(velocity.y(), velocity.x());
  const double candidate_a = track->yaw_rad;
  const double candidate_b = track->yaw_rad + M_PI;
  const double error_a = std::abs(ads_common::angle_diff(candidate_a, velocity_heading));
  const double error_b = std::abs(ads_common::angle_diff(candidate_b, velocity_heading));
  track->heading_rad = ads_common::normalize_angle(error_a <= error_b ? candidate_a : candidate_b);
  track->heading_resolved = true;
}

bool Tracker::IsOccludedByAnotherTrack(
  const Track & track, const Eigen::Vector2d & sensor_position) const
{
  // 视线段：传感器 → 航迹预测位置。与任何一条**别的已确认**航迹的盒子相交
  // 即为被遮挡。只认已确认的遮挡者 —— 拿一个未确认的噪点簇当"挡住我的东西"
  // 会让噪点间接续命别的航迹。
  const Eigen::Vector2d target = track.position();
  for (const Track & other : tracks_) {
    if (other.id == track.id || !other.confirmed) {
      continue;
    }
    // 线段-OBB 相交（slab 法，精确）：转到盒子自身坐标系做 2D AABB 裁剪。
    const double cos_yaw = std::cos(other.yaw_rad);
    const double sin_yaw = std::sin(other.yaw_rad);
    const Eigen::Vector2d to_p0 = sensor_position - other.position();
    const Eigen::Vector2d to_p1 = target - other.position();
    const Eigen::Vector2d p0(
      cos_yaw * to_p0.x() + sin_yaw * to_p0.y(), -sin_yaw * to_p0.x() + cos_yaw * to_p0.y());
    const Eigen::Vector2d p1(
      cos_yaw * to_p1.x() + sin_yaw * to_p1.y(), -sin_yaw * to_p1.x() + cos_yaw * to_p1.y());
    const Eigen::Vector2d direction = p1 - p0;
    const double half[2] = {0.5 * other.length_m, 0.5 * other.width_m};
    double t_min = 0.0;
    double t_max = 1.0;
    bool hit = true;
    for (int axis = 0; axis < 2 && hit; ++axis) {
      if (std::abs(direction[axis]) < 1e-9) {
        if (std::abs(p0[axis]) > half[axis]) {
          hit = false;  // 平行于该轴且在板外 —— 不可能相交
        }
        continue;
      }
      double t1 = (-half[axis] - p0[axis]) / direction[axis];
      double t2 = (half[axis] - p0[axis]) / direction[axis];
      if (t1 > t2) {
        std::swap(t1, t2);
      }
      t_min = std::max(t_min, t1);
      t_max = std::min(t_max, t2);
      if (t_min > t_max) {
        hit = false;
      }
    }
    if (hit) {
      return true;
    }
  }
  return false;
}

void Tracker::MergeDuplicateTracks()
{
  // O(n²)。园区场景一帧十几条航迹，而整条感知流水线实测才 5 ms ——
  // 这里的开销可以忽略，不值得为它引入空间索引。
  std::vector<char> removed(tracks_.size(), 0);
  for (std::size_t a = 0; a < tracks_.size(); ++a) {
    if (removed[a] != 0) {
      continue;
    }
    for (std::size_t b = a + 1; b < tracks_.size(); ++b) {
      if (removed[b] != 0) {
        continue;
      }
      if ((tracks_[a].position() - tracks_[b].position()).norm() > params_.merge_distance_m) {
        continue;
      }
      // ⚠️ 速度一致性**只在两条都成熟时**才判（hits ≥ mature_hits）。
      //    新建的航迹初速恒为 0，拿它去比一条 −4 m/s 的确认航迹必然超门限 ⟹
      //    重复永远合不掉，而"新建的那条"正是重复最常见的来源。P9-S5c 把
      //    「已确认」改成「成熟」：刚凑够 3 帧确认的航迹速度还没收敛（实测差
      //    2 m/s），一条 0.15–0.5 m 外的车身碎片航迹确认那一帧起与正主并存
      //    1–2 帧、评测在两者间摆 ⟹ 每次记 2 次 ID 切换。年轻的那条只按距离
      //    并：ODD 里两个目标中心最近 1.75 m（车与行人），1.0 m 内的年轻航迹
      //    只能是重复。理由见 merge_speed_mps / mature_hits 的注释。
      if (tracks_[a].hits >= params_.mature_hits && tracks_[b].hits >= params_.mature_hits) {
        if ((tracks_[a].velocity() - tracks_[b].velocity()).norm() > params_.merge_speed_mps) {
          continue;  // 位置近但速度截然不同 ⟹ 是擦身而过的两个目标
        }
      }
      // 留命中多的那条；打平留 id 小的（更老 ⟹ 下游积累的历史更长）。
      const bool keep_b = tracks_[b].hits > tracks_[a].hits ||
                          (tracks_[b].hits == tracks_[a].hits && tracks_[b].id < tracks_[a].id);
      // ⚠️ **不合并尺寸**：两条航迹各自的 length/width 是在各自的轴向约定下
      //    记的，直接取 max 可能把"长"和"宽"混起来。留下来的那条命中更多、
      //    记忆更可信，用它的就好。少写十行，也少一处会悄悄出错的地方。
      removed[keep_b ? a : b] = 1;
      if (keep_b) {
        break;  // a 已被并掉，不必再与后面的比
      }
    }
  }

  // ⚠️ 显式重建而不是 `remove_if` + 捕获一个自增下标：后者依赖谓词被
  //    "按顺序、每个元素恰好一次"调用。标准确实这么要求，但那种写法一眼看不出
  //    对错，而这里多写三行就完全显然。**能跑但脆的写法不值得省这三行。**
  std::vector<Track> kept;
  kept.reserve(tracks_.size());
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (removed[i] == 0) {
      kept.push_back(tracks_[i]);
    }
  }
  tracks_.swap(kept);
}

void Tracker::Update(
  const std::vector<Detection> & detections, double dt_s, const Eigen::Vector2d & sensor_position)
{
  ads_common::RequireFinitePositive(dt_s, "Tracker::Update", "dt_s");
  ads_common::RequireFinite(sensor_position.x(), "Tracker::Update", "sensor_position.x");
  ads_common::RequireFinite(sensor_position.y(), "Tracker::Update", "sensor_position.y");
  for (const Detection & detection : detections) {
    ads_common::RequireFinite(detection.position.x(), "Tracker::Update", "detection.x");
    ads_common::RequireFinite(detection.position.y(), "Tracker::Update", "detection.y");
    ads_common::RequireFinite(detection.yaw_rad, "Tracker::Update", "detection.yaw");
    // 尺寸参与补全的算术，非有限值会静默地把中心推到 NaN，
    // 而 NaN 位置在马氏距离里被 isfinite 拦下 ⟹ 表现是"目标突然全丢"。
    ads_common::RequireFinite(detection.length_m, "Tracker::Update", "detection.length_m");
    ads_common::RequireFinite(detection.width_m, "Tracker::Update", "detection.width_m");
    ads_common::RequireFinite(detection.height_m, "Tracker::Update", "detection.height_m");
  }

  Predict(dt_s);

  std::vector<int> assignment;
  Associate(detections, sensor_position, &assignment);

  std::vector<char> detection_used(detections.size(), 0);
  for (std::size_t t = 0; t < tracks_.size(); ++t) {
    if (assignment[t] >= 0) {
      const bool was_confirmed = tracks_[t].confirmed;
      ApplyUpdate(detections[assignment[t]], sensor_position, &tracks_[t]);
      detection_used[assignment[t]] = 1;
      if (!was_confirmed && tracks_[t].confirmed) {
        // 刚确认：若它出生时是某条正在丢失的旧航迹的重锚候选，此刻兑现。
        SupersedeReanchoredTrack(tracks_[t]);
      }
    } else if (
      tracks_[t].confirmed && tracks_[t].hits >= params_.mature_hits && !tracks_[t].superseded &&
      tracks_[t].velocity().norm() <= params_.coast_max_speed_mps &&
      IsOccludedByAnotherTrack(tracks_[t], sensor_position) &&
      tracks_[t].occluded_misses < params_.max_occluded_misses) {
      // 被别的已确认航迹挡住视线：未命中**不计入删除计数**，按恒速滑行。
      // 看不见 ≠ 消失 —— 这一条解决「遮挡 0.5–1.8 s vs 删除窗口 0.5 s」的
      // 设计冲突，见 TrackerParams::max_occluded_misses 的推导。
      // 只对已确认**且成熟**（hits ≥ mature_hits）的航迹滑行：未确认的本来就还
      // 不算"存在"；刚确认的速度还是噪声，滑行是按速度外推 3 s —— 外推噪声就是
      // 制造鬼影（Gazebo 实测：车身碎片 3 帧确认后躲在车框后按 5.5 m/s 的假速度
      // 滑进自车车道）。见 TrackerParams::mature_hits。
      // ⚠️ 还要速度在 ODD 物理上限之内：滑行是在**外推**状态，外推一个
      //    物理上不可能的状态就是在制造幽灵（实测 11.9 m/s 的假航迹
      //    靠滑行飞越 35 m 横穿车道）。见 TrackerParams::coast_max_speed_mps。
      // ⚠️ 还不能是**被自己的重锚航迹取代**的航迹（superseded，见下面
      //    "U 转鬼影"那段）：挡住它视线的"另一条航迹"就是它自己换了个框，
      //    给它 3 s 滑行等于放出一个 4.4×1.8 的鬼影贴车道带走。
      ++tracks_[t].occluded_misses;
    } else {
      ++tracks_[t].consecutive_misses;
      // ⚠️ 未命中时**不重置** hits —— 确认判据用的是**累计**命中而不是连续。
      //    这一条直接来自 S1 的实测：目标在连续帧之间闪烁（命中率 33–74%），
      //    要求连续 3 帧命中的话概率只有 12.5%，目标可能到消失都没被确认。
      //    见 tracker.hpp 的文件头。
    }
  }

  // 删除连续未命中太久的航迹。
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const Track & track) { return track.consecutive_misses >= params_.max_misses; }),
    tracks_.end());

  // 没配上的检测各起一条新航迹（未确认）。
  for (std::size_t d = 0; d < detections.size(); ++d) {
    if (detection_used[d] != 0) {
      continue;
    }
    Track track;
    track.id = next_id_++;
    track.state.head<2>() = detections[d].position;
    // 初速度取 0，但**协方差给大**：新航迹对速度一无所知，
    // 给小协方差等于宣称"我确定它是静止的"，随后几帧的观测会被压住，
    // 速度要好几帧才涨起来 —— 而那期间朝向消歧一直是 false。
    track.covariance = Eigen::Matrix4d::Identity();
    track.covariance(0, 0) = params_.measurement_stddev_m * params_.measurement_stddev_m;
    track.covariance(1, 1) = track.covariance(0, 0);
    track.covariance(2, 2) = 25.0;  // (5 m/s)²，覆盖园区目标的速度范围
    track.covariance(3, 3) = 25.0;
    track.yaw_rad = detections[d].yaw_rad;
    track.length_m = detections[d].length_m;
    track.width_m = detections[d].width_m;
    track.height_m = detections[d].height_m;
    track.hits = 1;
    track.confirmed = params_.confirm_hits <= 1;
    // ---- U 转鬼影（P9-S3 Gazebo 实测，P9-S5c 修）：记下"我可能是谁的重锚" ----
    // 目标原地掉头 / 露出另一面时，L-Shape 轴向翻转 + 先验锚定能让框中心一步
    // 跳 ~2 m：旧航迹配不上（卡方门）、这里起一条新航迹；旧航迹继续外推，
    // 预测位置落在新盒子后面 ⟹ 被 §6.5 判"遮挡"⟹ 滑行 3 s ⟹ 鬼影。
    // 真遮挡与重锚的可分签名：**遮挡者是刚出生的，且出生时离正在丢失的旧航迹
    // ≤ anchor_shift_max（同一目标换个框能造成的最大中心位移）**。这一帧只记
    // 候选，兑现放在新航迹**确认**那一帧（噪点簇活不到确认，不许它替别人判死刑）。
    double nearest_m = params_.anchor_shift_max_m;
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
      if (!tracks_[t].confirmed || assignment[t] >= 0) {
        continue;
      }
      const double dist_m = (tracks_[t].position() - detections[d].position).norm();
      if (dist_m <= nearest_m) {
        nearest_m = dist_m;
        track.reanchor_of_id = tracks_[t].id;
      }
    }
    tracks_.push_back(track);
  }

  // ⚠️ 放在**建完新航迹之后**：重复最常见的来源就是"这一帧没配上、于是新建
  //    了一条"，而那条新航迹此刻就叠在老航迹上。放在前面的话它要多活一帧，
  //    而只要它凑够 confirm_hits 就会被发给下游 —— 规划于是看到两个障碍物。
  MergeDuplicateTracks();
}

std::vector<Track> Tracker::ConfirmedTracks() const
{
  std::vector<Track> confirmed;
  for (const Track & track : tracks_) {
    if (!track.confirmed) {
      continue;
    }
    // ⚠️ 年轻航迹的未命中帧**照发**，但位置用 Track::published_position()（钉在最近
    //    一次观测的位置，不外推）—— 见那里的说明。曾试过"年轻航迹未命中帧不发"：
    //    鬼影是没了，但 U 转后那条年轻的正主丢一两帧就从发布里消失，评测配到旁边的
    //    碎片航迹上 ⟹ ID 切换 +2、检测率 −2%。连续性要保，外推不能给。
    confirmed.push_back(track);
  }
  return confirmed;
}

}  // namespace ads_perception
