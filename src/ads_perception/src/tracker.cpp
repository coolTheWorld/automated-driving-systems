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
  if (params_.confirm_hits <= 0 || params_.max_misses <= 0) {
    throw std::invalid_argument("TrackerParams: confirm_hits 与 max_misses 必须为正");
  }
}

void Tracker::Predict(double dt_s)
{
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

Eigen::Vector2d Tracker::CompletedCenter(
  const Detection & detection, const Track & track, const Eigen::Vector2d & sensor_position) const
{
  if (!AxesConsistent(detection.yaw_rad, track.yaw_rad)) {
    return detection.position;
  }
  // 观测**小于**记忆：看不见的那截藏在背离传感器的一侧，把中心补回去。
  return detection.position + AnchorOffset(
                                detection.yaw_rad, track.length_m - detection.length_m,
                                track.width_m - detection.width_m, detection.position,
                                sensor_position);
}

Eigen::Vector2d Tracker::TrackAnchorShift(
  const Detection & detection, const Track & track, const Eigen::Vector2d & sensor_position) const
{
  if (!AxesConsistent(detection.yaw_rad, track.yaw_rad)) {
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
  return AnchorOffset(
    detection.yaw_rad, detection.length_m - track.length_m, detection.width_m - track.width_m,
    detection.position, sensor_position);
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
      const Eigen::Vector2d anchored =
        tracks_[t].position() + TrackAnchorShift(detections[d], tracks_[t], sensor_position);
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
  const bool axes_consistent = AxesConsistent(detection.yaw_rad, track->yaw_rad);
  const Eigen::Vector2d measurement = CompletedCenter(detection, *track, sensor_position);
  // 重新锚定：只挪**位置**，不动速度也不动协方差 —— 这是一次坐标改写，
  // 不是一次观测。改协方差等于宣称"我对位置更不确定了"，而事实相反。
  track->state.head<2>() += TrackAnchorShift(detection, *track, sensor_position);

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

  track->yaw_rad = detection.yaw_rad;
  if (axes_consistent) {
    // 记住已观测到的最大值：**"我至少看到过这么大"是一个物理上单调的事实**，
    // 而"这一帧看到多少"随遮挡起伏。上限见 max_extent_m 的注释。
    track->length_m = std::min(params_.max_extent_m, std::max(track->length_m, detection.length_m));
    track->width_m = std::min(params_.max_extent_m, std::max(track->width_m, detection.width_m));
  } else {
    // 轴向翻了 ⟹ 记忆里的"长"和当帧的"长"根本不是同一条轴，合并会沿错误的
    // 轴外扩。此时**丢掉记忆重新开始**，而不是硬凑 —— 宁可退回旧行为。
    track->length_m = detection.length_m;
    track->width_m = detection.width_m;
  }
  track->height_m = detection.height_m;
  ++track->hits;
  track->consecutive_misses = 0;
  if (track->hits >= params_.confirm_hits) {
    track->confirmed = true;
  }
  ResolveHeading(track);
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
  }

  Predict(dt_s);

  std::vector<int> assignment;
  Associate(detections, sensor_position, &assignment);

  std::vector<char> detection_used(detections.size(), 0);
  for (std::size_t t = 0; t < tracks_.size(); ++t) {
    if (assignment[t] >= 0) {
      ApplyUpdate(detections[assignment[t]], sensor_position, &tracks_[t]);
      detection_used[assignment[t]] = 1;
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
    tracks_.push_back(track);
  }
}

std::vector<Track> Tracker::ConfirmedTracks() const
{
  std::vector<Track> confirmed;
  for (const Track & track : tracks_) {
    if (track.confirmed) {
      confirmed.push_back(track);
    }
  }
  return confirmed;
}

}  // namespace ads_perception
