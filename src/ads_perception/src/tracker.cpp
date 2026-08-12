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

void Tracker::Associate(const std::vector<Detection> & detections, std::vector<int> * assignment)
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
      const Eigen::Vector2d innovation = detections[d].position - tracks_[t].position();
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

void Tracker::ApplyUpdate(const Detection & detection, Track * track)
{
  const double measurement_variance = params_.measurement_stddev_m * params_.measurement_stddev_m;

  Eigen::Matrix<double, 2, 4> observation = Eigen::Matrix<double, 2, 4>::Zero();
  observation(0, 0) = 1.0;
  observation(1, 1) = 1.0;

  Eigen::Matrix2d innovation_covariance = observation * track->covariance * observation.transpose();
  innovation_covariance(0, 0) += measurement_variance;
  innovation_covariance(1, 1) += measurement_variance;

  const Eigen::Matrix<double, 4, 2> gain =
    track->covariance * observation.transpose() * innovation_covariance.inverse();
  const Eigen::Vector2d innovation = detection.position - track->position();
  track->state += gain * innovation;

  // Joseph 形式：(I−KH) P (I−KH)ᵀ + K R Kᵀ。
  // 结构上就是对称半正定的，代价是多一次 4×4 乘法 —— 与 ESKF 那边同一个理由。
  const Eigen::Matrix4d factor = Eigen::Matrix4d::Identity() - gain * observation;
  Eigen::Matrix2d measurement_noise = Eigen::Matrix2d::Identity() * measurement_variance;
  track->covariance =
    factor * track->covariance * factor.transpose() + gain * measurement_noise * gain.transpose();

  track->yaw_rad = detection.yaw_rad;
  track->length_m = detection.length_m;
  track->width_m = detection.width_m;
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

void Tracker::Update(const std::vector<Detection> & detections, double dt_s)
{
  ads_common::RequireFinitePositive(dt_s, "Tracker::Update", "dt_s");
  for (const Detection & detection : detections) {
    ads_common::RequireFinite(detection.position.x(), "Tracker::Update", "detection.x");
    ads_common::RequireFinite(detection.position.y(), "Tracker::Update", "detection.y");
    ads_common::RequireFinite(detection.yaw_rad, "Tracker::Update", "detection.yaw");
  }

  Predict(dt_s);

  std::vector<int> assignment;
  Associate(detections, &assignment);

  std::vector<char> detection_used(detections.size(), 0);
  for (std::size_t t = 0; t < tracks_.size(); ++t) {
    if (assignment[t] >= 0) {
      ApplyUpdate(detections[assignment[t]], &tracks_[t]);
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
