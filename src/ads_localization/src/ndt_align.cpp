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

#include "ads_localization/ndt_align.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "ads_common/numeric_checks.hpp"
#include "ads_localization/eskf.hpp"

namespace ads_localization
{

namespace
{

/// 把 6 维增量 (δt, δθ) 变成一个位姿增量，右乘到当前估计上。
Eigen::Isometry3d ApplyIncrement(
  const Eigen::Isometry3d & pose, const Eigen::Matrix<double, 6, 1> & increment)
{
  Eigen::Isometry3d updated = Eigen::Isometry3d::Identity();
  // 旋转右乘（body 系增量），与 ESKF 的姿态注入是同一个约定。
  updated.linear() =
    pose.linear() * QuaternionFromRotationVector(increment.tail<3>()).toRotationMatrix();
  updated.translation() = pose.translation() + increment.head<3>();
  return updated;
}

}  // namespace

void NdtAlignParams::Validate() const
{
  if (max_iterations <= 0) {
    throw std::invalid_argument(
      "NdtAlignParams::max_iterations 必须为正，收到 " + std::to_string(max_iterations));
  }
  ads_common::RequireFinitePositive(
    translation_epsilon_m, "NdtAlignParams", "translation_epsilon_m");
  ads_common::RequireFinitePositive(rotation_epsilon_rad, "NdtAlignParams", "rotation_epsilon_rad");
  ads_common::RequireFinitePositive(max_step_m, "NdtAlignParams", "max_step_m");
  ads_common::RequireFinitePositive(levenberg_damping, "NdtAlignParams", "levenberg_damping");
  ads_common::RequireFinitePositive(min_normal_diversity, "NdtAlignParams", "min_normal_diversity");
  ads_common::RequireFinitePositive(
    min_absolute_eigenvalue, "NdtAlignParams", "min_absolute_eigenvalue");
  ads_common::RequireFinitePositive(min_inlier_ratio, "NdtAlignParams", "min_inlier_ratio");
  ads_common::RequireFinitePositive(covariance_scale, "NdtAlignParams", "covariance_scale");
}

NdtScoreTerms ComputeNdtScoreTerms(
  const NdtGrid & map, const std::vector<Eigen::Vector3d> & scan_body,
  const Eigen::Isometry3d & pose)
{
  NdtScoreTerms terms;
  const Eigen::Matrix3d rotation = pose.linear();

  std::vector<const NdtVoxel *> neighbors;
  neighbors.reserve(27);

  for (const Eigen::Vector3d & point_body : scan_body) {
    const Eigen::Vector3d point_map = pose * point_body;
    neighbors.clear();
    // ⚠️ 必须收邻域而不是只取包含该点的那一个体素。只取一个的话，
    //    点跨过边界的瞬间换了一个高斯，代价函数是**锯齿状**的 ——
    //    实测牛顿步方向完全正确却一步都走不动。见 NdtGrid::CollectNeighbors。
    map.CollectNeighbors(point_map, neighbors);
    if (neighbors.empty()) {
      // 周围一个非空体素都没有 —— 跳过而不是记 0 分。草地上的实测点在地图里
      // 本来就没有对应，硬给分等于让地图的「洞」去拉扯位姿。
      continue;
    }
    ++terms.inlier_count;

    for (const NdtVoxel * voxel : neighbors) {
      const Eigen::Vector3d residual = point_map - voxel->mean;
      const Eigen::Vector3d weighted = voxel->inverse_covariance * residual;
      const double mahalanobis = residual.dot(weighted);
      const double weight = std::exp(-0.5 * mahalanobis);

      // s = −exp(−q/2)，越小越好。
      terms.score -= weight;

      // 雅可比 ∂x/∂ξ = [I, −R̂[p]×]（在 ξ = 0 处）。
      Eigen::Matrix<double, 3, 6> jacobian;
      jacobian.leftCols<3>() = Eigen::Matrix3d::Identity();
      jacobian.rightCols<3>() = -rotation * SkewSymmetric(point_body);

      // ∂s/∂ξ = exp(−q/2) · eᵀΣ⁻¹J  →  列向量形式 JᵀΣ⁻¹e·w
      terms.gradient += weight * (jacobian.transpose() * weighted);

      // Gauss-Newton 信息阵 = Σ w·JᵀΣ⁻¹J。见头文件里关于「为什么丢掉那一项」
      // 的说明 —— 它同时是牛顿方向的保证和输出协方差的正确对象。
      terms.information += weight * (jacobian.transpose() * voxel->inverse_covariance * jacobian);
      // 法向散布：几何退化的唯一可靠依据，见头文件「退化检测」一节。
      terms.normal_scatter += weight * (voxel->normal * voxel->normal.transpose());
    }
  }
  return terms;
}

NdtAlignResult AlignNdt(
  const NdtGrid & map, const std::vector<Eigen::Vector3d> & scan_body,
  const Eigen::Isometry3d & initial_guess, const NdtAlignParams & params)
{
  params.Validate();
  if (scan_body.empty()) {
    throw std::invalid_argument("AlignNdt: 扫描点集为空");
  }
  for (const Eigen::Vector3d & p : scan_body) {
    for (int i = 0; i < 3; ++i) {
      // ⚠️ gpu_lidar 的无回波射线返回 ±inf 不是 NaN，两者都要拦。
      ads_common::RequireFinite(p[i], "AlignNdt", "scan point");
    }
  }
  if (!initial_guess.matrix().allFinite()) {
    throw std::invalid_argument("AlignNdt: 初值位姿含非有限值");
  }

  NdtAlignResult result;
  result.pose = initial_guess;

  const double scan_size = static_cast<double>(scan_body.size());
  NdtScoreTerms terms = ComputeNdtScoreTerms(map, scan_body, result.pose);

  for (int iteration = 0; iteration < params.max_iterations; ++iteration) {
    result.iterations = iteration + 1;

    if (terms.inlier_count == 0) {
      break;  // 与地图完全没有重叠，下面按退化处理
    }

    // 阻尼后求牛顿步。H 半正定 + λI 之后严格正定，LDLT 够用且比求逆稳。
    Eigen::Matrix<double, 6, 6> damped = terms.information;
    damped.diagonal().array() += params.levenberg_damping;
    const Eigen::Matrix<double, 6, 1> step = damped.ldlt().solve(-terms.gradient);
    if (!step.allFinite()) {
      break;  // H 病态到解不出来，交给下面的退化判定
    }

    // 限制单步平移。病态时 H⁻¹ 可能解出一个把扫描甩到几百米外的步长。
    Eigen::Matrix<double, 6, 1> limited = step;
    const double translation_norm = limited.head<3>().norm();
    if (translation_norm > params.max_step_m) {
      limited *= params.max_step_m / translation_norm;
    }

    // 回溯线搜索：Gauss-Newton 方向一定是下降方向，但步长可能过冲
    // （代价函数是高斯的和，非二次）。折半直到代价真的下降。
    double scale = 1.0;
    bool improved = false;
    Eigen::Isometry3d candidate_pose = result.pose;
    NdtScoreTerms candidate_terms = terms;
    for (int trial = 0; trial < 10; ++trial) {
      candidate_pose = ApplyIncrement(result.pose, limited * scale);
      candidate_terms = ComputeNdtScoreTerms(map, scan_body, candidate_pose);
      if (candidate_terms.score < terms.score) {
        improved = true;
        break;
      }
      scale *= 0.5;
    }
    if (!improved) {
      // 折半十次仍不下降 —— 已经在极小值附近（或代价函数在这里不连续，
      // 见 NdtGrid::At 关于「只查一个体素」的说明）。当作收敛。
      result.converged = true;
      break;
    }

    result.pose = candidate_pose;
    terms = candidate_terms;

    const Eigen::Matrix<double, 6, 1> taken = limited * scale;
    if (
      taken.head<3>().norm() < params.translation_epsilon_m &&
      taken.tail<3>().norm() < params.rotation_epsilon_rad) {
      result.converged = true;
      break;
    }
  }

  result.score = terms.score;
  result.inlier_ratio = static_cast<double>(terms.inlier_count) / scan_size;
  result.information = terms.information;

  // ---- 退化判定 ---------------------------------------------------------
  // **几何退化看法向散布，不看信息阵条件数。**
  //
  // P4-1 决策一的原话：约束来自表面法向，一个平面只约束沿它法向的那一个自由度。
  // 所以判据就是「匹配上的体素法向是否平行」。
  //
  // ⚠️ 曾经用信息阵条件数做这件事，那是错的（2026-08-10 实测推翻）：
  //    纯地面点云上 H 报出 σ_x = 8.7 mm，条件数只比结构齐全的情形差 3 倍 ——
  //    因为**体素离散化伪造了面内信息**（每格高斯的均值落在该格质心，
  //    整张地图在面内构成 2 m 点阵，扫描平移会撞上纹波）。
  //    那个信息相对离散化后的地图是真的，相对物理世界是假的。
  //    换成法向散布之后两种情形差 6 个数量级。
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> normal_solver(terms.normal_scatter);
  if (normal_solver.info() == Eigen::Success) {
    const Eigen::Vector3d spread = normal_solver.eigenvalues();
    result.normal_diversity = spread.maxCoeff() > 0.0 ? spread.minCoeff() / spread.maxCoeff() : 0.0;
  }
  result.degenerate = result.normal_diversity < params.min_normal_diversity;

  // 信息阵的特征值只作为**诊断量**与数值保险，不承担几何退化判定。
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(terms.information);
  if (solver.info() == Eigen::Success) {
    const Eigen::Matrix<double, 6, 1> eigenvalues = solver.eigenvalues();
    result.smallest_eigenvalue = eigenvalues.minCoeff();
    const double largest = eigenvalues.maxCoeff();
    result.condition_number =
      largest > 0.0 ? largest / std::max(result.smallest_eigenvalue, 1e-300) : 0.0;
    if (result.smallest_eigenvalue < params.min_absolute_eigenvalue) {
      result.degenerate = true;  // H 病态到解不出来
    }
  } else {
    result.degenerate = true;
  }

  if (result.inlier_ratio < params.min_inlier_ratio) {
    result.degenerate = true;  // 与地图基本没有重叠
  }

  if (!result.degenerate) {
    Eigen::Matrix<double, 6, 6> damped = terms.information;
    damped.diagonal().array() += params.levenberg_damping;
    result.covariance = params.covariance_scale * damped.inverse();
    if (!result.covariance.allFinite()) {
      result.degenerate = true;
      result.covariance.setZero();
    }
  }
  // 退化时协方差保持全零 —— 它是「不要用这个结果」的显式信号，
  // 而不是一个看起来很小、会被下游当成"很准"的数。

  return result;
}

}  // namespace ads_localization
