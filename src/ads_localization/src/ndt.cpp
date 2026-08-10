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

#include "ads_localization/ndt.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "ads_common/numeric_checks.hpp"

namespace ads_localization
{

namespace
{

/// 累加器：先收点，够数了再一次算均值与协方差。
struct VoxelAccumulator
{
  Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d sum_outer{Eigen::Matrix3d::Zero()};
  int count{0};
};

}  // namespace

void NdtGridParams::Validate() const
{
  ads_common::RequireFinitePositive(voxel_size_m, "NdtGridParams", "voxel_size_m");
  ads_common::RequireFinitePositive(
    eigenvalue_ratio_floor, "NdtGridParams", "eigenvalue_ratio_floor");
  if (eigenvalue_ratio_floor >= 1.0) {
    throw std::invalid_argument(
      "NdtGridParams::eigenvalue_ratio_floor 必须 < 1（它是相对 λ_max 的比例），收到 " +
      std::to_string(eigenvalue_ratio_floor));
  }
  // 少于 4 个点时样本协方差的自由度 ≤ 0，算出来的东西没有统计意义。
  if (min_points_per_voxel < 4) {
    throw std::invalid_argument(
      "NdtGridParams::min_points_per_voxel 至少要 4（三个点恰好张成一个平面，"
      "样本协方差自由度为 0），收到 " +
      std::to_string(min_points_per_voxel));
  }
}

int64_t NdtGrid::EncodeIndex(int64_t ix, int64_t iy, int64_t iz)
{
  // 每轴偏移到非负再打包。±2^20 个体素，2 m 时是 ±2000 km。
  constexpr int64_t kOffset = 1 << 20;
  constexpr int64_t kSpan = 1 << 21;
  return ((ix + kOffset) * kSpan + (iy + kOffset)) * kSpan + (iz + kOffset);
}

NdtGrid::NdtGrid(const std::vector<Eigen::Vector3d> & points, const NdtGridParams & params)
: params_(params)
{
  params_.Validate();
  if (points.empty()) {
    throw std::invalid_argument("NdtGrid: 点集为空");
  }

  const double inv_size = 1.0 / params_.voxel_size_m;

  // ---- 第一遍：按体素累加 -----------------------------------------------
  std::unordered_map<int64_t, VoxelAccumulator> accumulators;
  accumulators.reserve(points.size() / 8 + 1);
  for (const Eigen::Vector3d & p : points) {
    // ⚠️ 必须显式判有限性。CLAUDE.md 记着两次教训：用比较去拦非有限值
    //    一条都拦不住（NaN 参与任何比较都返回 false），而 gpu_lidar 的
    //    无回波射线返回的是 **±inf 不是 NaN**。
    for (int i = 0; i < 3; ++i) {
      ads_common::RequireFinite(p[i], "NdtGrid", "point");
    }
    const int64_t ix = static_cast<int64_t>(std::floor(p.x() * inv_size));
    const int64_t iy = static_cast<int64_t>(std::floor(p.y() * inv_size));
    const int64_t iz = static_cast<int64_t>(std::floor(p.z() * inv_size));
    VoxelAccumulator & acc = accumulators[EncodeIndex(ix, iy, iz)];
    acc.sum += p;
    acc.sum_outer += p * p.transpose();
    ++acc.count;
  }

  // ---- 第二遍：拟合高斯 -------------------------------------------------
  voxels_.reserve(accumulators.size());
  for (const auto & [key, acc] : accumulators) {
    if (acc.count < params_.min_points_per_voxel) {
      ++discarded_sparse_;
      continue;
    }

    const double n = static_cast<double>(acc.count);
    const Eigen::Vector3d mean = acc.sum / n;
    // 无偏样本协方差：E[xxᵀ] − μμᵀ，再按 n/(n−1) 修正。
    Eigen::Matrix3d covariance = (acc.sum_outer - n * mean * mean.transpose()) / (n - 1.0);
    // 数值上强制对称：上面那个式子在浮点下会有微小的不对称，
    // 而自伴随特征分解要求严格对称，否则结果没有定义。
    covariance = 0.5 * (covariance + covariance.transpose()).eval();

    // ---- 特征值下限（本文件头「坑二」）---------------------------------
    // 路面点在体素内共面，协方差沿法向的特征值 ≈ 0，求逆会炸。
    // 把小于 λ_max·ratio 的特征值抬上去，几何含义是「给这个高斯在最扁的
    // 方向上强加一个最小厚度」—— 等价于承认那个方向上它没有信息。
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      // 走到这里说明协方差里有非有限值，而入口已经查过点了 ——
      // 属于不该发生的情形，宁可炸也不要给一个错的高斯。
      throw std::runtime_error("NdtGrid: 体素协方差的特征分解失败");
    }
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    const double floor_value = eigenvalues.maxCoeff() * params_.eigenvalue_ratio_floor;

    bool regularized = false;
    for (int i = 0; i < 3; ++i) {
      if (eigenvalues[i] < floor_value) {
        eigenvalues[i] = floor_value;
        regularized = true;
      }
    }

    NdtVoxel voxel;
    voxel.mean = mean;
    voxel.point_count = acc.count;
    voxel.was_regularized = regularized;
    // 法向 = 最小特征值对应的特征向量。SelfAdjointEigenSolver 的特征值
    // 按升序排列，所以取第 0 列。**在抬下限之前取** —— 抬完之后
    // 三个特征值可能相等，特征向量就不再有几何含义了。
    voxel.normal = solver.eigenvectors().col(0).normalized();
    if (regularized) {
      ++regularized_;
      const Eigen::Matrix3d & vectors = solver.eigenvectors();
      covariance = vectors * eigenvalues.asDiagonal() * vectors.transpose();
      covariance = 0.5 * (covariance + covariance.transpose()).eval();
    }
    voxel.inverse_covariance = covariance.inverse();

    // 抬完下限之后仍然算不出有限的逆，说明这个体素彻底病了 —— 丢掉而不是
    // 让它进去污染代价函数。这一支正常情况下走不到，留着是因为
    // 「NDT 收敛到一个错位姿」比「NDT 不收敛」危险得多。
    if (!voxel.inverse_covariance.allFinite()) {
      ++discarded_sparse_;
      continue;
    }
    voxels_.emplace(key, voxel);
  }

  if (voxels_.empty()) {
    throw std::invalid_argument(
      "NdtGrid: 一个体素都没建起来（" + std::to_string(discarded_sparse_) +
      " 个因点数不足被丢弃）。voxel_size_m 是不是太小了？");
  }
}

void NdtGrid::CollectNeighbors(
  const Eigen::Vector3d & point, std::vector<const NdtVoxel *> & out) const
{
  const double inv_size = 1.0 / params_.voxel_size_m;
  const int64_t cx = static_cast<int64_t>(std::floor(point.x() * inv_size));
  const int64_t cy = static_cast<int64_t>(std::floor(point.y() * inv_size));
  const int64_t cz = static_cast<int64_t>(std::floor(point.z() * inv_size));
  for (int64_t dx = -1; dx <= 1; ++dx) {
    for (int64_t dy = -1; dy <= 1; ++dy) {
      for (int64_t dz = -1; dz <= 1; ++dz) {
        const auto it = voxels_.find(EncodeIndex(cx + dx, cy + dy, cz + dz));
        if (it != voxels_.end()) {
          out.push_back(&it->second);
        }
      }
    }
  }
}

const NdtVoxel * NdtGrid::At(const Eigen::Vector3d & point) const
{
  const double inv_size = 1.0 / params_.voxel_size_m;
  const int64_t ix = static_cast<int64_t>(std::floor(point.x() * inv_size));
  const int64_t iy = static_cast<int64_t>(std::floor(point.y() * inv_size));
  const int64_t iz = static_cast<int64_t>(std::floor(point.z() * inv_size));
  const auto it = voxels_.find(EncodeIndex(ix, iy, iz));
  return it == voxels_.end() ? nullptr : &it->second;
}

}  // namespace ads_localization
