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

#include "ads_perception/ground_segmentation.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "ads_common/numeric_checks.hpp"

namespace ads_perception
{

namespace
{

/// 由三个点定一个平面。共线时返回 false（法向退化）。
bool PlaneFromThreePoints(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b, const Eigen::Vector3d & c,
  Eigen::Vector3d * normal, double * offset)
{
  const Eigen::Vector3d cross = (b - a).cross(c - a);
  const double norm = cross.norm();
  // ⚠️ 三点共线（或几乎共线）时叉积趋于零，单位化会放大数值噪声，
  //    得到一个方向随机的"法向"。阈值不能取 0 —— 取 1e-9 是因为
  //    雷达点间距是厘米级，正常三点的叉积模远大于它。
  if (!(norm > 1e-9)) {
    return false;
  }
  *normal = cross / norm;
  // 统一让法向指向 +z 半空间，这样坡度检查只要看 normal.z() 就行，
  // 不必对 ±n 各判一次。平面本身不受影响（n 与 −n 是同一个平面）。
  if (normal->z() < 0.0) {
    *normal = -*normal;
  }
  *offset = -normal->dot(a);
  return true;
}

}  // namespace

GroundSegmentationResult SegmentGround(
  const std::vector<Eigen::Vector3d> & points_sensor, const GroundSegmentationParams & params)
{
  if (params.max_iterations <= 0) {
    throw std::invalid_argument(
      "GroundSegmentationParams::max_iterations 必须为正，收到 " +
      std::to_string(params.max_iterations));
  }
  ads_common::RequireFinitePositive(
    params.distance_threshold_m, "GroundSegmentationParams", "distance_threshold_m");
  ads_common::RequireFinitePositive(
    params.max_slope_rad, "GroundSegmentationParams", "max_slope_rad");
  ads_common::RequireFinite(params.max_height_m, "GroundSegmentationParams", "max_height_m");

  GroundSegmentationResult result;
  result.is_ground.assign(points_sensor.size(), 0U);
  if (points_sensor.empty()) {
    return result;  // found = false
  }

  // ⚠️ 非有限值**抛异常而不是跳过**。理由见头文件：一个 inf 混进拟合会让
  //    法向变成 NaN，而 NaN 参与任何比较都返回 false —— 结果是所有点都被
  //    判成非地面，下游看到"满屏障碍物"，没人会想到根因是几个 inf。
  for (const Eigen::Vector3d & point : points_sensor) {
    for (int i = 0; i < 3; ++i) {
      ads_common::RequireFinite(point[i], "SegmentGround", "point");
    }
  }

  // ---- 只在 max_height_m 以下采样 -------------------------------------
  // 车顶、建筑、树冠的点不参与**采样**（但仍然参与内点计数 —— 一个真正的
  // 地面平面本来就不该把它们算进去，而如果算进去了，那说明平面选错了，
  // 这个信息不该被藏起来）。
  std::vector<std::size_t> candidates;
  candidates.reserve(points_sensor.size());
  for (std::size_t i = 0; i < points_sensor.size(); ++i) {
    if (points_sensor[i].z() <= params.max_height_m) {
      candidates.push_back(i);
    }
  }
  if (candidates.size() < 3) {
    return result;  // 连三个点都凑不出来
  }

  // 固定种子 —— RANSAC 依赖随机采样，结果不可复现就没法做回归测试。
  std::mt19937 rng(params.seed);
  std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);
  const double min_normal_z = std::cos(params.max_slope_rad);

  int best_inliers = 0;
  Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
  double best_offset = 0.0;

  for (int iteration = 0; iteration < params.max_iterations; ++iteration) {
    const Eigen::Vector3d & a = points_sensor[candidates[pick(rng)]];
    const Eigen::Vector3d & b = points_sensor[candidates[pick(rng)]];
    const Eigen::Vector3d & c = points_sensor[candidates[pick(rng)]];

    Eigen::Vector3d normal;
    double offset = 0.0;
    if (!PlaneFromThreePoints(a, b, c, &normal, &offset)) {
      continue;
    }
    // ⚠️ **坡度检查必须在计数之前。** 放在之后（"先选内点最多的，再看它是不是
    //    太陡"）的话，一堵墙会先赢下比较、再被淘汰，而那一轮就白费了 ——
    //    更糟的是若最优的几个候选全是墙，最后会退回一个内点很少的平面。
    if (normal.z() < min_normal_z) {
      continue;
    }

    int inliers = 0;
    for (const Eigen::Vector3d & point : points_sensor) {
      if (std::abs(normal.dot(point) + offset) <= params.distance_threshold_m) {
        ++inliers;
      }
    }
    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_normal = normal;
      best_offset = offset;
    }
  }

  if (best_inliers < params.min_inliers) {
    return result;  // found = false，宁可说"没找到"也不给一个拟合噪声的平面
  }

  // ---- 用全部内点做一次最小二乘精化 -----------------------------------
  // 三点定的平面只用了 3 个样本，噪声直接进结果。用全部内点重估一次，
  // 法向取协方差最小特征值对应的特征向量（也就是主成分分析的最小方向）。
  // 这一步便宜（一次 3×3 特征分解），而它把法向精度提高一个量级 ——
  // 而法向的误差会以「距离阈值被斜着切」的方式吃掉有效裕度。
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  std::vector<const Eigen::Vector3d *> inlier_points;
  inlier_points.reserve(static_cast<std::size_t>(best_inliers));
  for (const Eigen::Vector3d & point : points_sensor) {
    if (std::abs(best_normal.dot(point) + best_offset) <= params.distance_threshold_m) {
      inlier_points.push_back(&point);
      centroid += point;
    }
  }
  centroid /= static_cast<double>(inlier_points.size());

  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (const Eigen::Vector3d * point : inlier_points) {
    const Eigen::Vector3d delta = *point - centroid;
    scatter += delta * delta.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  if (solver.info() == Eigen::Success) {
    Eigen::Vector3d refined = solver.eigenvectors().col(0);  // 最小特征值 → 平面法向
    if (refined.z() < 0.0) {
      refined = -refined;
    }
    // ⚠️ 精化后**再查一次坡度**。内点集里混进墙面点时，精化会把法向拽歪；
    //    不复查的话，一个通过了坡度检查的候选可能精化成一个陡平面。
    if (refined.z() >= min_normal_z) {
      best_normal = refined;
      best_offset = -refined.dot(centroid);
    }
  }

  // ---- 用最终平面标注 -------------------------------------------------
  result.found = true;
  result.normal = best_normal;
  result.offset_m = best_offset;
  for (std::size_t i = 0; i < points_sensor.size(); ++i) {
    const bool is_ground =
      std::abs(best_normal.dot(points_sensor[i]) + best_offset) <= params.distance_threshold_m;
    result.is_ground[i] = is_ground ? 1U : 0U;
    result.ground_count += is_ground ? 1 : 0;
  }
  return result;
}

}  // namespace ads_perception
