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

#include "ads_perception/lshape_fit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "ads_common/numeric_checks.hpp"

namespace ads_perception
{

LShapeBox FitLShape(const std::vector<Eigen::Vector3d> & points, const LShapeFitParams & params)
{
  ads_common::RequireFinitePositive(params.angle_step_rad, "LShapeFitParams", "angle_step_rad");
  ads_common::RequireFinitePositive(params.min_distance_m, "LShapeFitParams", "min_distance_m");
  if (params.min_points < 3) {
    throw std::invalid_argument(
      "LShapeFitParams::min_points 至少要 3 —— 两个点定不出矩形，收到 " +
      std::to_string(params.min_points));
  }

  LShapeBox box;
  if (static_cast<int>(points.size()) < params.min_points) {
    return box;  // valid = false
  }
  for (const Eigen::Vector3d & point : points) {
    for (int i = 0; i < 3; ++i) {
      ads_common::RequireFinite(point[i], "FitLShape", "point");
    }
  }

  // ---- 高度直接取 z 的极差 ---------------------------------------------
  // 高度**不参与**朝向搜索：车绕 z 轴转动不改变 z 分布，把它塞进二维
  // 搜索里只会让每次打分多算一维、结果一模一样。
  double z_min = points.front().z();
  double z_max = points.front().z();
  for (const Eigen::Vector3d & point : points) {
    z_min = std::min(z_min, point.z());
    z_max = std::max(z_max, point.z());
  }
  box.height_m = z_max - z_min;

  // ---- 搜索朝向 --------------------------------------------------------
  // 只搜 [0, π/2)：矩形转 90° 还是同一个矩形（长宽互换），再搜就是重复劳动。
  const int steps = std::max(1, static_cast<int>(std::ceil((M_PI / 2.0) / params.angle_step_rad)));
  double best_score = -std::numeric_limits<double>::infinity();
  double best_theta = 0.0;
  double best_c1_min = 0.0;
  double best_c1_max = 0.0;
  double best_c2_min = 0.0;
  double best_c2_max = 0.0;

  std::vector<double> projection_1(points.size());
  std::vector<double> projection_2(points.size());

  for (int step = 0; step < steps; ++step) {
    const double theta = step * params.angle_step_rad;
    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);

    double c1_min = std::numeric_limits<double>::infinity();
    double c1_max = -std::numeric_limits<double>::infinity();
    double c2_min = std::numeric_limits<double>::infinity();
    double c2_max = -std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < points.size(); ++i) {
      // e1 = (cosθ, sinθ)，e2 = (−sinθ, cosθ) —— 一对正交轴。
      projection_1[i] = points[i].x() * cos_t + points[i].y() * sin_t;
      projection_2[i] = -points[i].x() * sin_t + points[i].y() * cos_t;
      c1_min = std::min(c1_min, projection_1[i]);
      c1_max = std::max(c1_max, projection_1[i]);
      c2_min = std::min(c2_min, projection_2[i]);
      c2_max = std::max(c2_max, projection_2[i]);
    }

    // closeness：每个点到**最近那条边**的距离，取 Σ 1/max(d, d0)。
    //
    // ⚠️ 为什么是 1/d 而不是 −d（"距离和最小"）：
    //    1/d 让**贴得很紧的点**主导得分，而那正是 L 形的两条可见边；
    //    用 −d 的话，远离边的点（矩形内部的空白）会稀释信号，
    //    结果偏向"把所有点圈起来的最小矩形"—— 那退化成了最小面积准则。
    double score = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      const double d1 = std::min(projection_1[i] - c1_min, c1_max - projection_1[i]);
      const double d2 = std::min(projection_2[i] - c2_min, c2_max - projection_2[i]);
      score += 1.0 / std::max(std::min(d1, d2), params.min_distance_m);
    }

    if (score > best_score) {
      best_score = score;
      best_theta = theta;
      best_c1_min = c1_min;
      best_c1_max = c1_max;
      best_c2_min = c2_min;
      best_c2_max = c2_max;
    }
  }

  // ---- 由最优 θ 还原矩形 -----------------------------------------------
  const double cos_t = std::cos(best_theta);
  const double sin_t = std::sin(best_theta);
  const double mid_1 = 0.5 * (best_c1_min + best_c1_max);
  const double mid_2 = 0.5 * (best_c2_min + best_c2_max);
  // 中心从两个轴上的中点反变换回来：p = mid1·e1 + mid2·e2。
  box.center.x() = mid_1 * cos_t - mid_2 * sin_t;
  box.center.y() = mid_1 * sin_t + mid_2 * cos_t;

  const double extent_1 = best_c1_max - best_c1_min;
  const double extent_2 = best_c2_max - best_c2_min;

  // ⚠️ **长轴必须是较长的那一条**，否则 length/width 会随 θ 落在哪个
  //    90° 区间而互换 —— 而尺寸分类正是按 length/width 判的，
  //    互换之后一辆 4.4×1.8 的车会被当成 1.8×4.4 的"横着的东西"，
  //    分类直接错。搜索范围是 [0, π/2)，两种情况都会出现。
  if (extent_1 >= extent_2) {
    box.length_m = extent_1;
    box.width_m = extent_2;
    box.yaw_rad = best_theta;
  } else {
    box.length_m = extent_2;
    box.width_m = extent_1;
    // 长轴是 e2，它比 e1 转了 90°。
    box.yaw_rad = best_theta + M_PI / 2.0;
  }

  // 归一化到 [0, π)。
  //
  // ⚠️ **这不是"丢掉了信息"，而是如实表达**：矩形有 180° 对称性，
  //    yaw 与 yaw+π 描述的是同一个矩形。把它留在 [0, 2π) 会让下游
  //    误以为这里给出了车头朝向 —— 而那个信息根本不存在（见头文件）。
  while (box.yaw_rad >= M_PI) {
    box.yaw_rad -= M_PI;
  }
  while (box.yaw_rad < 0.0) {
    box.yaw_rad += M_PI;
  }

  box.score = best_score;
  box.valid = true;
  return box;
}

}  // namespace ads_perception
