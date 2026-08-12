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

#include "ads_perception/euclidean_cluster.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "ads_common/numeric_checks.hpp"

namespace ads_perception
{

namespace
{

/// 体素下标（三个 int32）。
struct VoxelKey
{
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

/// 体素 key 的哈希。
///
/// ⚠️ **这里的质量只影响效率，不影响正确性**（2026-08-11 故障注入实测确认）。
///    `unordered_map` 在桶内还会用 `operator==` 精确比较 key，所以
///    **哈希碰撞 ≠ key 相同** —— 撞到一起的两个体素不会被当成同一个。
///
///    （这段注释原本写的是「碰撞会让两个毫不相干的目标连成一簇」，
///     那是**错的**。把哈希换成三项相加之后，10 条用例一条都没红。
///     写错的后果不是代码坏了，而是下一个人以为这里守着一个不存在的风险，
///     于是不敢动它 —— 那比没有注释更糟。）
///
///    仍然用三素数混合，是因为相加哈希会让 (1,2,3) 与 (3,2,1) 落进同一个桶，
///    而那种下标组合在点云里非常常见 —— 桶一长，查找就退化成线性扫描。
struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    // 三个大素数混合。这是标准的空间哈希做法 ——
    // 直接相加的话 (1,2,3) 与 (3,2,1) 会碰撞，而那在点云里非常常见。
    const std::size_t hx = static_cast<std::size_t>(static_cast<std::uint32_t>(key.x)) * 73856093U;
    const std::size_t hy = static_cast<std::size_t>(static_cast<std::uint32_t>(key.y)) * 19349663U;
    const std::size_t hz = static_cast<std::size_t>(static_cast<std::uint32_t>(key.z)) * 83492791U;
    return hx ^ hy ^ hz;
  }
};

/// 坐标 → 体素下标。
///
/// ⚠️ 用 `std::floor` 而不是强制类型转换：后者对负数是**向零取整**，
///    于是 −0.3 和 +0.3 会落进同一格，格子在原点处宽了一倍。
///
/// ⚠️ **但这同样只影响效率，不影响正确性**（2026-08-11 故障注入实测确认）。
///    格子变宽之后 3×3×3 邻域**依然**覆盖半径 tolerance 的球
///    （格 0 覆盖 (−0.5, 0.5) 时，邻域是 (−1.0, 1.0) ⊇ 任意点的 ±0.5），
///    而多进来的候选会被随后的距离检查拦掉。
///    实测把 floor 换成截断，10 条用例一条都没红。
///
///    （这段注释原本写的是「原点附近的目标会被多连进来一些点」，那是**错的**。）
///
///    仍然用 floor，是因为它让格子均匀 —— 原点那格装两倍的点会让
///    那一格的邻域查询变慢，而那是白付的代价。
VoxelKey ToVoxel(const Eigen::Vector3d & point, double size_m)
{
  return VoxelKey{
    static_cast<std::int32_t>(std::floor(point.x() / size_m)),
    static_cast<std::int32_t>(std::floor(point.y() / size_m)),
    static_cast<std::int32_t>(std::floor(point.z() / size_m))};
}

}  // namespace

std::vector<Cluster> ClusterEuclidean(
  const std::vector<Eigen::Vector3d> & points, const EuclideanClusterParams & params)
{
  ads_common::RequireFinitePositive(params.tolerance_m, "EuclideanClusterParams", "tolerance_m");
  if (params.min_cluster_size <= 0) {
    throw std::invalid_argument(
      "EuclideanClusterParams::min_cluster_size 必须为正，收到 " +
      std::to_string(params.min_cluster_size));
  }
  if (params.max_cluster_size < params.min_cluster_size) {
    throw std::invalid_argument(
      "EuclideanClusterParams::max_cluster_size 不能小于 min_cluster_size —— "
      "那样一个簇都不会输出，而现场表现是「感知什么都没看见」。");
  }

  std::vector<Cluster> clusters;
  if (points.empty()) {
    return clusters;
  }

  // ⚠️ 非有限值抛异常。inf 会让体素下标溢出，而溢出后的行为是未定义的：
  //    可能把两个毫不相干的区域连成一簇，看起来只是"聚类效果不好"。
  for (const Eigen::Vector3d & point : points) {
    for (int i = 0; i < 3; ++i) {
      ads_common::RequireFinite(point[i], "ClusterEuclidean", "point");
    }
  }

  // ---- 建体素索引 -------------------------------------------------------
  // 边长恰好取 tolerance，这样半径 tolerance 的球必被 3×3×3 邻域包住。
  std::unordered_map<VoxelKey, std::vector<int>, VoxelKeyHash> grid;
  grid.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    grid[ToVoxel(points[i], params.tolerance_m)].push_back(static_cast<int>(i));
  }

  const double tolerance_squared = params.tolerance_m * params.tolerance_m;
  std::vector<std::uint8_t> visited(points.size(), 0U);
  std::vector<int> queue;  // BFS 队列。复用同一块内存，避免每个簇都重新分配。

  for (std::size_t seed = 0; seed < points.size(); ++seed) {
    if (visited[seed] != 0U) {
      continue;
    }
    queue.clear();
    queue.push_back(static_cast<int>(seed));
    visited[seed] = 1U;

    // ⚠️ 用下标而不是迭代器遍历 queue：BFS 过程中会往 queue 里 push_back，
    //    而那可能触发 realloc，迭代器立刻失效。这个错误的症状是随机崩溃
    //    或随机漏点，且**只在簇比较大的时候**出现。
    for (std::size_t head = 0; head < queue.size(); ++head) {
      const int current = queue[head];
      const VoxelKey center = ToVoxel(points[current], params.tolerance_m);

      for (std::int32_t dx = -1; dx <= 1; ++dx) {
        for (std::int32_t dy = -1; dy <= 1; ++dy) {
          for (std::int32_t dz = -1; dz <= 1; ++dz) {
            const auto found = grid.find(VoxelKey{center.x + dx, center.y + dy, center.z + dz});
            if (found == grid.end()) {
              continue;
            }
            for (const int candidate : found->second) {
              if (visited[candidate] != 0U) {
                continue;
              }
              // 比较平方距离，省一次 sqrt。tolerance 恒为正，平方不改变序。
              if ((points[candidate] - points[current]).squaredNorm() <= tolerance_squared) {
                visited[candidate] = 1U;
                queue.push_back(candidate);
              }
            }
          }
        }
      }
    }

    const int size = static_cast<int>(queue.size());
    if (size < params.min_cluster_size || size > params.max_cluster_size) {
      continue;  // 太小 = 噪点；太大 = 地面分割失败，见头文件
    }

    Cluster cluster;
    cluster.indices = queue;
    cluster.min_corner = points[queue.front()];
    cluster.max_corner = points[queue.front()];
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const int index : queue) {
      const Eigen::Vector3d & point = points[index];
      sum += point;
      cluster.min_corner = cluster.min_corner.cwiseMin(point);
      cluster.max_corner = cluster.max_corner.cwiseMax(point);
    }
    cluster.centroid = sum / static_cast<double>(size);
    clusters.push_back(std::move(cluster));
  }

  return clusters;
}

}  // namespace ads_perception
