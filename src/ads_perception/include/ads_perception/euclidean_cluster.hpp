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

#ifndef ADS_PERCEPTION__EUCLIDEAN_CLUSTER_HPP_
#define ADS_PERCEPTION__EUCLIDEAN_CLUSTER_HPP_

// =============================================================================
//  欧式聚类：把非地面点分成一个个「目标」
//
//  ## 算法：体素栅格 + 广度优先搜索
//
//      把空间切成边长 = tolerance 的立方格 → 每个点落进一格
//      从任一未访问点出发 BFS：只在它所在格的 **3×3×3 邻域**里找候选，
//      距离 < tolerance 的归入同簇
//
//  为什么栅格边长**恰好取 tolerance**：这样半径 tolerance 的球必然被
//  3×3×3 邻域完全包住 —— 不会漏掉任何一个应当相连的点。
//  取得更小就要搜更大的邻域（`ceil(tolerance/voxel)` 层），得不偿失。
//
//  ⚠️ **不用 PCL 的 KdTree。** 那是重型依赖（CLAUDE.md「先问后做」），
//     而园区一帧只有 2–5 万点，栅格 hash 的常数比 KdTree 小得多 ——
//     KdTree 的优势要到点数上百万、且查询半径远小于点云尺度时才显现。
//
//  ## ⚠️ tolerance 的账：它被**雷达线间距**从下面顶着
//
//  同一个目标的点之所以能连成一簇，靠的是相邻两条扫描线上的点足够近。
//  32 线 / 垂直 FOV 35° ⟹ 线角间隔 1.129°，于是距离 d 处：
//
//      距离      线间距     水平点间距
//      10 m     0.197 m     0.035 m
//      20 m     0.394 m     0.070 m
//      25 m     0.493 m     0.087 m
//      30 m     0.591 m     0.105 m
//
//  **线间距是主导**（水平点间距只有它的 1/5.6）。tolerance 必须大于它，
//  否则同一个目标会被**按扫描线拆成好几个簇** —— 而每个碎片都凑不够
//  min_cluster_size，于是目标**整个消失**。
//
//  另一侧：判据要求相距 1.0 m 的两个目标必须分开，所以 tolerance < 1.0。
//
//  取 **0.5 m**：覆盖到 25 m（线间距 0.493），对 1.0 m 判据留 2 倍余量。
//  ⚠️ **代价是 30 m 外的目标会被拆簇**（线间距 0.591 > 0.5）——
//     那是角分辨率的直接后果，不是参数没调好。想连上就得调到 0.6+，
//     代价是分辨两个近距离目标的能力变差。
//     S1 体检实测检测距离本来就只有 20–25 m，所以这个取舍是合算的。
//
//  ## ⚠️ 质心与包围盒中心**不是一回事**
//
//  雷达只打得到目标**朝向自己**的那几个面，背面完全没有点。于是：
//    · 质心偏向雷达一侧（有点的那半边拉着它）
//    · 轴对齐包围盒也只覆盖看得见的部分
//  两个量都会**系统性地偏向雷达**，而偏多少取决于目标朝向与距离。
//
//  本结构**两个都给**，因为下游用途不同：S3 的 L-Shape 拟合要用原始点，
//  而快速判重、可视化用包围盒。**不要以为其中一个就是"目标中心"。**
// =============================================================================

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace ads_perception
{

/// 聚类参数。
struct EuclideanClusterParams
{
  /// 邻域半径，m。**这是本模块最重要的参数**，两侧都有代价，见文件头的账。
  ///
  /// 调大 → 相邻目标被并成一簇（两个行人变成"一辆车"，而尺寸分类会据此
  ///        给出 VEHICLE，再传给 P6 预测选错运动模型）；
  /// 调小 → 同一目标被按扫描线拆碎，每片都凑不够 min_cluster_size，
  ///        **目标整个消失**（比拆成两个更危险 —— 后者至少还看得见）。
  double tolerance_m{0.5};

  /// 成簇的最小点数。**按 S1 的实测点数定，不是拍脑袋**。
  ///
  /// `scripts/check_perception_input.py` 实测（2026-08-11，命中帧的中位点数）：
  ///     锥桶 20–25 m: 7 点   |  行人 20–25 m: 7 点  |  NPC 车 20–25 m: 40 点
  ///
  /// 取 **5**：保住 20–25 m 的锥桶与行人（7 点，留 1.4 倍余量）。
  /// ⚠️ 调到 10 → 锥桶和行人在 20 m 外**直接看不见**，而症状是
  ///    「车快撞上了才开始绕」，看起来像规划器反应慢。
  ///    **这个参数是安全关键值，与 ndt.max_innovation_m 同型。**
  /// 调到 3 → 虚警变多（三个噪点也能成簇）。
  int min_cluster_size{5};

  /// 成簇的最大点数。超过它的簇被丢弃。
  ///
  /// 取 20000：一帧总共才 2–5 万点，一个占了近半数点的"目标"只可能是
  /// **地面分割失败**（把整片地面留给了聚类）。此时宁可丢掉，
  /// 也不要给下游一个几十米长的"障碍物" —— 那会让车立刻刹停。
  int max_cluster_size{20000};
};

/// 一个簇。
struct Cluster
{
  /// 属于本簇的点在**输入数组**里的下标。
  std::vector<int> indices;

  /// 质心。⚠️ 它**偏向雷达一侧**（背面没有点），不是目标的几何中心。
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};

  /// 轴对齐包围盒。⚠️ 同样只覆盖看得见的部分。
  Eigen::Vector3d min_corner{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_corner{Eigen::Vector3d::Zero()};

  /// 包围盒中心 = (min + max) / 2。与 `centroid` **不是一回事** ——
  /// 质心受点密度影响（近处的面点多），包围盒中心只看极值。
  Eigen::Vector3d box_center() const { return 0.5 * (min_corner + max_corner); }
};

/// 把点云聚成若干簇。
///
/// @param points 输入点（本项目里是**地面分割之后的非地面点**）。
/// @param params 参数。
/// @return 满足大小约束的簇；**顺序不保证稳定**，调用方不要依赖它。
/// @throws std::invalid_argument 参数非法，或输入含非有限值。
///
/// ⚠️ 非有限值**抛异常而不是跳过**，与 `SegmentGround` 同一个理由：
/// 一个 inf 会让它自己的体素下标溢出，而溢出之后的行为是未定义的 ——
/// 可能把两个毫不相干的区域连成一簇，而那看起来只是"聚类效果不好"。
std::vector<Cluster> ClusterEuclidean(
  const std::vector<Eigen::Vector3d> & points, const EuclideanClusterParams & params);

}  // namespace ads_perception

#endif  // ADS_PERCEPTION__EUCLIDEAN_CLUSTER_HPP_
