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

// =============================================================================
//  欧式聚类的 L1 判据（CP-P5-A ③④⑤）
//
//  ⚠️ 这里的点云是**按真实雷达的角分辨率**采样出来的，不是均匀网格。
//     理由与地面分割那组用例同源：均匀采样的点云上，聚类怎么写都对 ——
//     而真实点云里同一个目标的相邻两线相隔 0.4–0.6 m（比水平点间距大 5.6 倍），
//     **那才是 tolerance 被顶住的地方**。用均匀网格测，等于把最难的部分绕开了。
//
//  ## 故障注入实测（2026-08-11，写完立刻做的）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 只搜自己那一格，不搜 3×3×3 邻域 | **红 5 条** |
//  | `ToVoxel` 用截断代替 `floor` | **绿 —— 抓不到** |
//  | 哈希改成三项相加 | **绿 —— 抓不到** |
//
//  ⚠️ **后两条抓不到，是因为它们本来就不影响正确性** —— 而我在实现里
//     原本把它们的后果写成了「两个毫不相干的目标被连成一簇」，**那是错的**：
//       · 格子变宽后 3×3×3 邻域**依然**覆盖半径 tolerance 的球，
//         多进来的候选会被距离检查拦掉；
//       · `unordered_map` 桶内还会用 `operator==` 精确比较 key，
//         **哈希碰撞 ≠ key 相同**。
//     两者影响的都是**效率**。注释已改。
//
//     这件事本身值得记：**注释里声称的后果如果是错的，比没有注释更糟** ——
//     它会让下一个人以为那里守着一个不存在的风险，于是既不敢动、
//     也不会去补真正需要的判据。
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "ads_perception/euclidean_cluster.hpp"

namespace
{

using ads_perception::Cluster;
using ads_perception::ClusterEuclidean;
using ads_perception::EuclideanClusterParams;

// 与 config/vehicle_params.yaml 的雷达一致。
constexpr double kLidarHeightM = 1.6;
constexpr double kVerticalStepRad = (0.1745 - (-0.4363)) / 31.0;  // 1.129°
constexpr double kHorizontalStepRad = 0.2 * M_PI / 180.0;

/// 按**真实角分辨率**在一个长方体的可见面上打点。
///
/// 这不是"造得像一点"，而是这组用例的前提：均匀网格会让 tolerance 的
/// 上下界都失去意义（见文件头）。
std::vector<Eigen::Vector3d> ScanBox(
  const Eigen::Vector3d & center, double length, double width, double height, double noise_stddev)
{
  std::vector<Eigen::Vector3d> points;
  std::mt19937 rng(static_cast<std::uint32_t>(std::abs(center.x()) * 977 + std::abs(center.y())));
  std::normal_distribution<double> noise(0.0, noise_stddev);

  const double distance = std::hypot(center.x(), center.y());
  // 该距离上一条扫描线的高度步长、一列的水平步长。
  const double dz = distance * std::tan(kVerticalStepRad);
  const double dy = distance * kHorizontalStepRad;

  // 只打**朝向雷达**的那个面（x 较小的一侧）—— 真雷达看不到背面。
  const double face_x = center.x() - length / 2.0;
  for (double z = center.z() - height / 2.0; z <= center.z() + height / 2.0; z += dz) {
    // 雷达打不到高于自己俯角范围的部分，也打不到地面以下。
    if (z < 0.05) {
      continue;
    }
    for (double y = center.y() - width / 2.0; y <= center.y() + width / 2.0; y += dy) {
      points.emplace_back(face_x + noise(rng), y + noise(rng), z + noise(rng));
    }
  }
  return points;
}

/// 把若干个点集拼起来。
std::vector<Eigen::Vector3d> Concat(const std::vector<std::vector<Eigen::Vector3d>> & parts)
{
  std::vector<Eigen::Vector3d> all;
  for (const auto & part : parts) {
    all.insert(all.end(), part.begin(), part.end());
  }
  return all;
}

}  // namespace

// ---------------------------------------------------------------------------
//  CP-P5-A ③：检测框中心的误差
// ---------------------------------------------------------------------------
TEST(EuclideanCluster, LocatesTheVisibleFaceOfABoxAccurately)
{
  // ⚠️ **这条判据量的是「可见面的位置」，不是「目标中心」。**
  //    雷达只打得到朝向自己的那个面，背面完全没有点 —— 所以簇的包围盒
  //    中心必然偏向雷达一侧，偏移量约等于**目标进深的一半**。
  //    对 4.4 m 长的车正对雷达时，那就是 2.2 m —— 拿它比"目标中心"
  //    会得到一个看起来很糟的数，而系统其实完全正常。
  //
  //    真正的目标中心要靠 S3 的 L-Shape 拟合（用两条边推出完整轮廓）。
  //    **这一层能保证的只有：可见面被定位得准。**
  for (const double distance : {10.0, 20.0, 25.0}) {
    const Eigen::Vector3d center(distance, 0.0, 0.75);
    const auto points = ScanBox(center, 4.4, 1.8, 1.5, 0.01);
    ASSERT_FALSE(points.empty());

    const auto clusters = ClusterEuclidean(points, EuclideanClusterParams{});
    ASSERT_EQ(clusters.size(), 1U) << distance << " m 处应当只有一个簇";

    // 可见面的真值位置（x = center.x − length/2）。
    const double truth_face_x = center.x() - 4.4 / 2.0;
    const Eigen::Vector3d box_center = clusters[0].box_center();
    const double face_error = std::abs(box_center.x() - truth_face_x);
    const double lateral_error = std::abs(box_center.y() - center.y());

    printf(
      "[          ] %.0f m：%zu 点，可见面误差 %.4f m，横向误差 %.4f m\n", distance, points.size(),
      face_error, lateral_error);

    // CP-P5-A ③：< 0.2 m。
    EXPECT_LT(face_error, 0.2) << distance << " m 处可见面定位偏了";
    EXPECT_LT(lateral_error, 0.2) << distance << " m 处横向定位偏了";
  }
}

// ---------------------------------------------------------------------------
//  CP-P5-A ④：相距 1.0 m 的两个目标必须分开
// ---------------------------------------------------------------------------
TEST(EuclideanCluster, SeparatesTwoTargetsOneMetreApart)
{
  // 两个行人（0.4×0.4×1.7），**外廓**相距 1.0 m。
  // ⚠️ 判据说的是「相距 1.0 m」——量的是**外廓间距**不是中心距，
  //    因为聚类看的是最近点。中心距要再加两个半宽。
  const double gap = 1.0;
  const double width = 0.4;
  const auto left = ScanBox({20.0, +(gap + width) / 2.0, 0.85}, 0.4, width, 1.7, 0.01);
  const auto right = ScanBox({20.0, -(gap + width) / 2.0, 0.85}, 0.4, width, 1.7, 0.01);
  const auto points = Concat({left, right});

  const auto clusters = ClusterEuclidean(points, EuclideanClusterParams{});
  printf(
    "[          ] 外廓相距 %.1f m 的两个行人（各 %zu / %zu 点）→ %zu 个簇\n", gap, left.size(),
    right.size(), clusters.size());

  // ⚠️ 并成一簇的后果不只是"少一个目标"：尺寸分类会按合并后的包围盒
  //    给出 VEHICLE，而 P6 预测据此选车道跟随模型 —— 两个行人会被预测成
  //    一辆沿车道行驶的车。**症状出现在两个模块之外。**
  EXPECT_EQ(clusters.size(), 2U) << "相距 1 m 的两个目标被并成一簇了";
}

TEST(EuclideanCluster, StillSeparatesThemWhenTheyAreFurtherApart)
{
  // 对照：拉到 2 m 更该分开。这条防的是"上一条恰好靠运气过了"。
  const auto left = ScanBox({20.0, +1.2, 0.85}, 0.4, 0.4, 1.7, 0.01);
  const auto right = ScanBox({20.0, -1.2, 0.85}, 0.4, 0.4, 1.7, 0.01);
  const auto clusters = ClusterEuclidean(Concat({left, right}), EuclideanClusterParams{});
  EXPECT_EQ(clusters.size(), 2U);
}

// ---------------------------------------------------------------------------
//  ⚠️ tolerance 被雷达线间距顶着 —— 这条是本文件的核心
// ---------------------------------------------------------------------------
TEST(EuclideanCluster, KeepsATargetWholeAcrossScanLinesUpToTwentyFiveMetres)
{
  // 25 m 处线间距 0.493 m，而默认 tolerance = 0.5 —— 只差 1.4%。
  // 这条用例守的正是那个余量：tolerance 一旦调小到 0.45，
  // 车就会被**按扫描线拆成三四个碎片**，每片都凑不够 min_cluster_size，
  // 于是**整个目标消失**（比拆成两个更危险 —— 后者至少还看得见）。
  const auto points = ScanBox({25.0, 0.0, 0.75}, 4.4, 1.8, 1.5, 0.01);
  const auto clusters = ClusterEuclidean(points, EuclideanClusterParams{});
  printf("[          ] 25 m 处的车：%zu 点 → %zu 个簇\n", points.size(), clusters.size());
  EXPECT_EQ(clusters.size(), 1U) << "目标被按扫描线拆开了 —— tolerance 小于线间距？";

  // 反向：把 tolerance 调到线间距**以下**，必须能看到拆簇。
  // ⚠️ 这不是"测试参数"，是证明上面那条判据**有区分力** ——
  //    如果调小也不拆，说明这个场景根本没有跨线连接的问题，
  //    那么上面那条用例就什么都没验。
  EuclideanClusterParams tight;
  tight.tolerance_m = 0.30;  // < 0.493
  tight.min_cluster_size = 1;
  const auto fragmented = ClusterEuclidean(points, tight);
  printf("[          ] tolerance 调到 0.30（< 线间距 0.493）→ %zu 个簇\n", fragmented.size());
  EXPECT_GT(fragmented.size(), 1U)
    << "tolerance 小于线间距却没拆簇 —— 那说明这个场景测不出跨线连接";
}

// ---------------------------------------------------------------------------
//  min_cluster_size：S1 实测定的那个值
// ---------------------------------------------------------------------------
TEST(EuclideanCluster, KeepsTheSparseTargetsThatSOneMeasured)
{
  // S1 体检实测：锥桶与行人在 20–25 m 的命中帧里只有 **7 点**。
  // min_cluster_size = 5 必须放它们过去。
  // ⚠️ 调到 10 的话它们直接消失，而症状是「车快撞上了才开始绕」——
  //    看起来像规划器反应慢。**这个参数是安全关键值。**
  std::vector<Eigen::Vector3d> sparse;
  std::mt19937 rng(5);
  std::normal_distribution<double> noise(0.0, 0.01);
  for (int i = 0; i < 7; ++i) {
    sparse.emplace_back(20.0 + noise(rng), 0.05 * i + noise(rng), 0.5 + noise(rng));
  }
  EXPECT_EQ(ClusterEuclidean(sparse, EuclideanClusterParams{}).size(), 1U)
    << "7 个点的稀疏目标被丢掉了 —— min_cluster_size 太大";

  EuclideanClusterParams strict;
  strict.min_cluster_size = 10;
  EXPECT_TRUE(ClusterEuclidean(sparse, strict).empty())
    << "min_cluster_size = 10 却仍然输出了 7 点的簇";
}

TEST(EuclideanCluster, DropsOversizedClustersThatMeanGroundSegmentationFailed)
{
  // 一个占了几万点的"目标"只可能是地面分割失败。
  // 宁可丢掉也不要给下游一个几十米长的障碍物 —— 那会让车立刻刹停。
  std::vector<Eigen::Vector3d> flood;
  for (int i = 0; i < 300; ++i) {
    for (int j = 0; j < 300; ++j) {
      flood.emplace_back(0.1 * i, 0.1 * j, 0.0);
    }
  }
  EuclideanClusterParams params;
  params.max_cluster_size = 20000;
  const auto clusters = ClusterEuclidean(flood, params);
  printf("[          ] 9 万点的「地面」→ %zu 个簇\n", clusters.size());
  EXPECT_TRUE(clusters.empty()) << "超大簇没被丢掉";
}

// ---------------------------------------------------------------------------
//  边界与防御
// ---------------------------------------------------------------------------
TEST(EuclideanCluster, ThrowsOnNonFiniteInput)
{
  // ⚠️ inf 会让体素下标溢出，而溢出后的行为是未定义的：可能把两个毫不
  //    相干的区域连成一簇，看起来只是"聚类效果不好"。
  auto points = ScanBox({15.0, 0.0, 0.75}, 1.0, 1.0, 1.0, 0.01);
  for (const double poison : {std::numeric_limits<double>::infinity(), std::nan("")}) {
    auto polluted = points;
    polluted[3].y() = poison;
    EXPECT_THROW(ClusterEuclidean(polluted, EuclideanClusterParams{}), std::invalid_argument);
  }
}

TEST(EuclideanCluster, RejectsContradictoryParameters)
{
  EuclideanClusterParams bad;
  bad.min_cluster_size = 50;
  bad.max_cluster_size = 10;
  // ⚠️ 这个配置下**一个簇都不会输出**，而现场表现是「感知什么都没看见」——
  //    没有任何报错。所以构造时就拒绝。
  EXPECT_THROW(ClusterEuclidean({{0, 0, 0}}, bad), std::invalid_argument);

  EuclideanClusterParams zero_tolerance;
  zero_tolerance.tolerance_m = 0.0;
  EXPECT_THROW(ClusterEuclidean({{0, 0, 0}}, zero_tolerance), std::invalid_argument);
}

TEST(EuclideanCluster, HandlesNegativeCoordinatesWithoutMergingDistantRegions)
{
  // 负坐标（车后方、地面以下）不许把远处的目标连起来。
  //
  // ⚠️ **这条抓不到 floor / 哈希的写法**（2026-08-11 故障注入实测）：
  //    把 floor 换成截断、把哈希换成三项相加，10 条用例**一条都没红**。
  //    因为那两处**本来就不影响正确性** ——
  //      · 格子变宽后 3×3×3 邻域依然覆盖半径 tolerance 的球；
  //      · `unordered_map` 桶内还会用 operator== 精确比较，碰撞 ≠ key 相同。
  //    两者影响的都是**效率**。（实现里原本的注释把后果写成了
  //    「目标被连成一簇」，是错的，已改。）
  //
  //    真正影响正确性的是 **3×3×3 邻域**：把它缩成只搜自己那一格，
  //    本文件红 5 条。那才是这一层的骨架。
  const auto front = ScanBox({20.0, 0.0, 0.85}, 0.4, 0.4, 1.7, 0.01);
  std::vector<Eigen::Vector3d> behind;
  for (const auto & point : front) {
    behind.emplace_back(-point.x(), -point.y(), point.z());
  }
  const auto clusters = ClusterEuclidean(Concat({front, behind}), EuclideanClusterParams{});
  printf("[          ] 前后各一个目标（相距 40 m）→ %zu 个簇\n", clusters.size());
  EXPECT_EQ(clusters.size(), 2U) << "相距 40 m 的两个目标被连成一簇 —— 体素哈希碰撞？";
}

// ---------------------------------------------------------------------------
//  CP-P5-A ⑤：耗时
// ---------------------------------------------------------------------------
TEST(EuclideanCluster, StaysWithinTheTimeBudget)
{
  // ⚠️ **点数必须接近真实，否则这条判据没有代表性。**
  //    S1 体检实测一帧 23499 点、非地面约占一半 ⟹ 目标约 12000 点。
  //    第一版只造了 2545 点、耗时 0.72 ms —— 那个数好看但说明不了问题，
  //    因为 BFS 的邻域查询开销随**点密度**超线性增长（同一格里的点越多，
  //    每次 3×3×3 扫描要比较的候选就越多），而点密度由距离决定。
  //
  //    近处目标点多（10 m 处一个 1×1×1.6 的盒子约 228 点，25 m 处只有 35），
  //    所以堆点数靠**把目标放近**，不是靠增加数量。
  //    ⚠️ 摆成**网格**而不是一条斜线：第一版沿斜线排、横向间距 0.31 m
  //       < tolerance 0.5，于是 60 个盒子**全连成一个簇**。点数是对的，
  //       但簇结构不真实 —— 而"一个大簇"和"几十个小簇"的邻域查询开销
  //       并不一样（前者的 visited 判断命中率高得多）。
  //       网格间距 2 m > tolerance，保证它们各自独立。
  std::vector<std::vector<Eigen::Vector3d>> parts;
  for (int row = 0; row < 8; ++row) {
    for (int col = 0; col < 8; ++col) {
      const double distance = 6.0 + 2.0 * row;
      const double lateral = -7.0 + 2.0 * col;
      parts.push_back(ScanBox({distance, lateral, 0.85}, 1.0, 1.0, 1.6, 0.01));
    }
  }
  const auto points = Concat(parts);
  printf("[          ] 非地面点 %zu 个（S1 实测一帧约 12000）\n", points.size());

  const auto started = std::chrono::steady_clock::now();
  const auto clusters = ClusterEuclidean(points, EuclideanClusterParams{});
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  printf(
    "[          ] 聚类耗时 %.2f ms，得到 %zu 个簇（应当接近 64）\n", elapsed_ms, clusters.size());

  // CP-P5-A ⑤ 的预算是「地面分割 + 聚类 < 40 ms」，地面分割实测 1.18 ms，
  // 所以这里留 35 ms。⚠️ 按 Release 定 —— CI 跑的正是 Release。
  EXPECT_LT(elapsed_ms, 35.0) << "聚类超预算，S3/S4 就没空间了";
}
