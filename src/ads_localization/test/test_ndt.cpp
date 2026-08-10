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
//  NDT 体素化的 L1 测试
//
//  判据分两类：
//    ① 高斯拟合对不对        → 与解析解比对（从已知分布采样）
//    ② **退化情形有没有被处理** → 共面点云（协方差 rank 2）、稀疏体素
//
//  第 ② 类是本文件的核心。它守的是 P4 全阶段那条主线在体素尺度上的重现：
//  一块路面上的点全在同一平面里，协方差沿法向的特征值 ≈ 0，求逆会炸 ——
//  或者更糟，给出天文数字的权重让那**一个**体素主导整个代价函数，
//  而配准照样"收敛"，只是收敛到一个错位姿。
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "ads_localization/ndt.hpp"
#include "ads_localization/point_cloud_io.hpp"

namespace
{

using ads_localization::NdtGrid;
using ads_localization::NdtGridParams;
using ads_localization::NdtVoxel;

/// 在一个体素中心附近，按给定协方差采样若干点。
std::vector<Eigen::Vector3d> SampleGaussian(
  const Eigen::Vector3d & mean, const Eigen::Vector3d & std_dev, int count, uint32_t seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> unit(0.0, 1.0);
  std::vector<Eigen::Vector3d> points;
  points.reserve(count);
  for (int i = 0; i < count; ++i) {
    points.emplace_back(
      mean.x() + std_dev.x() * unit(rng), mean.y() + std_dev.y() * unit(rng),
      mean.z() + std_dev.z() * unit(rng));
  }
  return points;
}

}  // namespace

// =============================================================================
//  1. 高斯拟合：与解析解比对
// =============================================================================

TEST(NdtGrid, RecoversTheDistributionItWasBuiltFrom)
{
  // 从一个已知的各向异性高斯采样，体素化之后应当把它还原回来。
  // 判据用**协方差的逆**而不是协方差本身，因为那才是配准真正用到的量。
  // ⚠️ 均值必须落在体素**内部**而不是坐标原点附近：体素索引用 floor(p/size)，
  //    所以哪怕只有一个点的坐标是 −0.001，它也会掉进索引 −1 的那个体素里。
  //    第一版把均值放在 (1.0, 0.5, 0.25)，采样的尾巴跨过了原点，
  //    于是 grid.size() 是 4 而不是 1 —— 那是用例写错了，不是代码错了。
  const Eigen::Vector3d mean(50.0, 50.0, 50.0);
  const Eigen::Vector3d std_dev(0.30, 0.20, 0.10);

  NdtGridParams params;
  params.voxel_size_m = 100.0;  // 一个大体素装下全部点，避免被切开
  NdtGrid grid(SampleGaussian(mean, std_dev, 20000, 12345), params);

  ASSERT_EQ(grid.size(), 1u);
  const NdtVoxel * voxel = grid.At(mean);
  ASSERT_NE(voxel, nullptr);

  EXPECT_NEAR(voxel->mean.x(), mean.x(), 0.01);
  EXPECT_NEAR(voxel->mean.y(), mean.y(), 0.01);
  EXPECT_NEAR(voxel->mean.z(), mean.z(), 0.01);

  // 协方差应当是 diag(σ²)，其逆是 diag(1/σ²)。2 万个样本下相对误差在 3% 量级。
  const Eigen::Matrix3d covariance = voxel->inverse_covariance.inverse();
  for (int i = 0; i < 3; ++i) {
    const double expected = std_dev[i] * std_dev[i];
    EXPECT_NEAR(covariance(i, i), expected, 0.05 * expected)
      << "第 " << i << " 个对角元：" << covariance(i, i) << " vs " << expected;
  }
  // 各向异性的分布**不该**被正则化 —— 最小/最大特征值比是 (0.10/0.30)² = 0.111，
  // 远大于 0.01 的下限。这条同时验了「下限不会误伤正常体素」。
  EXPECT_FALSE(voxel->was_regularized);
  EXPECT_EQ(grid.regularized_voxels(), 0u);
}

TEST(NdtGrid, SplitsPointsIntoTheExpectedVoxels)
{
  // 体素索引用 floor(p / size)，所以 [0,2) 与 [2,4) 是两个体素。
  // 边界点归属写错（比如用了 round）的症状是相邻体素各少一半点，
  // 而两个体素看起来都"有内容"。
  NdtGridParams params;
  params.voxel_size_m = 2.0;
  params.min_points_per_voxel = 4;

  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 10; ++i) {
    points.emplace_back(0.5 + 0.01 * i, 0.5, 0.5);   // 体素 (0,0,0)
    points.emplace_back(2.5 + 0.01 * i, 0.5, 0.5);   // 体素 (1,0,0)
    points.emplace_back(-0.5 - 0.01 * i, 0.5, 0.5);  // 体素 (-1,0,0)
  }
  NdtGrid grid(points, params);
  EXPECT_EQ(grid.size(), 3u);
  ASSERT_NE(grid.At(Eigen::Vector3d(1.0, 1.0, 1.0)), nullptr);
  ASSERT_NE(grid.At(Eigen::Vector3d(3.0, 1.0, 1.0)), nullptr);
  ASSERT_NE(grid.At(Eigen::Vector3d(-1.0, 1.0, 1.0)), nullptr);
  EXPECT_EQ(grid.At(Eigen::Vector3d(100.0, 0.0, 0.0)), nullptr) << "空体素必须返回 nullptr";
}

// =============================================================================
//  2. 退化情形 —— 本文件的核心
// =============================================================================

TEST(NdtGrid, CoplanarPointsAreRegularizedInsteadOfExploding)
{
  // **这是 P4 那条主线在体素尺度上的重现。**
  //
  // 一块路面上的点全在 z = 常数 的平面里，协方差沿 z 的特征值 ≈ 0。
  // 不做正则化的话，协方差的逆在 z 方向上是天文数字 ——
  // 这个体素会以巨大的权重主导整个代价函数，而配准照样"收敛"，
  // 只是收敛到一个错位姿。
  NdtGridParams params;
  params.voxel_size_m = 100.0;
  params.eigenvalue_ratio_floor = 0.01;

  // 同样把平面挪到体素内部，理由见上一个用例里关于 floor 的说明。
  std::mt19937 rng(777);
  std::uniform_real_distribution<double> plane(49.0, 51.0);
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 5000; ++i) {
    points.emplace_back(plane(rng), plane(rng), 50.0);  // 严格共面
  }

  NdtGrid grid(points, params);
  ASSERT_EQ(grid.size(), 1u);
  EXPECT_EQ(grid.regularized_voxels(), 1u) << "共面体素必须被识别为退化并修正";

  const NdtVoxel * voxel = grid.At(Eigen::Vector3d(50.0, 50.0, 50.0));
  ASSERT_NE(voxel, nullptr);
  EXPECT_TRUE(voxel->was_regularized);
  EXPECT_TRUE(voxel->inverse_covariance.allFinite()) << "协方差的逆必须是有限的";

  // 修正之后最小特征值应当恰好是 λ_max·ratio，条件数因此被钉在 1/ratio。
  const Eigen::Matrix3d covariance = voxel->inverse_covariance.inverse();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  const Eigen::Vector3d eigenvalues = solver.eigenvalues();
  const double condition = eigenvalues.maxCoeff() / eigenvalues.minCoeff();
  EXPECT_NEAR(condition, 1.0 / params.eigenvalue_ratio_floor, 1.0)
    << "条件数 " << condition << "，应当被下限钉在 " << (1.0 / params.eigenvalue_ratio_floor);
}

TEST(NdtGrid, SparseVoxelsAreDiscardedAndCounted)
{
  // 稀疏体素必须丢掉**并且报出数量**。静默丢弃会让人以为地图覆盖是完整的，
  // 而 NDT 在"有洞"的那一段飘掉时，没人会想到是建图阶段丢的
  // —— 这是 CLAUDE.md 里「No silent caps」那条。
  NdtGridParams params;
  params.voxel_size_m = 2.0;
  params.min_points_per_voxel = 6;

  std::vector<Eigen::Vector3d> points;
  // 体素 A：8 个点，够
  for (int i = 0; i < 8; ++i) {
    points.emplace_back(0.2 + 0.05 * i, 0.3, 0.4 + 0.03 * i);
  }
  // 体素 B：3 个点，不够
  for (int i = 0; i < 3; ++i) {
    points.emplace_back(10.2 + 0.05 * i, 0.3, 0.4);
  }

  NdtGrid grid(points, params);
  EXPECT_EQ(grid.size(), 1u);
  EXPECT_EQ(grid.discarded_sparse_voxels(), 1u);
  EXPECT_EQ(grid.At(Eigen::Vector3d(10.5, 0.5, 0.5)), nullptr) << "点数不足的体素不该被建出来";
}

TEST(NdtGridParamsValidation, IllegalParametersAreRejected)
{
  const std::vector<Eigen::Vector3d> points{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
                                            {1, 1, 0}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};

  NdtGridParams zero_size;
  zero_size.voxel_size_m = 0.0;
  EXPECT_THROW(NdtGrid(points, zero_size), std::invalid_argument);

  NdtGridParams nan_floor;
  nan_floor.eigenvalue_ratio_floor = std::nan("");
  EXPECT_THROW(NdtGrid(points, nan_floor), std::invalid_argument);

  // ratio ≥ 1 意味着"最小特征值 ≥ 最大特征值"，所有体素都被抹成球，
  // NDT 退化成点到点心距离 —— 法向信息全丢，而它不会报错。
  NdtGridParams unit_floor;
  unit_floor.eigenvalue_ratio_floor = 1.0;
  EXPECT_THROW(NdtGrid(points, unit_floor), std::invalid_argument);

  // 少于 4 个点时样本协方差自由度 ≤ 0，算出来没有统计意义。
  NdtGridParams too_few;
  too_few.min_points_per_voxel = 3;
  EXPECT_THROW(NdtGrid(points, too_few), std::invalid_argument);

  NdtGridParams ok;
  EXPECT_THROW(NdtGrid({}, ok), std::invalid_argument) << "空点集必须拒绝";
}

TEST(NdtGrid, RejectsNonFinitePoints)
{
  // ⚠️ gpu_lidar 的无回波射线返回的是 **±inf 不是 NaN**（CLAUDE.md 有专门一条），
  //    所以两者都要拦。用比较去拦是拦不住的 —— NaN 参与任何比较都返回 false。
  NdtGridParams params;
  for (const double bad :
       {std::nan(""), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    std::vector<Eigen::Vector3d> points{{0, 0, 0}, {1, 0, 0}, {0, 1, 0},
                                        {0, 0, 1}, {1, 1, 0}, {bad, 0, 0}};
    EXPECT_THROW(NdtGrid(points, params), std::invalid_argument) << "漏掉了 " << bad;
  }
}

// =============================================================================
//  3. 真实地图的端到端体检
// =============================================================================

TEST(NdtGridOnCampusMap, MatchesTheNumbersMeasuredWhenTheMapWasDesigned)
{
  // 用**真实的** maps/campus_cloud.pcd 跑一遍，把 S1 设计阶段量出来的那几个
  // 数字固化成判据。它们变了就说明地图或体素化被动过 ——
  // 而地图变了却没人注意到，正是「NDT 稳定地收敛到一个错位姿」的来源。
  const std::vector<Eigen::Vector3d> points = ads_localization::LoadPcdAscii(ADS_CAMPUS_CLOUD_PCD);
  EXPECT_EQ(points.size(), 82573u) << "点云地图的点数变了，是不是改了 campus_map.yaml？";

  NdtGridParams params;  // voxel 2 m、min 6 点、floor 0.01
  NdtGrid grid(points, params);

  printf(
    "[          ] 非空体素 %zu，稀疏丢弃 %zu，退化修正 %zu（%.1f%%）\n", grid.size(),
    grid.discarded_sparse_voxels(), grid.regularized_voxels(),
    100.0 * static_cast<double>(grid.regularized_voxels()) / static_cast<double>(grid.size()));

  // S1 体检量到 4354 个非空体素、其中 8.3% 少于 5 点。这里的阈值是 6，
  // 所以丢弃数会略多一些。判据取范围而不是精确值：精确值会因为
  // 体素边界上的浮点归属而抖动，而我们要抓的是"数量级对不对"。
  EXPECT_GT(grid.size(), 3500u) << "非空体素太少，地图是不是被截断了？";
  EXPECT_LT(grid.discarded_sparse_voxels(), grid.size() / 2)
    << "丢弃的体素比留下的还多，min_points_per_voxel 是不是太大了？";

  // **地图里必须有相当比例的体素是退化的** —— 那正是路面。
  // 这个数接近 0 反而说明有问题：要么点云里没有路面，要么正则化没生效。
  EXPECT_GT(grid.regularized_voxels(), 0u) << "一个退化体素都没有？路面点本该是共面的";
}

TEST(PointCloudIo, RejectsMalformedFiles)
{
  EXPECT_THROW(ads_localization::LoadPcdAscii("/nonexistent/nope.pcd"), std::runtime_error);
}
