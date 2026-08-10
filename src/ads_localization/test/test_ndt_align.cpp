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
//  NDT 配准的 L1 测试
//
//  三类判据，按「能抓住什么」排：
//
//    ① 解析梯度 vs 数值微分     —— 消除符号错误。符号写反的症状是
//                                 「配准朝反方向跑然后卡住」，看起来像
//                                 收敛域太小，会把人引到调参数上去。
//    ② 恢复已知变换             —— 端到端，用**真实的** campus_cloud.pcd
//    ③ **纯地面点云必须报退化** —— P4-1 决策一那条主线在配准这一步的落点
//
//  第 ③ 类是本文件的核心。加路侧结构之前，整张地图就是纯地面；
//  那时 NDT 会「收敛」到一个任意的位姿，而且看起来完全正常。
//  这条用例把那个几何事实固化成一个机械判据。
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
#include "ads_localization/ndt_align.hpp"
#include "ads_localization/point_cloud_io.hpp"

namespace
{

using ads_localization::AlignNdt;
using ads_localization::ComputeNdtScoreTerms;
using ads_localization::NdtAlignParams;
using ads_localization::NdtAlignResult;
using ads_localization::NdtGrid;
using ads_localization::NdtGridParams;

Eigen::Isometry3d MakePose(const Eigen::Vector3d & translation, double yaw_rad)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.translation() = translation;
  return pose;
}

/// 与 ndt_align.cpp 内部那个增量作用方式一致 —— 数值微分必须沿同一条路径扰动，
/// 否则比的是两个不同的函数。
Eigen::Isometry3d ApplyIncrementForTest(
  const Eigen::Isometry3d & pose, const Eigen::Matrix<double, 6, 1> & increment)
{
  const Eigen::Vector3d rotation_vec = increment.tail<3>();
  const double angle = rotation_vec.norm();
  Eigen::Matrix3d delta = Eigen::Matrix3d::Identity();
  if (angle > 0.0) {
    delta = Eigen::AngleAxisd(angle, rotation_vec / angle).toRotationMatrix();
  }
  Eigen::Isometry3d updated = Eigen::Isometry3d::Identity();
  updated.linear() = pose.linear() * delta;
  updated.translation() = pose.translation() + increment.head<3>();
  return updated;
}

/// 一张只有单个体素、内含各向异性高斯的"地图"。
/// 用它做梯度对账是有意的：整张图只有一个体素 → 扫描点绝不会跨体素边界，
/// 于是代价函数处处光滑，数值微分才有意义
/// （真实地图上点跨过边界时代价函数是不连续的，见 NdtGrid::At 的说明）。
NdtGrid MakeSingleVoxelMap()
{
  std::mt19937 rng(4242);
  std::normal_distribution<double> unit(0.0, 1.0);
  std::vector<Eigen::Vector3d> points;
  points.reserve(8000);
  for (int i = 0; i < 8000; ++i) {
    points.emplace_back(50.0 + 0.40 * unit(rng), 50.0 + 0.25 * unit(rng), 50.0 + 0.15 * unit(rng));
  }
  NdtGridParams params;
  params.voxel_size_m = 200.0;
  return NdtGrid(points, params);
}

}  // namespace

// =============================================================================
//  1. 解析梯度 vs 数值微分
// =============================================================================

TEST(NdtScoreTerms, AnalyticGradientMatchesNumericalDifferentiation)
{
  // **这条用例存在的唯一目的是消除符号错误。**
  //
  // NDT 的梯度里有三处符号容易写反：score 的负号、exp 的负指数、
  // 雅可比里 −R[p]× 的负号。任何一处写反，配准都会朝反方向跑然后卡住 ——
  // 而那个现象看起来像「收敛域太小」，会把人引到调 max_step / 阻尼上去。
  //
  // 数值微分不关心推导，只问「函数在这个方向上到底怎么变」，所以它能
  // 一次性把三处符号全部钉死。
  const NdtGrid map = MakeSingleVoxelMap();

  // 扫描点故意放在偏离地图均值的地方，保证残差非零、梯度不为零。
  const std::vector<Eigen::Vector3d> scan_body{
    {0.30, 0.10, -0.20}, {-0.15, 0.25, 0.10}, {0.05, -0.30, 0.15}, {0.20, 0.20, 0.05}};
  const Eigen::Isometry3d pose = MakePose({49.7, 50.2, 49.9}, 0.15);

  const auto terms = ComputeNdtScoreTerms(map, scan_body, pose);
  ASSERT_EQ(terms.inlier_count, static_cast<int>(scan_body.size()));

  constexpr double kStep = 1e-6;
  for (int axis = 0; axis < 6; ++axis) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    delta[axis] = kStep;
    const double plus =
      ComputeNdtScoreTerms(map, scan_body, ApplyIncrementForTest(pose, delta)).score;
    const double minus =
      ComputeNdtScoreTerms(map, scan_body, ApplyIncrementForTest(pose, -delta)).score;
    const double numerical = (plus - minus) / (2.0 * kStep);

    // 中心差分的截断误差是 O(h²)，浮点噪声是 O(eps/h)；h=1e-6 时两者都在
    // 1e-8 量级。判据取绝对 1e-5 加相对 1e-4，远松于噪声但远严于任何符号错误
    // （符号错会让两者差**两倍**）。
    EXPECT_NEAR(terms.gradient[axis], numerical, 1e-5 + 1e-4 * std::abs(numerical))
      << "第 " << axis << " 个分量：解析 " << terms.gradient[axis] << "，数值 " << numerical;
  }
}

TEST(NdtScoreTerms, InformationMatrixIsSymmetricPositiveSemiDefinite)
{
  // Gauss-Newton 形式 Σ w·JᵀΣ⁻¹J **结构上**就是对称半正定的。
  // 它不是的话，说明写成了完整 Hessian（含那个负半定项）——
  // 那样牛顿方向可能是上坡的，而现象只是「有时候不收敛」。
  const NdtGrid map = MakeSingleVoxelMap();
  const std::vector<Eigen::Vector3d> scan_body{
    {0.30, 0.10, -0.20}, {-0.15, 0.25, 0.10}, {0.05, -0.30, 0.15}};
  const auto terms = ComputeNdtScoreTerms(map, scan_body, MakePose({49.8, 50.1, 50.0}, -0.1));

  const Eigen::Matrix<double, 6, 6> & info = terms.information;
  EXPECT_LT((info - info.transpose()).cwiseAbs().maxCoeff(), 1e-12) << "信息阵不对称";
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(info);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-12)
    << "信息阵有负特征值 " << solver.eigenvalues().minCoeff() << " —— 它必须是半正定的";
}

// =============================================================================
//  2. 恢复已知变换（真实地图，端到端）
// =============================================================================

namespace
{

/// 从真实地图里截一段当"扫描"：取车位附近半径内的点，变换回 body 系，
/// **再加上雷达量级的测距噪声**。
///
/// ⚠️ 那个噪声不是装饰，去掉它这个用例会以一种极隐蔽的方式失效
///    （2026-08-10 实测，花了四轮诊断才定位）：
///
///    点云地图里杆件按 0.1 m、墙面按 0.5 m 采样，而 NDT 体素是 2.0 m ——
///    **两者整除**。于是大量点的 z 坐标恰好落在体素边界上（z = 2.0、4.0、6.0…）。
///    而绕 z 的旋转与水平平移都**不改变 z**，所以只有在「一个字都没动」的
///    那个位姿上，这些边界点才稳定地属于某个体素；任何无穷小的扰动都会
///    让它们跨界。实测：位移 3×10⁻¹¹ m 时 inlier 从 4950 掉到 4349，
///    得分从 −1266 跳到 −933 —— 代价函数在初值处**人为地**出现一个尖峰，
///    线搜索于是第一步就找不到下降方向，配准一动不动。
///
///    这与 CLAUDE.md 里 P3-S2 那条「测试的采样步长与被测结构的周期可通约」
///    是同一个坑，只是换了个面孔。**凡是"离散结构 × 规则采样"，先查可通约性。**
///
///    真雷达有测距噪声（vehicle_params.yaml 里 lidar.noise_stddev_m = 0.01），
///    加上它之后边界巧合自然消失 —— 这不是为了过测试而调参，
///    是把场景改成真实的样子。
std::vector<Eigen::Vector3d> CarveScan(
  const std::vector<Eigen::Vector3d> & map_points, const Eigen::Isometry3d & truth, double radius_m,
  uint32_t seed = 20260810)
{
  const Eigen::Isometry3d world_to_body = truth.inverse();
  std::mt19937 rng(seed);
  std::normal_distribution<double> range_noise(0.0, 0.01);  // = lidar.noise_stddev_m
  std::vector<Eigen::Vector3d> scan;
  for (const Eigen::Vector3d & p : map_points) {
    if ((p - truth.translation()).norm() <= radius_m) {
      const Eigen::Vector3d noisy =
        p + Eigen::Vector3d(range_noise(rng), range_noise(rng), range_noise(rng));
      scan.push_back(world_to_body * noisy);
    }
  }
  return scan;
}

}  // namespace

TEST(NdtAlign, RecoversAKnownTransformOnTheCampusMap)
{
  const std::vector<Eigen::Vector3d> map_points =
    ads_localization::LoadPcdAscii(ADS_CAMPUS_CLOUD_PCD);
  const NdtGrid map(map_points, NdtGridParams{});

  // 自车 spawn 位姿（见 worlds/campus_loop.sdf），车头朝 +x。
  const Eigen::Isometry3d truth = MakePose({30.0, -51.75, 0.0}, 0.0);
  const std::vector<Eigen::Vector3d> scan = CarveScan(map_points, truth, 30.0);
  ASSERT_GT(scan.size(), 1000u) << "截出来的扫描点太少，地图是不是变了？";

  // 初值给偏 0.8 m + 3°，量级上对应 ESKF 在两帧雷达之间的推算误差。
  const Eigen::Isometry3d guess =
    MakePose(truth.translation() + Eigen::Vector3d(0.8, -0.5, 0.0), 3.0 * M_PI / 180.0);

  const NdtAlignResult result = AlignNdt(map, scan, guess, NdtAlignParams{});

  printf(
    "[          ] 迭代 %d 次，inlier %.1f%%，法向散布 %.4g（判据 1e-3）\n", result.iterations,
    100.0 * result.inlier_ratio, result.normal_diversity);

  EXPECT_FALSE(result.degenerate) << "结构齐全的地图上不该报退化";
  const double position_error = (result.pose.translation() - truth.translation()).norm();
  const double yaw_error =
    std::abs(Eigen::AngleAxisd(result.pose.linear().transpose() * truth.linear()).angle());
  printf("[          ] 位置残差 %.4f m，姿态残差 %.5f rad\n", position_error, yaw_error);

  // 扫描点直接取自地图，所以理论最优就是真值。判据留一个体素尺寸的
  // 十分之一（0.2 m）—— 再紧就会被「只查一个体素」造成的代价函数
  // 不连续性影响（见 NdtGrid::At 的说明）。
  EXPECT_LT(position_error, 0.20) << "位置没收敛回真值";
  EXPECT_LT(yaw_error, 0.02) << "姿态没收敛回真值";
}

TEST(NdtAlign, ReportsAPlausibleCovarianceWhenItSucceeds)
{
  const std::vector<Eigen::Vector3d> map_points =
    ads_localization::LoadPcdAscii(ADS_CAMPUS_CLOUD_PCD);
  const NdtGrid map(map_points, NdtGridParams{});
  const Eigen::Isometry3d truth = MakePose({30.0, -51.75, 0.0}, 0.0);
  const std::vector<Eigen::Vector3d> scan = CarveScan(map_points, truth, 30.0);

  const NdtAlignResult result = AlignNdt(map, scan, truth, NdtAlignParams{});
  ASSERT_FALSE(result.degenerate);

  // 协方差必须对称正定 —— 它要喂进 ESKF 当观测噪声，不定的话卡尔曼增益
  // 会给出荒谬的方向。
  EXPECT_LT((result.covariance - result.covariance.transpose()).cwiseAbs().maxCoeff(), 1e-12);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(result.covariance);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

// =============================================================================
//  3. 退化 —— 本文件的核心
// =============================================================================

TEST(NdtAlign, GroundOnlyMapIsReportedAsDegenerate)
{
  // **这是 P4-1 决策一那条主线在配准这一步的落点。**
  //
  // 加路侧结构之前，campus_loop 里所有点的法向都是 (0,0,1)。
  // 那样的地图只约束 z/roll/pitch，x/y/yaw 完全不可观 ——
  // 代价函数沿这三个方向是**平的**，配准会「收敛」到一个任意的位姿，
  // 而残差、迭代次数、协方差看起来全都正常。
  //
  // 这条用例把那个几何事实变成一个机械判据：**必须报退化**。
  std::mt19937 rng(20260810);
  std::uniform_real_distribution<double> spread(-40.0, 40.0);
  std::normal_distribution<double> thickness(0.0, 0.01);  // 与雷达噪声同量级

  std::vector<Eigen::Vector3d> ground;
  ground.reserve(40000);
  for (int i = 0; i < 40000; ++i) {
    ground.emplace_back(spread(rng), spread(rng), thickness(rng));
  }
  const NdtGrid map(ground, NdtGridParams{});

  // 扫描同样只有地面。
  std::vector<Eigen::Vector3d> scan;
  for (int i = 0; i < 4000; ++i) {
    scan.emplace_back(spread(rng) * 0.5, spread(rng) * 0.5, thickness(rng));
  }

  const NdtAlignResult result =
    AlignNdt(map, scan, Eigen::Isometry3d::Identity(), NdtAlignParams{});

  printf(
    "[          ] 纯地面：法向散布 %.4g，信息阵条件数 %.4g，inlier %.1f%%\n",
    result.normal_diversity, result.condition_number, 100.0 * result.inlier_ratio);

  EXPECT_TRUE(result.degenerate) << "纯地面点云上必须报退化。法向散布 " << result.normal_diversity;
  // ⚠️ 顺带把「为什么不能用信息阵条件数」钉住：它在这里**看起来很正常**。
  // 走过一次弯路才发现的 —— 体素离散化会伪造面内信息。
  EXPECT_LT(result.normal_diversity, 1e-4) << "法向散布应当极小（所有法向都是 +z）";
  // 退化时协方差保持全零 —— 那是「不要用这个结果」的显式信号，
  // 而不是一个看起来很小、会被下游当成"很准"的数。
  EXPECT_TRUE(result.covariance.isZero()) << "退化时不该给出协方差";
}

TEST(NdtAlign, CorridorIsReportedAsDegenerateAlongItsAxis)
{
  // **走廊**：两面平行的墙 + 地面。这是自动驾驶里最经典的 NDT 失效场景 ——
  // 隧道、长直的两侧连续建筑、货架间的巷道。
  //
  // 法向只有 ±y（墙）和 +z（地面）两种，**沿走廊方向（x）完全没有约束**。
  // 配准会沿 x 自由滑动，而残差、迭代次数、inlier 比例全都正常。
  //
  // ⚠️ 这条用例是故障注入逼出来的：把体素法向取成**最大**特征值方向
  //    （而不是最小）时，纯地面那条用例照样通过 —— 因为地面上两种取法
  //    碰巧给出同样的判定。走廊能把它们分开。
  std::mt19937 rng(20260811);
  std::uniform_real_distribution<double> along(-40.0, 40.0);
  std::uniform_real_distribution<double> height(0.0, 4.0);
  std::uniform_real_distribution<double> across(-5.0, 5.0);
  std::normal_distribution<double> jitter(0.0, 0.01);

  std::vector<Eigen::Vector3d> corridor;
  for (int i = 0; i < 20000; ++i) {
    corridor.emplace_back(along(rng), across(rng), jitter(rng));         // 地面
    corridor.emplace_back(along(rng), -5.0 + jitter(rng), height(rng));  // 左墙
    corridor.emplace_back(along(rng), 5.0 + jitter(rng), height(rng));   // 右墙
  }
  const NdtGrid map(corridor, NdtGridParams{});

  std::vector<Eigen::Vector3d> scan;
  for (int i = 0; i < 3000; ++i) {
    scan.emplace_back(along(rng) * 0.4, across(rng), jitter(rng));
    scan.emplace_back(along(rng) * 0.4, -5.0 + jitter(rng), height(rng));
    scan.emplace_back(along(rng) * 0.4, 5.0 + jitter(rng), height(rng));
  }

  const NdtAlignResult result =
    AlignNdt(map, scan, Eigen::Isometry3d::Identity(), NdtAlignParams{});
  printf(
    "[          ] 走廊：法向散布 %.4g，inlier %.1f%%\n", result.normal_diversity,
    100.0 * result.inlier_ratio);

  EXPECT_TRUE(result.degenerate) << "走廊沿轴向不可观，必须报退化。法向散布 "
                                 << result.normal_diversity;
  EXPECT_TRUE(result.covariance.isZero());
}

TEST(NdtAlign, NoOverlapWithTheMapIsReportedAsDegenerate)
{
  // 扫描完全落在地图之外（车开到了没建图的地方）。
  // 此时哪怕 H 的条件数正常，结果也不可信 —— inlier 比例是那道闸。
  const NdtGrid map = MakeSingleVoxelMap();
  const std::vector<Eigen::Vector3d> scan{
    {0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, {0.0, 0.1, 0.0}, {0.0, 0.0, 0.1}};
  // 初值把扫描摆到离那个体素 1000 m 外
  const NdtAlignResult result =
    AlignNdt(map, scan, MakePose({1000.0, 1000.0, 0.0}, 0.0), NdtAlignParams{});

  EXPECT_TRUE(result.degenerate);
  EXPECT_DOUBLE_EQ(result.inlier_ratio, 0.0);
  EXPECT_TRUE(result.covariance.isZero());
}

TEST(NdtAlignValidation, IllegalInputsAreRejected)
{
  const NdtGrid map = MakeSingleVoxelMap();
  const std::vector<Eigen::Vector3d> scan{{0.0, 0.0, 0.0}};

  EXPECT_THROW(
    AlignNdt(map, {}, Eigen::Isometry3d::Identity(), NdtAlignParams{}), std::invalid_argument)
    << "空扫描必须拒绝";

  NdtAlignParams bad;
  bad.max_iterations = 0;
  EXPECT_THROW(AlignNdt(map, scan, Eigen::Isometry3d::Identity(), bad), std::invalid_argument);

  // ⚠️ gpu_lidar 的无回波射线返回 ±inf 不是 NaN，两者都要拦。
  for (const double poison : {std::nan(""), std::numeric_limits<double>::infinity()}) {
    const std::vector<Eigen::Vector3d> poisoned{{0.0, 0.0, 0.0}, {poison, 0.0, 0.0}};
    EXPECT_THROW(
      AlignNdt(map, poisoned, Eigen::Isometry3d::Identity(), NdtAlignParams{}),
      std::invalid_argument)
      << "漏掉了 " << poison;
  }
}

// =============================================================================
//  诊断用（默认不跑）：看清楚配准在真实地图上每一步到底发生了什么
// =============================================================================

TEST(NdtAlignDiagnostics, DISABLED_StepByStepOnCampusMap)
{
  const std::vector<Eigen::Vector3d> map_points =
    ads_localization::LoadPcdAscii(ADS_CAMPUS_CLOUD_PCD);
  const NdtGrid map(map_points, NdtGridParams{});
  const Eigen::Isometry3d truth = MakePose({30.0, -51.75, 0.0}, 0.0);
  const std::vector<Eigen::Vector3d> scan = CarveScan(map_points, truth, 30.0);
  printf("扫描点 %zu 个\n", scan.size());

  for (const double offset : {0.0, 0.05, 0.2, 0.5, 0.94}) {
    const Eigen::Isometry3d guess =
      MakePose(truth.translation() + Eigen::Vector3d(offset, 0.0, 0.0), 0.0);
    const auto terms = ComputeNdtScoreTerms(map, scan, guess);
    printf(
      "  x 偏移 %.2f m：得分 %12.2f  梯度范数 %10.3g  inlier %d\n", offset, terms.score,
      terms.gradient.norm(), terms.inlier_count);
  }

  // 沿 +x 走一小步，看得分到底往哪边变 —— 如果偏移 0.94 时得分比偏移 0 还好，
  // 那说明代价函数在这里有个假极小，问题在地图/权重而不在优化器。
  printf("\n沿 x 扫一遍得分（真值在 0.0）：\n");
  for (double dx = -1.5; dx <= 1.5001; dx += 0.25) {
    const auto terms = ComputeNdtScoreTerms(
      map, scan, MakePose(truth.translation() + Eigen::Vector3d(dx, 0.0, 0.0), 0.0));
    printf("  dx=%+.2f  得分 %12.2f  inlier %d\n", dx, terms.score, terms.inlier_count);
  }
}

TEST(NdtAlignDiagnostics, DISABLED_CompareInformationOfGoodAndDegenerateCases)
{
  const auto report = [](const char * label, const Eigen::Matrix<double, 6, 6> & info) {
    const Eigen::Matrix<double, 6, 6> cov = info.inverse();
    printf(
      "%s\n  σ 平移 (%.4g %.4g %.4g) m   σ 旋转 (%.4g %.4g %.4g) rad\n", label,
      std::sqrt(cov(0, 0)), std::sqrt(cov(1, 1)), std::sqrt(cov(2, 2)), std::sqrt(cov(3, 3)),
      std::sqrt(cov(4, 4)), std::sqrt(cov(5, 5)));
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> s(info);
    printf(
      "  信息阵特征值 %.3g … %.3g（条件数 %.3g）\n", s.eigenvalues().minCoeff(),
      s.eigenvalues().maxCoeff(), s.eigenvalues().maxCoeff() / s.eigenvalues().minCoeff());
  };

  const std::vector<Eigen::Vector3d> map_points =
    ads_localization::LoadPcdAscii(ADS_CAMPUS_CLOUD_PCD);
  const NdtGrid good_map(map_points, NdtGridParams{});
  const Eigen::Isometry3d truth = MakePose({30.0, -51.75, 0.0}, 0.0);
  const std::vector<Eigen::Vector3d> good_scan = CarveScan(map_points, truth, 30.0);
  report("【结构齐全】", ComputeNdtScoreTerms(good_map, good_scan, truth).information);

  std::mt19937 rng(20260810);
  std::uniform_real_distribution<double> spread(-40.0, 40.0);
  std::normal_distribution<double> thickness(0.0, 0.01);
  std::vector<Eigen::Vector3d> ground;
  for (int i = 0; i < 40000; ++i) {
    ground.emplace_back(spread(rng), spread(rng), thickness(rng));
  }
  const NdtGrid flat_map(ground, NdtGridParams{});
  std::vector<Eigen::Vector3d> flat_scan;
  for (int i = 0; i < 4000; ++i) {
    flat_scan.emplace_back(spread(rng) * 0.5, spread(rng) * 0.5, thickness(rng));
  }
  report(
    "【纯地面】",
    ComputeNdtScoreTerms(flat_map, flat_scan, Eigen::Isometry3d::Identity()).information);
}
