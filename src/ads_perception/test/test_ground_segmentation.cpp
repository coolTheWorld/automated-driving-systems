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
//  地面分割的 L1 判据 —— **全部跑在带坡度的合成地面上**
//
//  ⚠️ 这不是"顺便也测测斜面"，而是这组用例存在的**唯一理由**。
//
//  campus 世界的地面是理想平面，在那上面 RANSAC 与「z < 0.3 就是地面」
//  给出完全相同的结果。也就是说：**平地上的用例全绿，什么都没证明** ——
//  把整个 RANSAC 换成一行阈值判断也照样通过。
//
//  这与 P3-S2「采样步长与段长可通约 → 用例全绿但什么都没测」、
//  P4「不给扫描加噪声 → NDT 一步都不动」是同一类问题：
//  **理想化的场景会让判据失去区分力。**
//
//  ## 故障注入实测（2026-08-11，写完立刻做的）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 把 RANSAC 整个换成「z < 0.3 就是地面」 | **红 2 条**（坡度扫描 + 锥桶） |
//  | 去掉坡度检查（墙可以赢） | **红 1 条**（墙那条） |
//  | 去掉最小二乘精化 | **红 1 条**（坡度扫描，拟合角度超差） |
//
//  第一条是这组用例存在的**全部理由** —— 它证明判据能分辨 RANSAC 与
//  一行阈值判断。在平地上做同样的注入，**一条都不会红**。
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

#include "ads_perception/ground_segmentation.hpp"

namespace
{

using ads_perception::GroundSegmentationParams;
using ads_perception::GroundSegmentationResult;
using ads_perception::SegmentGround;

/// 一帧合成点云：带坡度的地面 + 若干立方体障碍物。
struct Scene
{
  std::vector<Eigen::Vector3d> points;
  /// 与 points 等长的真值标注：1 = 地面。
  std::vector<std::uint8_t> truth_is_ground;
  /// 与 points 等长：障碍物点的**离地高度**（地面点填 −1）。
  /// ⚠️ 判据要按它分层，见 Score::leak_above 的说明。
  std::vector<double> height_above_ground_m;
};

/// 造一个坡度为 slope_rad 的地面（绕 y 轴倾斜，即沿 x 方向上坡）。
///
/// @param slope_rad     坡度
/// @param noise_stddev  测距噪声。**必须非零** —— 无噪声的完美平面上，
///                      三点定的平面就精确等于真平面，最小二乘精化那一步
///                      永远无事可做，于是那段代码删掉也不会有任何用例变红。
Scene MakeSlopedGround(double slope_rad, double noise_stddev = 0.01)
{
  Scene scene;
  std::mt19937 rng(1234);
  std::normal_distribution<double> noise(0.0, noise_stddev);
  const double tangent = std::tan(slope_rad);

  // 地面：40 m × 20 m，0.25 m 网格 → 12800 点。与实测一帧的量级相当。
  for (double x = -10.0; x <= 30.0; x += 0.25) {
    for (double y = -10.0; y <= 10.0; y += 0.25) {
      scene.points.emplace_back(x, y, x * tangent + noise(rng));
      scene.truth_is_ground.push_back(1U);
      scene.height_above_ground_m.push_back(-1.0);
    }
  }
  return scene;
}

/// 按**真实雷达角分辨率**造地面点 —— 密度随距离 1/d² 衰减。
///
/// 每条扫描线打在地面上画一个圆：第 k 条线的俯角 θ_k 决定半径 d_k = h/tan(θ_k)，
/// 圆上的点按水平角分辨率分布。于是近处的圆密集、远处稀疏 ——
/// 那正是均匀网格造不出来、而 RANSAC 的「内点最多」判据会被它带偏的性质。
Scene MakeLidarLikeGround(double slope_rad = 0.0, double noise_stddev = 0.02)
{
  Scene scene;
  std::mt19937 rng(9876);
  std::normal_distribution<double> noise(0.0, noise_stddev);
  const double sensor_height_m = 1.6;
  const double vertical_step_rad = (0.1745 - (-0.4363)) / 31.0;
  const double horizontal_step_rad = 0.2 * M_PI / 180.0;
  const double tangent = std::tan(slope_rad);

  // 只有向下的线才打得到地面。俯角从 25° 到接近 0（最远）。
  for (int line = 0; line < 32; ++line) {
    const double elevation_rad = -0.4363 + line * vertical_step_rad;
    if (elevation_rad >= -0.01) {
      break;  // 水平或向上的线打不到地面
    }
    const double radius_m = sensor_height_m / std::tan(-elevation_rad);
    if (radius_m > 60.0) {
      continue;  // 超出量程
    }
    // 一圈的点数 = 2π / 水平角分辨率，但只取前方半圈（车后方也有，量级相同）。
    const int samples = static_cast<int>(2.0 * M_PI / horizontal_step_rad);
    for (int i = 0; i < samples; ++i) {
      const double azimuth_rad = i * horizontal_step_rad;
      const double x = radius_m * std::cos(azimuth_rad);
      const double y = radius_m * std::sin(azimuth_rad);
      scene.points.emplace_back(x, y, x * tangent + noise(rng));
      scene.truth_is_ground.push_back(1U);
      scene.height_above_ground_m.push_back(-1.0);
    }
  }
  return scene;
}

/// 往场景里加一个立方体（底面贴在坡面上）。
void AddBox(
  Scene * scene, double center_x, double center_y, double length, double width, double height,
  double slope_rad)
{
  std::mt19937 rng(static_cast<std::uint32_t>(center_x * 1000.0 + center_y));
  std::normal_distribution<double> noise(0.0, 0.01);
  const double tangent = std::tan(slope_rad);
  // ⚠️ **底面必须贴着坡面，不能水平地插在坡里。**（2026-08-11 实测踩到）
  //    第一版用 `base_z = center_x · tan(坡度)` 给整个盒子一个**水平**底面，
  //    于是长 4.4 m 的车在 5° 坡上一端翘起 0.19 m、另一端**埋进坡里** 0.19 m。
  //    埋进去那一端 dz = 0.30 的点实际离坡面只有 0.11 m < 阈值 0.15，
  //    被正确地判成了地面 —— 而用例把它算成了算法的漏判。
  //
  //    **是夹具错了，不是算法错了。** 而这个错误只在 5° 上暴露
  //    （1°/3° 的埋深 0.04/0.11 m 还够不着阈值），也就是说
  //    **坡度扫描不能只测一个角度** —— 那正是这组用例要扫 1/3/5 的原因。
  // 只造侧面与顶面的点（雷达打不到内部），0.05 m 间隔。
  for (double dx = -length / 2; dx <= length / 2; dx += 0.05) {
    for (double dy = -width / 2; dy <= width / 2; dy += 0.05) {
      for (double dz = 0.10; dz <= height; dz += 0.05) {
        const bool on_shell = std::abs(std::abs(dx) - length / 2) < 0.03 ||
                              std::abs(std::abs(dy) - width / 2) < 0.03 ||
                              std::abs(dz - height) < 0.03;
        if (!on_shell) {
          continue;
        }
        // 每个点用**它自己的 x** 算坡面高度 —— 这才是"贴着坡面放"。
        const double x = center_x + dx;
        scene->points.emplace_back(x, center_y + dy, x * tangent + dz + noise(rng));
        scene->truth_is_ground.push_back(0U);
        scene->height_above_ground_m.push_back(dz);
      }
    }
  }
}

/// 召回率与误判率。
/// 分层高度，m。**判据的分界线**，见 leak_above 的说明。
constexpr double kLeakBandM = 0.30;

struct Score
{
  double ground_recall{0.0};    ///< 真地面里被判成地面的比例
  double non_ground_leak{0.0};  ///< **真障碍物**里被误判成地面的比例（参考量）
  /// 离地 **kLeakBandM 以上**的障碍物点里被判成地面的**个数**。
  ///
  /// ⚠️ **这才是判据该量的东西，`non_ground_leak` 不是。**（2026-08-11 实测定的）
  ///
  /// 实测漏判点的高度分布：dz=0.10 m 漏 70.8%、0.15 m 漏 48.2%、0.20 m 漏 14.4%、
  /// 0.25 m 漏 2.1%、**再往上一个都不漏**。也就是说漏判**全部**来自
  /// 障碍物贴地的那一圈 —— 那是 `distance_threshold_m = 0.15` 的**定义使然**，
  /// 不是缺陷：离地面 15 cm 以内的点本来就该被当成地面。
  ///
  /// 于是「总漏判率 < 5%」这个判据把两件完全不同的事混在一起：
  ///   · 底部一圈被吸收 —— 正常，下游还剩 94% 的点，够成簇
  ///   · **整个障碍物被吞掉** —— 灾难，车会撞上去
  /// 实测总漏判率 5.3–5.8%，卡在 5% 上，而系统其实是对的。
  /// **这与 CP-P2-B 那条「判据量的是路不是车」是同一类错误。**
  ///
  /// 换成分层之后判据反而**更严**：0.3 m 以上**一个都不许漏**（不是「少于 5%」）。
  int leak_above{0};
  int total_above{0};
};

Score Evaluate(const Scene & scene, const GroundSegmentationResult & result)
{
  int ground_total = 0;
  int ground_hit = 0;
  int obstacle_total = 0;
  int obstacle_leak = 0;
  for (std::size_t i = 0; i < scene.points.size(); ++i) {
    if (scene.truth_is_ground[i] != 0U) {
      ++ground_total;
      ground_hit += result.is_ground[i] != 0U ? 1 : 0;
    } else {
      ++obstacle_total;
      obstacle_leak += result.is_ground[i] != 0U ? 1 : 0;
    }
  }
  Score score;
  score.ground_recall = ground_total > 0 ? static_cast<double>(ground_hit) / ground_total : 0.0;
  score.non_ground_leak =
    obstacle_total > 0 ? static_cast<double>(obstacle_leak) / obstacle_total : 0.0;
  for (std::size_t i = 0; i < scene.points.size(); ++i) {
    if (scene.height_above_ground_m[i] < kLeakBandM) {
      continue;
    }
    ++score.total_above;
    score.leak_above += result.is_ground[i] != 0U ? 1 : 0;
  }
  return score;
}

}  // namespace

// ---------------------------------------------------------------------------
//  CP-P5-A 判据 ①②：带坡度地面的召回率与误判率
// ---------------------------------------------------------------------------
TEST(GroundSegmentation, RecoversSlopedGroundAtOneThreeAndFiveDegrees)
{
  for (const double degrees : {1.0, 3.0, 5.0}) {
    const double slope_rad = degrees * M_PI / 180.0;
    Scene scene = MakeSlopedGround(slope_rad);
    // 三个障碍物：一辆车、一个锥桶、一个行人 —— 尺寸与 P5 的实际目标一致。
    AddBox(&scene, 12.0, 2.0, 4.4, 1.8, 1.5, slope_rad);
    AddBox(&scene, 18.0, -3.0, 0.5, 0.5, 0.8, slope_rad);
    AddBox(&scene, 8.0, -5.0, 0.4, 0.4, 1.7, slope_rad);

    const GroundSegmentationResult result = SegmentGround(scene.points, GroundSegmentationParams{});
    ASSERT_TRUE(result.found) << degrees << "° 坡上没找到地面";

    const Score score = Evaluate(scene, result);
    // 拟合出来的坡度应当接近真值。normal.z() = cos(坡度)。
    const double fitted_deg = std::acos(result.normal.z()) * 180.0 / M_PI;
    printf(
      "[          ] %.0f° 坡：拟合 %.3f°，地面召回 %.4f，总漏判 %.4f（参考），"
      "0.3 m 以上漏判 %d/%d\n",
      degrees, fitted_deg, score.ground_recall, score.non_ground_leak, score.leak_above,
      score.total_above);

    EXPECT_NEAR(fitted_deg, degrees, 0.3) << "拟合的坡度偏了 —— 法向精化那一步有问题？";
    // CP-P5-A ①：地面点召回 > 95%
    EXPECT_GT(score.ground_recall, 0.95) << degrees << "° 坡上地面召回不足";
    // CP-P5-A ②（**已按实测重定义**，见 Score::leak_above）：
    // 离地 0.3 m 以上的障碍物点**一个都不许**被判成地面。
    EXPECT_EQ(score.leak_above, 0) << degrees << "° 坡上把障碍物的主体吞进地面了";
    // 守卫：底部一圈被吸收是正常的，但不许失控。
    EXPECT_LT(score.non_ground_leak, 0.10) << degrees << "° 坡上底部吸收失控";
  }
}

// ---------------------------------------------------------------------------
//  ⚠️ 坡度检查：一堵墙不许赢
// ---------------------------------------------------------------------------
TEST(GroundSegmentation, RejectsAWallEvenWhenItHasMoreInliersThanTheGround)
{
  // 场景刻意做成**墙面点比地面点多**：只按"内点最多"选平面的话墙会赢。
  Scene scene;
  std::mt19937 rng(77);
  std::normal_distribution<double> noise(0.0, 0.01);

  // 地面：20 m × 10 m，0.5 m 网格 → 861 点
  for (double x = 0.0; x <= 20.0; x += 0.5) {
    for (double y = -5.0; y <= 5.0; y += 0.5) {
      scene.points.emplace_back(x, y, noise(rng));
      scene.truth_is_ground.push_back(1U);
      scene.height_above_ground_m.push_back(-1.0);
    }
  }
  // 墙：x = 10 的竖直面，20 m 宽 × 8 m 高，0.1 m 网格 → 16281 点（远多于地面）
  for (double y = -10.0; y <= 10.0; y += 0.1) {
    for (double z = 0.0; z <= 8.0; z += 0.1) {
      scene.points.emplace_back(10.0 + noise(rng), y, z);
      scene.truth_is_ground.push_back(0U);
      scene.height_above_ground_m.push_back(z);
    }
  }

  const GroundSegmentationResult result = SegmentGround(scene.points, GroundSegmentationParams{});
  ASSERT_TRUE(result.found);
  printf(
    "[          ] 墙 %d 点 vs 地面 861 点：选中平面的法向 z = %.4f（地面应 ≈1，墙应 ≈0）\n",
    static_cast<int>(scene.points.size()) - 861, result.normal.z());

  // ⚠️ 这一条是本文件里最不能省的。选中墙面之后，真正的地面点全部变成
  //    「非地面」，聚类会把整片地面连成一个几十米的巨簇 ——
  //    症状是「感知输出一个巨大的障碍物」，而人会去查聚类参数。
  EXPECT_GT(result.normal.z(), 0.96) << "选中了一个陡平面 —— 坡度检查没生效？";
  const Score score = Evaluate(scene, result);
  EXPECT_GT(score.ground_recall, 0.95) << "地面没被识别出来";
  EXPECT_EQ(score.leak_above, 0) << "墙面主体被当成了地面";

  // P9-S1 仪器断言：墙场景里坡度门必然拒绝过候选（RANSAC 抽到墙面三点组
  // 的概率占优）。计数器 = 0 说明仪器断线 —— 诊断哨兵自己也要被验。
  EXPECT_GT(result.slope_rejected_count, 0) << "坡度拒绝计数器没接上";
  EXPECT_GT(result.pool_count, 0);
  printf(
    "[          ] 仪器读数：pool=%d slope_rejected=%d ground/pool=%.2f\n", result.pool_count,
    result.slope_rejected_count, static_cast<double>(result.ground_count) / result.pool_count);
}

// ---------------------------------------------------------------------------
//  ⚠️ 矮障碍物不许被距离阈值吞掉
// ---------------------------------------------------------------------------
TEST(GroundSegmentation, KeepsAConeThatIsOnlySlightlyTallerThanTheThreshold)
{
  // 锥桶只有 0.8 m 高，而距离阈值是 0.15 m —— 它底部的点离地面很近。
  // 这一条守的是「阈值调大到 0.4 会把矮障碍物吞进地面」那个后果
  // （见 distance_threshold_m 的注释）。
  Scene scene = MakeSlopedGround(0.0);
  AddBox(&scene, 15.0, 0.0, 0.5, 0.5, 0.8, 0.0);

  const GroundSegmentationResult result = SegmentGround(scene.points, GroundSegmentationParams{});
  ASSERT_TRUE(result.found);
  const Score score = Evaluate(scene, result);
  printf("[          ] 0.8 m 锥桶：漏判率 %.4f\n", score.non_ground_leak);
  // 底部 0.15 m 那一圈确实会被算成地面（阈值就是这么定的），
  // 但**大部分**点必须留下来，否则聚类凑不够 min_cluster_size。
  EXPECT_LT(score.non_ground_leak, 0.25) << "锥桶被吞掉太多，聚类会凑不够点数";
}

// ---------------------------------------------------------------------------
//  边界与防御
// ---------------------------------------------------------------------------
TEST(GroundSegmentation, ReportsNotFoundInsteadOfFittingNoise)
{
  // 只有 50 个点，低于 min_inliers = 100。
  // ⚠️ 此时必须报 found = false，而不是给一个拟合自噪声的平面 ——
  //    后者会让下游把真障碍物当地面滤掉，而且**看起来完全正常**。
  Scene scene;
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> spread(-5.0, 5.0);
  for (int i = 0; i < 50; ++i) {
    scene.points.emplace_back(spread(rng), spread(rng), spread(rng));
  }
  const GroundSegmentationResult result = SegmentGround(scene.points, GroundSegmentationParams{});
  EXPECT_FALSE(result.found) << "点太少却报告找到了地面";

  // 空输入同理。
  EXPECT_FALSE(SegmentGround({}, GroundSegmentationParams{}).found);
}

TEST(GroundSegmentation, ThrowsOnNonFiniteInputInsteadOfSilentlyPoisoningThePlane)
{
  // ⚠️ 一个 inf 混进拟合会让法向变成 NaN，而 NaN 参与任何比较都返回 false
  //    —— 于是**所有点都被判成非地面**，下游看到"满屏障碍物"，
  //    而没有人会想到根因是几个 inf。gpu_lidar 的无回波射线返回的正是 inf。
  Scene scene = MakeSlopedGround(0.0);
  for (const double poison : {std::numeric_limits<double>::infinity(), std::nan("")}) {
    std::vector<Eigen::Vector3d> polluted = scene.points;
    polluted[100].z() = poison;
    EXPECT_THROW(SegmentGround(polluted, GroundSegmentationParams{}), std::invalid_argument);
  }
}

TEST(GroundSegmentation, IsReproducibleAcrossRuns)
{
  // RANSAC 依赖随机采样。种子固定 ⟹ 同样的输入必须给同样的输出，
  // 否则判据会随机红绿，而那比判据错了更难查。
  Scene scene = MakeSlopedGround(3.0 * M_PI / 180.0);
  AddBox(&scene, 12.0, 2.0, 4.4, 1.8, 1.5, 3.0 * M_PI / 180.0);
  const GroundSegmentationResult first = SegmentGround(scene.points, GroundSegmentationParams{});
  const GroundSegmentationResult second = SegmentGround(scene.points, GroundSegmentationParams{});
  ASSERT_TRUE(first.found);
  EXPECT_EQ(first.ground_count, second.ground_count);
  EXPECT_NEAR(first.normal.z(), second.normal.z(), 1e-15);
  EXPECT_NEAR(first.offset_m, second.offset_m, 1e-15);
}

// ---------------------------------------------------------------------------
//  CP-P5-A 判据 ⑤：单帧耗时
// ---------------------------------------------------------------------------
TEST(GroundSegmentation, StaysWithinTheTimeBudget)
{
  // 2–5 万点是实测一帧的量级（S1 体检时点云 23499 点）。
  Scene scene = MakeSlopedGround(2.0 * M_PI / 180.0);
  AddBox(&scene, 12.0, 2.0, 4.4, 1.8, 1.5, 2.0 * M_PI / 180.0);
  printf("[          ] 场景点数 %zu\n", scene.points.size());

  const auto started = std::chrono::steady_clock::now();
  const GroundSegmentationResult result = SegmentGround(scene.points, GroundSegmentationParams{});
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  ASSERT_TRUE(result.found);
  printf("[          ] 单帧耗时 %.2f ms\n", elapsed_ms);

  // CP-P5-A ⑤：< 40 ms（给 S3/S4 留预算，全链路总共 100 ms）。
  // ⚠️ Debug 构建会慢好几倍，这条判据默认按 Release 定 —— CI 跑的正是 Release。
  EXPECT_LT(elapsed_ms, 40.0) << "地面分割超预算，S3/S4 就没空间了";
}

// ---------------------------------------------------------------------------
//  ⚠️ 帧间稳定性 —— CP-P5-B 首轮失败的根因就在这里
// ---------------------------------------------------------------------------
//  实测（2026-08-11，Gazebo 全栈）：同一场景相邻两帧，非地面点 5056 vs 3871
//  （波动 77%），而簇数随之 8 vs 48、最大簇占非地面点的 87% vs 4%。
//  多出来的那批点是**残留的地面点**，它们铺在地上把所有东西连成一片
//  ⟹ 欠分割 ⟹ 巨簇的中心离任何真值目标都远 ⟹ 记成"漏检"。
//
//  RANSAC 每帧独立跑，随机采样不同 ⟹ 选中的平面不同。
//  **换种子等价于换一帧输入**，所以这条用例扫种子。
//
//  ⚠️ 前面那些用例抓不到它：它们只跑默认种子，而问题恰恰是
//     "某些采样会给出次优平面"。**单种子的用例对随机算法没有区分力。**
// ---------------------------------------------------------------------------
TEST(GroundSegmentation, GivesTheSameGroundAcrossDifferentRandomSeeds)
{
  // ⚠️ **地面必须按真实的角分辨率造，不能用均匀网格。**（2026-08-11 实测）
  //
  //    第一版用 40×20 m 的 0.25 m 均匀网格，20 个种子的波动只有 **0.1%** ——
  //    完全复现不了实测的 77%。**判据在那个场景上没有区分力。**
  //
  //    差别在于**点密度**：真实雷达的地面点按 1/d² 衰减（近处每平方米上千点、
  //    远处稀疏），而 RANSAC 的判据是「内点最多」—— 于是一个**贴合近处、
  //    在远处偏离**的平面可能比真平面内点还多。均匀网格里不存在这个偏向，
  //    所以任何倾斜都会立刻损失大量内点、被淘汰。
  //
  //    这是本仓库第三次栽在"合成数据太干净"上（前两次：地面分割必须带坡度、
  //    聚类必须按真实角分辨率采样）。**规律是：凡是判据要抓的现象依赖
  //    输入的某个统计性质，合成数据就必须把那个性质造出来。**
  Scene scene = MakeLidarLikeGround();
  AddBox(&scene, 12.0, 2.0, 4.4, 1.8, 1.5, 0.0);
  AddBox(&scene, 8.0, -3.0, 0.5, 0.5, 0.8, 0.0);
  AddBox(&scene, 18.0, -5.0, 0.4, 0.4, 1.7, 0.0);
  AddBox(&scene, 5.0, 4.0, 2.0, 0.6, 1.0, 0.0);

  std::vector<int> ground_counts;
  std::vector<double> tilts_deg;
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    GroundSegmentationParams params;
    params.seed = seed;
    const GroundSegmentationResult result = SegmentGround(scene.points, params);
    ASSERT_TRUE(result.found) << "种子 " << seed << " 没找到地面";
    ground_counts.push_back(result.ground_count);
    tilts_deg.push_back(std::acos(result.normal.z()) * 180.0 / M_PI);
  }

  const int lowest = *std::min_element(ground_counts.begin(), ground_counts.end());
  const int highest = *std::max_element(ground_counts.begin(), ground_counts.end());
  const double mean =
    std::accumulate(ground_counts.begin(), ground_counts.end(), 0.0) / ground_counts.size();
  const double spread = (highest - lowest) / mean;
  const double worst_tilt = *std::max_element(tilts_deg.begin(), tilts_deg.end());

  printf(
    "[          ] 20 个种子：地面点 %d–%d（均值 %.0f，波动 %.1f%%），最大倾角 %.3f°\n", lowest,
    highest, mean, spread * 100.0, worst_tilt);

  // ⚠️ **判据是「波动」不是「均值」。** 均值好看说明不了问题 ——
  //    实测那两帧的均值也在正常范围内，坏的是它们之间差了 77%。
  EXPECT_LT(spread, 0.05) << "不同采样给出的地面差太多 —— 残留点会把目标连成一片";
  // 地面是平的，任何一次采样都不该拟合出倾斜平面。
  EXPECT_LT(worst_tilt, 1.0) << "某些采样拟合出了倾斜的地面";
}
