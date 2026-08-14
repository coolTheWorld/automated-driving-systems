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

#ifndef ADS_PERCEPTION__GROUND_SEGMENTATION_HPP_
#define ADS_PERCEPTION__GROUND_SEGMENTATION_HPP_

// =============================================================================
//  地面分割：把一帧点云分成「地面」与「非地面」
//
//  这是感知流水线的第一步，也是**最容易被高估的一步**。
//
//  ## ⚠️ 为什么本项目的判据必须跑在**带坡度**的合成地面上
//
//  campus 世界的地面是**理想平面**（ground_plane + 0.01 m 厚的路面板）。
//  在理想平面上，RANSAC 与最朴素的「`z < 0.3` 就是地面」**给出完全相同的结果**
//  —— 把本文件整个换成一行阈值判断，所有闭环判据照样全绿。
//
//  也就是说：**在 Gazebo 上验不出这个模块写没写对。**
//  这与 P3-S2「采样步长与段长可通约 → 用例全绿但什么都没测」、
//  P4「不给扫描加噪声 → NDT 一步都不动」是同一类问题：
//  **理想化的场景会让判据失去区分力**（plan.md P5-1 决策二）。
//
//  所以 L1 判据跑在 1° / 3° / 5° 的合成斜面上，并做故障注入验红。
//  ⚠️ 不要为此给 campus_loop.sdf 加坡度 —— 那是三个检查点的回归基线。
//
//  ## 算法：RANSAC 平面拟合
//
//      重复 N 次：随机取 3 点定一个平面 → 数有多少点落在 ±d 之内 → 留最优
//      最后用**全部内点**做一次最小二乘精化
//
//  为什么不用更强的 Patchwork/GPF：SPEC §3.2 的选型原则是「先经典后可解释」，
//  而园区地面就是一个平面。Patchwork 的价值在于**分区处理起伏地形**，
//  那是 CARLA 那一半（P8）才会遇到的问题。
//
//  ## ⚠️ 坡度检查不是装饰
//
//  只按「内点最多」选平面的话，**一堵墙会赢** —— 建筑立面上的点又多又共面。
//  选中墙面之后，真正的地面点全部变成「非地面」，聚类会把整片地面连成一个
//  巨大的簇。症状是「感知输出一个几十米的障碍物」，而人会去查聚类参数。
//  所以候选平面的法向与 +z 夹角超过 `max_slope_rad` 就直接淘汰。
// =============================================================================

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace ads_perception
{

/// 地面分割参数。
struct GroundSegmentationParams
{
  /// RANSAC 迭代次数。
  ///
  /// 取 100 的依据：设地面点占比 p，随机 3 点全是地面点的概率是 p³。
  /// 园区场景 p ≈ 0.5（S1 实测点云里地面约占一半），p³ = 0.125，
  /// 100 次里至少一次全中的概率是 1 − (1−0.125)¹⁰⁰ ≈ 1 − 1.4e-6。
  /// 调小到 20 → 那个概率掉到 93%，也就是**每 14 帧就有一帧找不到地面**，
  /// 而症状是那一帧的地面点全变成障碍物 —— 闪烁的虚警。
  /// 调大只是线性增加耗时（每次迭代要遍历全部点）。
  int max_iterations{100};

  /// 内点距离阈值，m。点到平面的距离小于它就算地面。
  ///
  /// 取 0.15：雷达测距噪声 σ=1 cm，但**路面板本身有 0.01 m 厚**，
  /// 且远处的点受角分辨率影响，投影误差随距离增长。
  /// 调小到 0.05 → 远处的地面点被判成障碍物，30 m 外全是虚警；
  /// 调大到 0.4 → **矮障碍物被吞进地面**（锥桶才 0.8 m 高，
  /// 它底部 0.4 m 的点会被当成地面，剩下的点数不够成簇）。
  double distance_threshold_m{0.15};

  /// 候选平面允许的最大坡度（法向与 +z 的夹角），rad。
  ///
  /// 取 0.26 rad ≈ 15°：园区道路坡度不会超过这个数（SPEC §2 的 ODD 是
  /// 封闭园区），而建筑立面是 90°、路缘石侧面也远大于它。
  /// ⚠️ **这一条不是装饰**：不检查的话一堵墙会因为内点多而赢，
  ///    然后真正的地面全部变成障碍物。见文件头。
  /// 调大到 60° → 墙面开始有机会；调小到 5° → 真有缓坡的地方分不出地面。
  double max_slope_rad{0.26};

  /// 只在这个高度以下找地面（传感器系，z 向上），m。
  ///
  /// base_link 原点在地面高度，所以地面点的 z ≈ 0。取 1.0 是为了
  /// ① 加速（车顶、建筑、树冠的点直接不参与采样）；
  /// ② **防止把车顶或货箱顶面当成地面** —— 那些也是水平大平面。
  /// 调大 → 上面那个风险回来；调小到 0.2 → 有坡度时上坡段的地面点被排除。
  double max_height_m{1.0};

  /// 找到的地面内点少于它就判定「没有地面」。
  ///
  /// 取 100：S1 实测一帧约 2 万点、地面约占一半，正常情况下远超它。
  /// 低于 100 说明这一帧根本不是正常的路面场景（传感器坏了、车翻了），
  /// 此时**宁可报告没找到**也不要给一个拟合自噪声的平面 ——
  /// 后者会让下游把真障碍物当地面滤掉。
  int min_inliers{100};

  /// 随机种子。**固定**，因为 RANSAC 的结果依赖随机采样，
  /// 而一个结果不可复现的算法没法做回归测试 —— 判据会随机红绿。
  std::uint32_t seed{20260811U};
};

/// 地面分割结果。
struct GroundSegmentationResult
{
  /// 是否找到了地面。为 false 时下面的字段全部无意义。
  ///
  /// ⚠️ **调用方必须查它。** 不查的话，"没找到地面" 会被当成
  /// "所有点都是障碍物"，下游立刻输出一大片虚警并让车刹停。
  bool found{false};

  /// 平面方程 `n·x + d = 0` 的系数，`n` 已单位化且**指向 +z 半空间**。
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  double offset_m{0.0};

  /// 与输入点等长：1 = 地面，0 = 非地面。
  ///
  /// 用 `std::vector<std::uint8_t>` 而不是 `std::vector<bool>`：后者是位压缩
  /// 特化，取地址、迭代器语义都跟别的容器不一样，而这份数据要按下标反复索引。
  std::vector<std::uint8_t> is_ground;

  int ground_count{0};

  // ---- P9-S1 诊断计数（域移植仪器：CARLA 生成世界上分割失效的三嫌疑
  //      —— 挂高偏差 / mesh 非平面 / walker 点稀 —— 要靠这些数字裁决）----
  /// 参与采样的点数（z < max_height 的池子）。ground_count/pool_count
  /// 是「路面占比」哨兵 —— Gazebo 基线约 0.5，掉到 0.1 级 = 分割失效。
  int pool_count{0};
  /// 被坡度门拒绝的候选平面轮数。持续高位 = RANSAC 总在抽到墙/斜面
  /// （嫌疑 2 的哨兵）。
  int slope_rejected_count{0};
};

/// 把点云分成地面与非地面。
///
/// @param points_sensor 输入点，**传感器系**（本项目 = base_link，z 向上、原点在地面高度）。
/// @param params        参数。
/// @return 分割结果；`found` 为 false 时只有该字段有意义。
/// @throws std::invalid_argument 参数非法，或输入含非有限值。
///
/// ⚠️ 输入的非有限值会**抛异常而不是被跳过**。理由与 `AlignNdt` 一样：
/// `gpu_lidar` 的无回波射线返回 ±inf，那种点混进平面拟合会让法向变成 NaN，
/// 而 NaN 参与任何比较都返回 false —— 于是**所有点都会被判成非地面**，
/// 下游看到的是"满屏障碍物"，没有人会想到根因是几个 inf。
GroundSegmentationResult SegmentGround(
  const std::vector<Eigen::Vector3d> & points_sensor, const GroundSegmentationParams & params);

}  // namespace ads_perception

#endif  // ADS_PERCEPTION__GROUND_SEGMENTATION_HPP_
