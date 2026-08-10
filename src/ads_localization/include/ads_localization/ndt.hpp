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

#ifndef ADS_LOCALIZATION__NDT_HPP_
#define ADS_LOCALIZATION__NDT_HPP_

// =============================================================================
//  NDT（Normal Distributions Transform）—— 先验点云地图的体素高斯表示
//
//  ## 一句话
//
//  把地图切成固定大小的体素，每个体素里的点用**一个三维高斯**（均值 + 协方差）
//  概括。之后配准就不再是「点找最近点」，而是「把扫描点代进这些高斯里求似然」——
//  代价函数因此是**处处可微**的，可以用牛顿法而不是 ICP 那种离散的最近邻迭代。
//
//  ## 本文件只做「地图这一侧」
//
//  配准迭代在 ndt_align.hpp（P4-S3 的后半）。分开是因为**体素化本身有两个
//  独立的坑**，各自需要专门的判据，混在一个文件里会看不清是哪一边出的问题。
//
//  ## 坑一：稀疏体素拟合不出可靠的高斯
//
//  三个点在三维里恰好确定一个平面，协方差矩阵**秩亏且样本协方差无意义**。
//  实测（`maps/campus_cloud.pcd`，2 m 体素）：4354 个非空体素里
//  **8.3%（363 个）少于 5 个点**。这些必须按点数阈值丢掉，
//  否则求逆时得到的是数值噪声放大器。
//
//  ## 坑二：路面点在体素内**共面**，协方差 rank 2
//
//  这是 P4 全阶段的那条主线（见 plan.md P4-1 决策一）在体素尺度上的重现：
//  一块路面上的点全在同一个平面里，协方差矩阵沿平面法向的特征值 ≈ 0，
//  **求逆会炸**（或给出天文数字的权重，让那一个体素主导整个代价函数）。
//
//  处理办法是**特征值下限**：把小于 `λ_max · ratio` 的特征值抬到那个值。
//  几何含义是「给这个高斯在最扁的方向上强加一个最小厚度」——
//  等价于承认「这个体素只告诉你法向那一个自由度，别的方向它没有信息」。
//
//  ⚠️ 下限调太小 → 求逆仍然病态，配准会被个别体素绑架；
//     调太大  → 所有体素都被抹成球，NDT 退化成"点到点心距离"，失去法向信息。
//
//  完整推导与参数见 docs/modules/localization.md。**改这个文件前先读它。**
// =============================================================================

#include <Eigen/Core>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace ads_localization
{

/// 一个体素里那份高斯。
struct NdtVoxel
{
  Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
  /// 协方差的**逆**。存逆而不是协方差本身：配准每次迭代对每个点都要用它，
  /// 而求逆在建图时做一次就够了。
  Eigen::Matrix3d inverse_covariance{Eigen::Matrix3d::Identity()};
  /// 落在这个体素里的点数。留着是为了诊断，不参与计算。
  int point_count{0};
  /// 这个体素的协方差是否被特征值下限修正过（= 它原本是退化的）。
  /// 统计这个数能直接量出「地图里有多大比例是平面」。
  bool was_regularized{false};
};

/// 体素化参数。
struct NdtGridParams
{
  /// 体素边长，m。
  ///
  /// 调大 → 每个体素点更多、高斯更稳，但空间分辨率下降，
  ///        代价函数变平缓（收敛域变大但精度变差）。
  /// 调小 → 分辨率高、精度好，但每个体素点数变少，
  ///        实测 2 m 时已有 8.3% 的体素少于 5 点，再小这个比例会迅速上升。
  double voxel_size_m{2.0};

  /// 建一个体素所需的最少点数。低于它的体素直接丢弃。
  ///
  /// 为什么是 6 而不是 3：三个点在三维里恰好张成一个平面，
  /// 样本协方差自由度为 0，**数学上就没有意义**。
  /// 取 6 给了三倍冗余，同时实测只丢掉 8.3% 的体素。
  int min_points_per_voxel{6};

  /// 特征值下限比例：小于 `λ_max · ratio` 的特征值被抬到这个值。
  ///
  /// 0.01 是 PCL 的默认值，含义是「最扁的方向至少有最长方向 1% 的厚度」。
  /// 见文件头「坑二」关于调大调小的说明。
  double eigenvalue_ratio_floor{0.01};

  /// 校验参数合法，非法则抛 std::invalid_argument。
  void Validate() const;
};

/// 体素化之后的地图。构造完就是只读的。
class NdtGrid
{
public:
  /// @param points map 系的点。
  /// @param params 体素化参数。
  /// @throws std::invalid_argument 参数非法，或点集为空 / 含非有限值。
  NdtGrid(const std::vector<Eigen::Vector3d> & points, const NdtGridParams & params);

  /// 查询包含该点的体素。**没有则返回 nullptr** —— 调用方必须判空。
  ///
  /// ⚠️ 只查「包含该点的那一个」体素，不查邻域。
  ///    代价是代价函数在体素边界上**不连续**（点跨过边界时换了一个高斯），
  ///    牛顿法因此需要一个足够好的初值 —— 本项目由 ESKF 的预测位姿提供。
  ///    查 27 邻域能把代价函数抹平滑，但每个点的开销涨 27 倍，
  ///    在有好初值的前提下不划算。这条写进 docs 的边界节。
  const NdtVoxel * At(const Eigen::Vector3d & point) const;

  /// 非空体素数。
  size_t size() const noexcept { return voxels_.size(); }

  /// 因点数不足被丢弃的体素数。**必须能被外部看到** ——
  /// 静默丢弃会让人以为地图覆盖是完整的，而 NDT 在"有洞"的那一段飘掉时
  /// 没人会想到是建图阶段丢的。
  size_t discarded_sparse_voxels() const noexcept { return discarded_sparse_; }

  /// 协方差被特征值下限修正过的体素数（= 原本退化的那些）。
  /// 这个比例直接量出「地图里有多大比例是平面」。
  size_t regularized_voxels() const noexcept { return regularized_; }

  double voxel_size_m() const noexcept { return params_.voxel_size_m; }

private:
  /// 把体素的整数索引编码成一个 int64 当哈希键。
  /// 每轴留 21 位（±100 万个体素，2 m 时是 ±2000 km），本项目远远用不完。
  static int64_t EncodeIndex(int64_t ix, int64_t iy, int64_t iz);

  NdtGridParams params_;
  std::unordered_map<int64_t, NdtVoxel> voxels_;
  size_t discarded_sparse_{0};
  size_t regularized_{0};
};

}  // namespace ads_localization

#endif  // ADS_LOCALIZATION__NDT_HPP_
