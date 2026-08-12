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

#ifndef ADS_PERCEPTION__LSHAPE_FIT_HPP_
#define ADS_PERCEPTION__LSHAPE_FIT_HPP_

// =============================================================================
//  L-Shape 拟合：从一个簇算出**带朝向**的矩形
//
//  ## 为什么需要它 —— 轴对齐包围盒不够用
//
//  S2 的聚类给出的是轴对齐包围盒（AABB）。一辆斜停 45° 的车，它的 AABB
//  比车本身大 40%，而规划做碰撞检查时那 40% 是**凭空多出来的障碍**，
//  症状是「车绕一个不存在的东西」。更要命的是 AABB **没有朝向**，
//  而 P6 预测必须知道车头朝哪才能选车道跟随模型。
//
//  ## 算法：搜索朝向角，取「贴合度」最高的那个
//
//  车从雷达看过去通常只露出两个面（一个长边 + 一个短边），形成 L 形。
//  于是：枚举朝向 θ，把点投影到 (cosθ, sinθ) 与 (−sinθ, cosθ) 两个轴上，
//  得到一个外接矩形；**点贴着矩形边越紧，这个 θ 越可能是真的**。
//
//      score(θ) = Σ_i 1 / max(d_i, d0)      d_i = 点 i 到最近那条边的距离
//
//  取 score 最大的 θ。这是 Zhang 等人 2017 的 "closeness" 准则。
//
//  ⚠️ **为什么不用「最小面积」准则**（那是最容易想到的）：
//     最小面积对**只看得见一个面**的目标会退化 —— 那时点近似共线，
//     任何包住它们的窄矩形面积都接近零，于是朝向由噪声决定。
//     而"只看得见一个面"在本项目里是常态：S1 体检实测锥桶在 20 m 外
//     只有一条扫描线。closeness 在同样的情形下至少会选择让点贴着长边，
//     结果更稳定。
//
//  ## ⚠️⚠️ 180° 二义性：这一层**解决不了**，而且不该假装解决
//
//  拟合出来的是一个**矩形**，而矩形有 180° 对称性：把车头车尾对调，
//  点云一模一样。**几何上无解** —— 一帧静止的点云里没有任何信息能区分
//  "车头朝东" 和 "车头朝西"。
//
//  所以 `LShapeBox::yaw_rad` 是**长轴的方向**（值域 `[0, π)`），
//  **不是「车头朝向」**。两者差一个 0 或 π，而这里给不出是哪个。
//
//  消歧要靠**运动**：目标动起来之后，速度方向就是车头方向。那是 S4
//  跟踪模块的事（它有跨帧的速度估计），本层只诚实地把二义性标出来。
//
//  ⚠️ **不要在这里"猜"一个朝向**（比如"总是取靠近雷达的那一端当车头"）。
//     猜错的概率是 50%，而错了之后 P6 会预测出一条**逆行**的轨迹，
//     P7 的让行决策据此判断"对方要过来"——**症状出现在两个模块之外**，
//     而没有任何一层会报错。宁可标"未定"，让下游显式处理。
// =============================================================================

#include <Eigen/Core>

#include <vector>

namespace ads_perception
{

/// L-Shape 拟合参数。
struct LShapeFitParams
{
  /// 朝向搜索的角步长，rad。矩形有 90° 周期，所以只搜 `[0, π/2)`。
  ///
  /// 取 1° = 0.01745：判据要求朝向误差 < 10°，1° 的量化误差留了 10 倍余量。
  /// 调小到 0.5° → 搜索次数翻倍（每次 O(n)），而精度提升被点云噪声淹没；
  /// 调大到 5° → 量化误差直接吃掉一半判据余量。
  double angle_step_rad{0.01745};

  /// closeness 得分里的距离下限，m。防止贴边点的 `1/d` 爆掉。
  ///
  /// 取 0.01 = 雷达测距噪声 σ。比噪声小的"贴合"没有物理意义，
  /// 只是让某个恰好落在边上的点独占整个得分。
  double min_distance_m{0.01};

  /// 少于这么多点就不拟合（返回 `valid = false`）。
  ///
  /// 取 4：三个点必定共面，任何朝向都能"完美贴合"，拟合出来的角度是噪声。
  /// ⚠️ 但**不要因此把它调到 10** —— S1 体检实测锥桶/行人在 20–25 m
  ///    只有 7 点，调大等于让它们在那个距离上失去朝向。
  ///    朝向不可靠时下游可以退回用 AABB，而目标整个消失就没得救了。
  int min_points{4};
};

/// 拟合出来的带朝向矩形（俯视图）。
struct LShapeBox
{
  /// 拟合是否成功。为 false 时其余字段无意义。
  bool valid{false};

  /// 矩形中心（x, y），map 或传感器系 —— 与输入点同系。
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};

  /// 长轴方向，值域 `[0, π)`。
  ///
  /// ⚠️⚠️ **这是「轴向」不是「车头朝向」。** 矩形有 180° 对称性，
  /// 一帧点云里没有任何信息能区分车头朝哪 —— 见文件头。
  /// 真正的朝向要由 S4 用速度消歧。**直接把它当 heading 用会有 50% 的机会
  /// 让 P6 预测出一条逆行轨迹，而没有任何一层会报错。**
  double yaw_rad{0.0};

  /// 沿长轴的尺寸（`length_m ≥ width_m` 恒成立）。
  double length_m{0.0};
  double width_m{0.0};

  /// 高度，直接取簇内点 z 的极差。
  ///
  /// ⚠️ 它**不是**目标的真实高度：雷达打不到目标顶面以上，也常常打不到
  /// 贴地那一圈（被地面分割吸收了，S2 实测底部 0.15 m 内的点会被算成地面）。
  /// 所以这个值**系统性偏小**。分类阈值必须按这个偏小的值来定，
  /// 而不是按目标的标称高度 —— 否则行人（1.7 m 标称、实测约 1.5）会落空。
  double height_m{0.0};

  /// 最优 θ 处的 closeness 得分。**只作诊断**，不要拿它当置信度：
  /// 它随点数线性增长，两个不同大小的目标之间没有可比性。
  double score{0.0};
};

/// 对一个簇做 L-Shape 拟合。
///
/// @param points  簇内的点（**三维**；拟合只用 x/y，高度从 z 的极差取）。
/// @param params  参数。
/// @return 拟合结果；点数不足时 `valid = false`。
/// @throws std::invalid_argument 参数非法，或输入含非有限值。
LShapeBox FitLShape(const std::vector<Eigen::Vector3d> & points, const LShapeFitParams & params);

}  // namespace ads_perception

#endif  // ADS_PERCEPTION__LSHAPE_FIT_HPP_
