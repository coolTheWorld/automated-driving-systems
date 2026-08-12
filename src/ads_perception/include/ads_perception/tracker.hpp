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

#ifndef ADS_PERCEPTION__TRACKER_HPP_
#define ADS_PERCEPTION__TRACKER_HPP_

// =============================================================================
//  多目标跟踪：恒速卡尔曼滤波 + 匈牙利关联 + 航迹生命周期
//
//  ## ⚠️ 它是**线性卡尔曼滤波**，不是 EKF
//
//  plan.md 里写的是「恒速 EKF」，但状态转移（匀速直线）与观测（直接测位置）
//  **都是线性的** —— 没有需要线性化的地方，所以 EKF 会退化成 KF。
//  这里如实叫它 KF。用 EKF 这个名字会让人以为存在非线性、去找雅可比。
//  （P4 的 ESKF 是真 EKF：姿态活在流形上。两者不是一回事。）
//
//      状态 x = (px, py, vx, vy)      观测 z = (px, py)
//      F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]      H = [I₂ 0]
//
//  ## ⚠️⚠️ 生命周期参数直接来自 S1 的实测，不是抄来的默认值
//
//  S1 体检发现：**目标在连续帧之间闪烁**（同一个锥桶连续三帧 8 点→8 点→0 点），
//  实测命中率只有 33–74%。这把生命周期从"锦上添花"变成了**必需品**，
//  也推翻了教科书上那个「连续 N 帧命中才确认」的写法：
//
//      p=0.5 时「连续 N 帧命中」的概率     N=2: 25%  N=3: 12.5%  N=4: 6.2%
//      ⟹ N=3 平均要等 8 次机会（0.8 s）才确认一个目标，而它可能只在
//        视野里待几秒。**这不是保守，是看不见。**
//
//  所以本实现：
//    · **确认用「累计命中数」**（不要求连续）—— p=0.5 时平均 6 帧达成 3 次命中
//    · **删除用「连续未命中」** —— 目标真的消失了才该删
//      p=0.5 时连续 5 帧未命中的概率 3.1%，那是可接受的误删率
//
//  ## 180° 二义性在这里消歧
//
//  S3 的 L-Shape 只能给出**轴向**（`[0, π)`）—— 一帧点云里没有信息能区分
//  车头朝哪。本层有跨帧的速度估计，于是：
//
//      速度足够大  ⟹ 车头 = 速度方向（取与轴向差 < 90° 的那一个）
//      速度太小    ⟹ **仍然无解**，如实标 `heading_resolved = false`
//
//  ⚠️ 静止目标的朝向**永远定不下来**，这不是缺陷而是物理事实。
//     下游拿到 `heading_resolved = false` 时应当退回用轴向 + 不确定性，
//     而不是当成一个确定的朝向 —— 那会有 50% 的机会让 P6 预测出逆行轨迹。
// =============================================================================

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace ads_perception
{

/// 跟踪参数。
struct TrackerParams
{
  /// 过程噪声：**加速度**的标准差，m/s²。
  ///
  /// 恒速模型假设目标不加速，而实际会 —— 这个量就是"允许它多不守规矩"。
  /// 取 2.0：园区目标（车 1.5 m/s²、行人变向）的量级。
  /// 调大 → 滤波器更信观测，速度估计跟得快但更抖；
  /// 调小 → 更平滑，但目标一变向就跟丢（新息持续超门限 → 关联失败 → 航迹删除）。
  double process_accel_stddev_mps2{2.0};

  /// 观测噪声：检测位置的标准差，m。
  ///
  /// 取 0.3：S2 实测**可见面**定位误差只有 0.003 m，但那是"面"的位置；
  /// 而检测给出的是包围盒中心，它随目标朝向与遮挡变化（雷达打不到背面），
  /// 帧间抖动远大于面的定位精度。0.3 是给这个抖动留的。
  /// 调小 → 滤波器过度相信每一帧的中心，速度估计被抖动放大；
  /// 调大 → 迟钝，真机动跟不上。
  double measurement_stddev_m{0.3};

  /// 关联门限：马氏距离平方的上限（卡方，2 自由度）。
  ///
  /// 取 9.21 = χ²(2, 99%)。**这次可以用卡方分布表**，因为这里的协方差
  /// 是滤波器自己按已知的 Q/R 递推出来的 —— 与 P4 那个未标定的 NDT 协方差
  /// 不同（那里只能用固定阈值，见 localization.md §10.6）。
  /// 调小 → 目标机动时关联失败，航迹断裂后重建 ⟹ **ID 跳变**；
  /// 调大 → 密集场景下配错对象，两个目标的 ID 互换。
  double gate_chi_square{9.21};

  /// 累计命中多少次才算「确认」。**不要求连续**，理由见文件头。
  ///
  /// 取 3：p=0.5 时平均 6 帧（0.6 s）达成。
  /// 调大到 5 → 平均 10 帧，快速穿过视野的目标可能到消失都没被确认；
  /// 调小到 1 → 一个噪点簇就能生成一条确认航迹（虚警）。
  int confirm_hits{3};

  /// 连续未命中多少帧就删除航迹。
  ///
  /// 取 5（0.5 s）：p=0.5 时误删概率 3.1%。
  /// 调小到 3 → 误删概率 12.5%，闪烁场景下 ID 会频繁跳变；
  /// 调大到 10 → 目标真的走了之后还留着"幽灵航迹"，而规划会绕它。
  int max_misses{5};

  /// 速度超过它才用来消歧朝向，m/s。
  ///
  /// 取 0.5：低于这个速度，速度方向由噪声主导 —— 拿噪声去定朝向
  /// 还不如老实说"不知道"。行人正常步速 1.2 m/s，所以走动的人能被消歧。
  double heading_min_speed_mps{0.5};
};

/// 一次检测（S2 聚类 + S3 拟合的产物）。
struct Detection
{
  /// 位置（x, y）—— 本项目里用 L-Shape 的矩形中心。
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  /// 长轴方向，`[0, π)`。⚠️ 是**轴向**不是朝向，见 lshape_fit.hpp。
  double yaw_rad{0.0};
  double length_m{0.0};
  double width_m{0.0};
  double height_m{0.0};
};

/// 一条航迹。
struct Track
{
  std::uint32_t id{0};

  /// 状态 (px, py, vx, vy)。
  Eigen::Vector4d state{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Identity()};

  /// 最近一次关联上的检测的尺寸与轴向。
  double yaw_rad{0.0};
  double length_m{0.0};
  double width_m{0.0};
  double height_m{0.0};

  /// 消歧之后的**车头朝向**，`[−π, π)`。
  ///
  /// ⚠️ 只有 `heading_resolved` 为 true 时才有意义。为 false 时目标太慢，
  /// 朝向**物理上无解** —— 下游应当退回用 `yaw_rad`（轴向）+ 不确定性。
  double heading_rad{0.0};
  bool heading_resolved{false};

  int hits{0};  ///< 累计命中次数（**不要求连续**）
  int consecutive_misses{0};
  bool confirmed{false};  ///< 累计命中达到 confirm_hits

  Eigen::Vector2d position() const { return state.head<2>(); }
  Eigen::Vector2d velocity() const { return state.tail<2>(); }
};

/// 多目标跟踪器。**有状态**，逐帧调用 `Update`。
class Tracker
{
public:
  explicit Tracker(const TrackerParams & params = TrackerParams{});

  /// 推进一帧。
  ///
  /// @param detections 本帧的检测。**允许为空**（那正是"目标闪烁"时的常态，
  ///                   此时所有航迹只做预测，靠 max_misses 决定去留）。
  /// @param dt_s       距上一帧的时间，s。必须为正。
  /// @throws std::invalid_argument dt 非正或非有限，或检测含非有限值。
  void Update(const std::vector<Detection> & detections, double dt_s);

  /// 全部航迹（含未确认的）。
  const std::vector<Track> & tracks() const { return tracks_; }

  /// 只要**已确认**的航迹 —— 这才是该发给下游的东西。
  ///
  /// ⚠️ 未确认的航迹**不要发**：它们可能只是噪点簇，而下游（规划）
  /// 会对每一个障碍物做碰撞检查，虚警的代价是车无故刹停。
  std::vector<Track> ConfirmedTracks() const;

private:
  void Predict(double dt_s);
  void Associate(const std::vector<Detection> & detections, std::vector<int> * assignment);
  void ApplyUpdate(const Detection & detection, Track * track);
  void ResolveHeading(Track * track) const;

  TrackerParams params_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_{1};
};

}  // namespace ads_perception

#endif  // ADS_PERCEPTION__TRACKER_HPP_
