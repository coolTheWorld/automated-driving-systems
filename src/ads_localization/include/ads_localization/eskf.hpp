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

#ifndef ADS_LOCALIZATION__ESKF_HPP_
#define ADS_LOCALIZATION__ESKF_HPP_

// =============================================================================
//  误差状态卡尔曼滤波（Error-State Kalman Filter, ESKF）
//
//  ## 为什么是「误差状态」而不是直接对状态做 KF
//
//  姿态活在流形 SO(3) 上，不是向量空间。直接把四元数的四个分量塞进 KF 状态里，
//  卡尔曼更新会做一次线性组合 `x + K·y` —— 那个结果**几乎必然不再是单位四元数**，
//  于是每步都要强行归一化，而归一化这个操作不在滤波器的模型里，
//  协方差因此描述的不再是实际的不确定度。更糟的是四元数有 4 个分量却只有
//  3 个自由度，协方差矩阵必然奇异。
//
//  误差状态的做法是：把姿态拆成「一个大的名义值 q̂」和「一个小的误差 δθ」，
//      q = q̂ ⊗ q{δθ}
//  滤波器只对 **δθ（三维小角度向量，李代数）** 做线性运算 —— 它是货真价实的
//  向量空间，协方差在切空间上有良好定义。名义值 q̂ 单独用四元数精确积分，
//  永远不参与线性化。每次更新后把估出来的 δθ「注入」回 q̂ 再清零，
//  于是线性化点永远在原点附近，一阶近似始终成立。
//
//  这是 Sola《Quaternion kinematics for the error-state Kalman filter》的标准做法。
//
//  ## 15 维状态
//
//      索引  0..2   δp    位置误差（map 系，m）
//      索引  3..5   δv    速度误差（map 系，m/s）
//      索引  6..8   δθ    姿态误差（**body 系局部小角度**，rad）
//      索引  9..11  δb_a  加速度计零偏误差（body 系，m/s²）
//      索引 12..14  δb_g  陀螺零偏误差（body 系，rad/s）
//
//  ⚠️ **δθ 用局部（body）约定，不是全局（map）约定。** 两者的 F 矩阵符号不同，
//     混用的症状是滤波器在转弯时缓慢发散，而直线行驶完全正常 ——
//     因为 [ω]× 项只有转弯时才非零。本文件里所有推导都基于 q = q̂ ⊗ q{δθ}。
//
//  ## 为什么保留完整 15 维而不退化成 2D
//
//  园区场景 roll/pitch ≈ 0 是**结果**不是**假设**。让滤波器自己估出来接近 0，
//  比硬约束成 0 更能暴露问题（比如 IMU 装反了、重力方向搞错了）。
//  而且 roll/pitch 恰好是地面点云唯一可观的那几个自由度（见 P4-1 决策一），
//  扔掉它们等于把 NDT 给出的信息丢掉一半。
//
//  完整推导（F 矩阵逐块、Q 的离散化、观测雅可比、注入与重置）见
//  docs/modules/localization.md。**改这个文件前先读它。**
// =============================================================================

// ⚠️ Eigen 的头文件**没有扩展名**，cpplint 因此把它们归成「C 系统头」，
//    必须排在 C++ 标准库之前 —— 与 <gtest/gtest.h>、<tinyxml2.h> 同一个坑，
//    见 CLAUDE.md 的 lint 陷阱表。放反了报 include_order 且看不出所以然。
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>

namespace ads_localization
{

/// 一帧 IMU 采样。比力与角速度都在 **body 系**（= base_link，x 前 y 左 z 上）。
struct ImuSample
{
  /// 采样时刻，s。用仿真钟（SPEC §5 禁止用墙钟做算法时序）。
  double time_s{0.0};

  /// 比力（specific force），m/s²。**不是加速度** ——
  /// 静止时它读到的是 +g（沿 body 的 z 轴向上），因为加速度计测的是
  /// 「支撑力」而不是运动。这个符号搞反的症状是滤波器认为车在以 2g 下坠。
  Eigen::Vector3d accel_mps2{Eigen::Vector3d::Zero()};

  /// 角速度，rad/s。
  Eigen::Vector3d gyro_rad_s{Eigen::Vector3d::Zero()};
};

/// 名义状态。滤波器**不对它做线性运算** —— 它只被精确积分和「注入」修正。
struct NominalState
{
  Eigen::Vector3d position_m{Eigen::Vector3d::Zero()};             ///< map 系
  Eigen::Vector3d velocity_mps{Eigen::Vector3d::Zero()};           ///< map 系
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};  ///< body → map
  Eigen::Vector3d accel_bias_mps2{Eigen::Vector3d::Zero()};        ///< body 系
  Eigen::Vector3d gyro_bias_rad_s{Eigen::Vector3d::Zero()};        ///< body 系
};

/// 滤波器参数。**全部必须显式给值**：聚合初始化会把漏掉的项填 0，
/// 而噪声填 0 的后果是滤波器「过度自信」—— 它会拒绝一切观测，
/// 位置一路漂走而协方差始终报告「很准」。Validate() 就是拦这个的。
struct EskfParams
{
  // ---- IMU 噪声（**离散**标准差，与 config/vehicle_params.yaml 的 noise 段同源）
  /// 陀螺白噪声 σ，rad/s。调大 → 姿态随机游走快、滤波器更依赖绝对观测。
  double gyro_noise_rad_s{0.0};
  /// 加速度计白噪声 σ，m/s²。
  double accel_noise_mps2{0.0};
  /// 陀螺零偏的随机游走强度，rad/s/√s。对应 SDF 的 dynamic_bias_stddev。
  /// 调大 → 滤波器认为零偏变化快，会更快地跟着观测改零偏估计（也更容易被噪声带偏）。
  double gyro_bias_rw_rad_s{0.0};
  /// 加速度计零偏的随机游走强度，m/s²/√s。
  double accel_bias_rw_mps2{0.0};

  // ---- 初始不确定度（协方差的对角初值）
  double init_position_std_m{0.0};
  double init_velocity_std_mps{0.0};
  double init_attitude_std_rad{0.0};
  double init_accel_bias_std_mps2{0.0};
  double init_gyro_bias_std_rad_s{0.0};

  /// 重力大小，m/s²。map 是 ENU（z 朝上），所以重力矢量是 (0, 0, −g)。
  double gravity_mps2{9.80665};

  /// 单帧 IMU 允许的最大时间间隔，s。超过就认为中间丢帧了 ——
  /// 此时继续积分等于用**一个**采样代表一整段时间，误差按 dt² 涨。
  /// 0.5 s 是给 100 Hz 的 IMU 留的 50 倍余量，触发它一定是真出事了。
  double max_imu_dt_s{0.5};

  /// 检查每一项都是有限正数，否则抛 std::invalid_argument。
  ///
  /// ⚠️ 必须用 ads_common::RequireFinitePositive 而不是 `if (x <= 0)` ——
  ///    NaN 参与**任何**比较都返回 false，所以后者对 NaN 恒为假、一条都拦不住。
  ///    本项目已经在 vehicle_cmd_bridge 和 ads_control 上各咬过一次。
  void Validate() const;
};

/// 误差状态卡尔曼滤波器。纯 C++17 + Eigen，**无 ROS 依赖**。
class Eskf
{
public:
  /// 误差状态维数。
  static constexpr int kDim = 15;

  // 各块在误差状态向量里的起始索引。用具名常量而不是散落的魔数 ——
  // 写错一个索引的症状是某个量"就是估不出来"，而其余一切正常。
  static constexpr int kIdxPosition = 0;
  static constexpr int kIdxVelocity = 3;
  static constexpr int kIdxAttitude = 6;
  static constexpr int kIdxAccelBias = 9;
  static constexpr int kIdxGyroBias = 12;

  using Vector15d = Eigen::Matrix<double, kDim, 1>;
  using Matrix15d = Eigen::Matrix<double, kDim, kDim>;

  /// @param params 噪声与初值参数，构造时即校验。
  /// @param initial_state 名义状态初值（通常来自 GNSS + 初始朝向）。
  /// @throws std::invalid_argument 参数非法，或初值里有非有限量。
  Eskf(const EskfParams & params, const NominalState & initial_state);

  /// 用一帧 IMU 推进名义状态并传播协方差（**中值积分**）。
  ///
  /// 第一次调用只记下时间基准，不做积分 —— 中值积分需要**两帧**才定义得出来。
  ///
  /// @throws std::invalid_argument 采样里有非有限量，或时间倒流 / 间隔超限。
  void Predict(const ImuSample & imu);

  /// GNSS 位置观测（map 系，3 维）。
  /// @param position_m 观测到的位置。
  /// @param std_dev_m  三个轴各自的标准差（东/北/天）。**不是方差。**
  void UpdateGnssPosition(const Eigen::Vector3d & position_m, const Eigen::Vector3d & std_dev_m);

  /// GNSS 测速观测（map 系 ENU，3 维）。
  ///
  /// ⚠️ **这是一路与定位完全独立的观测，不要因为"都来自 GNSS"就以为它冗余。**
  /// 定位解算的是伪距（米级），测速解算的是**载波多普勒** —— 后者精度高
  /// 两个数量级（本项目 0.05 m/s vs 2 m）。不用它等于白扔掉最精确的那一路。
  ///
  /// 它对**位置**精度的贡献是间接但显著的：速度估得准 → 两次 GNSS 定位之间的
  /// 推算更准 → 滤波器能把更长时间窗内的定位观测平均掉。
  ///
  /// 观测模型 h(x) = v（map 系），所以雅可比只有速度块是单位阵 ——
  /// 它**不依赖姿态**，这一点与轮速观测正相反（那个要投到车体轴上）。
  void UpdateGnssVelocity(
    const Eigen::Vector3d & velocity_mps, const Eigen::Vector3d & std_dev_mps);

  /// 位姿观测（6 维：位置 + 姿态），供 **NDT 配准结果**喂回滤波器。
  ///
  /// @param position_m   map 系位置。
  /// @param orientation  body → map 的姿态。
  /// @param covariance   6×6 观测协方差，**顺序 = 位置 xyz、姿态 xyz**
  ///                     （与 NdtAlignResult::covariance 一致）。
  ///
  /// 姿态残差是 `Log(q̂⁻¹ ⊗ q_meas)` —— **不是四元数相减**。
  /// 相减在小角度下"看起来也能用"，但它不是流形上的差，
  /// 大角度时会给出错误的方向，而现象只是"收敛得慢"。
  ///
  /// ⚠️ NDT 报 degenerate 时**不要调用这个函数**。退化意味着代价函数沿某些
  ///    方向是平的，那几个方向上的位姿是任意的 —— 喂进来会把滤波器带偏，
  ///    而协方差看起来完全正常。判断该由调用方（localization_node）做，
  ///    因为只有它知道那次配准的上下文。
  void UpdatePose(
    const Eigen::Vector3d & position_m, const Eigen::Quaterniond & orientation,
    const Eigen::Matrix<double, 6, 6> & covariance);

  /// 轮速观测：车体纵向速度（body 系 x 分量，1 维）。
  ///
  /// 为什么只取 x 而不是三维：轮速计只测沿车轮滚动方向的速度。
  /// 把它当成三维速度观测（y、z 补 0）是在偷偷引入「无侧滑」的强假设 ——
  /// 那在低速园区大致成立，但它会让滤波器对侧向速度过度自信，
  /// 而侧滑恰恰是转弯时最该被 IMU 观测到的量。
  void UpdateWheelSpeed(double speed_mps, double std_dev_mps);

  const NominalState & state() const noexcept { return nominal_; }
  const Matrix15d & covariance() const noexcept { return covariance_; }

  /// 最近一次 Predict 的时间戳，s。没收到过 IMU 时返回 NaN。
  double time_s() const noexcept { return time_s_; }

  /// 是否已经收到过至少一帧 IMU（即时间基准是否建立）。
  bool initialized() const noexcept { return has_last_imu_; }

private:
  /// 通用的误差状态更新：给定观测残差、雅可比、观测噪声，做一次卡尔曼更新，
  /// 然后立刻把误差注入名义状态并重置。
  template <int kMeasDim>
  void ApplyUpdate(
    const Eigen::Matrix<double, kMeasDim, 1> & residual,
    const Eigen::Matrix<double, kMeasDim, kDim> & jacobian,
    const Eigen::Matrix<double, kMeasDim, kMeasDim> & noise_cov);

  /// 把误差状态注入名义状态，然后把误差清零、并按重置雅可比修正协方差。
  void InjectAndReset(const Vector15d & error);

  EskfParams params_;
  NominalState nominal_;
  Matrix15d covariance_{Matrix15d::Zero()};

  ImuSample last_imu_{};
  bool has_last_imu_{false};
  double time_s_{std::numeric_limits<double>::quiet_NaN()};
};

/// 反对称矩阵 [v]×，满足 [v]× w = v × w。
///
/// 它在本文件里出现三次（F 的两块 + 观测雅可比），三处必须是同一个约定：
///     [v]× = [[  0, -v3,  v2],
///             [ v3,   0, -v1],
///             [-v2,  v1,   0]]
/// 符号写反的症状是姿态误差往反方向修正 —— 滤波器不发散，但收敛得出奇地慢。
Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d & v);

/// 由小角度向量构造四元数（右乘增量）。|θ| 很小时退化成 [1, θ/2] 并归一化。
Eigen::Quaterniond QuaternionFromRotationVector(const Eigen::Vector3d & rotation_vec);

}  // namespace ads_localization

#endif  // ADS_LOCALIZATION__ESKF_HPP_
