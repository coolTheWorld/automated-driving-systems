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

#include "ads_localization/eskf.hpp"

#include <stdexcept>
#include <string>

#include "ads_common/numeric_checks.hpp"

namespace ads_localization
{

namespace
{

/// 小角度阈值。低于它就用一阶展开，避免 sin(θ)/θ 在 θ→0 时 0/0。
/// 1e-8 rad 对应 100 Hz 下 1e-6 rad/s 的角速度 —— 远低于任何真实陀螺的噪声，
/// 所以这条分支只在"车完全静止且噪声恰好抵消"时才走到。
constexpr double kSmallAngleRad = 1e-8;

void RequireFiniteVector(const Eigen::Vector3d & v, const char * what, const char * name)
{
  for (int i = 0; i < 3; ++i) {
    ads_common::RequireFinite(v[i], what, name);
  }
}

}  // namespace

Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Quaterniond QuaternionFromRotationVector(const Eigen::Vector3d & rotation_vec)
{
  const double angle = rotation_vec.norm();
  if (angle < kSmallAngleRad) {
    // 一阶展开 [1, θ/2]，再归一化。归一化不能省：连续几千步的一阶展开
    // 会让模长缓慢偏离 1，而四元数模长漂了之后旋转矩阵不再正交，
    // 症状是位置缓慢发散而姿态看起来正常。
    Eigen::Quaterniond q(
      1.0, 0.5 * rotation_vec.x(), 0.5 * rotation_vec.y(), 0.5 * rotation_vec.z());
    q.normalize();
    return q;
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vec / angle));
}

void EskfParams::Validate() const
{
  // ⚠️ 一律走 RequireFinitePositive，不要写 `if (x <= 0) throw`。
  //    NaN 参与任何比较都返回 false，后者对 NaN 恒为假 —— 一条都拦不住。
  //    本项目已经在 vehicle_cmd_bridge 与 ads_control 上各咬过一次。
  ads_common::RequireFinitePositive(gyro_noise_rad_s, "EskfParams", "gyro_noise_rad_s");
  ads_common::RequireFinitePositive(accel_noise_mps2, "EskfParams", "accel_noise_mps2");
  ads_common::RequireFinitePositive(gyro_bias_rw_rad_s, "EskfParams", "gyro_bias_rw_rad_s");
  ads_common::RequireFinitePositive(accel_bias_rw_mps2, "EskfParams", "accel_bias_rw_mps2");

  // 初始不确定度必须**严格为正**。填 0 的意思是「这个量我完全确定」，
  // 于是卡尔曼增益恒为 0、对应的状态永远不会被观测修正 ——
  // 滤波器安静地退化成纯推算，而协方差一路报告「很准」。
  ads_common::RequireFinitePositive(init_position_std_m, "EskfParams", "init_position_std_m");
  ads_common::RequireFinitePositive(init_velocity_std_mps, "EskfParams", "init_velocity_std_mps");
  ads_common::RequireFinitePositive(init_attitude_std_rad, "EskfParams", "init_attitude_std_rad");
  ads_common::RequireFinitePositive(
    init_accel_bias_std_mps2, "EskfParams", "init_accel_bias_std_mps2");
  ads_common::RequireFinitePositive(
    init_gyro_bias_std_rad_s, "EskfParams", "init_gyro_bias_std_rad_s");

  ads_common::RequireFinitePositive(gravity_mps2, "EskfParams", "gravity_mps2");
  ads_common::RequireFinitePositive(max_imu_dt_s, "EskfParams", "max_imu_dt_s");
}

Eskf::Eskf(const EskfParams & params, const NominalState & initial_state)
: params_(params), nominal_(initial_state)
{
  params_.Validate();

  RequireFiniteVector(initial_state.position_m, "Eskf 初值", "position_m");
  RequireFiniteVector(initial_state.velocity_mps, "Eskf 初值", "velocity_mps");
  RequireFiniteVector(initial_state.accel_bias_mps2, "Eskf 初值", "accel_bias_mps2");
  RequireFiniteVector(initial_state.gyro_bias_rad_s, "Eskf 初值", "gyro_bias_rad_s");
  ads_common::RequireFinite(initial_state.orientation.norm(), "Eskf 初值", "orientation");
  if (initial_state.orientation.norm() < 1e-6) {
    throw std::invalid_argument("Eskf 初值: orientation 是零四元数，无法归一化");
  }
  nominal_.orientation.normalize();

  // 协方差初值：对角，各块用各自的初始标准差平方。
  covariance_.setZero();
  const auto fill = [this](int start, double std_dev) {
    covariance_.block<3, 3>(start, start) = Eigen::Matrix3d::Identity() * (std_dev * std_dev);
  };
  fill(kIdxPosition, params_.init_position_std_m);
  fill(kIdxVelocity, params_.init_velocity_std_mps);
  fill(kIdxAttitude, params_.init_attitude_std_rad);
  fill(kIdxAccelBias, params_.init_accel_bias_std_mps2);
  fill(kIdxGyroBias, params_.init_gyro_bias_std_rad_s);
}

void Eskf::Predict(const ImuSample & imu)
{
  ads_common::RequireFinite(imu.time_s, "Eskf::Predict", "time_s");
  RequireFiniteVector(imu.accel_mps2, "Eskf::Predict", "accel_mps2");
  RequireFiniteVector(imu.gyro_rad_s, "Eskf::Predict", "gyro_rad_s");

  // 第一帧只建立时间基准。**中值积分需要两帧才定义得出来** ——
  // 拿一帧就积分等于把该帧当成整段区间的代表，那正是中值积分要避免的事。
  if (!has_last_imu_) {
    last_imu_ = imu;
    has_last_imu_ = true;
    time_s_ = imu.time_s;
    return;
  }

  const double dt = imu.time_s - last_imu_.time_s;
  if (dt <= 0.0) {
    throw std::invalid_argument(
      "Eskf::Predict: 时间没有前进（dt=" + std::to_string(dt) +
      " s）。仿真里这通常意味着**两套仿真同时在发 /clock**，"
      "所有测量值都会作废 —— 先去查残留进程，不要改滤波器。");
  }
  if (dt > params_.max_imu_dt_s) {
    throw std::invalid_argument(
      "Eskf::Predict: IMU 间隔 " + std::to_string(dt) + " s 超过上限 " +
      std::to_string(params_.max_imu_dt_s) +
      " s。中间丢帧了，继续积分的误差按 dt² 涨，宁可让上层降级。");
  }

  // ---- 名义状态：中值积分 -------------------------------------------------
  // 前后两帧各自扣掉零偏，取平均作为区间代表。
  // 相比欧拉法（只用区间起点），中值法把角度积分的截断误差从 O(dt²) 降到 O(dt³)。
  // 在 100 Hz、转向 0.5 rad/s 的条件下这一项很小，但它**不花什么代价**，
  // 而且 P4 之后要跑几百秒的闭环，截断误差是会累积的。
  const Eigen::Vector3d accel_0 = last_imu_.accel_mps2 - nominal_.accel_bias_mps2;
  const Eigen::Vector3d accel_1 = imu.accel_mps2 - nominal_.accel_bias_mps2;
  const Eigen::Vector3d gyro_0 = last_imu_.gyro_rad_s - nominal_.gyro_bias_rad_s;
  const Eigen::Vector3d gyro_1 = imu.gyro_rad_s - nominal_.gyro_bias_rad_s;
  const Eigen::Vector3d gyro_mid = 0.5 * (gyro_0 + gyro_1);

  const Eigen::Matrix3d rot_old = nominal_.orientation.toRotationMatrix();
  const Eigen::Quaterniond delta_q = QuaternionFromRotationVector(gyro_mid * dt);
  Eigen::Quaterniond orientation_new = nominal_.orientation * delta_q;
  orientation_new.normalize();
  const Eigen::Matrix3d rot_new = orientation_new.toRotationMatrix();

  // map 是 ENU（z 朝上），所以重力矢量指向 −z。
  //
  // ⚠️ 加速度计测的是**比力**不是加速度：静止时它读 +g（沿 body 的 z 轴），
  //    所以世界系加速度 = R·f + g_vec，两项在静止时恰好抵消为 0。
  //    这里把 g_vec 的符号写反的症状是车"以 2g 下坠"，位置几秒内就飞掉。
  const Eigen::Vector3d gravity(0.0, 0.0, -params_.gravity_mps2);
  const Eigen::Vector3d accel_world = 0.5 * (rot_old * accel_0 + rot_new * accel_1) + gravity;

  nominal_.position_m += nominal_.velocity_mps * dt + 0.5 * accel_world * dt * dt;
  nominal_.velocity_mps += accel_world * dt;
  nominal_.orientation = orientation_new;
  // 零偏在名义状态里不随时间变化 —— 它的漂移只体现在误差状态的过程噪声 Q 里。

  // ---- 误差状态：协方差传播 ----------------------------------------------
  // 连续时间误差动力学（局部角误差约定，Sola §5）：
  //     δṗ   = δv
  //     δv̇   = −R [a]× δθ − R δb_a + n_a
  //     δθ̇   = −[ω]× δθ − δb_g + n_g
  //     δḃ_a = n_ba
  //     δḃ_g = n_bg
  // 离散化取一阶 F = I + F_c·dt，只有姿态块用**精确**的 R{ω dt}ᵀ ——
  // 它一样便宜，而姿态是唯一会被长时间大角速度放大的那一块。
  const Eigen::Vector3d accel_mid = 0.5 * (accel_0 + accel_1);

  Matrix15d transition = Matrix15d::Identity();
  transition.block<3, 3>(kIdxPosition, kIdxVelocity) = Eigen::Matrix3d::Identity() * dt;
  transition.block<3, 3>(kIdxVelocity, kIdxAttitude) = -rot_old * SkewSymmetric(accel_mid) * dt;
  transition.block<3, 3>(kIdxVelocity, kIdxAccelBias) = -rot_old * dt;
  transition.block<3, 3>(kIdxAttitude, kIdxAttitude) = delta_q.toRotationMatrix().transpose();
  transition.block<3, 3>(kIdxAttitude, kIdxGyroBias) = -Eigen::Matrix3d::Identity() * dt;

  // 过程噪声。白噪声项按 σ²dt² 进（速度/姿态是噪声的**积分**），
  // 零偏随机游走按 σ²dt 进（它本身就是布朗运动）。
  //
  // ⚠️ 这四项**都不能填 0**。全 0 的 Q 意味着滤波器认为自己的预测毫无误差，
  //    协方差会单调收缩到 0，卡尔曼增益随之趋于 0 —— 之后任何观测都被无视，
  //    位置一路漂走而滤波器始终报告「很准」。CP-P4-A 第 3 条就是拦这个的。
  Matrix15d process_noise = Matrix15d::Zero();
  const double dt2 = dt * dt;
  process_noise.block<3, 3>(kIdxVelocity, kIdxVelocity) =
    Eigen::Matrix3d::Identity() * (params_.accel_noise_mps2 * params_.accel_noise_mps2 * dt2);
  process_noise.block<3, 3>(kIdxAttitude, kIdxAttitude) =
    Eigen::Matrix3d::Identity() * (params_.gyro_noise_rad_s * params_.gyro_noise_rad_s * dt2);
  process_noise.block<3, 3>(kIdxAccelBias, kIdxAccelBias) =
    Eigen::Matrix3d::Identity() * (params_.accel_bias_rw_mps2 * params_.accel_bias_rw_mps2 * dt);
  process_noise.block<3, 3>(kIdxGyroBias, kIdxGyroBias) =
    Eigen::Matrix3d::Identity() * (params_.gyro_bias_rw_rad_s * params_.gyro_bias_rw_rad_s * dt);

  covariance_ = transition * covariance_ * transition.transpose() + process_noise;
  // 强制对称。P 在数学上恒对称，但 FPFᵀ 的浮点舍入会让它慢慢失去对称性，
  // 而不对称的 P 会让后面的 S 求逆给出无意义的增益。代价是一次加法。
  covariance_ = 0.5 * (covariance_ + covariance_.transpose()).eval();

  last_imu_ = imu;
  time_s_ = imu.time_s;
}

void Eskf::UpdateGnssPosition(const Eigen::Vector3d & position_m, const Eigen::Vector3d & std_dev_m)
{
  RequireFiniteVector(position_m, "Eskf::UpdateGnssPosition", "position_m");
  for (int i = 0; i < 3; ++i) {
    ads_common::RequireFinitePositive(std_dev_m[i], "Eskf::UpdateGnssPosition", "std_dev_m");
  }

  // 观测模型 h(x) = p，对误差状态求导只有位置块非零。
  Eigen::Matrix<double, 3, kDim> jacobian = Eigen::Matrix<double, 3, kDim>::Zero();
  jacobian.block<3, 3>(0, kIdxPosition) = Eigen::Matrix3d::Identity();

  const Eigen::Vector3d residual = position_m - nominal_.position_m;
  const Eigen::Matrix3d noise_cov = std_dev_m.cwiseProduct(std_dev_m).asDiagonal();
  ApplyUpdate<3>(residual, jacobian, noise_cov);
}

void Eskf::UpdateGnssVelocity(
  const Eigen::Vector3d & velocity_mps, const Eigen::Vector3d & std_dev_mps)
{
  RequireFiniteVector(velocity_mps, "Eskf::UpdateGnssVelocity", "velocity_mps");
  for (int i = 0; i < 3; ++i) {
    ads_common::RequireFinitePositive(std_dev_mps[i], "Eskf::UpdateGnssVelocity", "std_dev_mps");
  }

  // h(x) = v，直接就是 map 系速度 —— 雅可比只有速度块，且**与姿态无关**。
  // 这与轮速观测正相反：那个测的是车体轴上的投影，所以要对姿态求导。
  Eigen::Matrix<double, 3, kDim> jacobian = Eigen::Matrix<double, 3, kDim>::Zero();
  jacobian.block<3, 3>(0, kIdxVelocity) = Eigen::Matrix3d::Identity();

  const Eigen::Vector3d residual = velocity_mps - nominal_.velocity_mps;
  const Eigen::Matrix3d noise_cov = std_dev_mps.cwiseProduct(std_dev_mps).asDiagonal();
  ApplyUpdate<3>(residual, jacobian, noise_cov);
}

void Eskf::UpdatePose(
  const Eigen::Vector3d & position_m, const Eigen::Quaterniond & orientation,
  const Eigen::Matrix<double, 6, 6> & covariance)
{
  RequireFiniteVector(position_m, "Eskf::UpdatePose", "position_m");
  ads_common::RequireFinite(orientation.norm(), "Eskf::UpdatePose", "orientation");
  if (orientation.norm() < 1e-6) {
    throw std::invalid_argument("Eskf::UpdatePose: orientation 是零四元数");
  }
  if (!covariance.allFinite()) {
    throw std::invalid_argument("Eskf::UpdatePose: 观测协方差含非有限值");
  }

  Eigen::Quaterniond measured = orientation;
  measured.normalize();

  // 观测模型：位置直接可观，姿态在**局部误差**约定下也直接可观。
  Eigen::Matrix<double, 6, kDim> jacobian = Eigen::Matrix<double, 6, kDim>::Zero();
  jacobian.block<3, 3>(0, kIdxPosition) = Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(3, kIdxAttitude) = Eigen::Matrix3d::Identity();

  // 姿态残差 = Log(q̂⁻¹ ⊗ q_meas)。
  //
  // ⚠️ **不是四元数相减。** 相减在小角度下看起来也能用，但它不是流形上的差；
  //    大角度时方向就错了，而现象只是"收敛得慢"。
  //    这与 ESKF 注入时必须右乘是同一条道理。
  const Eigen::Quaterniond delta = nominal_.orientation.conjugate() * measured;
  const Eigen::AngleAxisd delta_axis(delta.normalized());
  // AngleAxis 的角度在 [0, π]；轴取反可以表示另一半，所以这里不需要额外的
  // 象限处理 —— 但要保证角度落在 (−π, π]，否则残差会绕远路。
  double angle = delta_axis.angle();
  Eigen::Vector3d axis = delta_axis.axis();
  if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  Eigen::Matrix<double, 6, 1> residual;
  residual.head<3>() = position_m - nominal_.position_m;
  residual.tail<3>() = axis * angle;

  ApplyUpdate<6>(residual, jacobian, covariance);
}

void Eskf::UpdateWheelSpeed(double speed_mps, double std_dev_mps)
{
  ads_common::RequireFinite(speed_mps, "Eskf::UpdateWheelSpeed", "speed_mps");
  ads_common::RequireFinitePositive(std_dev_mps, "Eskf::UpdateWheelSpeed", "std_dev_mps");

  // 观测模型 h(x) = (Rᵀ v)_x，即车体纵向速度。
  //
  // 推导（R = R̂(I + [δθ]×)）：
  //     v_body = (I − [δθ]×) R̂ᵀ (v̂ + δv)
  //            ≈ v̂_body + R̂ᵀ δv − [δθ]× v̂_body
  //            = v̂_body + R̂ᵀ δv + [v̂_body]× δθ
  // 所以 ∂/∂δv = R̂ᵀ，∂/∂δθ = [v̂_body]×，各取第 0 行。
  //
  // ⚠️ **姿态那一项在本项目里几乎不起作用，而这一点很反直觉**（2026-08-07 实测）。
  //
  //    [v_body]× 的**第 0 行**是 (0, −v_z, v_y)。车不侧滑时 v_body = (速度, 0, 0)，
  //    于是这一行**恒等于零** —— 纵向速度观测对姿态的耦合在无侧滑时精确为 0。
  //    它只能拾取 v_y / v_z 上的估计噪声。
  //
  //    实测（无 GNSS、只靠轮速辅助的 120 s 推算）：**删掉这一项漂移反而更小**，
  //    62.0 m → 30.6 m —— 因为保留它等于把速度噪声喂进航向。
  //
  //    仍然保留它，理由是：① 它数学上是对的；② 真车与 Gazebo 在大横向加速度下
  //    确实有侧滑，那时 v_body.y ≠ 0，这一项就有了物理内容。
  //    但**不要指望有测试守着它** —— 一个恒为零的项没有可观测的行为差异。
  //    （曾经在这里写过「漏掉它会让转弯时航向慢慢偏」，那是错的，已被注入实验推翻。）
  const Eigen::Matrix3d rot_transpose = nominal_.orientation.toRotationMatrix().transpose();
  const Eigen::Vector3d velocity_body = rot_transpose * nominal_.velocity_mps;

  Eigen::Matrix<double, 1, kDim> jacobian = Eigen::Matrix<double, 1, kDim>::Zero();
  jacobian.block<1, 3>(0, kIdxVelocity) = rot_transpose.row(0);
  jacobian.block<1, 3>(0, kIdxAttitude) = SkewSymmetric(velocity_body).row(0);

  Eigen::Matrix<double, 1, 1> residual;
  residual << speed_mps - velocity_body.x();
  Eigen::Matrix<double, 1, 1> noise_cov;
  noise_cov << std_dev_mps * std_dev_mps;
  ApplyUpdate<1>(residual, jacobian, noise_cov);
}

template <int kMeasDim>
void Eskf::ApplyUpdate(
  const Eigen::Matrix<double, kMeasDim, 1> & residual,
  const Eigen::Matrix<double, kMeasDim, kDim> & jacobian,
  const Eigen::Matrix<double, kMeasDim, kMeasDim> & noise_cov)
{
  const Eigen::Matrix<double, kMeasDim, kMeasDim> innovation_cov =
    jacobian * covariance_ * jacobian.transpose() + noise_cov;

  // 新息协方差必须可逆。它奇异意味着「该方向上完全没有不确定度」，
  // 而观测噪声 R 是正定的，所以只有 P 被污染成 NaN/Inf 才会走到这里。
  const Eigen::Matrix<double, kMeasDim, kMeasDim> innovation_inv = innovation_cov.inverse();
  if (!innovation_inv.allFinite()) {
    throw std::runtime_error(
      "Eskf: 新息协方差不可逆 —— 协方差矩阵已经被污染。"
      "根因通常在上游（喂进来的观测或 IMU 含非有限值），不在这一步。");
  }

  const Eigen::Matrix<double, kDim, kMeasDim> gain =
    covariance_ * jacobian.transpose() * innovation_inv;
  const Vector15d error = gain * residual;

  // ---- Joseph 形式的协方差更新 ------------------------------------------
  //     P = (I − KH) P (I − KH)ᵀ + K R Kᵀ
  //
  // 而不是更常见的简写 P = (I − KH) P。两者在**精确算术下等价**，
  // 但简写形式只有在 K 恰好是最优增益时才成立，且它的结果不保证对称 ——
  // 浮点舍入累积几千步之后 P 会失去正定性，症状是增益突然变得荒谬、
  // 滤波器一步跳飞。Joseph 形式的右边是两个「XPXᵀ」型的和，
  // **结构上就是对称半正定的**，代价是多一次 15×15 乘法。
  //
  // CP-P4-A 第 4 条「协方差始终正定对称」正是靠这一行成立的。
  const Matrix15d identity_minus_kh = Matrix15d::Identity() - gain * jacobian;
  covariance_ = identity_minus_kh * covariance_ * identity_minus_kh.transpose() +
                gain * noise_cov * gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose()).eval();

  InjectAndReset(error);
}

void Eskf::InjectAndReset(const Vector15d & error)
{
  // ---- 注入：把误差加回名义状态 ------------------------------------------
  nominal_.position_m += error.segment<3>(kIdxPosition);
  nominal_.velocity_mps += error.segment<3>(kIdxVelocity);
  nominal_.accel_bias_mps2 += error.segment<3>(kIdxAccelBias);
  nominal_.gyro_bias_rad_s += error.segment<3>(kIdxGyroBias);

  // 姿态是**右乘**，不是相加 —— 它活在流形上。
  const Eigen::Vector3d delta_theta = error.segment<3>(kIdxAttitude);
  nominal_.orientation = nominal_.orientation * QuaternionFromRotationVector(delta_theta);
  nominal_.orientation.normalize();

  // ---- 重置：误差归零，并把协方差搬到新的线性化点上 ----------------------
  //
  // ⚠️ **这一步最容易被漏掉，而漏掉之后滤波器看起来完全正常。**
  //
  // 误差状态被注入名义状态之后就必须清零，否则同一份修正会在下一次更新里
  // 被重复计入 —— 症状是缓慢发散而不是立刻出错。本实现里误差是局部变量，
  // 天然"清零"了，所以真正需要显式做的是**协方差那一半**：
  //
  // 线性化点从 q̂ 挪到了 q̂ ⊗ q{δθ}，δθ 的协方差要跟着做一次坐标变换，
  // 雅可比是 G_θ = I − ½[δθ]×（Sola eq. 285）。
  //
  // 这一项通常很小（δθ 是小角度），很多实现直接省掉；这里保留，
  // 因为省掉之后**没有任何现象**能提示你省掉了 —— 它只是让协方差
  // 系统性地略微偏乐观，而"偏乐观"正是滤波器最难被发现的那类错误。
  Matrix15d reset_jacobian = Matrix15d::Identity();
  reset_jacobian.block<3, 3>(kIdxAttitude, kIdxAttitude) =
    Eigen::Matrix3d::Identity() - 0.5 * SkewSymmetric(delta_theta);
  covariance_ = reset_jacobian * covariance_ * reset_jacobian.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose()).eval();
}

}  // namespace ads_localization
