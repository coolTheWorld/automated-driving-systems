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
//  localization_node —— 定位模块的 ROS 包装层
//
//  订阅  /imu            sensor_msgs/Imu           100 Hz，ESKF 的预测源
//  订阅  /gnss           sensor_msgs/NavSatFix     10 Hz，绝对位置观测
//  订阅  /odom           nav_msgs/Odometry         轮速推算，取纵向速度当观测
//  订阅  /lidar/points   sensor_msgs/PointCloud2   10 Hz，NDT 配准
//  发布  TF  map → odom                            **本节点的主产出**
//  发布  /localization/pose         PoseWithCovarianceStamped
//  发布  /localization/diagnostics  DiagnosticArray
//
//  ## 为什么发 map→odom 而不是 map→base_link
//
//  `odom→base_link` 由 Gazebo 的轮速里程计给（高频、平滑、但漂移）。
//  定位负责估计 `map→base_link`，两者相除得到 `map→odom`：
//
//      T(map→odom) = T(map→base_link)_估计 ∘ T(odom→base_link)⁻¹
//
//  这是 ROS 的标准分工，好处是**下游拿到的位姿始终连续**：定位跳变时跳的是
//  map→odom（低频修正量），而 odom→base_link 一直平滑。
//  直接发 map→base_link 会让 TF 树上 base_link 有两个父节点，tf2 直接报错。
//
//  ## ⚠️ 本节点禁止订阅 /ego_pose_gt
//
//  那是真值，只给评测脚本用。一份「标准答案」流进被考核的一方，
//  考试就失去意义 —— 而它看起来仍然是绿的。
//
//  ## 已知的未实现项（不要假设它们能用）
//
//  · **在途初始对准**：冷启动的航向来自参数 `initial_yaw_rad`（粗略先验）。
//    真车靠双天线 GNSS 或运动对准拿这个量，本阶段不做。
//    位置初值来自带噪声的 GNSS，所以滤波器仍要靠自己走完全程。
//  · **扫描的运动畸变补偿**：车 5 m/s、雷达 10 Hz → 一帧内位移 0.5 m。
//  · **NDT 输出协方差未标定**：见 NdtAlignParams::covariance_scale 的警告。
//
//  推导与参数见 docs/modules/localization.md。**改本文件前先读它。**
// =============================================================================

// ⚠️ Eigen 的头**没有扩展名**，cpplint 把它归成「C 系统头」，必须排在
//    C++ 标准库之前 —— 与 <gtest/gtest.h>、<tinyxml2.h> 同源的第四个实例。
//    见 CLAUDE.md 的 lint 陷阱表。
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// tf2_ros 用 .hpp 而不是 .h —— 后者在 Jazzy 里是待淘汰的兼容 shim，
// 而且会被 cpplint 按后缀误判成 C 系统头。
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "ads_localization/eskf.hpp"
#include "ads_localization/geodetic.hpp"
#include "ads_localization/ndt.hpp"
#include "ads_localization/ndt_align.hpp"
#include "ads_localization/point_cloud_io.hpp"

namespace
{

/// 定位当前靠什么在工作。**必须能被外部看到** ——
/// 「位置还行」和「位置还行但已经退化成纯推算」在数值上一模一样，
/// 只有把状态发出来才分得开。
enum class LocalizationState
{
  kInitializing,   ///< 还没收到足够的传感器数据
  kNdtAided,       ///< 正常：IMU + 轮速 + GNSS + NDT
  kGnssOnly,       ///< NDT 退化或失败，退回 GNSS + IMU
  kDeadReckoning,  ///< GNSS 也没有，纯推算 —— 误差会按 t^1.5 涨
};

const char * StateName(LocalizationState state)
{
  switch (state) {
    case LocalizationState::kInitializing:
      return "INITIALIZING";
    case LocalizationState::kNdtAided:
      return "NDT_AIDED";
    case LocalizationState::kGnssOnly:
      return "GNSS_ONLY";
    case LocalizationState::kDeadReckoning:
      return "DEAD_RECKONING";
  }
  return "UNKNOWN";
}

}  // namespace

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode() : Node("localization_node")
  {
    // ---- 大地原点 -------------------------------------------------------
    // ⚠️ 必须与世界文件的 <spherical_coordinates> 一致。两者不一致的症状是
    //    定位稳定地偏一个常量，而没有任何模块报错。
    //    由 launch 从 config/campus_map.yaml 传进来 —— 本包不依赖 ads_map。
    origin_.latitude_deg = declare_parameter<double>("geo_origin.latitude_deg", 0.0);
    origin_.longitude_deg = declare_parameter<double>("geo_origin.longitude_deg", 0.0);
    origin_.elevation_m = declare_parameter<double>("geo_origin.elevation_m", 0.0);
    origin_.Validate();

    // ---- ESKF 参数（与 config/vehicle_params.yaml 的 noise 段同源）------
    ads_localization::EskfParams eskf_params;
    eskf_params.gyro_noise_rad_s = declare_parameter<double>("eskf.gyro_noise_rad_s", 8.7e-4);
    eskf_params.accel_noise_mps2 = declare_parameter<double>("eskf.accel_noise_mps2", 1.0e-2);
    eskf_params.gyro_bias_rw_rad_s = declare_parameter<double>("eskf.gyro_bias_rw_rad_s", 4.8e-5);
    eskf_params.accel_bias_rw_mps2 = declare_parameter<double>("eskf.accel_bias_rw_mps2", 1.0e-4);
    eskf_params.init_position_std_m = declare_parameter<double>("eskf.init_position_std_m", 3.0);
    eskf_params.init_velocity_std_mps =
      declare_parameter<double>("eskf.init_velocity_std_mps", 1.0);
    eskf_params.init_attitude_std_rad =
      declare_parameter<double>("eskf.init_attitude_std_rad", 0.10);
    eskf_params.init_accel_bias_std_mps2 =
      declare_parameter<double>("eskf.init_accel_bias_std_mps2", 0.05);
    eskf_params.init_gyro_bias_std_rad_s =
      declare_parameter<double>("eskf.init_gyro_bias_std_rad_s", 5.0e-3);
    eskf_params.max_imu_dt_s = declare_parameter<double>("eskf.max_imu_dt_s", 0.5);
    eskf_params_ = eskf_params;

    // ---- 观测噪声 -------------------------------------------------------
    gnss_horizontal_std_m_ = declare_parameter<double>("gnss.horizontal_std_m", 2.0);
    gnss_vertical_std_m_ = declare_parameter<double>("gnss.vertical_std_m", 4.0);
    wheel_speed_std_mps_ = declare_parameter<double>("wheel.speed_std_mps", 0.05);

    // ---- GNSS 天线的杆臂 -------------------------------------------------
    //
    // ⚠️⚠️ **GNSS 报的是天线的位置，不是 base_link 的位置。**
    //
    // 本车天线装在 (0.5, 0, 1.6)（config/vehicle_params.yaml 的 sensors.gnss），
    // 所以直接把 GNSS 当 base_link 用会引入一个**系统性**偏差：
    //   竖直 1.6 m —— 常量，会把 NDT 的初值顶出收敛域；
    //   水平 0.5 m —— **随车头方向旋转**，于是它是一个随航向变化的偏差，
    //                 看起来像"定位在某些朝向上偏得多一些"。
    //
    // 这个偏差不会让滤波器发散，只会让它稳定地偏一点 —— 于是所有人去调 Q，
    // 而错在一次坐标变换里。CP-P4-B 实测正是栽在这里：连续两轮的初始 z
    // 都是 1.58 / 1.59，与天线高度 1.6 严丝合缝。
    //
    //     p_base = p_天线 − R · r_天线
    lever_arm_body_ = Eigen::Vector3d(
      declare_parameter<double>("gnss.lever_arm_x_m", 0.0),
      declare_parameter<double>("gnss.lever_arm_y_m", 0.0),
      declare_parameter<double>("gnss.lever_arm_z_m", 0.0));

    // ---- 冷启动的航向先验 ------------------------------------------------
    // ⚠️ **在途初始对准未实现。** 真车靠双天线 GNSS 或运动对准拿这个量。
    //    这里给一个粗略先验，位置初值仍来自带 2 m 噪声的 GNSS，
    //    所以滤波器还是要靠自己走完全程。
    initial_yaw_rad_ = declare_parameter<double>("initial_yaw_rad", 0.0);

    // ---- 初始化前先把 GNSS 平均掉 ---------------------------------------
    //
    // ⚠️⚠️ **这一条是 CP-P4-B 五轮实测炸出来的，是整个 S4 最关键的一处。**
    //
    // 第一版用**单次** GNSS 定位当初值（σ=2 m）。结果是双峰的：同一份配置
    // 跑五轮，定位横向误差在 0.0664 m 和 78 m 之间跳，差别只是开机那一枪
    // GNSS 的运气。
    //
    // 机理：初值偏 1.6 m 时落在 NDT 收敛域之外（体素 2 m），
    // NDT 于是**收敛到一个错误的局部极小** —— 而它法向散布正常、
    // inlier 正常、协方差很紧，**一个警告都不报**。
    // 滤波器锁死在那个错位姿上，此后一路发散。
    // 实测第 5 轮：前 973 拍全是 NDT_AIDED，横向误差却稳定在 0.79 m 不收敛。
    //
    // 「NDT 稳定地收敛到一个错误的位姿，比不收敛危险得多」——
    // 这句话写在 ndt.hpp 的开头，而我第一版正是踩了它。
    //
    // 修法：GNSS 的位置噪声是**白噪声**（SDF 里只有 stddev，没有 bias 项，
    // 见 gen_vehicle_model.py 的 _navsat_noise），所以平均 N 次能把 σ 降
    // √N 倍。取 30（10 Hz 下 3 s）→ σ 从 2 m 降到 0.37 m，稳稳落在收敛域内。
    // 车在栈起来之后本来就要静止约 30 s 才发目标点，这 3 s 不花任何代价。
    //
    // ⚠️ 调小 → 初值方差回升，双峰行为回来；调大 → 起步更慢，
    //    且若车在此期间已经动了，平均就没有意义（本项目里它不会动）。
    init_gnss_samples_ = declare_parameter<int>("init_gnss_samples", 30);

    // ---- NDT -------------------------------------------------------------
    const std::string map_path = declare_parameter<std::string>("map_pcd_path", "");
    ads_localization::NdtGridParams grid_params;
    grid_params.voxel_size_m = declare_parameter<double>("ndt.voxel_size_m", 2.0);
    grid_params.min_points_per_voxel = declare_parameter<int>("ndt.min_points_per_voxel", 6);
    // 迭代上限。**这是控制单帧耗时的正确杠杆** —— 它砍的是"怎么也收敛不了
    // 的那些帧"，而不是信息量。
    //
    // 实测（2026-08-10，CP-P4-B）：上限 30 时中位 47.7 ms、99% 分位 129.4 ms，
    // 1.65% 的帧超判据（100 ms）。有 ESKF 的预测位姿作初值时正常帧只要 3–5 次，
    // 用满 30 次的帧本来也没收敛好。
    ndt_params_.max_iterations = declare_parameter<int>("ndt.max_iterations", 30);
    // 线搜索折半次数。每折半一次就是一次**全量**打分，所以最坏情况是
    // max_iterations × 这个数 次全量打分。
    //
    // ⚠️ **试过砍到 4，结果更差，别再试**（2026-08-10 实测）：
    //    耗时没降反升（129 → 165 ms），因为步长被迫变小之后要迭代更多次；
    //    而且更多帧到不了 epsilon 判据。**这不是一个安全的省时杠杆。**
    ndt_params_.max_line_search_trials = declare_parameter<int>("ndt.max_line_search_trials", 10);
    ndt_params_.min_normal_diversity = declare_parameter<double>("ndt.min_normal_diversity", 1e-3);
    ndt_params_.min_inlier_ratio = declare_parameter<double>("ndt.min_inlier_ratio", 0.2);
    // ⚠️ **已标定**（2026-08-11，NEES 实测，见 scripts/calibrate_ndt_covariance.py）。
    //    AlignNdt 输出的是 scale × H⁻¹，而 H 只与真实协方差**成正比**，比例未知。
    //    标定方法：NEES = eᵀΣ⁻¹e 对真值，理论期望 = 自由度 6；
    //    实测中位数 4.331 / 5.175（两轮，各 324 / 367 个样本）⟹ scale ≈ 0.72 / 0.86。
    //    取 0.8。
    //
    // ⚠️ **用中位数反解，不用均值**：NEES 分布是重尾的 —— 实测均值 10.5 / 11.8，
    //    而 95 分位高达 48 / 52（理论值的 8 倍）。少数「半收敛却带着毫米级协方差」
    //    的帧主导了均值，拿均值标会把 scale 推大一倍，让 NDT 整体失去话语权。
    //
    // ⚠️ **那条重尾没有被这次标定解决** —— 它是「NDT 报告成功但其实锁偏了」的
    //    直接证据，要靠**严格卡方新息门限**拦（P8 前置第 2 条，仍未做）。
    //    标定只是让它有了可用的前提。
    //
    // 调大 → NDT 说了不算，位姿被 GNSS（σ=2 m）拉着走；
    // 调小 → 滤波器过度相信 NDT，锁偏时会被硬拽过去。
    ndt_params_.covariance_scale = declare_parameter<double>("ndt.covariance_scale", 0.8);
    // 扫描降采样：每 N 个点取一个。
    //
    // **这个数是量出来的，不是猜的**（2026-08-10，CP-P4-B 首跑）：
    //   stride=4（14400 点）：中位 47.7 ms、90% 分位 69.4、**1.65% 的帧超 100 ms**
    //   而 CP-P4-B 的判据是单帧 < 100 ms（10 Hz 实时性）。
    //
    // ⚠️ **试过改成 8，结果是负面的，别再试**（2026-08-10 实测）：
    //   耗时确实降到 78 ms，但**最大横向误差从 0.0664 m 涨到 1.7167 m**，
    //   而且日志里冒出「NDT 退化（法向散布 0.000743 / 0.000564 / 0.000332）」。
    //
    //   根因：**均匀降采样对不同类别的点不等价**。地面点冗余度极高，
    //   而杆件本来就只有几个点（半径 0.15 m，20 m 处每线才 4.3 个点）。
    //   按同一个比例扔，扔掉的恰好是**唯一提供 x/y/yaw 约束**的那些 ——
    //   法向散布于是塌到判据以下，NDT 被拒，退回 GNSS（σ=2 m）。
    //   实测那一跑里有 199 拍处在 GNSS_ONLY。
    //
    //   真正该动的杠杆是**迭代次数**（见下面的 ndt.max_iterations）：
    //   它只砍尾部，不动信息量。若将来还要再快，正确做法是
    //   **地面点分割**（地面只贡献 z/roll/pitch，那几个自由度 IMU 的重力
    //   矢量已经给了），而不是均匀抽稀。
    //
    // ⚠️ 静止时只要 11 ms、动起来要 48 ms，差的是**迭代次数**：
    //    静止时 ESKF 的预测位姿几乎就是答案，3 次就收敛；
    //    动起来初值差一点，要十几次。所以「耗时」是随工况变的，
    //    拿静止时的数去估算实时性会低估三四倍。
    // ⚠️ **按类别降采样，不能一视同仁。**
    //
    // 地面点占扫描的大头，而它只贡献 z/roll/pitch —— 那几个自由度 IMU 的
    // 重力矢量已经给了。结构点（杆件、墙面）才是 x/y/yaw 的唯一来源，
    // 而杆件本来就只有几个点（半径 0.15 m，20 m 处每线才 4.3 个）。
    //
    // 实测教训（2026-08-10）：均匀 stride 从 4 改到 8，耗时 129→78 ms，
    // 但横向误差 0.0664→1.72 m —— 扔掉的恰好是唯一有信息的那些。
    //
    // 结构点取 4、地面点取 24：结构点的密度与那次拿到 0.0664 m 的基线相同，
    // 而地面点被抽掉 6/7 —— **总点数降下来了，信息量一点没少**。
    // 这就是「按类别降采样」与「均匀降采样」的全部区别。
    scan_stride_ = std::max<int>(1, static_cast<int>(declare_parameter<int>("ndt.scan_stride", 4)));
    ground_stride_ =
      std::max<int>(1, static_cast<int>(declare_parameter<int>("ndt.ground_stride", 24)));
    // base_link 原点在**地面高度**，所以地面点的 z ≈ 0（路面板 1 cm + 雷达噪声）。
    // 0.30 m 远高于噪声，又低于任何有意义的结构。
    ground_height_m_ = declare_parameter<double>("ndt.ground_height_m", 0.30);
    // 点云比这个还旧就直接丢。
    //
    // ⚠️⚠️ **这一条是 CP-P4-B 实测炸出来的，是 S4 最关键的一处防线。**
    //   NDT 一旦慢过雷达周期（100 ms），单线程执行器就排队积压，
    //   处理的点云越来越旧 —— 而 NDT 会老老实实把那帧旧扫描配准好，
    //   返回**它拍摄时刻**的位姿。于是估计越落越远，**全程 NDT_AIDED、
    //   横向误差只有 3 cm、没有任何报警**，而纵向误差以车速增长。
    //   实测第 1 轮：车冲过目标 24 m，/localization/pose 的发布率掉到一半。
    //
    //   丢掉旧帧之后最坏情况退化成「NDT 更新少了」，由 GNSS 兜住 ——
    //   那是可以接受的降级；而「用旧帧算出一个自信的错位姿」不可接受。
    max_cloud_age_s_ = declare_parameter<double>("ndt.max_cloud_age_s", 0.15);

    // ---- 粗配准网格：失锁之后把初值拉回收敛域 ---------------------------
    //
    // ⚠️⚠️ **这是 CP-P4-B 实测暴露的最后一个失效模式。**
    //
    // 精配准的收敛域被高斯的"薄"限死了：2 m 体素、特征值下限 0.01
    // → σ_n = 0.058 m。横向误差到 0.3 m 时，结构体素的马氏距离平方是
    // (0.3/0.058)² = 26.8，权重 exp(−13.4) ≈ 1.5e-6 —— **结构项直接消失**，
    // 法向散布只剩地面的 +z（实测 1.4e-06），NDT 于是被**自己的退化判据**
    // 拒掉，误差继续长，**再也回不来**。
    //
    // 粗网格要同时放大**两个**量才有用：
    //   体素 6 m       → σ_max ≈ 1.7 m
    //   特征值下限 0.05 → σ_n = 1.7 × √0.05 ≈ 0.39 m，收敛域约 1 m
    // 只放大体素而不放宽下限，高斯还是同样地薄，白搭。
    //
    // 内存代价：体素数约为精网格的 1/27，可忽略。
    // 时间代价：**只在失锁时跑**，稳态不花钱。
    ads_localization::NdtGridParams coarse_params;
    coarse_params.voxel_size_m = declare_parameter<double>("ndt.coarse_voxel_size_m", 6.0);
    coarse_params.eigenvalue_ratio_floor =
      declare_parameter<double>("ndt.coarse_eigenvalue_ratio_floor", 0.05);
    coarse_params.min_points_per_voxel = grid_params.min_points_per_voxel;
    // 连续几帧被拒才启动恢复。取 3（0.3 s）—— 太小会在正常抖动时白跑粗配准，
    // 太大则失锁之后要等更久，而误差在这期间一直在长。
    recovery_after_failures_ = declare_parameter<int>("ndt.recovery_after_failures", 3);
    // 开机后头几帧**无条件**走粗→精。
    //
    // ⚠️⚠️ **「连续 N 帧被拒」这个触发条件抓不到开机锁错** ——
    //    锁错的定义就是「NDT 报告成功」，它不会连续失败。
    //    实测第 2 轮：55 帧被拒但从未连续 3 帧，恢复一次都没触发，而定位错了 28 m。
    //
    //    最脆弱的时刻是**开机第一帧**：那时初值只有 GNSS 的平均值
    //    （σ=0.37 m，2σ 就是 0.74 m），而精配准的收敛域只有约 0.3 m
    //    （受 σ_n = 0.058 m 的薄高斯限制）。一旦第一帧锁错，
    //    之后每一帧都从那个错位姿附近重新收敛，**再也回不来**。
    //
    //    车在这几帧里是静止的，粗配准那点开销白送。
    bootstrap_coarse_frames_ = declare_parameter<int>("ndt.bootstrap_coarse_frames", 5);

    // ---- 新息门限：NDT 的输出离滤波器预测太远就丢掉（S5）-----------------
    //
    // 这是**固定阈值**的版本，不是严格的卡方检验。区别与理由：
    //   严格卡方要算 d² = yᵀS⁻¹y，其中 S = HPHᵀ + R，而 R 就是 NDT 的输出
    //   协方差 —— `covariance_scale` 虽已按 NEES 标定到 0.8（2026-08-11，
    //   下面第 371 行附近），但那是**中位数对齐**，重尾还在（标定当天实测），
    //   拿它当严格卡方的 R 仍会误杀重尾里的好帧。R 偏小则好帧被误杀，
    //   偏大则坏帧照样放行，于是卡方分布表里那些数（6 自由度 95% 分位
    //   12.59）**还不能用**。严格卡方列为 P8 前置。
    //   在标定之前先装一个阈值随便填的保险丝，比不装更糟 ——
    //   它会让人以为有防线，而没人知道它挡在哪里。
    //
    // 固定阈值不需要标定，因为好帧与坏帧之间隔着一个数量级以上的空档。
    // **阈值来自实测，不是拍脑袋**（Gazebo 全栈闭环，2026-08-10，三轮）：
    //
    //   正常新息  中位数 0.027 m，90 分位 0.11–0.16 m
    //   每轮峰值  1.18 / 0.61 / 0.56 m ← 分母取最差的 1.18
    //   灾难性锁错 **28 m**（开机锁错）、**462 m**（迭代上限砍到 15）
    //
    // 取 3.0 m：对最差的好帧留 2.5 倍余量，距最轻的坏帧还有 9.3 倍。
    // ⚠️ 一度想取 1.5 m，那样对 1.18 m 只剩 **1.27 倍** —— 换一轮就可能误杀。
    //    误杀不是无害的：那一帧掉进 GNSS_ONLY（σ=2 m），而日志里只有一条
    //    3 s 一次的节流告警，很容易被当成噪声。
    //
    // ⚠️⚠️ **它是「防灾难」不是「保精度」。** 3 m 的锁错本身早就违反了
    //    SPEC §1 的 0.3 m，这个门限拦不住它 —— 它拦的是随后那个正反馈：
    //    错位姿 → 下一帧初值更差 → 错得更多 → 28 m → 462 m。
    //    「0.5 m 量级的锁错」要靠**标定过协方差的严格卡方**，
    //    那一条列为 P8（CARLA 验收）的前置条件，见 tasks/todo.md。
    //
    // 峰值的来源也实测过：最大那次开机 z 初值偏 −1.12 m（GNSS 高程 σ=4 m），
    // 而 z 恰好是 NDT 最强、GNSS 最弱的方向，于是 NDT 一上来就要拉回一米多。
    // **那是系统在正常工作，不是锁错** —— 这也是不能把阈值定太紧的直接理由。
    max_innovation_m_ = declare_parameter<double>("ndt.max_innovation_m", 3.0);
    // 转角实测最大 0.51°，10° 留 19 倍余量。它主要防的是走廊类重复几何下
    // 的 180° 翻转 —— 那种错误一旦发生就是半圈量级，不需要更紧的阈值。
    max_innovation_rad_ = declare_parameter<double>("ndt.max_innovation_rad", 0.175);  // 10°

    if (map_path.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "未提供 map_pcd_path，NDT 关闭 —— 定位会退化成 GNSS + IMU，"
        "精度到不了 SPEC §1 的 0.3 m（那正是要做 NDT 的理由）");
    } else {
      try {
        // 只读一次盘，精/粗两个网格从同一份点云构造。
        const std::vector<Eigen::Vector3d> map_cloud = ads_localization::LoadPcdAscii(map_path);
        ndt_map_ = std::make_unique<ads_localization::NdtGrid>(map_cloud, grid_params);
        // ⚠️⚠️ **粗网格的构造曾经整个缺失**（2026-08-12 复检发现）：上面那一大段
        //    注释推导了 coarse_params 该怎么取，参数也声明了，唯独没有这一行 ——
        //    于是 on_cloud 里 `ndt_coarse_map_ && …` 恒为假，开机 bootstrap 与
        //    失锁恢复**整条是死代码**。L1 用例全绿（它们自己构造粗网格），
        //    节点侧却零功能。CP-P4-B 的「恢复实测 0 次触发」当时被归因为
        //    「粗网格同样退化」—— 那个归因是错的：粗配准从未执行过，
        //    `coarse.degenerate` 根本没被观测过。
        //    教训：**参数声明得再讲究，也证明不了消费它的对象存在。**
        ndt_coarse_map_ = std::make_unique<ads_localization::NdtGrid>(map_cloud, coarse_params);
        RCLCPP_INFO(
          get_logger(),
          "点云地图已载入：精网格 %zu 个非空体素（稀疏丢弃 %zu，退化修正 %zu），"
          "粗网格 %zu 个（恢复/自举用）",
          ndt_map_->size(), ndt_map_->discarded_sparse_voxels(), ndt_map_->regularized_voxels(),
          ndt_coarse_map_->size());
      } catch (const std::exception & e) {
        // 地图载入失败必须**大声报**。静默降级的症状是「定位怎么调都到不了
        // 0.3 m」，而没人想到 NDT 根本没在跑。
        RCLCPP_ERROR(get_logger(), "点云地图载入失败：%s —— NDT 关闭", e.what());
      }
    }

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose", rclcpp::QoS(10));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/localization/diagnostics", rclcpp::QoS(10));
    // ⚠️ **诊断输出，不是算法接口。** 发的是 NDT 的**原始**输出位姿与协方差，
    //    在它被喂进滤波器**之前**。存在的唯一理由是标定 covariance_scale：
    //    要算 NEES 就必须拿到「这一帧 NDT 自己说它有多准」和「它实际差多少」，
    //    而 /localization/pose 是**滤波后**的，那一层已经把 NDT 和 GNSS 混在一起了。
    //    ⚠️ 算法节点不许订阅它 —— 它没有经过任何门限，是原料不是产品。
    ndt_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/ndt_pose", rclcpp::QoS(10));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { OnImu(std::move(msg)); });
    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gnss", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) { OnGnss(std::move(msg)); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { OnOdom(std::move(msg)); });
    // ⚠️ 点云用 **reliable + 深度 10**，不是 best-effort。CLAUDE.md 记着实测：
    //    best-effort 下静默丢帧，频率只有标称的 35%，且没有任何日志。
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { OnCloud(std::move(msg)); });

    RCLCPP_INFO(get_logger(), "localization_node 就绪，等待 GNSS 与 IMU 完成初始化");
  }

private:
  // ---------------------------------------------------------------------
  //  初始化：等到第一帧 GNSS 才能定位置初值
  // ---------------------------------------------------------------------
  void TryInitialize()
  {
    if (eskf_ || gnss_sample_count_ < init_gnss_samples_) {
      return;
    }
    ads_localization::NominalState init;
    // 用**平均值**而不是最后一枪，见构造函数里 init_gnss_samples_ 的说明。
    init.position_m = gnss_sum_ / static_cast<double>(gnss_sample_count_);
    init.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(initial_yaw_rad_, Eigen::Vector3d::UnitZ()));
    eskf_ = std::make_unique<ads_localization::Eskf>(eskf_params_, init);
    state_ = LocalizationState::kGnssOnly;
    RCLCPP_INFO(
      get_logger(), "定位已初始化：位置 (%.2f, %.2f, %.2f)，航向先验 %.3f rad", init.position_m.x(),
      init.position_m.y(), init.position_m.z(), initial_yaw_rad_);
  }

  void OnImu(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    TryInitialize();
    if (!eskf_) {
      return;
    }
    ads_localization::ImuSample sample;
    sample.time_s = rclcpp::Time(msg->header.stamp).seconds();
    sample.accel_mps2 = {
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    sample.gyro_rad_s = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    try {
      eskf_->Predict(sample);
    } catch (const std::exception & e) {
      // 时间倒流通常意味着**两套仿真同时在发 /clock**（CLAUDE.md 有专门一条）。
      // 节流打印，否则 100 Hz 会把日志刷爆。
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "IMU 预测失败：%s", e.what());
      return;
    }
    PublishOutputs();
  }

  void OnGnss(sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // NavSatFix 在无定位时会填 NaN。用比较拦不住（NaN 参与任何比较都为假），
    // GeodeticToLocal 内部用 RequireFinite 显式判。
    try {
      last_gnss_local_ =
        ads_localization::GeodeticToLocal(msg->latitude, msg->longitude, msg->altitude, origin_);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "GNSS 无效：%s", e.what());
      return;
    }
    // 杆臂补偿：把天线位置换算回 base_link。
    // 初始化之前还没有姿态估计，用航向先验代替 —— 车此时静止，
    // 那个先验就是它的真实朝向（误差只有先验本身的误差）。
    const Eigen::Matrix3d rotation =
      eskf_ ? eskf_->state().orientation.toRotationMatrix()
            : Eigen::Matrix3d(
                Eigen::AngleAxisd(initial_yaw_rad_, Eigen::Vector3d::UnitZ()).toRotationMatrix());
    last_gnss_local_ -= rotation * lever_arm_body_;

    have_gnss_ = true;
    last_gnss_time_ = now();

    if (!eskf_) {
      // 初始化之前只累加，不做别的 —— 车此时还没动，均值才是最好的估计。
      gnss_sum_ += last_gnss_local_;
      ++gnss_sample_count_;
      TryInitialize();
      return;
    }
    try {
      eskf_->UpdateGnssPosition(
        last_gnss_local_, {gnss_horizontal_std_m_, gnss_horizontal_std_m_, gnss_vertical_std_m_});
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "GNSS 更新失败：%s", e.what());
    }
  }

  void OnOdom(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // ⚠️ 只取**纵向速度**，不取位姿。/odom 的位姿在 odom 系且会漂移 ——
    //    拿它当 map 系用会差整整一个 map→odom 变换。
    last_odom_ = *msg;
    have_odom_ = true;
    if (!eskf_) {
      return;
    }
    try {
      eskf_->UpdateWheelSpeed(msg->twist.twist.linear.x, wheel_speed_std_mps_);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "轮速更新失败：%s", e.what());
    }
  }

  void OnCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!eskf_ || !ndt_map_) {
      return;
    }
    // ---- 陈旧点云直接丢 ---------------------------------------------
    const double age_s = (now() - rclcpp::Time(msg->header.stamp)).seconds();
    if (age_s > max_cloud_age_s_) {
      ++dropped_stale_clouds_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "点云已经旧了 %.0f ms（上限 %.0f），丢弃。NDT 跟不上雷达周期时"
        "用旧帧配准会返回一个**自信的错位姿**，比不更新危险得多。累计丢 %ld 帧",
        age_s * 1e3, max_cloud_age_s_ * 1e3, dropped_stale_clouds_);
      return;
    }

    // 扫描点在 base_link 系（lidar_preprocessor 已经做过变换）。
    std::vector<Eigen::Vector3d> scan;
    scan.reserve(msg->width * msg->height / static_cast<size_t>(scan_stride_) + 1);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    int index = 0;
    int ground_points = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++index) {
      // ⚠️ gpu_lidar 的无回波射线返回 **±inf 不是 NaN**（CLAUDE.md 有专门一条），
      //    两者都要滤。用比较拦不住 —— 必须先判有限性再做任何比较。
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) {
        continue;
      }
      const bool is_ground = *it_z < ground_height_m_;
      if (is_ground) {
        ++ground_points;
      }
      // 地面点抽得狠，结构点留得密。
      //
      // ⚠️ 地面点**不能全扔**：退化判据看的是匹配上的体素法向散布，
      //    全扔之后法向只剩水平方向，λ_min（竖直）→ 0，
      //    NDT 会被自己的退化判据误杀。留一小撮就够撑住那个自由度。
      if (index % (is_ground ? ground_stride_ : scan_stride_) != 0) {
        continue;
      }
      scan.emplace_back(*it_x, *it_y, *it_z);
    }
    last_scan_points_ = static_cast<int>(scan.size());
    last_ground_fraction_ =
      index > 0 ? static_cast<double>(ground_points) / static_cast<double>(index) : 0.0;
    if (scan.size() < 100) {
      return;
    }

    // NDT 的初值用 ESKF 的当前估计 —— 这是闭环：滤波器给配准初值，
    // 配准给滤波器观测。初值差超过收敛域就发散，所以不能用固定初值。
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.linear() = eskf_->state().orientation.toRotationMatrix();
    guess.translation() = eskf_->state().position_m;

    // ⚠️ 新息门限要比的是「NDT 输出 vs **滤波器预测**」，而下面 `guess` 会被
    //    粗配准的结果覆盖掉，所以必须**在那之前**留一份。
    //    拿覆盖后的 guess 去比，恢复时量到的是「精配准离粗配准多远」——
    //    那是另一个问题，而且恰好总是很小，于是门限在最该出手的时候失效。
    const Eigen::Isometry3d filter_prediction = guess;
    bool used_coarse = false;

    const auto started = std::chrono::steady_clock::now();
    ads_localization::NdtAlignResult result;
    try {
      // ---- 失锁恢复：先用粗网格把初值拉回精配准的收敛域 ----------------
      //
      // 只在连续被拒若干帧之后才跑。粗配准的结果**只当初值**，
      // 绝不直接喂给滤波器 —— 它的精度不够（体素 6 m），
      // 而一个不够准却带着协方差的观测正是把滤波器带偏的方式。
      const bool bootstrapping = ndt_frames_ < bootstrap_coarse_frames_;
      if (
        ndt_coarse_map_ &&
        (bootstrapping || consecutive_ndt_failures_ >= recovery_after_failures_)) {
        const auto coarse = ads_localization::AlignNdt(*ndt_coarse_map_, scan, guess, ndt_params_);
        if (!coarse.degenerate) {
          guess = coarse.pose;
          used_coarse = true;
          ++recovery_attempts_;
          if (bootstrapping) {
            // 开机自举是**预期路径**，用 INFO 不用 WARN —— 否则每次启动都刷
            // 5 条告警，真正的失锁恢复反而被淹没在里面。
            RCLCPP_INFO(
              get_logger(), "开机自举：第 %d/%d 帧粗→精（粗配准位移 %.3f m）", ndt_frames_ + 1,
              bootstrap_coarse_frames_,
              (coarse.pose.translation() - filter_prediction.translation()).norm());
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 3000,
              "NDT 已连续 %d 帧被拒，用粗网格重定初值（累计 %ld 次）", consecutive_ndt_failures_,
              recovery_attempts_);
          }
        }
      }
      result = ads_localization::AlignNdt(*ndt_map_, scan, guess, ndt_params_);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "NDT 失败：%s", e.what());
      ndt_ok_ = false;
      ++consecutive_ndt_failures_;
      return;
    }
    // 耗时用**墙钟**量，不是仿真钟 —— 这是「算法跑得够不够快」，
    // 不是算法时序（SPEC §5 禁的是后者）。
    last_ndt_ms_ =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    last_ndt_result_ = result;
    ++ndt_frames_;

    // ⚠️⚠️ **退化或没收敛的帧一律不喂给滤波器。**
    //
    //    退化：代价函数沿某些方向是平的，那几个方向上的位姿是任意的。
    //
    // ⚠️ **但不要顺手加上 `|| !result.converged`** —— 试过，结果更差
    //    （2026-08-10 实测，横向误差 0.0664 → 1.99 m，899 拍掉进 GNSS_ONLY）。
    //    车动起来时很多帧到迭代上限才停，位姿其实已经很好；
    //    `converged` 那个布尔量只说「步长小于 epsilon 了吗」，
    //    不说「位姿好不好」，拿它当闸门等于把大量好结果扔掉。
    //
    // ⚠️ 真正的隐患在另一处，而且更隐蔽：NDT 到了迭代上限仍会返回位姿，
    //    而它的协方差来自**最终迭代点处的信息阵** —— 那个量反映的是
    //    「代价函数在这里有多陡」，**不是「离真解有多远」**。
    //    半收敛的位姿带着毫米级协方差被喂进滤波器，把状态硬拽过去，
    //    下一帧初值更差…… 正反馈发散。
    //    实测把 max_iterations 从 30 砍到 15：横向误差 0.0664 m → **462 m**，
    //    车全程一拍 TRACKING 都没有。**所以迭代上限不能随便砍。**
    //
    //    正确的防线是**新息门限（卡方检验）**：位姿观测偏离滤波器预测
    //    超过若干 σ 就丢弃。那一条尚未实现，见 docs/modules/localization.md
    //    §11 的边界表。在它之前，max_iterations 必须留够（30）。
    if (result.degenerate) {
      ndt_ok_ = false;
      ++consecutive_ndt_failures_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "NDT 退化（法向散布 %.3g，inlier %.1f%%），本帧不用于更新；连续 %d 帧",
        result.normal_diversity, 100.0 * result.inlier_ratio, consecutive_ndt_failures_);
      return;
    }
    // ---- 新息门限（S5）---------------------------------------------------
    const ads_localization::PoseDelta innovation =
      ads_localization::ComparePoses(filter_prediction, result.pose);
    last_innovation_m_ = innovation.translation_m;
    last_innovation_rad_ = innovation.rotation_rad;
    // 记住峰值。**只诊断不判决** —— 但没有它就没法回答「门限还剩多少余量」。
    // 这条是 gen_map 数值精度那次的教训：只断言通过的话，余量从两位数倍
    // 缩到 1.5 倍也没人知道，直到某天突然变红而没人记得判据是怎么来的。
    // 恢复中的帧不计入：那一跳按定义就是大的，混进来峰值就没有意义了。
    if (!used_coarse && std::isfinite(innovation.translation_m)) {
      max_innovation_seen_m_ = std::max(max_innovation_seen_m_, innovation.translation_m);
    }

    // ⚠️ **恢复中的帧豁免门限，这是逃生口，不是漏洞。**
    //    粗→精只在开机头几帧或连续失败之后才跑，那时的全部目的就是
    //    「从一个已知很差的位姿跳回正确的地方」—— 那一跳按定义就是大的。
    //    没有这个豁免，一旦滤波器漂了，正确的 NDT 结果反而会被门限一直拒，
    //    **永久锁死在错误状态里**。这是固定门限最典型的自伤方式。
    if (!used_coarse) {
      // ⚠️ 必须先判有限性再比较。`NaN > limit` 恒为 false，也就是说
      //    只写下面那两个比较的话，一个 NaN 位姿会被**原样放行**。
      //    本仓库已经在 vehicle_cmd_bridge 和 ads_control 上各咬过一次。
      const bool finite =
        std::isfinite(innovation.translation_m) && std::isfinite(innovation.rotation_rad);
      if (
        !finite || innovation.translation_m > max_innovation_m_ ||
        innovation.rotation_rad > max_innovation_rad_) {
        ndt_ok_ = false;
        ++consecutive_ndt_failures_;
        ++rejected_innovation_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "NDT 新息过大（%.3f m / %.2f°，上限 %.2f m / %.2f°），本帧丢弃。"
          "NDT 到迭代上限也会返回位姿，而它的协方差说的是「这里有多陡」不是"
          "「离真解有多远」—— 半收敛的位姿带着毫米级协方差会把滤波器拽跑。"
          "累计丢 %ld 帧，连续 %d 帧",
          innovation.translation_m, innovation.rotation_rad * 180.0 / M_PI, max_innovation_m_,
          max_innovation_rad_ * 180.0 / M_PI, rejected_innovation_, consecutive_ndt_failures_ + 1);
        // 连续被拒会累加到 consecutive_ndt_failures_，达到 recovery_after_failures_
        // 之后自动触发粗→精重定位 —— 逃生口是现成的，这里什么都不用做。
        return;
      }
    }

    ndt_ok_ = true;
    last_ndt_success_time_ = now();
    consecutive_ndt_failures_ = 0;
    PublishNdtPose(result, msg->header.stamp);
    try {
      eskf_->UpdatePose(
        result.pose.translation(), Eigen::Quaterniond(result.pose.linear()), result.covariance);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "位姿更新失败：%s", e.what());
    }
  }

  /// 把 NDT 的**原始**输出发出去（标定用，见发布者声明处的警告）。
  void PublishNdtPose(
    const ads_localization::NdtAlignResult & result, const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    // ⚠️ 用**点云自己的时间戳**，不是 now()。标定要把它与真值按时间对齐，
    //    而 now() 比点云晚一个 NDT 的耗时（实测 40 ms，车 4 m/s 就是 16 cm）——
    //    那会被算成 NDT 的误差，把标定结果系统性地推大。
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = result.pose.translation().x();
    msg.pose.pose.position.y = result.pose.translation().y();
    msg.pose.pose.position.z = result.pose.translation().z();
    const Eigen::Quaterniond q(result.pose.linear());
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    // NDT 的 6×6 是 (δt, δθ) 排布，与 ROS 的 (x,y,z,rx,ry,rz) 顺序一致，直接搬。
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        msg.pose.covariance[i * 6 + j] = result.covariance(i, j);
      }
    }
    ndt_pose_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------
  //  输出
  // ---------------------------------------------------------------------
  void PublishOutputs()
  {
    if (!eskf_ || !have_odom_) {
      return;
    }
    UpdateState();

    const auto & nominal = eskf_->state();
    const rclcpp::Time stamp = now();

    // T(map→odom) = T(map→base_link)_估计 ∘ T(odom→base_link)⁻¹
    Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
    map_to_base.linear() = nominal.orientation.toRotationMatrix();
    map_to_base.translation() = nominal.position_m;

    Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
    odom_to_base.linear() =
      Eigen::Quaterniond(
        last_odom_.pose.pose.orientation.w, last_odom_.pose.pose.orientation.x,
        last_odom_.pose.pose.orientation.y, last_odom_.pose.pose.orientation.z)
        .normalized()
        .toRotationMatrix();
    odom_to_base.translation() = Eigen::Vector3d(
      last_odom_.pose.pose.position.x, last_odom_.pose.pose.position.y,
      last_odom_.pose.pose.position.z);

    const Eigen::Isometry3d map_to_odom = map_to_base * odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = map_to_odom.translation().x();
    tf.transform.translation.y = map_to_odom.translation().y();
    tf.transform.translation.z = map_to_odom.translation().z();
    const Eigen::Quaterniond q(map_to_odom.linear());
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf_broadcaster_->sendTransform(tf);

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = nominal.position_m.x();
    pose.pose.pose.position.y = nominal.position_m.y();
    pose.pose.pose.position.z = nominal.position_m.z();
    pose.pose.pose.orientation.w = nominal.orientation.w();
    pose.pose.pose.orientation.x = nominal.orientation.x();
    pose.pose.pose.orientation.y = nominal.orientation.y();
    pose.pose.pose.orientation.z = nominal.orientation.z();
    // ROS 的 6×6 协方差顺序是 (x,y,z,roll,pitch,yaw)，与 ESKF 的
    // (δp, δθ) 分块一致 —— 但两块在 15 维状态里不相邻，要分别搬。
    const auto & cov = eskf_->covariance();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        pose.pose.covariance[i * 6 + j] =
          cov(ads_localization::Eskf::kIdxPosition + i, ads_localization::Eskf::kIdxPosition + j);
        pose.pose.covariance[(i + 3) * 6 + (j + 3)] =
          cov(ads_localization::Eskf::kIdxAttitude + i, ads_localization::Eskf::kIdxAttitude + j);
      }
    }
    pose_pub_->publish(pose);

    PublishDiagnostics(stamp);
  }

  void UpdateState()
  {
    const double gnss_age_s =
      last_gnss_time_.nanoseconds() == 0 ? 1e9 : (now() - last_gnss_time_).seconds();
    // ⚠️ ndt_ok_ 必须带超时（2026-08-12 复检修复）：它原来是**无超时的锁存量**，
    //    只在下一帧 NDT 跑过之后才可能翻 false —— 而雷达断流/点云持续被
    //    stale-drop 时**根本没有下一帧**，状态机于是永远停在 NDT_AIDED，
    //    诊断谎报正常，实际早已在纯航位推算上漂。
    //    超时取 1.0 s = 雷达标称周期（0.1 s）的 10 倍，与规划器的障碍物
    //    过期阈值同一个取法。
    const double ndt_age_s =
      last_ndt_success_time_.nanoseconds() == 0 ? 1e9 : (now() - last_ndt_success_time_).seconds();
    if (ndt_ok_ && ndt_age_s > 1.0) {
      ndt_ok_ = false;
      RCLCPP_WARN(
        get_logger(), "NDT 已 %.1f s 没有成功帧（雷达断流？），降出 NDT_AIDED", ndt_age_s);
    }
    if (ndt_ok_ && ndt_map_) {
      state_ = LocalizationState::kNdtAided;
    } else if (gnss_age_s < gnss_timeout_s_) {
      state_ = LocalizationState::kGnssOnly;
    } else {
      state_ = LocalizationState::kDeadReckoning;
    }
  }

  void PublishDiagnostics(const rclcpp::Time & stamp)
  {
    // 1 Hz 就够 —— 这是给人看的，不是控制回路的一部分。
    if (stamp.nanoseconds() - last_diag_ns_ < 1'000'000'000LL) {
      return;
    }
    last_diag_ns_ = stamp.nanoseconds();

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "localization";
    status.hardware_id = "ads_localization";
    status.message = StateName(state_);
    status.level = state_ == LocalizationState::kNdtAided
                     ? diagnostic_msgs::msg::DiagnosticStatus::OK
                     : (state_ == LocalizationState::kDeadReckoning
                          ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                          : diagnostic_msgs::msg::DiagnosticStatus::WARN);

    const auto add = [&status](const std::string & key, double value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = std::to_string(value);
      status.values.push_back(kv);
    };
    const auto & cov = eskf_->covariance();
    add(
      "position_sigma_m",
      std::sqrt(
        cov(ads_localization::Eskf::kIdxPosition, ads_localization::Eskf::kIdxPosition) +
        cov(ads_localization::Eskf::kIdxPosition + 1, ads_localization::Eskf::kIdxPosition + 1)));
    add(
      "yaw_sigma_rad",
      std::sqrt(
        cov(ads_localization::Eskf::kIdxAttitude + 2, ads_localization::Eskf::kIdxAttitude + 2)));
    add("gyro_bias_z_rad_s", eskf_->state().gyro_bias_rad_s.z());
    add("ndt_time_ms", last_ndt_ms_);
    add("ndt_normal_diversity", last_ndt_result_.normal_diversity);
    add("ndt_inlier_ratio", last_ndt_result_.inlier_ratio);
    add("ndt_iterations", last_ndt_result_.iterations);
    add("scan_points", last_scan_points_);
    add("ground_fraction", last_ground_fraction_);
    add("dropped_stale_clouds", static_cast<double>(dropped_stale_clouds_));
    add("ndt_recovery_attempts", static_cast<double>(recovery_attempts_));
    add("consecutive_ndt_failures", consecutive_ndt_failures_);
    // 新息门限的三个量。**正常运行时 rejected_innovation 必须是 0** ——
    // 不是 0 说明要么真锁错了、要么阈值定得太紧在误杀好帧，两者都要查。
    add("ndt_innovation_m", last_innovation_m_);
    add("ndt_innovation_deg", last_innovation_rad_ * 180.0 / M_PI);
    add("ndt_rejected_innovation", static_cast<double>(rejected_innovation_));
    add("ndt_innovation_max_m", max_innovation_seen_m_);
    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  // ---- 状态 -------------------------------------------------------------
  ads_localization::GeoOrigin origin_;
  ads_localization::EskfParams eskf_params_;
  ads_localization::NdtAlignParams ndt_params_;
  std::unique_ptr<ads_localization::Eskf> eskf_;
  std::unique_ptr<ads_localization::NdtGrid> ndt_map_;
  /// 粗网格：只在失锁恢复时用，见构造函数里的说明。
  std::unique_ptr<ads_localization::NdtGrid> ndt_coarse_map_;
  int consecutive_ndt_failures_{0};
  int recovery_after_failures_{3};
  double max_innovation_m_{3.0};
  double max_innovation_rad_{0.175};
  double last_innovation_m_{0.0};
  double last_innovation_rad_{0.0};
  double max_innovation_seen_m_{0.0};
  int64_t rejected_innovation_{0};
  int bootstrap_coarse_frames_{5};
  int64_t ndt_frames_{0};
  int64_t recovery_attempts_{0};

  double gnss_horizontal_std_m_{2.0};
  double gnss_vertical_std_m_{4.0};
  double wheel_speed_std_mps_{0.05};
  double initial_yaw_rad_{0.0};
  /// GNSS 天线相对 base_link 的安装位置（body 系）。见构造函数里的说明。
  Eigen::Vector3d lever_arm_body_{Eigen::Vector3d::Zero()};
  double gnss_timeout_s_{2.0};
  int scan_stride_{4};
  int ground_stride_{24};
  double ground_height_m_{0.30};
  double max_cloud_age_s_{0.15};
  int64_t dropped_stale_clouds_{0};
  int last_scan_points_{0};
  double last_ground_fraction_{0.0};

  Eigen::Vector3d last_gnss_local_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gnss_sum_{Eigen::Vector3d::Zero()};
  int gnss_sample_count_{0};
  int init_gnss_samples_{30};
  bool have_gnss_{false};
  bool have_odom_{false};
  bool ndt_ok_{false};
  rclcpp::Time last_ndt_success_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gnss_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry last_odom_;
  ads_localization::NdtAlignResult last_ndt_result_;
  double last_ndt_ms_{0.0};
  int64_t last_diag_ns_{0};
  LocalizationState state_{LocalizationState::kInitializing};

  std::string map_frame_;
  std::string odom_frame_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  /// NDT 原始输出，**只给标定脚本**。见发布者声明处的警告。
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LocalizationNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_node"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
