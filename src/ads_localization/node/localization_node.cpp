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

    // ---- 冷启动的航向先验 ------------------------------------------------
    // ⚠️ **在途初始对准未实现。** 真车靠双天线 GNSS 或运动对准拿这个量。
    //    这里给一个粗略先验，位置初值仍来自带 2 m 噪声的 GNSS，
    //    所以滤波器还是要靠自己走完全程。
    initial_yaw_rad_ = declare_parameter<double>("initial_yaw_rad", 0.0);

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
    ndt_params_.covariance_scale = declare_parameter<double>("ndt.covariance_scale", 1.0);
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
    scan_stride_ = std::max<int>(1, static_cast<int>(declare_parameter<int>("ndt.scan_stride", 4)));

    if (map_path.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "未提供 map_pcd_path，NDT 关闭 —— 定位会退化成 GNSS + IMU，"
        "精度到不了 SPEC §1 的 0.3 m（那正是要做 NDT 的理由）");
    } else {
      try {
        ndt_map_ = std::make_unique<ads_localization::NdtGrid>(
          ads_localization::LoadPcdAscii(map_path), grid_params);
        RCLCPP_INFO(
          get_logger(), "点云地图已载入：%zu 个非空体素（稀疏丢弃 %zu，退化修正 %zu）",
          ndt_map_->size(), ndt_map_->discarded_sparse_voxels(), ndt_map_->regularized_voxels());
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
    if (eskf_ || !have_gnss_) {
      return;
    }
    ads_localization::NominalState init;
    init.position_m = last_gnss_local_;
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
    have_gnss_ = true;
    last_gnss_time_ = now();

    if (!eskf_) {
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
    // 扫描点在 base_link 系（lidar_preprocessor 已经做过变换）。
    std::vector<Eigen::Vector3d> scan;
    scan.reserve(msg->width * msg->height / static_cast<size_t>(scan_stride_) + 1);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    int index = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++index) {
      if (index % scan_stride_ != 0) {
        continue;
      }
      // ⚠️ gpu_lidar 的无回波射线返回 **±inf 不是 NaN**（CLAUDE.md 有专门一条），
      //    两者都要滤。用比较拦不住。
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) {
        continue;
      }
      scan.emplace_back(*it_x, *it_y, *it_z);
    }
    if (scan.size() < 100) {
      return;
    }

    // NDT 的初值用 ESKF 的当前估计 —— 这是闭环：滤波器给配准初值，
    // 配准给滤波器观测。初值差超过收敛域就发散，所以不能用固定初值。
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.linear() = eskf_->state().orientation.toRotationMatrix();
    guess.translation() = eskf_->state().position_m;

    const auto started = std::chrono::steady_clock::now();
    ads_localization::NdtAlignResult result;
    try {
      result = ads_localization::AlignNdt(*ndt_map_, scan, guess, ndt_params_);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "NDT 失败：%s", e.what());
      ndt_ok_ = false;
      return;
    }
    // 耗时用**墙钟**量，不是仿真钟 —— 这是「算法跑得够不够快」，
    // 不是算法时序（SPEC §5 禁的是后者）。
    last_ndt_ms_ =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    last_ndt_result_ = result;

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
    //    §10 的边界表。在它之前，max_iterations 必须留够（30）。
    if (result.degenerate) {
      ndt_ok_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "NDT 退化（法向散布 %.3g，inlier %.1f%%），本帧不用于更新", result.normal_diversity,
        100.0 * result.inlier_ratio);
      return;
    }
    ndt_ok_ = true;
    try {
      eskf_->UpdatePose(
        result.pose.translation(), Eigen::Quaterniond(result.pose.linear()), result.covariance);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "位姿更新失败：%s", e.what());
    }
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
    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  // ---- 状态 -------------------------------------------------------------
  ads_localization::GeoOrigin origin_;
  ads_localization::EskfParams eskf_params_;
  ads_localization::NdtAlignParams ndt_params_;
  std::unique_ptr<ads_localization::Eskf> eskf_;
  std::unique_ptr<ads_localization::NdtGrid> ndt_map_;

  double gnss_horizontal_std_m_{2.0};
  double gnss_vertical_std_m_{4.0};
  double wheel_speed_std_mps_{0.05};
  double initial_yaw_rad_{0.0};
  double gnss_timeout_s_{2.0};
  int scan_stride_{4};

  Eigen::Vector3d last_gnss_local_{Eigen::Vector3d::Zero()};
  bool have_gnss_{false};
  bool have_odom_{false};
  bool ndt_ok_{false};
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
