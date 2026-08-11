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
//  fake_sensors —— 不需要 Gazebo 的合成传感器，**测试夹具，不是仿真数据源**
//
//  发布  /clock          rosgraph_msgs/Clock       ← **它是本闭环的时间源**
//  发布  /imu            sensor_msgs/Imu           base_link 系，100 Hz
//  发布  /gnss           sensor_msgs/NavSatFix     天线位置（含杆臂），10 Hz
//  发布  /odom           nav_msgs/Odometry         odom 系，50 Hz
//  发布  TF              odom → base_link          50 Hz
//  发布  /lidar/points   sensor_msgs/PointCloud2   base_link 系，10 Hz
//  发布  /ego_pose_gt    nav_msgs/Odometry         真值，map → base_link
//
//  存在的理由：`fake_sensors + localization_node` 构成一个**完全不需要 GPU**
//  的定位闭环，于是可以进 CI（SPEC §8 的 L3-G）。与 ads_control 的 fake_vehicle
//  是同一套思路的定位版：
//
//      L1 单元测试（S2/S3）  验**算法**：毫秒级，不起 ROS
//      本闭环（S5）          验**接线**：不需要 GPU，半分钟，进 CI
//      CP-P4-B（S4）         验**真物理**：要 Gazebo + GPU + 人
//
//  ⚠️ **这一层全绿不代表定位准。** 三条结构性的原因，缺一不可地说明它为什么
//     替代不了 CP-P4-B：
//
//     ① **扫描是从先验地图里抠出来的**，也就是「世界」与「地图」完全同源。
//        真雷达打到的是草、树、临时车辆，而地图里没有；地图里的杆件位置
//        也不会与实物精确重合。这里没有任何模型失配，NDT 的日子过得太好。
//     ② **没有遮挡**：只按距离筛点，杆件背后的点照样进扫描。真雷达看不见。
//     ③ **没有运动畸变**：一帧点全部取自同一时刻的真值位姿。真雷达转一圈
//        要 100 ms，车 5 m/s 时首尾差半米 —— 那一条至今没做补偿（§11 边界表）。
//
//     所以这里的判据只到「链路通、状态机进了 NDT_AIDED、误差没有发散」，
//     **不到 CP-P4-B 的 0.30 m**。想当然地把判据收紧到那个量级，得到的
//     不是更强的保证，而是一个在 CI 上随机变红的测试。
//
//  ⚠️ **它不是 SIM_SOURCES 的一员，不要往 stack.launch.py 里加。**
//     与 fake_vehicle 同理：拿它当仿真器用，等于把 CP-P4-B 发现的那五个根因
//     （见 docs/modules/localization.md §10.2）全部隐藏掉。
//
//  ⚠️ **本节点自己必须 use_sim_time=false** —— 它是时钟的来源。
//     开了就成了鸡生蛋：等一个自己还没发布的时钟。
// =============================================================================

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "ads_localization/geodetic.hpp"
#include "ads_localization/point_cloud_io.hpp"

namespace ads_localization
{

/// @brief 合成传感器 + 解析真值轨迹，并**充当 /clock 的来源**。
class FakeSensors : public rclcpp::Node
{
public:
  // parameter_overrides 的优先级高于 launch 的 parameters=，所以外面写错也拦得住。
  FakeSensors()
  : Node(
      "fake_sensors",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", false)}))
  {
    // ---- 大地原点。与 localization_node 用**同一份** campus_map.yaml，由 launch 搬运 ----
    // 两处各填一遍的症状是定位稳定地偏一个常量，而没有任何模块报错。
    origin_.latitude_deg = declare_parameter<double>("geo_origin.latitude_deg", 0.0);
    origin_.longitude_deg = declare_parameter<double>("geo_origin.longitude_deg", 0.0);
    origin_.elevation_m = declare_parameter<double>("geo_origin.elevation_m", 0.0);
    origin_.Validate();

    // ---- 真值轨迹 ----
    // 起点取自车 spawn 位姿（launch 从世界文件读），沿园区南侧那条路开。
    start_x_m_ = declare_parameter<double>("truth.start_x_m", 30.0);
    start_y_m_ = declare_parameter<double>("truth.start_y_m", -51.75);
    start_yaw_rad_ = declare_parameter<double>("truth.start_yaw_rad", 0.0);
    cruise_speed_mps_ = declare_parameter<double>("truth.cruise_speed_mps", 4.0);
    accel_mps2_ = declare_parameter<double>("truth.accel_mps2", 1.5);

    // ⚠️ **开头必须先停一会儿**，不能一上来就走。
    //    localization_node 用**前 30 帧 GNSS 的平均**定位置初值（10 Hz → 3 s），
    //    车若在这 3 s 里以 4 m/s 前进，那个平均值对应的是 6 m 之前的位置 ——
    //    初值直接偏 6 m，而 GNSS 单帧 σ 才 2 m。症状是「初始化完就偏一大截，
    //    之后慢慢收敛」，看起来像滤波器调得不好，其实是**初始化时车在动**。
    //    真栈里车确实是静止的（control_node 在等目标点），这里如实复现。
    start_delay_s_ = declare_parameter<double>("truth.start_delay_s", 4.0);

    // 航向做一个**回到零**的正弦摆动，而不是直线。
    //
    // 为什么不能是纯直线：直线匀速下陀螺恒为 0、世界系加速度恒为 0，
    // 于是「陀螺三个轴接错了」「比力符号反了」这类接线错误**结构上测不出来**
    // —— 而那正是这一层唯一该负责的东西。
    //
    // 为什么不能是大幅机动：幅度一大车就开出杆件密集的走廊，扫描点数骤减，
    // 判据随之变脆。取 ψ̇ = A·sin(2πt/T)，A=0.05 rad/s、T=4 s，
    // 峰值航向 2A·T/(2π) ≈ 0.064 rad = 3.6°，横向偏移不到 0.5 m。
    yaw_rate_amplitude_rad_s_ = declare_parameter<double>("truth.yaw_rate_amplitude_rad_s", 0.05);
    yaw_rate_period_s_ = declare_parameter<double>("truth.yaw_rate_period_s", 4.0);

    // ---- 步长与倍率 ----
    // 步长 0.01 s = IMU 的 100 Hz。其余传感器按整数分频，见 step()。
    sim_step_s_ = declare_parameter<double>("sim_step_s", 0.01);
    // ⚠️ 与 fake_vehicle 不同，这里默认 **1.0 倍速**。
    //    定位闭环每 100 ms 要做一次 NDT（真仿真里实测 37–44 ms），
    //    倍速一开，一帧点云的墙钟预算跟着缩水，NDT 追不上就会触发
    //    localization_node 的「陈旧点云直接丢」——症状是状态机永远进不了
    //    NDT_AIDED，而日志里只有一条 3 s 一次的节流告警。
    //    也就是说：**调快它，测试会以一种看不出原因的方式变红。**
    real_time_factor_ = declare_parameter<double>("real_time_factor", 1.0);

    // ---- 传感器噪声。与 vehicle_params.yaml 同源，由 launch 搬运 ----
    gyro_noise_rad_s_ = declare_parameter<double>("noise.gyro_stddev_rad_s", 8.7e-4);
    accel_noise_mps2_ = declare_parameter<double>("noise.accel_stddev_mps2", 1.0e-2);
    gnss_horizontal_noise_m_ = declare_parameter<double>("noise.gnss_horizontal_stddev_m", 2.0);
    gnss_vertical_noise_m_ = declare_parameter<double>("noise.gnss_vertical_stddev_m", 4.0);
    lidar_noise_m_ = declare_parameter<double>("noise.lidar_stddev_m", 0.010);
    wheel_noise_mps_ = declare_parameter<double>("noise.wheel_stddev_mps", 0.02);

    // 常值零偏。**必须非零**：全零的话滤波器的零偏状态无事可做，
    // 而「零偏那几行根本没接上」这种错误照样全绿。
    gyro_bias_rad_s_ = Eigen::Vector3d(
      declare_parameter<double>("bias.gyro_x_rad_s", 4.0e-4),
      declare_parameter<double>("bias.gyro_y_rad_s", -3.0e-4),
      declare_parameter<double>("bias.gyro_z_rad_s", 6.0e-4));
    accel_bias_mps2_ = Eigen::Vector3d(
      declare_parameter<double>("bias.accel_x_mps2", 0.02),
      declare_parameter<double>("bias.accel_y_mps2", -0.015),
      declare_parameter<double>("bias.accel_z_mps2", 0.01));

    // 固定种子 —— CI 里的测试必须可复现。换种子等于换一个测试，
    // 「换个种子就红」说明判据太紧，那是判据的问题不是种子的问题。
    rng_.seed(static_cast<uint32_t>(declare_parameter<int>("random_seed", 20260810)));

    // ---- 杆臂。GNSS 报的是**天线**位置，不是 base_link ----
    lever_arm_body_ = Eigen::Vector3d(
      declare_parameter<double>("gnss.lever_arm_x_m", 0.0),
      declare_parameter<double>("gnss.lever_arm_y_m", 0.0),
      declare_parameter<double>("gnss.lever_arm_z_m", 0.0));

    // ---- 雷达 ----
    lidar_range_m_ = declare_parameter<double>("lidar.range_max_m", 30.0);
    lidar_mount_z_m_ = declare_parameter<double>("lidar.mount_z_m", 1.6);
    lidar_fov_min_rad_ = declare_parameter<double>("lidar.vertical_fov_min_rad", -0.4363);
    lidar_fov_max_rad_ = declare_parameter<double>("lidar.vertical_fov_max_rad", 0.1745);
    lidar_stride_ = std::max<int>(1, declare_parameter<int>("lidar.stride", 1));

    // ---- 先验点云。**扫描就是从它里面抠的** ----
    // 这一点在文件头写清楚了：世界与地图同源，没有任何模型失配。
    const std::string map_path = declare_parameter<std::string>("map_pcd_path", "");
    if (map_path.empty()) {
      throw std::invalid_argument("fake_sensors: 必须给 map_pcd_path");
    }
    map_points_ = LoadPcdAscii(map_path);
    if (map_points_.size() < 1000) {
      throw std::invalid_argument("fake_sensors: 先验点云太少，扫描不可能有结构");
    }

    if (sim_step_s_ <= 0.0 || real_time_factor_ <= 0.0 || cruise_speed_mps_ <= 0.0) {
      throw std::invalid_argument("fake_sensors: sim_step_s / real_time_factor / speed 必须为正");
    }

    x_m_ = start_x_m_;
    y_m_ = start_y_m_;
    yaw_rad_ = start_yaw_rad_;

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::QoS(50));
    gnss_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gnss", rclcpp::QoS(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
    truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ego_pose_gt", rclcpp::QoS(10));
    // ⚠️ 点云用 **reliable + 深度 10**，与本仓库仿真链路一致。
    //    best-effort 会静默丢帧（实测只剩标称的 35%），而症状是
    //    「NDT 偶尔不更新」—— 没有任何日志。见 CLAUDE.md 陷阱表。
    cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points", rclcpp::QoS(10).reliable());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ⚠️ 必须是 **wall** timer：本节点是时钟的来源，用节点时钟就是等自己。
    const auto wall_period = std::chrono::duration<double>(sim_step_s_ / real_time_factor_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(wall_period), [this]() { step(); });

    RCLCPP_INFO(
      get_logger(), "假传感器就绪：起点 (%.2f, %.2f, %.3f rad)，先验点云 %zu 点，%.1f 倍速", x_m_,
      y_m_, yaw_rad_, map_points_.size(), real_time_factor_);
  }

private:
  // ---------------------------------------------------------------------
  //  一拍
  // ---------------------------------------------------------------------
  void step()
  {
    // 先推进时钟再动车 —— 与 fake_vehicle 同一个理由：下游这一拍读到的
    // now() 才与随后发布的消息时间戳一致，否则 dt 恒偏差一个步长。
    sim_time_s_ += sim_step_s_;
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = ToRosTime(sim_time_s_);
    clock_pub_->publish(clock_msg);

    // ---- 真值轨迹：停一会儿 → 匀加速 → 巡航，全程叠一个正弦横摆 ----
    // 纵向加速度：起步阶段是常值 accel，到巡航速度后归零。
    // 它必须与下面 IMU 报的比力**用同一个变量**算 —— 两处各写一份的话，
    // 滤波器看到的加速度与真值轨迹对不上，表现为速度估计稳定地偏一点，
    // 而位置误差随之线性增长。没有任何一层会报错。
    const bool rolling = sim_time_s_ >= start_delay_s_;
    const double accel_cmd_mps2 = (rolling && speed_mps_ < cruise_speed_mps_) ? accel_mps2_ : 0.0;
    // 横摆只在车动起来之后加：静止时绕 z 转是非完整约束不允许的动作，
    // 而 NDT 在原地打转的扫描上照样能配准 —— 那会掩盖真实场景里没有的信息。
    const double yaw_rate_rad_s =
      rolling ? yaw_rate_amplitude_rad_s_ * std::sin(2.0 * M_PI * sim_time_s_ / yaw_rate_period_s_)
              : 0.0;

    // 导数取**更新前**的状态，这才是显式欧拉；与 fake_vehicle 的写法一致。
    const double speed_before_mps = speed_mps_;
    const double yaw_before_rad = yaw_rad_;
    x_m_ += speed_before_mps * std::cos(yaw_before_rad) * sim_step_s_;
    y_m_ += speed_before_mps * std::sin(yaw_before_rad) * sim_step_s_;
    yaw_rad_ = yaw_before_rad + yaw_rate_rad_s * sim_step_s_;
    speed_mps_ = std::min(speed_before_mps + accel_cmd_mps2 * sim_step_s_, cruise_speed_mps_);

    ++tick_;
    PublishImu(accel_cmd_mps2, speed_before_mps, yaw_rate_rad_s);
    // 分频。整数分频而不是各自开定时器：定时器之间的相位是随机的，
    // 而这里希望每一帧点云的时间戳都恰好落在某一拍 IMU 上 —— 时间对齐
    // 出问题时，「是不是分频没对齐」这个可能性一开始就不该存在。
    if (tick_ % 2 == 0) {
      PublishOdomAndTf();
    }
    if (tick_ % 10 == 0) {
      PublishGnss();
      PublishCloud();
      PublishTruth();
    }
  }

  // ---------------------------------------------------------------------
  //  IMU：比力 + 角速度，body 系
  // ---------------------------------------------------------------------
  void PublishImu(double longitudinal_mps2, double speed_mps, double yaw_rate_rad_s)
  {
    // 世界系加速度 = 切向 + 向心：
    //     a_world = a·(cosψ, sinψ, 0) + v·ψ̇·(−sinψ, cosψ, 0)
    // 转到 body 系（body x 沿 (cosψ,sinψ)、body y 沿 (−sinψ,cosψ)）后，
    // 两项正交地落在两个轴上：纵向 = a，横向 = v·ψ̇。**这是精确的，不是近似。**
    //
    // ⚠️ 比力 = Rᵀ(a_world − g)，其中 g = (0,0,−9.80665)。
    //    静止时它读到 +g（沿 body 的 z 向上），因为加速度计测的是**支撑力**。
    //    符号反了的症状是滤波器认为车在以 2g 下坠 —— 与 eskf.hpp 里
    //    ImuSample::accel_mps2 的注释是同一件事，两边必须一致。
    //    车身水平（roll=pitch=0），所以 Rᵀg 只落在 body z 上。
    const Eigen::Vector3d specific_force_body(
      longitudinal_mps2, speed_mps * yaw_rate_rad_s, kGravityMps2);
    const Eigen::Vector3d gyro_body(0.0, 0.0, yaw_rate_rad_s);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = ToRosTime(sim_time_s_);
    msg.header.frame_id = "base_link";
    const Eigen::Vector3d accel = specific_force_body + accel_bias_mps2_ + Noise(accel_noise_mps2_);
    const Eigen::Vector3d gyro = gyro_body + gyro_bias_rad_s_ + Noise(gyro_noise_rad_s_);
    msg.linear_acceleration.x = accel.x();
    msg.linear_acceleration.y = accel.y();
    msg.linear_acceleration.z = accel.z();
    msg.angular_velocity.x = gyro.x();
    msg.angular_velocity.y = gyro.y();
    msg.angular_velocity.z = gyro.z();
    imu_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------
  //  odom + TF(odom → base_link)
  // ---------------------------------------------------------------------
  void PublishOdomAndTf()
  {
    // ⚠️ odom 原点**刻意不等于 map 原点** —— 它放在自车起点，与 Gazebo 的
    //    AckermannSteering 一致（P1-S4 实测）。
    //
    //    如果让它俩重合，map→odom 恒为单位阵，于是
    //    「localization_node 把 map→base_link 错发成了 map→odom」这类错误
    //    **在数值上完全看不出来**。留一个非平凡的偏移，写错就立刻暴露。
    //
    //    这里 odom 不漂：本层验接线，「修正里程计漂移」的能力由 CP-P4-B
    //    在真仿真里验（那里的 /odom 是轮速推算，本来就漂）。
    const Eigen::Isometry3d map_to_base = TruthPose();
    const Eigen::Isometry3d odom_to_base = MapToOdomTruth().inverse() * map_to_base;

    const rclcpp::Time stamp = ToRosTime(sim_time_s_);
    const Eigen::Quaterniond q(odom_to_base.linear());

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = odom_to_base.translation().x();
    tf.transform.translation.y = odom_to_base.translation().y();
    tf.transform.translation.z = odom_to_base.translation().z();
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf_broadcaster_->sendTransform(tf);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = odom_to_base.translation().x();
    odom.pose.pose.position.y = odom_to_base.translation().y();
    odom.pose.pose.position.z = odom_to_base.translation().z();
    odom.pose.pose.orientation = tf.transform.rotation;
    // 轮速观测走 twist.linear.x（车身系纵向速度），与 gazebo_bridge 一致。
    odom.twist.twist.linear.x = speed_mps_ + Noise(wheel_noise_mps_).x();
    odom_pub_->publish(odom);
  }

  // ---------------------------------------------------------------------
  //  GNSS：天线位置（含杆臂）→ 经纬高
  // ---------------------------------------------------------------------
  void PublishGnss()
  {
    const Eigen::Isometry3d map_to_base = TruthPose();
    // 天线在 base_link 前上方，所以要**加**杆臂（节点侧再减回去）。
    // 这里加、那里减，两边用的是同一份 vehicle_params 参数 ——
    // 谁写反了都会表现为一个稳定的常量偏差，而不是发散。
    const Eigen::Vector3d antenna_map =
      map_to_base.translation() + map_to_base.linear() * lever_arm_body_;
    const Eigen::Vector3d noisy(
      antenna_map.x() + Noise(gnss_horizontal_noise_m_).x(),
      antenna_map.y() + Noise(gnss_horizontal_noise_m_).x(),
      antenna_map.z() + Noise(gnss_vertical_noise_m_).x());
    const Eigen::Vector3d llh = LocalToGeodetic(noisy, origin_);

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = ToRosTime(sim_time_s_);
    msg.header.frame_id = "base_link";
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.latitude = llh.x();
    msg.longitude = llh.y();
    msg.altitude = llh.z();
    gnss_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------
  //  雷达：从先验地图里按距离与俯仰角抠一帧，转到 base_link 并加测距噪声
  // ---------------------------------------------------------------------
  void PublishCloud()
  {
    const Eigen::Isometry3d map_to_base = TruthPose();
    const Eigen::Isometry3d base_to_map = map_to_base.inverse();
    const Eigen::Vector3d sensor_map =
      map_to_base.translation() +
      map_to_base.linear() * Eigen::Vector3d(0.0, 0.0, lidar_mount_z_m_);

    scratch_.clear();
    int index = 0;
    for (const auto & point_map : map_points_) {
      if (++index % lidar_stride_ != 0) {
        continue;
      }
      const Eigen::Vector3d from_sensor = point_map - sensor_map;
      const double range_m = from_sensor.norm();
      if (range_m > lidar_range_m_ || range_m < 0.5) {
        continue;
      }
      // 垂直视场：打不到的线本来就没有回波。不模拟遮挡（文件头已说明），
      // 但视场必须模拟 —— 否则近处地面点会多出一大片真雷达看不到的区域，
      // 而 localization_node 的 ground_fraction 判据是按真视场调出来的。
      const double elevation_rad =
        std::atan2(from_sensor.z(), std::hypot(from_sensor.x(), from_sensor.y()));
      if (elevation_rad < lidar_fov_min_rad_ || elevation_rad > lidar_fov_max_rad_) {
        continue;
      }
      // ⚠️ **必须加测距噪声**，而且不能省。
      //    先验点云按 0.1 m（杆件）/ 0.5 m（墙面）规则采样，而 NDT 体素 2.0 m
      //    —— 两者**整除**。无噪声时大量点的坐标恰好落在体素边界上，代价函数
      //    变成锯齿状：牛顿步方向完全正确却一步走不动，现象酷似「收敛域太小」。
      //    这是本仓库第二次踩「离散结构 × 规则采样」的可通约性（见 CLAUDE.md）。
      const Eigen::Vector3d in_body = base_to_map * point_map + Noise(lidar_noise_m_);
      scratch_.push_back(in_body);
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = ToRosTime(sim_time_s_);
    msg.header.frame_id = "base_link";
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(scratch_.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
    for (const auto & p : scratch_) {
      *it_x = static_cast<float>(p.x());
      *it_y = static_cast<float>(p.y());
      *it_z = static_cast<float>(p.z());
      ++it_x;
      ++it_y;
      ++it_z;
    }
    cloud_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------
  //  真值。⚠️ 仅供评测，算法节点禁止订阅（SPEC §4.1）
  // ---------------------------------------------------------------------
  void PublishTruth()
  {
    const Eigen::Isometry3d map_to_base = TruthPose();
    const Eigen::Quaterniond q(map_to_base.linear());

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = ToRosTime(sim_time_s_);
    msg.header.frame_id = "map";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = map_to_base.translation().x();
    msg.pose.pose.position.y = map_to_base.translation().y();
    msg.pose.pose.position.z = map_to_base.translation().z();
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.twist.twist.linear.x = speed_mps_;
    truth_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------
  //  小工具
  // ---------------------------------------------------------------------
  Eigen::Isometry3d TruthPose() const
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::AngleAxisd(yaw_rad_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(x_m_, y_m_, 0.0);
    return pose;
  }

  /// map → odom 的**真值**：odom 原点钉在自车起点（与 Gazebo 一致）。
  Eigen::Isometry3d MapToOdomTruth() const
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = Eigen::AngleAxisd(start_yaw_rad_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(start_x_m_, start_y_m_, 0.0);
    return pose;
  }

  /// 三个独立的零均值高斯样本。`.x()` 取一个，用作标量噪声。
  Eigen::Vector3d Noise(double stddev)
  {
    if (stddev <= 0.0) {
      return Eigen::Vector3d::Zero();
    }
    std::normal_distribution<double> dist(0.0, stddev);
    return Eigen::Vector3d(dist(rng_), dist(rng_), dist(rng_));
  }

  static rclcpp::Time ToRosTime(double seconds)
  {
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
  }

  /// 与 EskfParams::gravity_mps2 的默认值一致。两处不一致会表现为
  /// 一个恒定的竖直加速度偏差，最终吃进加速度计零偏 —— 不报错，只是估错。
  static constexpr double kGravityMps2 = 9.80665;

  GeoOrigin origin_;
  std::vector<Eigen::Vector3d> map_points_;
  std::vector<Eigen::Vector3d> scratch_;

  double start_x_m_{0.0};
  double start_y_m_{0.0};
  double start_yaw_rad_{0.0};
  double cruise_speed_mps_{4.0};
  double accel_mps2_{1.5};
  double start_delay_s_{4.0};
  double yaw_rate_amplitude_rad_s_{0.05};
  double yaw_rate_period_s_{4.0};

  double sim_step_s_{0.01};
  double real_time_factor_{1.0};

  double gyro_noise_rad_s_{0.0};
  double accel_noise_mps2_{0.0};
  double gnss_horizontal_noise_m_{0.0};
  double gnss_vertical_noise_m_{0.0};
  double lidar_noise_m_{0.0};
  double wheel_noise_mps_{0.0};
  Eigen::Vector3d gyro_bias_rad_s_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_mps2_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d lever_arm_body_{Eigen::Vector3d::Zero()};

  double lidar_range_m_{30.0};
  double lidar_mount_z_m_{1.6};
  double lidar_fov_min_rad_{-0.4363};
  double lidar_fov_max_rad_{0.1745};
  int lidar_stride_{1};

  double x_m_{0.0};
  double y_m_{0.0};
  double yaw_rad_{0.0};
  double speed_mps_{0.0};
  double sim_time_s_{0.0};
  int64_t tick_{0};
  std::mt19937 rng_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ads_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ads_localization::FakeSensors>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("fake_sensors"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
