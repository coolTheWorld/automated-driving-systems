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
//  perception_node —— 感知模块的 ROS 包装层
//
//  订阅  /lidar/points            sensor_msgs/PointCloud2   base_link 系，10 Hz
//  发布  /perception/obstacles    ads_msgs/ObstacleArray    **map 系**
//  发布  /perception/diagnostics  DiagnosticArray           各阶段点数与耗时
//
//  四阶段流水线（算法**一行都不在这里**，全在 lib/）：
//      地面分割 → 欧式聚类 → L-Shape 拟合 + 尺寸分类 → 匈牙利关联 + 跟踪
//
//  ## ⚠️ 坐标系：前三阶段在 base_link，跟踪在 map
//
//  · **地面分割必须在 base_link 做** —— 它假设地面 z ≈ 0，而那只在传感器系
//    成立（base_link 原点就定在地面高度）。搬到 map 系去做的话，
//    地面高度变成一个随位置变化的量，RANSAC 仍然能拟合，但
//    `max_height_m` 那道"只在这个高度以下找地面"的筛选就失效了。
//  · **跟踪必须在 map 系** —— 恒速模型要求惯性系。在 base_link 里跟踪的话，
//    自车一转弯**所有目标都在动**，恒速假设立刻破产，速度估计全是自车运动。
//
//  所以 L-Shape 拟合之后做一次 base_link → map 的变换，再喂给跟踪器。
//
//  ## ⚠️ 与真值发布器互斥
//
//  `obstacle_truth` 在 `publish_as_perception = true` 时也发
//  `/perception/obstacles`。两者**不能同时发** —— SPEC §3.3 的「每一段
//  有且只有一个发布者」在话题上同样成立，而 P4 实测过：多一个发布者
//  **不报错、数值上也不一定看得出来**。
//  由 launch 的 `perception:=true/false` 二选一，见 stack.launch.py。
//
//  ## ⚠️ 只发**已确认**的航迹
//
//  未确认的航迹可能只是噪点簇。下游（规划）会对每个障碍物做碰撞检查，
//  虚警的代价是车无故刹停 —— 而那看起来像"规划器有毛病"。
// =============================================================================

// ⚠️ Eigen 的头没有扩展名，cpplint 把它归成「C 系统头」，必须排在
//    C++ 标准库之前 —— 见 CLAUDE.md 的 lint 陷阱表。
#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "ads_msgs/msg/obstacle.hpp"
#include "ads_msgs/msg/obstacle_array.hpp"
#include "ads_perception/euclidean_cluster.hpp"
#include "ads_perception/ground_segmentation.hpp"
#include "ads_perception/lshape_fit.hpp"
#include "ads_perception/size_classifier.hpp"
#include "ads_perception/tracker.hpp"

namespace
{

/// 各阶段的耗时与规模，只用于诊断。
struct StageStats
{
  int input_points{0};
  int non_ground_points{0};
  // ---- P9-S1 域移植仪器（CARLA 上分割失效的三嫌疑靠这些数字裁决）----
  bool ground_found{false};
  double ground_height_m{0.0};   // 平面在原点的 z（= −offset/n_z）：挂高偏差哨兵
  double ground_slope_deg{0.0};  // 法向与 +z 夹角：mesh 非平面/斜面哨兵
  double ground_ratio{0.0};      // ground/pool：路面占比哨兵（Gazebo 基线 ~0.5）
  int slope_rejected{0};         // 坡度门拒绝轮数：总在抽到墙 = 嫌疑 2
  int razor_dropped{0};  // 剃刀门吞掉的框数：合法目标被吞的哨兵（P9 Gazebo 回归案）
  double razor_min_extent_m{1e9};  // 本帧被吞框里最大的那个 min(l,w)：门与目标剖面的距离
  double razor_max_range_m{0.0};  // 本帧被吞框里最远的距离：远距单环回波被吞的哨兵
  double razor_max_top_m{0.0};  // 本帧被吞框里最高的簇顶（离地）：吞到的是地面残留还是目标
  int clusters{0};
  int largest_cluster{0};
  int detections{0};
  int confirmed_tracks{0};
  double ground_ms{0.0};
  double cluster_ms{0.0};
  double fit_ms{0.0};
  double track_ms{0.0};
  double total_ms{0.0};
};

}  // namespace

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode() : Node("perception_node")
  {
    // ---- 地面分割 -------------------------------------------------------
    ground_params_.max_iterations = declare_parameter<int>("ground.max_iterations", 100);
    ground_params_.distance_threshold_m =
      declare_parameter<double>("ground.distance_threshold_m", 0.15);
    ground_params_.max_slope_rad = declare_parameter<double>("ground.max_slope_rad", 0.26);
    ground_params_.max_height_m = declare_parameter<double>("ground.max_height_m", 1.0);
    ground_params_.min_inliers = declare_parameter<int>("ground.min_inliers", 100);

    // ---- 聚类 -----------------------------------------------------------
    // ⚠️ tolerance 被雷达线间距从下面顶着（25 m 处 0.493 m），
    //    min_cluster_size 由 S1 的实测点数定（20–25 m 只有 7 点）。
    //    两个都是安全关键值，理由见 euclidean_cluster.hpp。
    cluster_params_.tolerance_m = declare_parameter<double>("cluster.tolerance_m", 0.5);
    cluster_params_.min_cluster_size = declare_parameter<int>("cluster.min_cluster_size", 5);
    cluster_params_.max_cluster_size = declare_parameter<int>("cluster.max_cluster_size", 20000);

    // ---- L-Shape --------------------------------------------------------
    fit_params_.angle_step_rad = declare_parameter<double>("lshape.angle_step_rad", 0.01745);
    fit_params_.min_points = declare_parameter<int>("lshape.min_points", 4);
    min_extent_m_ = declare_parameter<double>("cluster.min_extent_m", 0.1);
    razor_max_height_m_ = declare_parameter<double>("cluster.razor_max_height_m", 0.3);

    // ---- 跟踪 -----------------------------------------------------------
    tracker_params_.process_accel_stddev_mps2 =
      declare_parameter<double>("tracker.process_accel_stddev_mps2", 2.0);
    tracker_params_.measurement_stddev_m =
      declare_parameter<double>("tracker.measurement_stddev_m", 0.3);
    tracker_params_.gate_chi_square = declare_parameter<double>("tracker.gate_chi_square", 9.21);
    // ⚠️ 确认用**累计**命中而不是连续 —— S1 实测目标在连续帧之间闪烁，
    //    要求连续 3 帧命中的话概率只有 12.5%。见 tracker.hpp 的文件头。
    tracker_params_.confirm_hits = declare_parameter<int>("tracker.confirm_hits", 3);
    tracker_params_.max_misses = declare_parameter<int>("tracker.max_misses", 5);
    tracker_params_.max_occluded_misses = declare_parameter<int>("tracker.max_occluded_misses", 30);
    tracker_params_.heading_min_speed_mps =
      declare_parameter<double>("tracker.heading_min_speed_mps", 0.5);
    // 两道物理闸（P6-S0 加），推导见 tracker.hpp 对应参数的注释：
    // 前者封"尺寸差换算成位置修正"的关联虫洞，后者不让坏状态借遮挡滑行外推。
    tracker_params_.anchor_shift_max_m =
      declare_parameter<double>("tracker.anchor_shift_max_m", 2.2);
    tracker_params_.coast_max_speed_mps =
      declare_parameter<double>("tracker.coast_max_speed_mps", 8.33);
    tracker_ = std::make_unique<ads_perception::Tracker>(tracker_params_);
    max_misses_ = tracker_params_.max_misses;

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    // 点云比这么旧就丢。与 localization_node 那条同一个理由：
    // 用旧帧算出来的障碍物位置对应的是**过去**的时刻，而下游会当成现在的。
    max_cloud_age_s_ = declare_parameter<double>("max_cloud_age_s", 0.15);
    // 诊断发布周期，s。默认 1 Hz（给人看的，不是控制回路的一部分）。
    // ⚠️ 设 0 则**每帧都发** —— 排查"某个距离上目标消失"这类问题时必须这样，
    //    因为 1 Hz 只能采到十分之一的帧，而故障往往只持续几帧。
    diagnostics_period_s_ = declare_parameter<double>("diagnostics_period_s", 1.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    obstacle_pub_ =
      create_publisher<ads_msgs::msg::ObstacleArray>("/perception/obstacles", rclcpp::QoS(10));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/perception/diagnostics", rclcpp::QoS(10));

    // ⚠️ 点云用 **reliable + 深度 10**，与 lidar_preprocessor 的发布端一致。
    //    best-effort 会静默丢帧（实测只剩标称的 35%），而症状是
    //    「感知偶尔漏一帧」—— 没有任何日志。见 CLAUDE.md 陷阱表。
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { OnCloud(std::move(msg)); });

    RCLCPP_INFO(
      get_logger(), "perception_node 就绪：聚类容差 %.2f m，最小簇 %d 点，确认需 %d 次命中",
      cluster_params_.tolerance_m, cluster_params_.min_cluster_size, tracker_params_.confirm_hits);
  }

private:
  void OnCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto started = std::chrono::steady_clock::now();
    StageStats stats;

    // ---- 陈旧点云直接丢 -------------------------------------------------
    const double age_s = (now() - rclcpp::Time(msg->header.stamp)).seconds();
    if (age_s > max_cloud_age_s_) {
      ++dropped_stale_clouds_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "点云已经旧了 %.0f ms（上限 %.0f），丢弃。用旧帧算出来的障碍物位置对应的是"
        "**过去**的时刻，而下游会当成现在的。累计丢 %ld 帧",
        age_s * 1e3, max_cloud_age_s_ * 1e3, dropped_stale_clouds_);
      return;
    }

    // ---- 取 map ← base_link 的变换 --------------------------------------
    // ⚠️ 用**点云自己的**时间戳查，不是 now()：车 4 m/s 时差 100 ms 就是 0.4 m，
    //    而那个偏差会被算成检测误差。
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        map_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(0.05));
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "拿不到 %s ← %s 的变换：%s", map_frame_.c_str(),
        msg->header.frame_id.c_str(), e.what());
      return;
    }

    // ---- 读点 -----------------------------------------------------------
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->width * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      // ⚠️ gpu_lidar 的无回波射线返回 **±inf 不是 NaN**，两者都要滤。
      //    不滤的话下游的 SegmentGround / ClusterEuclidean 会抛异常
      //    （它们的契约就是这样），而那会让整个回调中断。
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) {
        continue;
      }
      points.emplace_back(*it_x, *it_y, *it_z);
    }
    stats.input_points = static_cast<int>(points.size());
    if (points.empty()) {
      return;
    }

    // ---- ① 地面分割（base_link 系，地面 z ≈ 0 是它的前提）--------------
    auto stage = std::chrono::steady_clock::now();
    ads_perception::GroundSegmentationResult ground =
      ads_perception::SegmentGround(points, ground_params_);
    // ---- 平面时间一致性门（P9-S2，「门成立要有空档」实测后上的门）----------
    // 停驻实测帧间平面差 0.5-0.7°；坏帧是 5°+ 的歪平面（6.9° 坡度门内），
    // 30 m 外高度差 3 m ⟹ 远段路整体被判非地面 ⟹ 28 m 巨板虚警。
    // 空档（0.7° vs 5°）近一个量级 ⟹ 门取 2°/0.15 m。上一块 ≤1 s 新时，
    // 越门的新平面**弃用**、按上一块重分类；found=false 同样兜底 ——
    // 10 Hz 管线里 0.1 s 陈旧的好平面 ≫ 没有平面。超龄则接受现实重置。
    const rclcpp::Time cloud_stamp(msg->header.stamp);
    const bool last_fresh = last_plane_valid_ && (cloud_stamp - last_plane_stamp_).seconds() < 1.0;
    bool replace_by_last = false;
    if (ground.found) {
      const double angle_rad =
        std::acos(std::clamp(ground.normal.dot(last_plane_normal_), -1.0, 1.0));
      const double height_m =
        std::abs(ground.normal.z()) > 1e-9 ? -ground.offset_m / ground.normal.z() : 1e9;
      const double last_height_m = std::abs(last_plane_normal_.z()) > 1e-9
                                     ? -last_plane_offset_ / last_plane_normal_.z()
                                     : 1e9;
      if (last_fresh && (angle_rad > 0.035 || std::abs(height_m - last_height_m) > 0.15)) {
        replace_by_last = true;
      }
    } else if (last_fresh) {
      replace_by_last = true;
    }
    if (replace_by_last) {
      ground.found = true;
      ground.normal = last_plane_normal_;
      ground.offset_m = last_plane_offset_;
      ground.ground_count = 0;
      ground.is_ground.assign(points.size(), 0U);
      for (std::size_t i = 0; i < points.size(); ++i) {
        if (
          std::abs(ground.normal.dot(points[i]) + ground.offset_m) <=
          ground_params_.distance_threshold_m) {
          ground.is_ground[i] = 1U;
          ++ground.ground_count;
        }
      }
      ++ground_held_count_;
    } else if (ground.found) {
      last_plane_normal_ = ground.normal;
      last_plane_offset_ = ground.offset_m;
      last_plane_stamp_ = cloud_stamp;
      last_plane_valid_ = true;
    }
    stats.ground_ms = Elapsed(&stage);
    stats.ground_found = ground.found;
    stats.slope_rejected = ground.slope_rejected_count;
    if (ground.pool_count > 0) {
      stats.ground_ratio = static_cast<double>(ground.ground_count) / ground.pool_count;
    }
    if (ground.found && std::abs(ground.normal.z()) > 1e-9) {
      // 平面 n·x + d = 0 在 x=y=0 处的 z = −d/n_z。base_link 里地面应 ≈ 0；
      // 系统性偏离 = 传感器挂载基准差（P9 嫌疑 1）的直接读数。
      stats.ground_height_m = -ground.offset_m / ground.normal.z();
      stats.ground_slope_deg = std::acos(std::clamp(ground.normal.z(), -1.0, 1.0)) * 180.0 / M_PI;
    }

    std::vector<Eigen::Vector3d> non_ground;
    if (ground.found) {
      non_ground.reserve(points.size() - ground.ground_count);
      for (std::size_t i = 0; i < points.size(); ++i) {
        if (ground.is_ground[i] == 0U) {
          non_ground.push_back(points[i]);
        }
      }
    } else {
      // ⚠️ 没找到地面时**不要把所有点都当障碍物** —— 那会让下游看到
      //    一大片虚警并立刻刹停。宁可这一帧什么都不报（航迹靠 max_misses
      //    撑住），也不要报一堆假的。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "这一帧没找到地面（%d 点），跳过。不把全部点当障碍物是有意的："
        "那会让规划看到一大片虚警并立刻刹停",
        stats.input_points);
      ++frames_without_ground_;
      return;
    }
    stats.non_ground_points = static_cast<int>(non_ground.size());

    // ---- ② 欧式聚类 ------------------------------------------------------
    const std::vector<ads_perception::Cluster> clusters =
      ads_perception::ClusterEuclidean(non_ground, cluster_params_);
    stats.cluster_ms = Elapsed(&stage);
    stats.clusters = static_cast<int>(clusters.size());
    // ⚠️ **最大簇的点数**：欠分割（多个目标被连成一片）的直接证据。
    //    只看簇数看不出来 —— 簇数少既可能是"目标少"也可能是"全连一起了"。
    for (const ads_perception::Cluster & cluster : clusters) {
      stats.largest_cluster =
        std::max(stats.largest_cluster, static_cast<int>(cluster.indices.size()));
    }

    // ---- ③ L-Shape 拟合 + 转到 map 系 -----------------------------------
    const double yaw = QuaternionYaw(transform.transform.rotation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const auto & translation = transform.transform.translation;

    std::vector<ads_perception::Detection> detections;
    std::vector<Eigen::Vector3d> cluster_points;
    detections.reserve(clusters.size());
    for (const ads_perception::Cluster & cluster : clusters) {
      cluster_points.clear();
      cluster_points.reserve(cluster.indices.size());
      for (const int index : cluster.indices) {
        cluster_points.push_back(non_ground[index]);
      }
      const ads_perception::LShapeBox box = ads_perception::FitLShape(cluster_points, fit_params_);
      if (!box.valid) {
        continue;
      }
      // ---- 剃刀条门（P9-S2，物理先验准入）--------------------------------
      // CARLA 生成路面的接缝/边线几何在 RANSAC 阈值上骑线，漏成 0.03 m 宽、
      // 1–2.5 m 长的**单环弧段**簇（P5 实测 374 帧车道内虚警的主体），
      // 并以短命航迹打碎跟踪器（ID 切换 47、速度误差 6.0 的上游）。
      // ODD（SPEC §2）里不存在最小水平尺寸 < 0.1 m 的目标 —— 行人 0.4、
      // 锥桶 0.5、车 1.8；这是按**物理**收的准入门，不是按场景调的补丁。
      // ⚠️ P9 Gazebo 回归案（2026-08-15）推翻了「合法目标远在门上」：L-Shape
      //    量的是**可见剖面**不是物体 —— 正对的盒状目标只露一个面，深度
      //    方向剩下的是雷达噪声（σ=1 cm），min(l,w) 实测 p50 0.047 / max
      //    0.097，**每一帧**都被 0.1 门吞掉（A/B：门 0.1 跟停 −5.19 m 撞车；
      //    门 0 全绿）。CARLA 网格有曲面所以从没露过马脚。
      //    真正的物理先验是「剃刀条是**一维**的」：既薄**又矮**（地面接缝
      //    残留一环点，竖向延展 ≈ 0）；而正对目标薄但**高**（车尾面 1.3 m、
      //    行人 1.5 m）。两个条件缺一不可 —— 只按薄收门等于把所有正对的
      //    盒子当剃刀条。
      if (
        std::min(box.length_m, box.width_m) < min_extent_m_ && box.height_m < razor_max_height_m_) {
        ++stats.razor_dropped;
        stats.razor_min_extent_m =
          std::min(stats.razor_min_extent_m, std::min(box.length_m, box.width_m));
        // 被吞框的距离与簇顶离地高度（观察用：吞到的是地面残留还是远处目标的单环）
        double top_m = -1e9;
        for (const auto & pt : cluster_points) {
          top_m = std::max(top_m, pt.z());
        }
        stats.razor_max_range_m = std::max(stats.razor_max_range_m, box.center.norm());
        stats.razor_max_top_m = std::max(stats.razor_max_top_m, top_m);
        continue;
      }

      // base_link → map。目标都在地面上，所以只需要平面旋转 + 平移。
      ads_perception::Detection detection;
      detection.position.x() = cos_yaw * box.center.x() - sin_yaw * box.center.y() + translation.x;
      detection.position.y() = sin_yaw * box.center.x() + cos_yaw * box.center.y() + translation.y;
      // ⚠️ 轴向也要转 —— 它是在 base_link 系里量的。
      //    忘了转的症状是"车直行时朝向对、一转弯全体目标的朝向跟着自车转"。
      detection.yaw_rad = box.yaw_rad + yaw;
      detection.length_m = box.length_m;
      detection.width_m = box.width_m;
      detection.height_m = box.height_m;
      detections.push_back(detection);
    }
    stats.fit_ms = Elapsed(&stage);
    stats.detections = static_cast<int>(detections.size());

    // ---- ④ 跟踪（map 系 —— 恒速模型要求惯性系）--------------------------
    const rclcpp::Time stamp(msg->header.stamp);
    double dt_s = 0.1;  // 首帧没有上一帧，用标称周期
    if (last_stamp_.nanoseconds() != 0) {
      dt_s = (stamp - last_stamp_).seconds();
    }
    if (!(dt_s > 0.0) || !std::isfinite(dt_s)) {
      // ⚠️ dt ≤ 0 通常意味着**两套仿真同时在发 /clock**（CLAUDE.md 有专门一条）。
      //    此时所有测量值都作废，该去查残留进程而不是在这里凑合。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "dt = %.4f s，跳过这一帧 —— 是不是有两套仿真在跑？",
        dt_s);
      return;
    }
    // ⚠️ **守卫通过之后才更新 last_stamp_**（2026-08-12 复检修复）：
    //    原来它在守卫**之前**无条件更新 —— 两套仿真的时间戳交错时，守卫只
    //    拦得住负 dt 的那一半帧，另一半带着 |时钟偏移| 量级的巨 dt 进跟踪器：
    //    Predict 沿旧速度外推 ~200 m，Q 按 dt⁴ 膨胀 6 个数量级，卡方门限对
    //    场内一切放行 —— 而日志只有"dt 负，跳过"，看着像守卫在正常拦截。
    last_stamp_ = stamp;
    // dt 上限：单仿真下长时间丢帧后的巨 dt 是真实流逝，但跟踪器的恒速外推
    // 在几秒之外已无意义（目标早换了机动），不如按"跟丢重来"处理。
    // 取 1.0 s = max_misses（0.5 s）的 2 倍：正常丢几帧到不了这里。
    if (dt_s > 1.0) {
      RCLCPP_WARN(
        get_logger(), "点云间隔 %.2f s 过大（时钟跳变或长时间断流），本帧按首帧处理", dt_s);
      tracker_ = std::make_unique<ads_perception::Tracker>(tracker_params_);
      dt_s = 0.1;
    }
    // 传感器在 map 系的位置 = base_link 原点（`transform` 正是 base_link → map）。
    //
    // ⚠️ 用 base_link 原点而不是雷达实际安装点（前方约 1.4 m）是有意的：
    //    补全只需要"哪一侧背离传感器"这个**方向**，而 1.4 m 的偏移在
    //    10 m 处只改变 8° 的视线角，不可能改变任何一侧的符号。
    //    真去查 base_link→lidar_link 的 TF 反而多一处可以失败的地方。
    tracker_->Update(detections, dt_s, Eigen::Vector2d(translation.x, translation.y));
    stats.track_ms = Elapsed(&stage);

    const std::vector<ads_perception::Track> tracks = tracker_->ConfirmedTracks();
    stats.confirmed_tracks = static_cast<int>(tracks.size());
    stats.total_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();

    PublishObstacles(tracks, msg->header.stamp);
    CountIdSwitches(tracks);
    PublishDiagnostics(stats, msg->header.stamp);
  }

  void PublishObstacles(
    const std::vector<ads_perception::Track> & tracks, const builtin_interfaces::msg::Time & stamp)
  {
    ads_msgs::msg::ObstacleArray array;
    array.header.stamp = stamp;
    array.header.frame_id = map_frame_;
    array.obstacles.reserve(tracks.size());

    for (const ads_perception::Track & track : tracks) {
      ads_msgs::msg::Obstacle obstacle;
      obstacle.header = array.header;
      obstacle.id = track.id;
      obstacle.classification = static_cast<std::uint8_t>(ads_perception::ClassifyBySize(
        track.length_m, track.width_m, track.height_m, classifier_params_));

      obstacle.pose.position.x = track.position().x();
      obstacle.pose.position.y = track.position().y();
      obstacle.pose.position.z = 0.5 * track.height_m;

      // ⚠️ 朝向：消歧成功就用车头朝向，否则**退回轴向**。
      //    轴向有 180° 二义性，但那是**如实**的 —— 猜一个的话有 50% 的
      //    机会让 P6 预测出一条逆行轨迹（见 lshape_fit.hpp 的文件头）。
      tf2::Quaternion quaternion;
      // 先验锚定的车：朝向用速度方向（heading_rad 是轴向消歧，正对时轴向
      // 是车宽向 —— 4.4 的框会横躺，见 Track::prior_heading_rad）。
      const double published_yaw = track.vehicle_prior_anchored
                                     ? track.prior_heading_rad
                                     : (track.heading_resolved ? track.heading_rad : track.yaw_rad);
      quaternion.setRPY(0.0, 0.0, published_yaw);
      obstacle.pose.orientation.x = quaternion.x();
      obstacle.pose.orientation.y = quaternion.y();
      obstacle.pose.orientation.z = quaternion.z();
      obstacle.pose.orientation.w = quaternion.w();
      // 语义标志一起发（P6-S3）：下游从此分得清 yaw 是车头朝向还是轴向。
      // 此前只有上面那行三目在静默二选一 —— 前置台账第 3 条的实体。
      obstacle.heading_resolved = track.heading_resolved;

      // 尺寸的发布分三档（P8-S2b，与中心的锚定约定**必须同一套**）：
      //   结构物 —— 当帧观测（记忆前提不成立，历史只会把框养虚胖；观测
      //   才是此刻真实占据的空间，低估结构对准入危险）；
      //   先验锚定的车 —— 长度用 ODD 车长垫底。中心已按 4.4 推到几何
      //   中心，长度若还报记忆的 1.9，下游算出的近边比真值远 1.25 m ——
      //   CP-P5-B 回归实测近边 p95 从 0.13 爆到 1.82、跟停间距缩到 3.55，
      //   「只推中心不给长度」是一次撕裂；
      //   其余 —— 记忆（P5 原语义）。
      if (track.is_structure) {
        obstacle.size_m.x = track.last_observed_length_m;
        obstacle.size_m.y = track.last_observed_width_m;
      } else if (track.vehicle_prior_anchored) {
        obstacle.size_m.x = std::max(track.length_m, tracker_params_.vehicle_prior_length_m);
        obstacle.size_m.y = track.width_m;
      } else {
        obstacle.size_m.x = track.length_m;
        obstacle.size_m.y = track.width_m;
      }
      obstacle.size_m.z = track.height_m;
      // 出口用 reported_velocity：结构物的内部速度是可见面滑移，不许出门
      // （P8-S2b，理由见 tracker.hpp）。
      obstacle.velocity_mps.x = track.reported_velocity().x();
      obstacle.velocity_mps.y = track.reported_velocity().y();
      obstacle.velocity_mps.z = 0.0;

      // 存在概率：按连续未命中衰减。刚命中的是 1.0，丢得越久越低。
      //
      // ⚠️ **SPEC §11 禁止拿它做碰撞检查的过滤。** 它只是信息 ——
      //    一个"存在概率 0.2"的东西照样会把车撞坏。
      obstacle.existence_probability = static_cast<float>(std::max(
        0.0, 1.0 - static_cast<double>(track.consecutive_misses) / std::max(1, max_misses_)));

      array.obstacles.push_back(obstacle);
    }
    obstacle_pub_->publish(array);
  }

  void CountIdSwitches(const std::vector<ads_perception::Track> & tracks)
  {
    // ID 切换 = 上一帧有、这一帧没了的航迹数。
    //
    // ⚠️ 它**不区分**"目标真的走了"和"航迹断了又重建" —— 前者是正常的。
    //    所以这个量只作诊断，CP-P5-B 的 ID 稳定性判据要在实测脚本里
    //    按真值配对来算，而不是拿这个数。**不要把诊断量当判据。**
    std::unordered_set<std::uint32_t> current;
    for (const ads_perception::Track & track : tracks) {
      current.insert(track.id);
    }
    for (const std::uint32_t id : previous_ids_) {
      if (current.count(id) == 0) {
        ++id_disappearances_;
      }
    }
    previous_ids_ = std::move(current);
  }

  void PublishDiagnostics(const StageStats & stats, const builtin_interfaces::msg::Time & stamp)
  {
    // 1 Hz 就够 —— 这是给人看的，不是控制回路的一部分。
    const rclcpp::Time now_stamp(stamp);
    const std::int64_t period_ns = static_cast<std::int64_t>(diagnostics_period_s_ * 1e9);
    if (now_stamp.nanoseconds() - last_diag_ns_ < period_ns) {
      return;
    }
    last_diag_ns_ = now_stamp.nanoseconds();

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "perception";
    status.hardware_id = "ads_perception";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = std::to_string(stats.confirmed_tracks) + " tracks";

    const auto add = [&status](const std::string & key, double value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = std::to_string(value);
      status.values.push_back(pair);
    };
    add("input_points", stats.input_points);
    add("non_ground_points", stats.non_ground_points);
    add("ground_found", stats.ground_found ? 1.0 : 0.0);
    add("ground_height_m", stats.ground_height_m);
    add("ground_slope_deg", stats.ground_slope_deg);
    add("ground_ratio", stats.ground_ratio);
    add("ground_slope_rejected", stats.slope_rejected);
    add("ground_held_count", ground_held_count_);
    add("razor_dropped", stats.razor_dropped);
    add("razor_min_extent_m", stats.razor_dropped > 0 ? stats.razor_min_extent_m : 0.0);
    add("razor_max_range_m", stats.razor_max_range_m);
    add("razor_max_top_m", stats.razor_dropped > 0 ? stats.razor_max_top_m : 0.0);
    add("clusters", stats.clusters);
    add("largest_cluster", stats.largest_cluster);
    add("detections", stats.detections);
    add("confirmed_tracks", stats.confirmed_tracks);
    // ⚠️ **分阶段耗时**，不是只发一个总数。CLAUDE.md 那条「频率低 = 算力不够
    //    是最容易犯的想当然」—— 没有分层数据的话，超预算时第一反应会是
    //    砍雷达分辨率，而 S3 的教训是那样只快了 4%（病根在 QoS）。
    add("ground_ms", stats.ground_ms);
    add("cluster_ms", stats.cluster_ms);
    add("fit_ms", stats.fit_ms);
    add("track_ms", stats.track_ms);
    add("total_ms", stats.total_ms);
    add("dropped_stale_clouds", static_cast<double>(dropped_stale_clouds_));
    add("frames_without_ground", static_cast<double>(frames_without_ground_));
    add("id_disappearances", static_cast<double>(id_disappearances_));
    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  static double Elapsed(std::chrono::steady_clock::time_point * since)
  {
    const auto now_point = std::chrono::steady_clock::now();
    const double milliseconds =
      std::chrono::duration<double, std::milli>(now_point - *since).count();
    *since = now_point;
    return milliseconds;
  }

  static double QuaternionYaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  ads_perception::GroundSegmentationParams ground_params_;
  ads_perception::EuclideanClusterParams cluster_params_;
  ads_perception::LShapeFitParams fit_params_;
  ads_perception::SizeClassifierParams classifier_params_;
  ads_perception::TrackerParams tracker_params_;
  std::unique_ptr<ads_perception::Tracker> tracker_;
  int max_misses_{5};

  std::string map_frame_;
  double max_cloud_age_s_{0.15};
  double diagnostics_period_s_{1.0};
  double min_extent_m_{0.0};
  double razor_max_height_m_{0.0};
  // 平面时间一致性门的状态（见回调内注释）
  bool last_plane_valid_{false};
  Eigen::Vector3d last_plane_normal_{Eigen::Vector3d::UnitZ()};
  double last_plane_offset_{0.0};
  rclcpp::Time last_plane_stamp_{0, 0, RCL_ROS_TIME};
  int ground_held_count_{0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::int64_t last_diag_ns_{0};
  std::int64_t dropped_stale_clouds_{0};
  std::int64_t frames_without_ground_{0};
  std::int64_t id_disappearances_{0};
  std::unordered_set<std::uint32_t> previous_ids_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<ads_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PerceptionNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception_node"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
