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
//  prediction_node —— ads_prediction 的 ROS 包装层（P6-S3）
//
//  订阅  /perception/obstacles     ads_msgs/ObstacleArray（map 系）
//  发布  /prediction/trajectories  ads_msgs/PredictedTrajectoryArray（map 系）
//        /prediction/markers      visualization_msgs/MarkerArray（RViz）
//        /prediction/diagnostics  DiagnosticArray（单帧耗时 + 模型分支计数）
//
//  节点只做三件事：消息翻译、逐 id 位置历史（位移一致性证据）、可视化。
//  算法全在 lib（零 ROS）。**不需要 TF** —— 输入输出都在 map 系，
//  少一处能失败的地方。
//
//  地图（决策二，用户拍板）：链接 libads_map 读本包外的 share/ads_map/
//  maps/campus.xodr —— 静态先验，与 map_node 读的是同一份文件。
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ads_map/lane_graph.hpp"
#include "ads_map/opendrive_parser.hpp"
#include "ads_msgs/msg/obstacle_array.hpp"
#include "ads_msgs/msg/predicted_trajectory_array.hpp"
#include "ads_prediction/lane_follow.hpp"
#include "ads_prediction/model_selector.hpp"
#include "ads_prediction/motion_model.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

/// 历史样本比位移窗多留的余量，s。窗口边界恰好落在两帧之间时，
/// 有余量才能取到"窗口外最近的那一帧"做基准。
constexpr double kHistoryMarginS = 0.5;

/// 目标从视野消失多久后丢弃其历史，s。感知的删除窗口 + 遮挡滑行上限
/// 是 3.5 s —— 超过它 ID 本来就会换（新 ID = 新历史），历史留着没有意义。
constexpr double kHistoryExpireS = 4.0;

double YawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class PredictionNode : public rclcpp::Node
{
public:
  PredictionNode() : rclcpp::Node("prediction_node")
  {
    // ---- 参数（与 config/prediction_params.yaml 一一对应，推导见那边）----
    motion_.horizon_s = declare_parameter<double>("horizon_s", 3.0);
    motion_.step_s = declare_parameter<double>("step_s", 0.2);
    motion_.sigma_pos0_m = declare_parameter<double>("sigma_pos0_m", 0.2);
    motion_.sigma_speed_mps = declare_parameter<double>("sigma_speed_mps", 0.15);
    motion_.pedestrian_lateral_accel_mps2 =
      declare_parameter<double>("pedestrian_lateral_accel_mps2", 0.5);
    motion_.vehicle_lateral_accel_mps2 =
      declare_parameter<double>("vehicle_lateral_accel_mps2", 0.8);
    motion_.lane_follow_lateral_accel_mps2 =
      declare_parameter<double>("lane_follow_lateral_accel_mps2", 0.3);
    motion_.static_start_accel_mps2 = declare_parameter<double>("static_start_accel_mps2", 1.5);
    lane_params_.match_max_lateral_m = declare_parameter<double>("match_max_lateral_m", 2.0);
    lane_params_.match_max_heading_rad = declare_parameter<double>("match_max_heading_rad", 0.5236);
    lane_params_.sample_step_m = declare_parameter<double>("sample_step_m", 0.5);
    lane_params_.lateral_decay_s = declare_parameter<double>("lateral_decay_s", 2.0);
    lane_params_.max_hypotheses = static_cast<int>(declare_parameter<int>("max_hypotheses", 4));
    selector_.min_dynamic_speed_mps = declare_parameter<double>("min_dynamic_speed_mps", 0.5);
    selector_.odd_max_speed_mps = declare_parameter<double>("odd_max_speed_mps", 8.33);
    selector_.odd_max_length_m = declare_parameter<double>("odd_max_length_m", 5.5);
    selector_.min_displacement_ratio = declare_parameter<double>("min_displacement_ratio", 0.5);
    selector_.displacement_window_s = declare_parameter<double>("displacement_window_s", 1.0);
    selector_.min_vehicle_length_m = declare_parameter<double>("min_vehicle_length_m", 2.5);

    // ---- 地图：与 map_node 同一份 share 文件（决策二的"静态先验"）----
    const std::string default_map_file =
      ament_index_cpp::get_package_share_directory("ads_map") + "/maps/campus.xodr";
    const std::string map_file = declare_parameter<std::string>("map_file", default_map_file);
    graph_ = std::make_unique<ads_map::LaneGraph>(ads_map::load_opendrive(map_file));

    obstacle_sub_ = create_subscription<ads_msgs::msg::ObstacleArray>(
      "/perception/obstacles", rclcpp::QoS(10),
      [this](const ads_msgs::msg::ObstacleArray::SharedPtr msg) { OnObstacles(msg); });
    trajectory_pub_ = create_publisher<ads_msgs::msg::PredictedTrajectoryArray>(
      "/prediction/trajectories", rclcpp::QoS(10));
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/prediction/markers", rclcpp::QoS(1));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/prediction/diagnostics", rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(), "预测节点就绪：视界 %.1f s、步长 %.1f s、地图 %s", motion_.horizon_s,
      motion_.step_s, map_file.c_str());
  }

private:
  void OnObstacles(const ads_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    const auto started = std::chrono::steady_clock::now();
    const double stamp_s =
      static_cast<double>(msg->header.stamp.sec) + 1e-9 * msg->header.stamp.nanosec;

    ads_msgs::msg::PredictedTrajectoryArray array;
    array.header = msg->header;

    int count_static = 0;
    int count_cv = 0;
    int count_lane = 0;
    for (const ads_msgs::msg::Obstacle & obstacle : msg->obstacles) {
      ads_prediction::TargetSnapshot target;
      target.id = obstacle.id;
      target.position = {obstacle.pose.position.x, obstacle.pose.position.y};
      target.yaw_rad = YawFromQuaternion(obstacle.pose.orientation);
      target.heading_resolved = obstacle.heading_resolved;
      target.velocity = {obstacle.velocity_mps.x, obstacle.velocity_mps.y};
      target.length_m = obstacle.size_m.x;
      target.width_m = obstacle.size_m.y;
      target.height_m = obstacle.size_m.z;
      target.net_displacement_1s_m = NetDisplacement(obstacle, stamp_s);

      std::vector<ads_prediction::PredictedPath> paths;
      switch (ads_prediction::SelectModel(target, selector_)) {
        case ads_prediction::ModelKind::kStatic:
          paths.push_back(ads_prediction::PredictStatic(target, motion_));
          ++count_static;
          break;
        case ads_prediction::ModelKind::kConstantVelocity:
          paths.push_back(ads_prediction::PredictConstantVelocity(
            target, motion_, motion_.pedestrian_lateral_accel_mps2));
          ++count_cv;
          break;
        case ads_prediction::ModelKind::kLaneFollow:
          paths = ads_prediction::PredictLaneFollow(target, *graph_, motion_, lane_params_);
          if (paths.empty()) {
            // 归属不成立（脱离车道 / 斜穿 / 逆行）——如实退恒速（决策五）。
            paths.push_back(ads_prediction::PredictConstantVelocity(
              target, motion_, motion_.vehicle_lateral_accel_mps2));
            ++count_cv;
          } else {
            ++count_lane;
          }
          break;
      }
      for (const ads_prediction::PredictedPath & path : paths) {
        array.trajectories.push_back(ToMessage(path));
      }
    }

    trajectory_pub_->publish(array);
    PublishMarkers(array);
    const double total_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    PublishDiagnostics(msg->header, total_ms, count_static, count_cv, count_lane);
    PruneHistories(stamp_s);
  }

  /// 逐 id 位置历史 → 过去约 1 s 的净位移（位移一致性证据，S1 体检的产物）。
  std::optional<double> NetDisplacement(const ads_msgs::msg::Obstacle & obstacle, double now_s)
  {
    History & history = histories_[obstacle.id];
    history.last_seen_s = now_s;
    history.samples.push_back({now_s, obstacle.pose.position.x, obstacle.pose.position.y});
    const double window_s = selector_.displacement_window_s;
    while (!history.samples.empty() &&
           history.samples.front().t_s < now_s - window_s - kHistoryMarginS) {
      history.samples.pop_front();
    }
    // 基准 = 窗口边界（now − window）之前最近的样本；没有 ⟹ 历史不够长。
    const Sample * base = nullptr;
    for (const Sample & sample : history.samples) {
      if (sample.t_s <= now_s - window_s) {
        base = &sample;
      } else {
        break;
      }
    }
    if (base == nullptr) {
      return std::nullopt;
    }
    return std::hypot(obstacle.pose.position.x - base->x_m, obstacle.pose.position.y - base->y_m);
  }

  void PruneHistories(double now_s)
  {
    for (auto it = histories_.begin(); it != histories_.end();) {
      it = (now_s - it->second.last_seen_s > kHistoryExpireS) ? histories_.erase(it) : ++it;
    }
  }

  ads_msgs::msg::PredictedTrajectory ToMessage(const ads_prediction::PredictedPath & path) const
  {
    ads_msgs::msg::PredictedTrajectory message;
    message.obstacle_id = path.target_id;
    message.model = static_cast<std::uint8_t>(path.model);
    message.probability = path.probability;
    message.points.reserve(path.points.size());
    for (const ads_prediction::PredictedPoint & point : path.points) {
      ads_msgs::msg::PredictedTrajectoryPoint out;
      out.t_s = point.t_s;
      out.x_m = point.position.x();
      out.y_m = point.position.y();
      out.heading_rad = point.heading_rad;
      out.speed_mps = point.speed_mps;
      out.sigma_along_m = point.sigma_along_m;
      out.sigma_cross_m = point.sigma_cross_m;
      message.points.push_back(out);
    }
    return message;
  }

  // ---------------------------------------------------------------------------
  //  RViz：每条轨迹一条 LINE_STRIP，t=1/2/3 s 处一个 2σ 椭圆（压扁的 SPHERE）。
  //  颜色按模型分：车道跟随绿、恒速橙、静态灰；透明度 = 假设概率 ——
  //  低概率分支自然更淡，不用看图例。
  // ---------------------------------------------------------------------------
  void PublishMarkers(const ads_msgs::msg::PredictedTrajectoryArray & array)
  {
    visualization_msgs::msg::MarkerArray markers;
    // 每帧先清场：目标数量逐帧在变，上一帧的多余 marker 不清会留残影。
    visualization_msgs::msg::Marker clear;
    clear.header = array.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 0;
    for (const ads_msgs::msg::PredictedTrajectory & trajectory : array.trajectories) {
      visualization_msgs::msg::Marker line;
      line.header = array.header;
      line.ns = "prediction";
      line.id = marker_id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.15;
      line.pose.orientation.w = 1.0;
      switch (trajectory.model) {
        case ads_msgs::msg::PredictedTrajectory::MODEL_LANE_FOLLOW:
          line.color.g = 0.9;
          break;
        case ads_msgs::msg::PredictedTrajectory::MODEL_CONSTANT_VELOCITY:
          line.color.r = 1.0;
          line.color.g = 0.6;
          break;
        default:  // STATIC：灰
          line.color.r = 0.6;
          line.color.g = 0.6;
          line.color.b = 0.6;
          break;
      }
      line.color.a = static_cast<float>(std::max(0.35, trajectory.probability));
      for (const auto & point : trajectory.points) {
        geometry_msgs::msg::Point p;
        p.x = point.x_m;
        p.y = point.y_m;
        p.z = 0.3;
        line.points.push_back(p);
      }
      if (line.points.size() >= 2) {
        markers.markers.push_back(line);
      }

      // 不确定椭圆：整秒时刻采样，画 2σ（判据 CP-P6-B ⑤ 用的量级）。
      for (const auto & point : trajectory.points) {
        const double t = point.t_s;
        if (std::fabs(t - std::round(t)) > 1e-6 || t < 0.5) {
          continue;
        }
        visualization_msgs::msg::Marker ellipse;
        ellipse.header = array.header;
        ellipse.ns = "prediction_uncertainty";
        ellipse.id = marker_id++;
        ellipse.type = visualization_msgs::msg::Marker::SPHERE;
        ellipse.action = visualization_msgs::msg::Marker::ADD;
        ellipse.pose.position.x = point.x_m;
        ellipse.pose.position.y = point.y_m;
        ellipse.pose.position.z = 0.05;
        ellipse.pose.orientation.z = std::sin(0.5 * point.heading_rad);
        ellipse.pose.orientation.w = std::cos(0.5 * point.heading_rad);
        ellipse.scale.x = std::max(0.05, 2.0 * point.sigma_along_m);
        ellipse.scale.y = std::max(0.05, 2.0 * point.sigma_cross_m);
        ellipse.scale.z = 0.02;
        ellipse.color = line.color;
        ellipse.color.a = 0.25;
        markers.markers.push_back(ellipse);
      }
    }
    marker_pub_->publish(markers);
  }

  void PublishDiagnostics(
    const std_msgs::msg::Header & header, double total_ms, int count_static, int count_cv,
    int count_lane)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "prediction";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    const auto add = [&status](const std::string & key, double value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = std::to_string(value);
      status.values.push_back(pair);
    };
    add("total_ms", total_ms);
    add("targets_static", count_static);
    add("targets_constant_velocity", count_cv);
    add("targets_lane_follow", count_lane);
    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  struct Sample
  {
    double t_s{0.0};
    double x_m{0.0};
    double y_m{0.0};
  };
  struct History
  {
    std::deque<Sample> samples;
    double last_seen_s{0.0};
  };

  ads_prediction::MotionModelParams motion_;
  ads_prediction::LaneFollowParams lane_params_;
  ads_prediction::SelectorParams selector_;
  std::unique_ptr<ads_map::LaneGraph> graph_;
  std::unordered_map<std::uint32_t, History> histories_;

  rclcpp::Subscription<ads_msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<ads_msgs::msg::PredictedTrajectoryArray>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PredictionNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("prediction_node"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
