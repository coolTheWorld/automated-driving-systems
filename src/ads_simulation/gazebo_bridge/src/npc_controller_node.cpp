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
//  npc_controller —— 让 P5 的动态目标沿航点走（**仿真道具，不是算法**）
//
//  订阅  /model/<name>/pose_gt   nav_msgs/Odometry      目标自己的真值位姿
//  发布  /model/<name>/cmd_vel   geometry_msgs/Twist    **车体系**速度指令
//
//  ## ⚠️ 为什么这个节点订阅真值是合法的
//
//  SPEC §4.1 那条禁令是「**算法节点**一律禁止订阅真值」——因为一份标准答案
//  流进被考核的一方，考试就失去意义。而本节点是**仿真侧的道具驱动**：
//  它和 Gazebo 里那辆 NPC 车是同一个东西的两半，不参与任何被考核的推理。
//  同理 `obstacle_truth` 也直接读 YAML。
//
//  判断标准很简单：**这个节点的输出会不会进到被评测的链路里？**
//  会 → 禁止用真值；不会 → 用真值反而更简单、更确定。
//
//  ## ⚠️ cmd_vel 是**车体系**，不是世界系（2026-08-11 实测确认）
//
//  探针：把模型 spawn 成 yaw = π（车头朝 −x），发 linear.x = **+4**，
//  实测它往 −x 走。也就是说 linear.x 的语义是「沿车头方向前进」。
//
//  这一条搞错了会怎样：控制器按世界系算速度 → 车**倒着开**。
//  而车是前后对称的，点云形状照样对、真值位置照样对，
//  唯一的破绽是车头朝向 —— 那正是 L-Shape 拟合要估的量。
//  于是 S3 会量到「朝向差 180°」，而人会去查 L-Shape 的二义性处理。
//
//  ## 控制律为什么可以这么简单
//
//  `VelocityControl` 插件是**运动学**的：给什么速度就走什么速度，没有轮胎、
//  没有滑移、没有执行机构延迟。所以「跑飞」在结构上不可能发生，
//  一个比例朝向环就够了。
//
//  ⚠️ **不要把 Stanley 搬过来。** 那是被测对象的控制器，它的调参结论
//  （k_e = 1.0 等）建立在真实动力学上。在一个理想积分器上再验一遍
//  既没有意义，又会让人误以为 NPC 的行为能反映控制器质量。
// =============================================================================

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ads_common/angles.hpp"

namespace gazebo_bridge
{

class NpcControllerNode : public rclcpp::Node
{
public:
  NpcControllerNode() : Node("npc_controller_node")
  {
    model_name_ = declare_parameter<std::string>("model_name", "");
    if (model_name_.empty()) {
      throw std::invalid_argument("npc_controller: 必须给 model_name");
    }

    // 航点由 launch 从 config/dynamic_actors.yaml 搬运。这里一个坐标都不写死 ——
    // 与 control_node / obstacle_truth 同一条规矩。
    const auto xs = declare_parameter<std::vector<double>>("waypoints_x_m", {});
    const auto ys = declare_parameter<std::vector<double>>("waypoints_y_m", {});
    if (xs.size() != ys.size() || xs.size() < 2) {
      throw std::invalid_argument(
        "npc_controller: waypoints_x_m / waypoints_y_m 长度必须相等且 ≥ 2，收到 " +
        std::to_string(xs.size()) + " / " + std::to_string(ys.size()));
    }
    for (size_t i = 0; i < xs.size(); ++i) {
      waypoints_.push_back({xs[i], ys[i]});
    }

    speed_mps_ = declare_parameter<double>("speed_mps", 0.0);
    if (!(speed_mps_ > 0.0)) {
      throw std::invalid_argument("npc_controller: speed_mps 必须为正");
    }

    // 走完最后一个航点之后循环回第一个。
    // 默认 true：验收跑的时长不确定，目标提前走完就消失的话，
    // 后半程的判据会因为「没有目标」而**恒真** —— 又是一个绿灯不代表对。
    loop_ = declare_parameter<bool>("loop", true);

    // 到达判定半径。取 1.0 m：比一个控制周期的位移（4 m/s × 0.1 s = 0.4 m）大，
    // 否则会在航点附近反复错过、原地打转。
    // 调大 → 转弯提前，路径被"切角"；调小 → 可能永远判不到达。
    arrival_radius_m_ = declare_parameter<double>("arrival_radius_m", 1.0);

    // 朝向环增益。运动学模型上这个环没有稳定性问题，取 2.0 让它在
    // 约 0.5 s 内对齐航向。调大 → 转向更急（点云里能看到侧面）；
    // 调小 → 转弯时会画一个大圈，可能压出车道。
    heading_gain_ = declare_parameter<double>("heading_gain", 2.0);

    // 朝向误差大的时候减速，避免边转边冲出去。
    // cos 加权：误差 90° 时速度归零，只原地转。
    // 关掉它（设 false）的话，掉头那一下会画一个半径 = v/ω 的圈。
    slow_down_when_turning_ = declare_parameter<bool>("slow_down_when_turning", true);

    const double rate_hz = declare_parameter<double>("control_rate_hz", 20.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/model/" + model_name_ + "/cmd_vel", rclcpp::QoS(10));
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/model/" + model_name_ + "/pose_gt", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { OnPose(std::move(msg)); });

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz)),
      [this]() { Step(); });

    RCLCPP_INFO(
      get_logger(), "NPC 控制器就绪：模型 %s，%zu 个航点，%.2f m/s%s", model_name_.c_str(),
      waypoints_.size(), speed_mps_, loop_ ? "，循环" : "");
  }

private:
  struct Waypoint
  {
    double x_m;
    double y_m;
  };

  void OnPose(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_m_ = msg->pose.pose.position.x;
    y_m_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    yaw_rad_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_pose_ = true;
  }

  void Step()
  {
    geometry_msgs::msg::Twist cmd;  // 默认全零 = 停住

    if (!have_pose_) {
      // 还没收到自己的位姿。发零速而不是"猜一个" ——
      // 仿真刚起来时目标乱窜会污染开头几帧的感知判据。
      cmd_pub_->publish(cmd);
      return;
    }
    if (finished_) {
      cmd_pub_->publish(cmd);
      return;
    }

    const Waypoint & target = waypoints_[index_];
    const double dx = target.x_m - x_m_;
    const double dy = target.y_m - y_m_;
    const double distance_m = std::hypot(dx, dy);

    if (distance_m < arrival_radius_m_) {
      ++index_;
      if (index_ >= waypoints_.size()) {
        if (loop_) {
          index_ = 0;
        } else {
          finished_ = true;
          RCLCPP_INFO(get_logger(), "%s 已走完全部航点，停住", model_name_.c_str());
          cmd_pub_->publish(cmd);
          return;
        }
      }
    }

    // 朝向误差。**必须用归一化的角度差** —— 直接相减的话，
    // 目标航向在 +π 而当前在 −π 时会算出 2π 的误差，车会朝反方向猛转一圈。
    // 这正是 ads_common::angles 存在的理由（P1 就下沉到公共包了）。
    // angle_diff(from, to) = 从 from 转到 to 的最短差，正值 = 左转。
    // 参数顺序**不能反** —— 反了误差符号就反，车会朝着背离航点的方向转。
    const double heading_error_rad = ads_common::angle_diff(yaw_rad_, std::atan2(dy, dx));

    // 朝向误差大时减速：cos 加权，90° 时归零（只转不走）。
    const double speed_scale =
      slow_down_when_turning_ ? std::max(0.0, std::cos(heading_error_rad)) : 1.0;

    // ⚠️ linear.x 是**车体系**前向速度（实测确认，见文件头）。
    cmd.linear.x = speed_mps_ * speed_scale;
    cmd.angular.z = heading_gain_ * heading_error_rad;
    cmd_pub_->publish(cmd);
  }

  std::string model_name_;
  std::vector<Waypoint> waypoints_;
  double speed_mps_{0.0};
  bool loop_{true};
  double arrival_radius_m_{1.0};
  double heading_gain_{2.0};
  bool slow_down_when_turning_{true};

  size_t index_{0};
  bool finished_{false};
  bool have_pose_{false};
  double x_m_{0.0};
  double y_m_{0.0};
  double yaw_rad_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gazebo_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<gazebo_bridge::NpcControllerNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("npc_controller"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
