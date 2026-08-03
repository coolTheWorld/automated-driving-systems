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
//  fake_vehicle —— 不需要 Gazebo 的假车，**测试夹具，不是仿真数据源**
//
//  订阅  /vehicle_cmd   ads_msgs/VehicleCmd
//  发布  /clock         rosgraph_msgs/Clock      ← **它是本闭环的时间源**
//  发布  /odom          nav_msgs/Odometry        odom 系
//  发布  TF             odom → base_link
//
//  存在的理由：`map_node + control_node + 假车` 构成一个**完全不需要 GPU**
//  的闭环，于是可以进 CI（SPEC §8 的 L3-G）。它验的是**节点接线** ——
//  话题名、QoS、TF 链、参数装配、时序 —— 而不是控制律，也不是真物理。
//
//      L1 单元测试（S2/S3）  验**控制律**：毫秒级，不起 ROS
//      本闭环（S5）          验**接线**：不需要 GPU，秒级，进 CI
//      CP-P2-B（S4）         验**真物理**：要 Gazebo + GPU + 人
//
//  三层各验各的。这一层能抓到的，正是 L1 结构上抓不到、而 CP-P2-B 又太贵的那类：
//  **`/route/path` 的 QoS 从 P1 一直错到 P2-S4 都没人发现**，因为唯一的订阅者
//  是 RViz 而人总是先起 RViz 再点目标点。有了这一层，那种 bug 一次推送就红。
//
//  ⚠️ **它不是 SIM_SOURCES 的一员，不要往 stack.launch.py 里加。**
//     没有传感器、没有轮胎、没有质量惯量 —— 它只会让"车按运动学走"这件事成立。
//     拿它当仿真器用，等于把 CP-P2-B 发现的那些真物理问题全部隐藏掉
//     （转向执行机构 1.2 s 滞后就是 L1 结构上不可能预测的，见 control.md §3.9）。
//
//  ⚠️ **本节点自己必须 `use_sim_time=false`** —— 它是时钟的**来源**。
//     开了就成了鸡生蛋：等一个自己还没发布的时钟。
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "ads_msgs/msg/vehicle_cmd.hpp"

namespace ads_control
{

/// @brief 运动学自行车模型的 ROS 包装 —— 定步长推进，并**充当 /clock 的来源**。
class FakeVehicle : public rclcpp::Node
{
public:
  // ⚠️ **在构造时就把 use_sim_time 钉成 false**，而不是靠 launch 传。
  //    它是 /clock 的来源，开了就是等一个自己还没发布的时钟 —— 而那个错误的
  //    表现是"节点起来了、话题都在、就是一条消息都不发"，最难查的那一类。
  //    parameter_overrides 的优先级高于 launch 的 parameters=，所以外面写错了也拦得住。
  FakeVehicle()
  : Node(
      "fake_vehicle",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", false)}))
  {
    // 车辆参数由 launch 从 vehicle_params.yaml 搬进来，这里一个数都不写死 ——
    // 与 control_node 同一条规矩。默认 0 会让下面的校验指名报错。
    wheelbase_m_ = declare_parameter<double>("geometry.wheelbase_m", 0.0);
    max_steer_rad_ = declare_parameter<double>("limits.max_steer_angle_rad", 0.0);
    max_speed_mps_ = declare_parameter<double>("limits.max_speed_mps", 0.0);

    x_m_ = declare_parameter<double>("initial.x_m", 0.0);
    y_m_ = declare_parameter<double>("initial.y_m", 0.0);
    heading_rad_ = declare_parameter<double>("initial.heading_rad", 0.0);

    // 每拍推进多少**仿真**时间。取 0.02 与控制周期一致：
    // 于是 control_node 的定时器每拍恰好触发一次，闭环是 1:1 的，
    // 不存在"一拍里跑了两次控制"或"跳过一拍"这种不确定性。
    sim_step_s_ = declare_parameter<double>("sim_step_s", 0.02);
    // 仿真时间相对墙钟的倍率。**这是 CI 时长与确定性之间的唯一旋钮**：
    //   调大 → 跑得快，但 control_node 每拍只有 sim_step/rtf 的墙钟时间可用，
    //          单拍耗时（实测 3–7 ms）一旦超过它，闭环就跟不上、结果不再稳定；
    //   调小 → 稳，但 CI 变慢。
    // 取 3.0：每拍留 6.7 ms 墙钟，是实测单拍耗时的约 2 倍余量。
    real_time_factor_ = declare_parameter<double>("real_time_factor", 3.0);

    if (wheelbase_m_ <= 0.0 || max_steer_rad_ <= 0.0 || max_speed_mps_ <= 0.0) {
      throw std::invalid_argument(
        "fake_vehicle: 车辆参数缺失（轴距/最大转角/最大速度必须为正）——"
        "launch 是不是没从 vehicle_params.yaml 搬进来？");
    }
    if (sim_step_s_ <= 0.0 || real_time_factor_ <= 0.0) {
      throw std::invalid_argument("fake_vehicle: sim_step_s 与 real_time_factor 必须为正");
    }

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cmd_sub_ = create_subscription<ads_msgs::msg::VehicleCmd>(
      "/vehicle_cmd", rclcpp::QoS(10), [this](ads_msgs::msg::VehicleCmd::SharedPtr msg) {
        // 与 gazebo_bridge 同样的做法：非有限值直接丢弃并告警，
        // 不能让它进到积分里 —— 一拍 NaN 会让位姿永久变成 NaN，车"消失"。
        if (!std::isfinite(msg->steer_angle_rad) || !std::isfinite(msg->accel_mps2)) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "收到非有限指令，已丢弃");
          return;
        }
        steer_rad_ = std::clamp(msg->steer_angle_rad, -max_steer_rad_, max_steer_rad_);
        accel_mps2_ = msg->accel_mps2;
      });

    // ⚠️ 必须是 **wall** timer：本节点是时钟的来源，用节点时钟就是等自己。
    const auto wall_period = std::chrono::duration<double>(sim_step_s_ / real_time_factor_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(wall_period), [this]() { step(); });

    RCLCPP_INFO(
      get_logger(), "假车就绪：起点 (%.2f, %.2f, %.3f rad)，步长 %.3f s × %.1f 倍速", x_m_, y_m_,
      heading_rad_, sim_step_s_, real_time_factor_);
  }

private:
  void step()
  {
    // ---- 先推进时钟，再动车 ----
    // 顺序有讲究：先发 /clock，下游（control_node）这一拍读到的 now() 才与
    // 随后发布的 odom/TF 时间戳一致。反过来的话下游会用**上一拍**的时间
    // 去处理这一拍的数据，dt 恒偏差一个步长。
    sim_time_s_ += sim_step_s_;
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = to_ros_time(sim_time_s_);
    clock_pub_->publish(clock_msg);

    // ---- 运动学自行车模型（前向欧拉，与 L1 夹具同一套式子）----
    // 导数全部取**更新前**的状态，这才是显式欧拉。
    const double speed_before_mps = speed_mps_;
    const double heading_before_rad = heading_rad_;
    x_m_ += speed_before_mps * std::cos(heading_before_rad) * sim_step_s_;
    y_m_ += speed_before_mps * std::sin(heading_before_rad) * sim_step_s_;
    heading_rad_ =
      heading_before_rad + speed_before_mps * std::tan(steer_rad_) / wheelbase_m_ * sim_step_s_;
    // 速度下限夹到 0：`VehicleCmd` 没有挡位字段，负加速度只能理解为减速，
    // 与 gazebo_bridge 的速度设定值下限一致（倒车不支持，control.md §7）。
    speed_mps_ = std::clamp(speed_before_mps + accel_mps2_ * sim_step_s_, 0.0, max_speed_mps_);

    publish_state();
  }

  void publish_state()
  {
    const rclcpp::Time stamp = to_ros_time(sim_time_s_);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading_rad_);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_m_;
    tf.transform.translation.y = y_m_;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_m_;
    odom.pose.pose.position.y = y_m_;
    odom.pose.pose.orientation = tf.transform.rotation;
    // 控制器只用 twist.linear.x（车身系纵向速度）。位姿一律走 TF。
    odom.twist.twist.linear.x = speed_mps_;
    odom_pub_->publish(odom);
  }

  static rclcpp::Time to_ros_time(double seconds)
  {
    const auto nanos = static_cast<int64_t>(seconds * 1e9);
    return rclcpp::Time(nanos, RCL_ROS_TIME);
  }

  double wheelbase_m_{0.0};
  double max_steer_rad_{0.0};
  double max_speed_mps_{0.0};
  double sim_step_s_{0.02};
  double real_time_factor_{3.0};

  double x_m_{0.0};
  double y_m_{0.0};
  double heading_rad_{0.0};
  double speed_mps_{0.0};
  double steer_rad_{0.0};
  double accel_mps2_{0.0};
  double sim_time_s_{0.0};

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<ads_msgs::msg::VehicleCmd>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ads_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ads_control::FakeVehicle>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("fake_vehicle"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
