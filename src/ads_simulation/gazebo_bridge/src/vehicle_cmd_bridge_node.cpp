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
//  vehicle_cmd_bridge —— /vehicle_cmd（规范指令）→ Gazebo 的 Twist
//
//  这是整个栈里**第一条反方向**的链路。前面七条都是 Gazebo → ROS，
//  这条是 ROS → Gazebo：算法（或键盘）说想怎么开，仿真器照做。
//
//  为什么需要一层转换，不能让上游直接发 Twist
//  ------------------------------------------
//  SPEC §4.1 规定 /vehicle_cmd 是「前轮转角 rad + 纵向加速度 m/s²」——
//  这是**车辆物理量**，和具体执行器无关。而 Gazebo 的 AckermannSteering
//  插件吃的是 Twist「速度 m/s + 横摆角速度 rad/s」，CARLA 吃的又是
//  「油门/刹车/方向盘」三个 [0,1] 归一化量。
//
//  让上游直接发 Twist 的后果：控制器代码里就会出现 Gazebo 的执行器约定，
//  换到 CARLA 时整个控制器都要改。所以转换必须发生在 bridge 层，
//  carla_bridge 做同样的事、消费同样的 /vehicle_cmd。
//
//  两个转换的数学
//  --------------
//  1. 加速度 → 速度：对指令加速度积分得到速度设定值。
//     插件不接受加速度，只接受速度，所以这一步躲不掉。
//  2. 转角 → 横摆角速度：自行车模型 ω = v·tan(δ) / L
//     （L = 轴距）。插件内部再用 δ = atan(L·ω/v) 反算回转角，
//     所以只要我们按这个公式给，插件拿到的就是我们想要的转角。
//
//     ⚠️ v = 0 时 ω 恒为 0 —— **静止时打方向车不会转**。
//        这不是 bug，阿克曼转向的真车原地打方向车也不动。
// =============================================================================

// include 分段规则（cpplint 强制）：C 标准库 → C++ 标准库 → 第三方，段间留空行。
// .clang-format 设了 IncludeBlocks: Preserve，只在段内排序，不会合并三段。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <ads_msgs/msg/vehicle_cmd.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ads_simulation
{

class VehicleCmdBridge : public rclcpp::Node
{
public:
  VehicleCmdBridge() : Node("vehicle_cmd_bridge")
  {
    // ---- 车辆参数（由 launch 从 config/vehicle_params.yaml 传入）----
    // 默认值全为 0：launch 忘了传参时车不会动，是"功能失效但行为安全"。
    // 给一个看似合理的默认值反而危险 —— 车会按错误的限值跑起来，
    // 而且没人会发现参数没传进来。
    wheelbase_m_ = declare_parameter<double>("geometry.wheelbase_m", 0.0);
    max_steer_rad_ = declare_parameter<double>("limits.max_steer_angle_rad", 0.0);
    max_speed_mps_ = declare_parameter<double>("limits.max_speed_mps", 0.0);
    max_accel_ = declare_parameter<double>("limits.max_accel_mps2", 0.0);
    max_decel_ = declare_parameter<double>("limits.max_decel_mps2", 0.0);
    emergency_decel_ = declare_parameter<double>("limits.emergency_decel_mps2", 0.0);

    // 看门狗：多久没收到 /vehicle_cmd 就当作"驾驶员失联"，自动刹车。
    //
    // ⚠️ 这是安全逻辑，没有开关，只有阈值（SPEC §11 禁止把安全逻辑放在
    //    可被关掉的分支里）。缺了它的后果很实际：teleop 进程崩了或者
    //    ssh 断了，车会**带着最后一条加速指令一直开下去**。
    // 调大：网络抖动时不易误刹车，但失联后继续行驶的时间更长。
    // 调小：更安全，但指令发布频率稍有波动就会被误判为失联。
    // 0.5 s 对应 teleop 的 20 Hz 发布率有 10 帧余量。
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.5);

    // 速度设定值允许超前实测速度多少。这是积分器的抗饱和（anti-windup）。
    //
    // 没有它的症状：车顶着墙时实际速度是 0，但设定值仍按指令加速度一路积到
    // max_speed；等车脱离障碍的瞬间会**猛地窜出去**。
    // 调大：接近纯开环，抗饱和失效；调小：正常加速时设定值被压住，加速变迟钝。
    // 1.0 m/s 的依据：插件按 max_accel 跟随设定值，正常加速时滞后约 0.2-0.5 m/s。
    setpoint_lead_mps_ = declare_parameter<double>("speed_setpoint_lead_mps", 1.0);

    const double rate_hz = declare_parameter<double>("control_rate_hz", 50.0);

    // 输出话题是 Gazebo 专用的中间话题，**不是** SPEC §4.1 的规范话题。
    // 名字里带 gazebo 前缀就是为了让人一眼看出它不是对外契约的一部分：
    // carla_bridge 那边对应的会是完全不同的东西（油门/刹车/方向盘）。
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/gazebo/cmd_vel", 10);

    sub_ = create_subscription<ads_msgs::msg::VehicleCmd>(
      "/vehicle_cmd", 10, std::bind(&VehicleCmdBridge::on_cmd, this, std::placeholders::_1));

    // 订阅里程计只为了抗饱和，不参与控制律本身。
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        measured_speed_mps_ = msg->twist.twist.linear.x;
      });

    // ⚠️ 用 create_timer + 节点时钟，不是 create_wall_timer。
    // 这里做的是对**仿真时间**的积分（v += a·dt），必须走仿真钟；
    // 用墙上时钟的话 RTF 偏离 1 时积出来的速度就是错的（SPEC §3.3）。
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz),
      std::bind(&VehicleCmdBridge::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "指令桥接已启动：/vehicle_cmd → /gazebo/cmd_vel  "
      "轴距 %.3f m  最大转角 %.3f rad  限速 %.2f m/s",
      wheelbase_m_, max_steer_rad_, max_speed_mps_);
    if (wheelbase_m_ <= 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "轴距为 0 —— launch 没传 geometry.wheelbase_m，转向换算无法进行，车只会直行。");
    }
  }

private:
  static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

  void on_cmd(const ads_msgs::msg::VehicleCmd::ConstSharedPtr msg)
  {
    // ---- 任务 4.3：指令限幅 ----
    // 上游发什么都得挡住。控制器有 bug、参数没调好、消息里是 NaN ——
    // 这些都会发生，而 bridge 是最后一道能拦住它的地方。

    // NaN 必须单独判：NaN 参与任何比较都返回 false，clamp 会原样放行，
    // 一路传到 Gazebo 让物理引擎解算出 NaN 位姿，车直接消失。
    //
    // ⚠️⚠️ **被丢弃的指令不许喂狗**（2026-08-12 复检发现的缺陷）：
    //    原来 `last_cmd_time_ = now()` 是本函数第一行，于是上游持续发 NaN 时
    //    （本仓库已实际发生过两次的故障形态），每条被丢弃的坏指令都在刷新
    //    看门狗，0.5 s 超时永不触发，车以**闩存的旧指令**加速到限速一直开 ——
    //    恰是看门狗注释声称要防止的后果。语义上，持续 NaN 流等价于失联：
    //    没有任何**有效**指令到达，就该按失联刹停。
    //    所以喂狗挪到校验之后：只有真指令才算「联系还在」。
    if (!std::isfinite(msg->steer_angle_rad) || !std::isfinite(msg->accel_mps2)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "收到非有限的指令（转角 %.3f，加速度 %.3f），已丢弃",
        msg->steer_angle_rad, msg->accel_mps2);
      return;
    }
    last_cmd_time_ = now();

    const double steer_raw = msg->steer_angle_rad;
    steer_rad_ = clamp(steer_raw, -max_steer_rad_, max_steer_rad_);
    if (std::abs(steer_raw - steer_rad_) > 1e-9) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "转角指令 %.3f rad 超出 ±%.3f rad，已截断为 %.3f rad",
        steer_raw, max_steer_rad_, steer_rad_);
    }

    // 加速度的下限用 emergency_decel 而不是 max_decel，这是有意的：
    //
    //   max_decel(3.0)       是**舒适性**约束，给规划器和常规控制器用
    //   emergency_decel(5.0) 是车辆的**物理**能力，安全模块紧急制动时用
    //
    // bridge 是车辆接口，它该模拟的是「这辆车能做到什么」，不是「谁被允许
    // 这么做」。在这里按 3.0 截断的话，将来的安全模块**物理上无法紧急制动** ——
    // 那是个比放过一条超舒适区指令严重得多的问题。
    // 越过舒适区但没到物理极限的指令：放行，但告警。
    const double accel_raw = msg->accel_mps2;
    accel_mps2_ = clamp(accel_raw, -emergency_decel_, max_accel_);
    if (std::abs(accel_raw - accel_mps2_) > 1e-9) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "加速度指令 %.3f m/s^2 超出物理极限 [%.3f, %.3f]，已截断为 %.3f", accel_raw,
        -emergency_decel_, max_accel_, accel_mps2_);
    } else if (accel_mps2_ < -max_decel_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "减速度 %.3f m/s^2 超过舒适限值 %.3f —— 仅紧急制动才应如此", accel_mps2_, max_decel_);
    }
  }

  void on_timer()
  {
    const rclcpp::Time t_now = now();

    // 首次进入时没有上一拍，跳过一轮，避免把一个巨大的 dt 积进去。
    if (last_tick_.nanoseconds() == 0) {
      last_tick_ = t_now;
      return;
    }
    double dt = (t_now - last_tick_).seconds();
    last_tick_ = t_now;
    // 仿真刚启动或被暂停再恢复时 dt 可能异常。负数直接跳过；
    // 过大的截断到一个控制周期，免得一拍积出几米每秒的速度跳变。
    if (dt <= 0.0) {
      return;
    }
    dt = std::min(dt, 0.1);

    // ---- 看门狗 ----
    double accel = accel_mps2_;
    double steer = steer_rad_;
    const bool have_cmd = last_cmd_time_.nanoseconds() != 0;
    if (!have_cmd || (t_now - last_cmd_time_).seconds() > cmd_timeout_s_) {
      // 失联：按舒适减速度刹停并回正。用 max_decel 而不是 emergency_decel ——
      // 失联本身不等于遇到危险，没必要每次都急刹。
      accel = -max_decel_;
      steer = 0.0;
      if (have_cmd) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "超过 %.2f s 没收到 /vehicle_cmd，自动减速停车",
          cmd_timeout_s_);
      }
    }

    // ---- 加速度 → 速度设定值 ----
    speed_setpoint_mps_ += accel * dt;

    // 抗饱和：设定值不许比实测速度超前太多（顶着墙时防止一路积到限速）。
    const double lead_cap = measured_speed_mps_ + setpoint_lead_mps_;
    // 下限固定为 0：VehicleCmd 没有挡位字段，负加速度只能理解为"减速"，
    // 无法与"倒车"区分（详见 ads_teleop 的说明）。所以刹到 0 为止。
    speed_setpoint_mps_ = clamp(speed_setpoint_mps_, 0.0, std::min(max_speed_mps_, lead_cap));

    // ---- 转角 → 横摆角速度（自行车模型）----
    // ω = v·tan(δ)/L。v=0 时 ω=0，静止无法转向 —— 物理事实，见文件头。
    const double yaw_rate =
      (wheelbase_m_ > 0.0) ? speed_setpoint_mps_ * std::tan(steer) / wheelbase_m_ : 0.0;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = speed_setpoint_mps_;
    twist.angular.z = yaw_rate;
    pub_->publish(twist);
  }

  double wheelbase_m_{0.0};
  double max_steer_rad_{0.0};
  double max_speed_mps_{0.0};
  double max_accel_{0.0};
  double max_decel_{0.0};
  double emergency_decel_{0.0};
  double cmd_timeout_s_{0.5};
  double setpoint_lead_mps_{1.0};

  // 最近一条（已限幅的）指令
  double steer_rad_{0.0};
  double accel_mps2_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  double speed_setpoint_mps_{0.0};
  double measured_speed_mps_{0.0};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<ads_msgs::msg::VehicleCmd>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ads_simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ads_simulation::VehicleCmdBridge>());
  rclcpp::shutdown();
  return 0;
}
