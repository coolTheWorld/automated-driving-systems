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
//  control_node —— ads_control 的 ROS 包装层（SPEC §3.3 的 node/ 那一半）
//
//  订阅  /route/path            nav_msgs/Path        map 系，**transient_local**
//  订阅  /odom                  nav_msgs/Odometry    只取纵向车速
//  依赖  TF  map → base_link    自车位姿（**禁止手写变换矩阵**，SPEC §5）
//  发布  /vehicle_cmd           ads_msgs/VehicleCmd  @ control_rate_hz
//  发布  /control/debug         MarkerArray          RViz 可视化
//  发布  /control/diagnostics   DiagnosticArray      给记录脚本和 rqt 用
//
//  本文件里**没有任何控制算法**：Stanley、速度剖面、速度环全在 lib/ 里，
//  已被 71 个不需要 ROS 的 L1 用例覆盖。这里只做四件事：
//  读参数、把 ROS 消息翻译成 lib 的调用、把结果翻译回 ROS 消息、**把失败讲清楚**。
//
//  ⚠️ 「把失败讲清楚」是这一层最容易被做成静默失败的地方，理由和 map_node 一模一样：
//     没路径、TF 拿不到、里程计断了、偏离路径太远 —— **四种原因，一种症状（车不动）**。
//     所以每一环单独判、单独报，且报的是**这一环**的原因。
//
//  ⚠️ 降级动作是「减速停车」而不是「停发指令」。停发会让 bridge 的看门狗在
//     0.5 s 后接管刹停 —— 结果一样，但日志里会多一条「超过 0.5 s 没收到
//     /vehicle_cmd」，把排查方向引向"控制节点是不是崩了"。
//     控制器还活着的时候，就该由它自己说明白。
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ads_common/reference_line.hpp"
#include "ads_control/speed_controller.hpp"
#include "ads_control/speed_profile.hpp"
#include "ads_control/stanley.hpp"
#include "ads_msgs/msg/vehicle_cmd.hpp"

namespace ads_control
{

// 参考线几何在 P3-S1 下沉到了 ads_common —— 那里有两个消费者：
// 控制侧把投影结果当横向/航向误差，规划侧把同一个结果当 Frenet 的 (s, d)。
//
// 只引入用到的这几个类型，**不是 using namespace**：后者会把 ads_common 的
// 全部符号（angle_diff 等）一并拉进本命名空间，日后本包若添了同名函数，
// 重载决议会**静默**改变而不报错。
using ads_common::PathPoint;
using ads_common::PathProjection;
using ads_common::Pose2D;
using ads_common::ReferenceLine;

namespace
{

/// 控制器所处的状态。**每一种都要能从日志和 /control/diagnostics 上认出来** ——
/// 它们的表象全是「车不动」，分不清就只能靠猜。
enum class ControlState
{
  kTracking,     ///< 正常跟踪
  kNoPath,       ///< 还没收到路径，或收到的是空路径
  kNoTransform,  ///< 拿不到 map → base_link
  kOdomStale,    ///< /odom 断了或太旧
  kOffPath,      ///< 横向误差超过安全阈值
  kGoalReached,  ///< 已到终点（**不是故障**，但同样要刹住并说清楚）
};

const char * StateName(ControlState state)
{
  switch (state) {
    case ControlState::kTracking:
      return "TRACKING";
    case ControlState::kNoPath:
      return "NO_PATH";
    case ControlState::kNoTransform:
      return "NO_TRANSFORM";
    case ControlState::kOdomStale:
      return "ODOM_STALE";
    case ControlState::kOffPath:
      return "OFF_PATH";
    case ControlState::kGoalReached:
      return "GOAL_REACHED";
  }
  return "UNKNOWN";
}

/// 可视化元素的抬高量，单位 m。车道图在 0.05、路径在 0.10（见 map_node），
/// 控制调试量再往上一层，免得和它们深度冲突（z-fighting）。
constexpr double kDebugElevationM = 0.15;

/// 把 double 塞进 DiagnosticStatus 的 key/value。
diagnostic_msgs::msg::KeyValue MakeKeyValue(const std::string & key, double value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  // %.9g：9 位有效数字。够记录 mm 级的误差，又不至于把 CSV 撑爆。
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.9g", value);
  kv.value = buffer;
  return kv;
}

diagnostic_msgs::msg::KeyValue MakeKeyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

}  // namespace

/// @brief 路径跟踪控制节点：Stanley 横向 + 速度剖面/PI 纵向。
class ControlNode : public rclcpp::Node
{
public:
  ControlNode() : Node("control_node")
  {
    // -------------------------------------------------------------------------
    //  参数
    // -------------------------------------------------------------------------
    // ⚠️ **车辆能力**（轴距、转角、转向速率、巡航、加减速）全部来自
    //    config/vehicle_params.yaml，由 launch 搬运进来，**这里一个都不许写死**。
    //    默认值一律给 0 / 空，让 lib 的构造函数在启动时指名报错 ——
    //    给一个"看着合理"的默认值，等于把"launch 忘了传"变成一个跑得起来、
    //    只是开得不对的系统（SPEC §4.1「行为漂移」）。
    //    参数名保持与 YAML 层级一致，看到日志就知道去改哪一行。
    const double wheelbase_m = declare_parameter<double>("geometry.wheelbase_m", 0.0);
    const double max_steer_angle_rad = declare_parameter<double>("limits.max_steer_angle_rad", 0.0);
    const double max_steer_rate_rad_s =
      declare_parameter<double>("limits.max_steer_rate_rad_s", 0.0);
    const double cruise_speed_mps = declare_parameter<double>("limits.cruise_speed_mps", 0.0);
    const double max_accel_mps2 = declare_parameter<double>("limits.max_accel_mps2", 0.0);
    const double max_decel_mps2 = declare_parameter<double>("limits.max_decel_mps2", 0.0);

    // 控制器自己的调参，来自 config/control_params.yaml。
    const double lateral_gain_inv_s = declare_parameter<double>("lateral.gain", 0.0);
    const double soft_speed_mps = declare_parameter<double>("lateral.soft_speed_mps", 0.0);
    const int search_window = declare_parameter<int>("lateral.search_window", 30);
    const double longitudinal_kp = declare_parameter<double>("longitudinal.kp", 0.0);
    // K_i 的默认值是 **0.0 而且那是对的**（control.md §4.4）：被控对象自带积分，
    // 纯 P 对常值目标就没有稳态误差。只有实测出稳态误差、且说得出它来自哪个
    // 物理量时才往上加。这也是唯一一个"默认 0 是合法配置"的参数。
    const double longitudinal_ki = declare_parameter<double>("longitudinal.ki", 0.0);
    const double max_lateral_accel_mps2 =
      declare_parameter<double>("profile.max_lateral_accel_mps2", 0.0);

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    goal_stop_distance_m_ = declare_parameter<double>("goal.stop_distance_m", 0.5);
    // 横向误差超过它就判定"偏离路径"并刹停。
    // 依据：车道半宽 1.750，车宽半 0.900 → 越界线 0.850 m。取 1.5 m 是
    // "已经压到相邻车道/绿化带"的量级，比 CP-P2-B 的 0.30 m 判据宽 5 倍 ——
    // 这里要的是**兜底**，不是精度考核，太紧会在正常暂态里误触发。
    max_lateral_error_m_ = declare_parameter<double>("safety.max_lateral_error_m", 1.5);
    // /odom 停多久算断了。0.5 s = bridge 看门狗的阈值，取同一个数是有意的：
    // 两层同时判定失联，日志能互相印证。
    odom_timeout_s_ = declare_parameter<double>("safety.odom_timeout_s", 0.5);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    if (search_window <= 0) {
      throw std::invalid_argument(
        "lateral.search_window 必须为正，收到 " + std::to_string(search_window));
    }
    search_window_ = static_cast<std::size_t>(search_window);
    wheelbase_m_ = wheelbase_m;

    // lib 的构造函数会逐项校验并**指名**报错（"StanleyParams::gain_inv_s 必须是
    // 有限正数，收到 0.000000"）。异常在 main 里被接住并打成 FATAL —— 启动即失败，
    // 而不是带着一组零参数跑起来。
    stanley_ = std::make_unique<StanleyController>(
      StanleyParams{lateral_gain_inv_s, soft_speed_mps, max_steer_angle_rad, max_steer_rate_rad_s});
    speed_controller_ = std::make_unique<SpeedController>(
      SpeedControllerParams{longitudinal_kp, longitudinal_ki, max_accel_mps2, max_decel_mps2});
    profile_params_ =
      SpeedProfileParams{cruise_speed_mps, max_lateral_accel_mps2, max_accel_mps2, max_decel_mps2};
    max_decel_mps2_ = max_decel_mps2;

    // -------------------------------------------------------------------------
    //  接口
    // -------------------------------------------------------------------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // ⚠️ **必须与 map_node 的发布端 QoS 匹配，且必须是 transient_local。**
    //    /route/path 只在收到 /goal_pose 时发一次。本节点晚于那一刻启动
    //    （或中途重启）时，volatile 订阅会**永远收不到**已经发布的那条路径 ——
    //    症状是车停着不动、topic echo 空，而 map_node 日志写着"路径已发布：N 个点"。
    //    P1 阶段唯一的订阅者是 RViz，人总是先起 RViz 再点目标点，所以从没露过马脚。
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/route/path", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::Path::SharedPtr msg) { on_path(std::move(msg)); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(std::move(msg)); });

    cmd_pub_ = create_publisher<ads_msgs::msg::VehicleCmd>("/vehicle_cmd", rclcpp::QoS(10));
    debug_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/control/debug", rclcpp::QoS(1));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/control/diagnostics", rclcpp::QoS(10));

    // ⚠️ create_timer + 节点时钟，不是 create_wall_timer。
    //    use_sim_time=true 时控制周期必须跟着仿真钟走，否则仿真被暂停或
    //    RTF 偏离 1 时，控制频率与"车走多远"就对不上了（SPEC §5）。
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / control_rate_hz_),
      [this]() { on_timer(); });

    RCLCPP_INFO(
      get_logger(),
      "control_node 就绪：%.1f Hz，轴距 %.3f m，k_e=%.2f，k_soft=%.2f，K_p=%.2f，K_i=%.2f",
      control_rate_hz_, wheelbase_m_, lateral_gain_inv_s, soft_speed_mps, longitudinal_kp,
      longitudinal_ki);
  }

private:
  // ---------------------------------------------------------------------------
  //  回调
  // ---------------------------------------------------------------------------

  void on_path(nav_msgs::msg::Path::SharedPtr msg)
  {
    // map_node 在路由失败时会发一条**空 Path** 来清掉 RViz 里的旧路径
    // （map_node.cpp:331）。那不是错误，是"现在没有目标"，所以按 INFO 处理。
    if (msg->poses.size() < 2) {
      path_.reset();
      profile_.reset();
      hint_.reset();
      RCLCPP_INFO(
        get_logger(), "收到 %zu 个点的路径（上游清空或路由失败），转入减速停车", msg->poses.size());
      return;
    }

    std::vector<Pose2D> poses;
    poses.reserve(msg->poses.size());
    for (const auto & stamped : msg->poses) {
      poses.push_back(
        {stamped.pose.position.x, stamped.pose.position.y, tf2::getYaw(stamped.pose.orientation)});
    }

    try {
      auto path = std::make_unique<ReferenceLine>(std::move(poses));
      auto profile = std::make_unique<SpeedProfile>(*path, profile_params_);
      path_ = std::move(path);
      profile_ = std::move(profile);
    } catch (const std::invalid_argument & e) {
      // **丢掉旧路径而不是继续用它。** 新路径来了说明目标变了，
      // 沿着旧路径继续开是"看起来在正常工作"的错误行为 —— 宁可刹停。
      path_.reset();
      profile_.reset();
      hint_.reset();
      RCLCPP_ERROR(get_logger(), "路径不可用，已丢弃并转入减速停车：%s", e.what());
      return;
    }

    // 换了路径 = 换了一套弧长与设定值序列：
    //   * 最近点提示必须清掉，否则会拿旧路径的索引去新路径上查（越界或乱跳）。
    //   * 速度环的积分必须清掉，否则上一条路径末端攒下的积分会带进新路径。
    //   * 转角**不清**：车轮此刻物理上就在那个角度上，清成 0 反而是撒谎，
    //     而且会让转向速率限幅从一个错误的起点开始。
    hint_.reset();
    speed_controller_->reset();
    goal_announced_ = false;

    RCLCPP_INFO(
      get_logger(), "收到新路径：%zu 个点，%.2f m，剖面首点 %.3f m/s，末点 %.3f m/s",
      path_->points().size(), path_->length_m(), profile_->speeds_mps().front(),
      profile_->speeds_mps().back());
  }

  void on_odom(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 只取纵向车速。位姿一律走 TF（SPEC §5：禁止手写变换矩阵）——
    // /odom 的位姿在 odom 系，拿它当 map 系用会差整整一个 map→odom 变换，
    // 而本项目那一段**不是单位变换**（P1-S4 实测，见 CLAUDE.md）。
    measured_speed_mps_ = msg->twist.twist.linear.x;
    last_odom_time_ = now();
  }

  // ---------------------------------------------------------------------------
  //  控制周期
  // ---------------------------------------------------------------------------

  void on_timer()
  {
    const rclcpp::Time tick_start = now();

    // 首次进入没有上一拍，跳过一轮 —— 否则 dt 会是从 1970 年算起的一个巨大值，
    // 一拍就把速率限幅的余量全部用光。
    if (last_tick_.nanoseconds() == 0) {
      last_tick_ = tick_start;
      return;
    }
    const double dt_s = (tick_start - last_tick_).seconds();
    last_tick_ = tick_start;
    // dt ≤ 0：仿真被暂停或时钟倒流。跳过这一拍而不是把负数喂给限幅器
    // （lib 会抛异常，而在控制回调里抛异常等于让节点死掉）。
    if (dt_s <= 0.0) {
      return;
    }

    // ---- 逐环检查，每一环单独报 ----
    if (!path_ || !profile_) {
      degrade(ControlState::kNoPath, dt_s, "没有可用路径（还没点目标点，或上游路由失败）");
      return;
    }

    if (last_odom_time_.nanoseconds() == 0) {
      degrade(ControlState::kOdomStale, dt_s, "还没收到过 /odom");
      return;
    }
    const double odom_age_s = (tick_start - last_odom_time_).seconds();
    if (odom_age_s > odom_timeout_s_) {
      degrade(
        ControlState::kOdomStale, dt_s,
        "/odom 已经 " + std::to_string(odom_age_s) + " s 没更新（阈值 " +
          std::to_string(odom_timeout_s_) + " s）");
      return;
    }

    geometry_msgs::msg::TransformStamped ego;
    try {
      ego = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      degrade(
        ControlState::kNoTransform, dt_s,
        std::string("拿不到 ") + map_frame_ + " → " + base_frame_ + "：" + e.what());
      return;
    }

    // ---- 前轴换算 ----
    // ⚠️ Stanley 的推导对象是**前轴**，而 base_link 在后轴（Autoware 惯例）。
    //    漏掉这一步的症状是低速正常、高速持续外偏，而人会去调增益。
    //    S2 量化过：用后轴投影跑 R=8 的弯，稳态外偏 **1.182 m**。
    const Pose2D rear{
      ego.transform.translation.x, ego.transform.translation.y,
      tf2::getYaw(ego.transform.rotation)};
    const Pose2D front = front_axle_pose(rear, wheelbase_m_);

    const PathProjection projection = path_->project(front, hint_, search_window_);
    hint_ = projection.index;

    if (std::abs(projection.lateral_error_m) > max_lateral_error_m_) {
      degrade(
        ControlState::kOffPath, dt_s,
        "横向误差 " + std::to_string(projection.lateral_error_m) + " m 超过安全阈值 " +
          std::to_string(max_lateral_error_m_) + " m");
      return;
    }

    // ---- 到达终点 ----
    // ⚠️ **必须用弧长判，不能用横向误差判。** 车冲过终点之后投影被夹到端点，
    //    横向误差只剩偏移量的横向分量，会趋于 0 —— 一个只看横向误差的上层
    //    会认为"跟得很好"，而车正在开向天边。由 path_tracking 的
    //    ClampsBeyondBothEnds 用例钉死了这个行为。
    const double remaining_m = path_->length_m() - projection.s_m;
    // ⚠️ **到终点的直线距离要单独算，不能用 remaining_m。**
    //    车冲过终点之后投影被夹到末点，remaining_m 恒为 0 —— 冲过去 20 m
    //    和恰好停在终点长得一模一样。这正是 path_tracking 头文件里交给 S4 的
    //    那个陷阱（ClampsBeyondBothEnds 用例钉住的行为）。
    //    量在**前轴**上：路径末点是"前轴该停在哪"，因为整条路径都是按前轴跟踪的。
    const PathPoint & tail = path_->points().back();
    const double goal_distance_m = std::hypot(front.x_m - tail.x_m, front.y_m - tail.y_m);
    const bool goal_reached = remaining_m <= goal_stop_distance_m_;
    if (goal_reached && !goal_announced_) {
      goal_announced_ = true;
      RCLCPP_INFO(
        get_logger(), "已到达终点附近（剩余 %.3f m ≤ %.3f m），目标速度归零", remaining_m,
        goal_stop_distance_m_);
    }

    // ---- 控制律 ----
    // 正常段：目标速度和**目标加速度**都查剖面。
    //
    // ⚠️ 前馈不是可选项。剖面在入弯前和终点前是**斜坡**，而纯 P 跟踪斜坡的
    //    稳态误差 = 斜率/K_p = 3.0/1.0 = 3.0 m/s —— S4 首测就是这么冲过终点
    //    4.26 m 的，同一个原因还把最大横向加速度顶到 2.113（入弯超速 0.85 m/s）。
    //
    // 到终点后：目标速度 0，前馈直接给 **−max_decel**，也就是"承诺全力停住"。
    //    这不是硬塞一个数 —— 它与 goal.stop_distance_m 是**自洽**的：
    //    剖面在距终点 d 处的速度是 √(2·a_dec·d)，而从那个速度按 a_dec 刹停
    //    需要的距离恰好又是 d。d = 0.5 → 进入时 1.73 m/s，刹停 0.5 m，正好停在终点。
    //    若这里沿用剖面前馈（随速度衰减），末端会拖成一条长尾巴，
    //    表现为"车蹭到终点"。
    double target_speed_mps = 0.0;
    double feedforward_accel_mps2 = -max_decel_mps2_;
    if (!goal_reached) {
      target_speed_mps = profile_->speed_at(projection);
      feedforward_accel_mps2 = profile_->target_accel_at(projection);
    }

    const double steer_rad = stanley_->update(
      projection.heading_error_rad, projection.lateral_error_m, measured_speed_mps_, dt_s);
    const double accel_mps2 = speed_controller_->update(
      target_speed_mps, feedforward_accel_mps2, measured_speed_mps_, dt_s);

    publish_command(steer_rad, accel_mps2);
    const double cycle_ms = (now() - tick_start).seconds() * 1e3;
    publish_debug(front, projection, target_speed_mps);
    publish_diagnostics(
      goal_reached ? ControlState::kGoalReached : ControlState::kTracking, "", projection,
      target_speed_mps, feedforward_accel_mps2, steer_rad, accel_mps2, remaining_m, goal_distance_m,
      cycle_ms);
  }

  /// 降级：**减速停车**并把原因说清楚。
  void degrade(ControlState state, double dt_s, const std::string & reason)
  {
    // 转角靠 lib 自己的速率限幅**平滑回正**：给控制律喂零误差，
    // 它算出的目标转角就是 0，而限幅器负责按 0.5 rad/s 走过去。
    // 在这里另写一遍 `prev + clamp(0 − prev, ±rate·dt)` 也能做到，
    // 但那就等于把限幅逻辑抄了第二份，两份迟早漂移。
    const double steer_rad = stanley_->update(0.0, 0.0, measured_speed_mps_, dt_s);
    // 加速度直接给舒适减速度的下限，不走速度环 —— 降级时我们不信任
    // "目标速度"这个量本身（可能压根没有路径），拿它去闭环没有意义。
    const double accel_mps2 = -max_decel_mps2_;
    speed_controller_->reset();

    publish_command(steer_rad, accel_mps2);

    // 限流打印：50 Hz 下不限流会把日志刷成滚屏，真正有用的那一行反而看不见。
    // 状态**变化**时立刻打一条，之后每 2 s 一条。
    if (state != last_state_) {
      RCLCPP_WARN(get_logger(), "[%s] 减速停车：%s", StateName(state), reason.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "[%s] 减速停车：%s", StateName(state), reason.c_str());
    }

    PathProjection empty;
    publish_diagnostics(
      state, reason, empty, 0.0, accel_mps2, steer_rad, accel_mps2, 0.0, 0.0, 0.0);
  }

  void publish_command(double steer_rad, double accel_mps2)
  {
    ads_msgs::msg::VehicleCmd cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = base_frame_;
    cmd.steer_angle_rad = steer_rad;
    cmd.accel_mps2 = accel_mps2;
    cmd_pub_->publish(cmd);
  }

  // ---------------------------------------------------------------------------
  //  可视化与诊断
  // ---------------------------------------------------------------------------

  void publish_debug(
    const Pose2D & front, const PathProjection & projection, double target_speed_mps)
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker base;
    base.header.frame_id = map_frame_;
    base.header.stamp = now();
    base.ns = "control";
    base.action = visualization_msgs::msg::Marker::ADD;
    base.pose.orientation.w = 1.0;

    // ① 前轴中心（黄球）。它应当**始终贴着车头**，而不是车身中心 ——
    //    肉眼一看就知道前轴换算有没有生效。
    visualization_msgs::msg::Marker front_marker = base;
    front_marker.id = 0;
    front_marker.type = visualization_msgs::msg::Marker::SPHERE;
    front_marker.scale.x = front_marker.scale.y = front_marker.scale.z = 0.35;
    front_marker.color.r = 1.0F;
    front_marker.color.g = 0.9F;
    front_marker.color.a = 0.9F;
    front_marker.pose.position.x = front.x_m;
    front_marker.pose.position.y = front.y_m;
    front_marker.pose.position.z = kDebugElevationM;
    markers.markers.push_back(front_marker);

    // ② 路径上的最近点（洋红球）。
    visualization_msgs::msg::Marker foot = front_marker;
    foot.id = 1;
    foot.color.r = 1.0F;
    foot.color.g = 0.1F;
    foot.color.b = 1.0F;
    foot.pose.position.x = projection.x_m;
    foot.pose.position.y = projection.y_m;
    markers.markers.push_back(foot);

    // ③ 两者之间的连线 = **横向误差本身**。线越长误差越大，不用读数字。
    visualization_msgs::msg::Marker error_line = base;
    error_line.id = 2;
    error_line.type = visualization_msgs::msg::Marker::LINE_LIST;
    error_line.scale.x = 0.10;
    error_line.color.r = 1.0F;
    error_line.color.g = 0.3F;
    error_line.color.a = 1.0F;
    geometry_msgs::msg::Point a;
    a.x = front.x_m;
    a.y = front.y_m;
    a.z = kDebugElevationM;
    geometry_msgs::msg::Point b;
    b.x = projection.x_m;
    b.y = projection.y_m;
    b.z = kDebugElevationM;
    error_line.points.push_back(a);
    error_line.points.push_back(b);
    markers.markers.push_back(error_line);

    // ④ 数字，浮在车头上方。目标速度和实测速度放一起 ——
    //    分成两个显示项的话，"跟不上"这件事要靠人两处对读才看得出来。
    visualization_msgs::msg::Marker text = base;
    text.id = 3;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.scale.z = 0.6;
    text.color.r = text.color.g = text.color.b = 1.0F;
    text.color.a = 1.0F;
    text.pose.position.x = front.x_m;
    text.pose.position.y = front.y_m;
    text.pose.position.z = 2.5;
    char buffer[160];
    std::snprintf(
      buffer, sizeof(buffer), "e = %+.3f m\npsi = %+.3f rad\nv = %.2f / %.2f m/s",
      projection.lateral_error_m, projection.heading_error_rad, measured_speed_mps_,
      target_speed_mps);
    text.text = buffer;
    markers.markers.push_back(text);

    debug_pub_->publish(markers);
  }

  void publish_diagnostics(
    ControlState state, const std::string & reason, const PathProjection & projection,
    double target_speed_mps, double feedforward_accel_mps2, double steer_rad, double accel_mps2,
    double remaining_m, double goal_distance_m, double cycle_ms)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "ads_control/control_node";
    status.hardware_id = "ego_vehicle";
    status.level = (state == ControlState::kTracking || state == ControlState::kGoalReached)
                     ? diagnostic_msgs::msg::DiagnosticStatus::OK
                     : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message =
      reason.empty() ? StateName(state) : (std::string(StateName(state)) + ": " + reason);

    status.values.push_back(MakeKeyValue("state", std::string(StateName(state))));
    status.values.push_back(MakeKeyValue("lateral_error_m", projection.lateral_error_m));
    status.values.push_back(MakeKeyValue("heading_error_rad", projection.heading_error_rad));
    status.values.push_back(MakeKeyValue("curvature_inv_m", projection.curvature_inv_m));
    status.values.push_back(MakeKeyValue("path_s_m", projection.s_m));
    status.values.push_back(MakeKeyValue("path_remaining_m", remaining_m));
    // 与 path_remaining_m 的区别见 on_timer 里的说明：冲过终点后前者恒为 0，后者继续增长。
    status.values.push_back(MakeKeyValue("goal_distance_m", goal_distance_m));
    status.values.push_back(MakeKeyValue("target_speed_mps", target_speed_mps));
    status.values.push_back(MakeKeyValue("feedforward_accel_mps2", feedforward_accel_mps2));
    status.values.push_back(MakeKeyValue("measured_speed_mps", measured_speed_mps_));
    status.values.push_back(MakeKeyValue("steer_angle_rad", steer_rad));
    status.values.push_back(MakeKeyValue("accel_mps2", accel_mps2));
    status.values.push_back(MakeKeyValue("cycle_time_ms", cycle_ms));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(status);
    diag_pub_->publish(array);

    last_state_ = state;
  }

  // ---- 参数 ----
  double wheelbase_m_{0.0};
  double control_rate_hz_{50.0};
  double goal_stop_distance_m_{0.5};
  double max_lateral_error_m_{1.5};
  double odom_timeout_s_{0.5};
  double max_decel_mps2_{0.0};
  std::size_t search_window_{30};
  std::string map_frame_;
  std::string base_frame_;
  SpeedProfileParams profile_params_{0.0, 0.0, 0.0, 0.0};

  // ---- 算法（全部来自 lib/，本文件不含任何控制逻辑）----
  std::unique_ptr<StanleyController> stanley_;
  std::unique_ptr<SpeedController> speed_controller_;
  std::unique_ptr<ReferenceLine> path_;
  std::unique_ptr<SpeedProfile> profile_;
  std::optional<std::size_t> hint_;

  // ---- 运行状态 ----
  double measured_speed_mps_{0.0};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  ControlState last_state_{ControlState::kNoPath};
  bool goal_announced_{false};

  // ---- 接口 ----
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ads_msgs::msg::VehicleCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ads_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ads_control::ControlNode>());
  } catch (const std::exception & e) {
    // 参数缺失/非法在这里落地。**打成 FATAL 并以非零码退出**，
    // 而不是让异常裸奔出去 —— 裸奔的话终端上只有一行 terminate called，
    // 完全看不出是哪个参数的问题，而 lib 明明已经把参数名写在异常里了。
    RCLCPP_FATAL(rclcpp::get_logger("control_node"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
