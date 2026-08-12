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
//  运动规划节点 —— ROS 包装层（P3-S4）
//
//  这一层**只做翻译**：ROS 消息 ↔ lib 的调用。规划算法一行都不在这里，
//  所以它没有自己的单元测试也不可怕 —— 该测的都在 lib 的 L1 用例里
//  （frenet / quintic / collision / lattice / speed_profile / trajectory）。
//
//    /route/path            (nav_msgs/Path, transient_local)  ──┐
//    /perception/obstacles  (ads_msgs/ObstacleArray)          ──┼─▶ /planning/trajectory
//    TF map → base_link                                       ──┘   (ads_msgs/Trajectory)
//
//  ⚠️ **没有障碍物话题也要正常工作。** P3-S5 才有真值障碍物发布器，
//     在那之前（以及 P5 感知挂掉时）本节点必须按"没有障碍物"规划，
//     而不是等在那儿。「等一个可能永远不来的话题」是很常见的死法。
// =============================================================================

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ads_msgs/msg/obstacle_array.hpp"
#include "ads_msgs/msg/trajectory.hpp"
#include "ads_planning/trajectory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ads_planning
{

class PlanningNode : public rclcpp::Node
{
public:
  PlanningNode() : Node("planning_node")
  {
    // -------------------------------------------------------------------------
    //  参数：全部来自 YAML，**不给算法参数留默认值**
    // -------------------------------------------------------------------------
    // 坐标系名和话题名给默认值（换环境时才动），但**算法参数不给** ——
    // 给了默认值，上层忘记加载 YAML 时就会静默用一组没人审过的数跑起来，
    // 而车照样能开，只是行为不是配置里写的那样。
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const double rate_hz = declare_parameter<double>("planning_rate_hz", 10.0);

    params_.lattice.max_lateral_offset_m = declare_parameter<double>("lateral.max_offset_m");
    params_.lattice.lateral_offset_step_m = declare_parameter<double>("lateral.offset_step_m");
    params_.lattice.min_horizon_m = declare_parameter<double>("longitudinal.min_horizon_m");
    params_.lattice.max_horizon_m = declare_parameter<double>("longitudinal.max_horizon_m");
    params_.lattice.horizon_step_m = declare_parameter<double>("longitudinal.horizon_step_m");
    params_.lattice.resample_step_m = declare_parameter<double>("trajectory.resample_step_m");
    params_.lattice.safety_margin_m = declare_parameter<double>("safety.margin_m");
    params_.stop_margin_m = declare_parameter<double>("safety.stop_margin_m");
    // 障碍物列表多久没更新就算过期，s。取 1.0 = 感知标称周期（0.1 s）的 10 倍：
    // 感知自己的航迹删除窗口是 0.5 s（max_misses），1.0 s 已是整条流水线死透。
    // 调小到 0.2 → 感知偶发慢一拍就误刹；调大到 5 → 动态目标按 20 m 前的位置判碰撞。
    obstacle_timeout_s_ = declare_parameter<double>("safety.obstacle_timeout_s", 1.0);
    params_.lattice.weight_offset = declare_parameter<double>("cost.weight_offset");
    params_.lattice.weight_curvature = declare_parameter<double>("cost.weight_curvature");
    params_.lattice.weight_clearance = declare_parameter<double>("cost.weight_clearance");
    params_.lattice.weight_consistency = declare_parameter<double>("cost.weight_consistency");

    // 车辆能力与外廓来自 vehicle_params.yaml（SPEC §4.1 的单一来源），
    // 由 launch 搬运进来。**不在 planning_params.yaml 里再抄一份** ——
    // 抄一份的症状是「改了车宽之后碰撞检查还用旧值」，而没有任何一层报错。
    params_.lattice.vehicle_length_m = declare_parameter<double>("vehicle.length_m");
    params_.lattice.vehicle_width_m = declare_parameter<double>("vehicle.width_m");
    params_.lattice.rear_overhang_m = declare_parameter<double>("vehicle.rear_overhang_m");
    params_.speed.cruise_speed_mps = declare_parameter<double>("speed.cruise_speed_mps");
    params_.speed.max_lateral_accel_mps2 =
      declare_parameter<double>("speed.max_lateral_accel_mps2");
    params_.speed.max_accel_mps2 = declare_parameter<double>("speed.max_accel_mps2");
    params_.speed.max_decel_mps2 = declare_parameter<double>("speed.max_decel_mps2");

    // -------------------------------------------------------------------------
    //  接口
    // -------------------------------------------------------------------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // ⚠️ **必须 transient_local**，与 map_node 的发布端匹配。
    //    /route/path 只在收到 /goal_pose 时发一次，本节点晚于那一刻启动
    //    （或中途重启）时，volatile 订阅会**永远收不到**已发布的那条路径 ——
    //    症状是车停着不动、topic echo 空，而 map_node 日志写着"路径已发布"。
    //    这个 bug 从 P1 活到 P2-S4 才被抓住，别再犯一次。
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/route/path", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::Path::SharedPtr msg) { on_path(std::move(msg)); });

    obstacle_sub_ = create_subscription<ads_msgs::msg::ObstacleArray>(
      "/perception/obstacles", rclcpp::QoS(10),
      [this](ads_msgs::msg::ObstacleArray::SharedPtr msg) { obstacles_ = std::move(msg); });

    // 轨迹是**周期性**的，深度 1 的 volatile 就够：晚到的订阅者等 100 ms
    // 就有新的一条，不需要 transient_local（那会让新订阅者先收到一条过期轨迹）。
    trajectory_pub_ =
      create_publisher<ads_msgs::msg::Trajectory>("/planning/trajectory", rclcpp::QoS(1));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/planning/diagnostics", rclcpp::QoS(10));

    // ⚠️ create_timer + 节点时钟，不是 create_wall_timer。
    //    use_sim_time=true 时规划周期必须跟着仿真钟走（SPEC §5）。
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz), [this]() { tick(); });

    RCLCPP_INFO(
      get_logger(), "planning_node 已启动：%.1f Hz，前视 %.1f–%.1f m，横向 ±%.2f m", rate_hz,
      params_.lattice.min_horizon_m, params_.lattice.max_horizon_m,
      params_.lattice.max_lateral_offset_m);
  }

private:
  void on_path(nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "/route/path 只有 %zu 个点，忽略", msg->poses.size());
      return;
    }
    std::vector<ads_common::Pose2D> poses;
    poses.reserve(msg->poses.size());
    for (const auto & stamped : msg->poses) {
      poses.push_back(
        {stamped.pose.position.x, stamped.pose.position.y, tf2::getYaw(stamped.pose.orientation)});
    }
    try {
      reference_line_ = std::make_unique<ads_common::ReferenceLine>(std::move(poses));
    } catch (const std::invalid_argument & e) {
      // 构造失败（重合点、非有限值）**不是**静默丢弃就完事的：
      // 它意味着上游采样有 bug，而保留旧参考线会让症状延后出现在别处。
      reference_line_.reset();
      RCLCPP_ERROR(get_logger(), "参考线构造失败：%s", e.what());
      return;
    }
    // 换了路径就必须忘掉上一周期的横向选择 —— 那个值是相对**旧**参考线的，
    // 拿它去做新参考线的一致性项等于凭空引入一个偏好。
    previous_offset_m_.reset();
    RCLCPP_INFO(
      get_logger(), "收到参考线：%zu 个点，%.1f m", reference_line_->points().size(),
      reference_line_->length_m());
  }

  /// @brief 把 ObstacleArray 翻译成规划层的矩形。
  ///
  /// @note 只取**静态**障碍物之外的一切也照样当静态处理 —— P3 的 lattice
  ///       是纯路径的，表达不了"等它过去"（planning.md §4.3）。
  ///       把动态障碍物当静态是**保守**方向（它现在在哪就绕哪），
  ///       但那不是正确处理，P6/P7 要加时间维度。这里如实按静态处理，
  ///       不做任何假装能处理动态的事。
  std::vector<Rectangle> current_obstacles() const
  {
    std::vector<Rectangle> rectangles;
    if (!obstacles_) {
      return rectangles;
    }
    rectangles.reserve(obstacles_->obstacles.size());
    for (const auto & obstacle : obstacles_->obstacles) {
      Rectangle rectangle;
      rectangle.center_x_m = obstacle.pose.position.x;
      rectangle.center_y_m = obstacle.pose.position.y;
      rectangle.heading_rad = tf2::getYaw(obstacle.pose.orientation);
      rectangle.length_m = obstacle.size_m.x;
      rectangle.width_m = obstacle.size_m.y;
      rectangles.push_back(rectangle);
    }
    return rectangles;
  }

  void tick()
  {
    if (!reference_line_) {
      publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::WARN, "还没收到 /route/path", {});
      return;
    }

    geometry_msgs::msg::TransformStamped ego;
    try {
      ego = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      publish_diagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        std::string("拿不到 ") + map_frame_ + " → " + base_frame_ + "：" + e.what(), {});
      return;
    }

    // ---- 障碍物过期检查（2026-08-12 复检补上的缺口）------------------------
    //
    // ⚠️ 原来这里什么都没有：回调只存指针、header.stamp 从未被读。
    //    感知进程死掉后，规划器**永远**拿着冻结的障碍物列表做碰撞检查 ——
    //    P5 起有 4 m/s 的动态目标，冻结 1 s = 按 4 m 前的位置判碰撞。
    //    对比：control_node 对 /odom 有超时降级，这里却对 /perception/obstacles
    //    没有 —— 同一类输入失效，两种待遇。
    //
    // 过期 ⟹ **不发轨迹**（与"规划抛异常"同一条降级路径）：下游 control_node
    // 落进 NO_PATH 分支刹停并保持。不选"继续按旧障碍物规划"—— 那是拿过期
    // 数据继续开；也不选"当没有障碍物"—— 那更糟。
    //
    // ⚠️ **从未收到过任何 ObstacleArray 时不判**（obstacles_ 为空指针直接放行）。
    //    这不是疏忽是边界：CP-P2-B 回归基线（obstacles:=none dynamic:=none）
    //    里整个话题**没有发布者**，那一跑必须照常规划。「链路存在但断了」
    //    与「链路本来就不存在」是两回事，前者才是故障。
    //    代价（如实记）：感知**从启动起就没发过一条**的场景抓不到 ——
    //    那看起来与基线跑一模一样。要区分得靠 launch 层告诉规划器
    //    「这一跑该有感知」，P6 接 /perception 进规划闭环时一并做。
    if (obstacles_) {
      const double age_s = (now() - rclcpp::Time(obstacles_->header.stamp)).seconds();
      if (age_s > obstacle_timeout_s_) {
        publish_diagnostics(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "障碍物列表已 " + std::to_string(age_s) +
            " s 没有更新 —— 感知/真值链路死了？本周期不发轨迹（下游会刹停）",
          {});
        return;
      }
    }

    const ads_common::Pose2D ego_pose{
      ego.transform.translation.x, ego.transform.translation.y,
      tf2::getYaw(ego.transform.rotation)};

    PlanResult result;
    try {
      // 起点用**后轴**位姿：lattice 的起点是车辆本体状态。
      // （控制侧的 Stanley 才用前轴 —— 那是控制律的推导对象，两者不是一回事。）
      const FrenetState start = to_frenet(*reference_line_, ego_pose, projection_hint_);
      result = plan(*reference_line_, start, current_obstacles(), params_, previous_offset_m_);
    } catch (const std::exception & e) {
      // 规划抛异常时**不发轨迹**，让下游走"没有轨迹"的降级分支（刹停）。
      // 发一条上一周期的旧轨迹看着更"连续"，但那是拿过期数据继续开。
      publish_diagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string("规划失败：") + e.what(), {});
      return;
    }

    if (result.status == PlanStatus::kRouteExhausted || result.points.empty()) {
      publish_diagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::OK, "参考线已走完，不再发布轨迹", result);
      return;
    }

    previous_offset_m_ = result.lateral_offset_m;
    publish_trajectory(result);
    publish_diagnostics(
      result.status == PlanStatus::kStopping ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                             : diagnostic_msgs::msg::DiagnosticStatus::OK,
      result.status == PlanStatus::kStopping ? "前方绕不过去，按停车轨迹执行" : "正常", result);
  }

  void publish_trajectory(const PlanResult & result)
  {
    ads_msgs::msg::Trajectory msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.status = result.status == PlanStatus::kStopping ? ads_msgs::msg::Trajectory::STATUS_STOPPING
                                                        : ads_msgs::msg::Trajectory::STATUS_OK;
    msg.lateral_offset_m = result.lateral_offset_m;
    msg.points.reserve(result.points.size());
    for (const TrajectoryPoint & point : result.points) {
      ads_msgs::msg::TrajectoryPoint out;
      out.x_m = point.x_m;
      out.y_m = point.y_m;
      out.heading_rad = point.heading_rad;
      out.curvature_1pm = point.curvature_inv_m;
      out.s_m = point.s_m;
      out.speed_mps = point.speed_mps;
      out.accel_mps2 = point.accel_mps2;
      msg.points.push_back(out);
    }
    trajectory_pub_->publish(msg);
  }

  void publish_diagnostics(
    unsigned char level, const std::string & message, const PlanResult & result)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = static_cast<signed char>(level);
    status.name = "ads_planning";
    status.message = message;

    const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      status.values.push_back(pair);
    };
    // 「27 条候选全被淘汰」和「一条候选都没生成」是完全不同的故障 ——
    // 两个计数都报出去，否则现场只能看到"车停了"。
    add("candidate_count", std::to_string(result.candidate_count));
    add("blocked_count", std::to_string(result.blocked_count));
    add("lateral_offset_m", std::to_string(result.lateral_offset_m));
    add("point_count", std::to_string(result.points.size()));
    if (result.status == PlanStatus::kStopping) {
      add("stop_clearance_m", std::to_string(result.stop_clearance_m));
    }

    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  std::string map_frame_;
  std::string base_frame_;
  PlanParams params_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<ads_msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<ads_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<ads_common::ReferenceLine> reference_line_;
  ads_msgs::msg::ObstacleArray::SharedPtr obstacles_;
  double obstacle_timeout_s_{1.0};
  /// 上一周期的横向选择，喂给代价函数的一致性项。换路径时必须清空。
  std::optional<double> previous_offset_m_;
  /// 最近点投影的局部搜索提示。**不是性能优化是正确性要求** ——
  /// 环线上自车前后必然有几何距离相近的两段路径，全局最近点会在两者间跳。
  std::optional<std::size_t> projection_hint_;
};

}  // namespace ads_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ads_planning::PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
