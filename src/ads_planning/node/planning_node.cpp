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
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ads_msgs/msg/obstacle_array.hpp"
#include "ads_msgs/msg/predicted_trajectory_array.hpp"
#include "ads_msgs/msg/trajectory.hpp"
#include "ads_planning/longitudinal.hpp"
#include "ads_planning/speed_profile.hpp"
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
    // 投影局部搜索半宽（段数）。30 段 × 0.5 m 采样 ≈ ±15 m —— 一个规划周期
    // （0.1 s × 5.5 m/s ≈ 0.6 m）远走不出去。与 control 的 lateral.search_window
    // 取同一个数。调小 → 车被扰动甩远后投影追不上；调大 → 退化向全局搜索。
    const int projection_window = declare_parameter<int>("projection.search_window", 30);
    if (projection_window <= 0) {
      throw std::invalid_argument("projection.search_window 必须为正");
    }
    projection_search_window_ = static_cast<std::size_t>(projection_window);
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

    // ---- 行为决策（P7-S3）。参数依据见 behavior.md §5 与 planning_params.yaml。
    BehaviorParams behavior_params;
    behavior_params.corridor_half_m = declare_parameter<double>("behavior.corridor_half_m");
    behavior_params.stand_off_m = declare_parameter<double>("behavior.stand_off_m");
    behavior_params.yield_margin_m = declare_parameter<double>("behavior.yield_margin_m");
    behavior_params.time_margin_s = declare_parameter<double>("behavior.time_margin_s");
    behavior_params.sigma_inflation_cap_m =
      declare_parameter<double>("behavior.sigma_inflation_cap_m");
    // front_offset 是**推导量**（后轴到车头面），从车辆几何算，不单独配 ——
    // 配两遍的症状是「改了车长之后跟停间距还按旧车头算」，没有一层会报错。
    behavior_params.front_offset_m =
      params_.lattice.vehicle_length_m - params_.lattice.rear_overhang_m;
    // 阻挡阈值同为推导量（= 车半宽 + 安全间距 − 最大横向偏移，本配置 0.55）。
    // 它与 lattice 的可行性判断共用同一组参数 —— 改了任何一个，两处一起变。
    behavior_params.blocking_half_m = params_.lattice.vehicle_width_m / 2.0 +
                                      params_.lattice.safety_margin_m -
                                      params_.lattice.max_lateral_offset_m;
    const int release_cycles = static_cast<int>(declare_parameter<int>("behavior.release_cycles"));
    arbiter_ = std::make_unique<BehaviorArbiter>(behavior_params, release_cycles);
    prediction_timeout_s_ = declare_parameter<double>("behavior.prediction_timeout_s", 1.0);

    // ---- 启动告知（P7 事实 12 的收口：这两个参数被注释承诺过两次而不存在）。
    // launch 层告诉规划器「这一跑**应当**有感知/预测」：为 true 时对应输入
    // 从未到达也按链路故障处理（不发轨迹 → 下游刹停），堵住
    // 「感知从启动起就没发过一条，看起来与基线跑一模一样」的洞。
    expect_perception_ = declare_parameter<bool>("expect_perception", false);
    expect_prediction_ = declare_parameter<bool>("expect_prediction", false);

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

    // P7-S3 起本节点消费预测（决策六）：stack.launch 的 prediction 开关
    // 从「纯旁路」变成了规划的输入源之一。没有预测时横穿判定为空、
    // 只剩跟车（感知近边）—— 降级方向正确，不是故障。
    prediction_sub_ = create_subscription<ads_msgs::msg::PredictedTrajectoryArray>(
      "/prediction/trajectories", rclcpp::QoS(10),
      [this](ads_msgs::msg::PredictedTrajectoryArray::SharedPtr msg) {
        predictions_ = std::move(msg);
      });

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
    // hint 是相对**旧**参考线的段号，换路径必须清掉 —— 只有首次跟踪
    // 新路径才该做全局搜索（reference_line.hpp 的使用约定）。
    projection_hint_.reset();
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

  /// @brief 组装行为层输入并仲裁一次。
  ///
  /// 输入翻译（ROS 消息 → lib 类型）都在这里，与 current_obstacles() 同一层职责。
  BehaviorArbiter::Decision run_behavior(const FrenetState & start)
  {
    // 感知快照 → TargetBox（走廊判定用近边，所以要长宽）。
    std::vector<TargetBox> targets;
    if (obstacles_) {
      targets.reserve(obstacles_->obstacles.size());
      for (const auto & obstacle : obstacles_->obstacles) {
        targets.push_back(
          {obstacle.id, obstacle.pose.position.x, obstacle.pose.position.y, obstacle.size_m.x,
           obstacle.size_m.y});
      }
    }
    // 预测 → 假设（多假设按任一冲突合成 —— 事实 11；
    // 概率与 existence 都不进安全判定，所以这里根本不搬运它们）。
    //
    // ⚠️ **STATIC 模型的假设不进横穿判定**（S4 实测改的）：STATIC 没有运动
    //    方向，它的威胁是**位置性的** —— 归阻挡判定（FOLLOW）与 lattice 的
    //    静态准入管，两条路都已存在。放进横穿判定的后果是起步律椭圆
    //    （2σ(3s)=13.9 m）把**路侧静物**变成永久让行：层 2 里每一根杆件、
    //    每一面墙都会让车停下，且 S05 实测行人穿完站在路肩时还拖着一个
    //    尾巴 YIELD（⑨ 数到 5 次的元凶之一）。
    std::vector<PredictionHypothesis> hypotheses;
    if (predictions_) {
      hypotheses.reserve(predictions_->trajectories.size());
      for (const auto & trajectory : predictions_->trajectories) {
        if (trajectory.model == ads_msgs::msg::PredictedTrajectory::MODEL_STATIC) {
          continue;
        }
        PredictionHypothesis hypothesis;
        hypothesis.obstacle_id = trajectory.obstacle_id;
        hypothesis.points.reserve(trajectory.points.size());
        for (const auto & point : trajectory.points) {
          hypothesis.points.push_back({point.t_s, point.x_m, point.y_m, point.sigma_cross_m});
        }
        hypotheses.push_back(std::move(hypothesis));
      }
    }
    // 无约束剖面（时间标注的输入）：参考线全线的曲率限速 + 终点归零。
    std::vector<double> arc_lengths_m;
    arc_lengths_m.reserve(reference_line_->points().size());
    for (const auto & point : reference_line_->points()) {
      arc_lengths_m.push_back(point.s_m);
    }
    const SpeedProfile profile(*reference_line_, params_.speed);
    return arbiter_->decide(
      *reference_line_, start.s_m, targets, hypotheses, arc_lengths_m, profile.speeds_mps());
  }

  void tick()
  {
    // 周期耗时用**墙钟**量：这是「规划算得够不够快」（CP-P3-B 判据 #7，
    // ≤ 50 ms = 10 Hz 的半周期），不是算法时序（SPEC §5 禁的是后者）。
    // ⚠️ 2026-08-12 复检发现：plan 表里这条判据从 P3 验收起就**没有任何
    //    instrumentation 在量它** —— 判据表有第 7 行，脚本与节点都没有对应物。
    tick_started_ = std::chrono::steady_clock::now();
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
    } else if (expect_perception_) {
      // launch 声明了「这一跑该有感知」而一条都没来过（P7-S3 收口的洞）。
      publish_diagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "expect_perception=true 但 /perception/obstacles 从未到达 —— 链路没接上？"
        "本周期不发轨迹（下游会刹停）",
        {});
      return;
    }

    // ---- 预测的过期/缺席检查（与障碍物同一套逻辑，P7-S3）--------------------
    if (predictions_) {
      const double age_s = (now() - rclcpp::Time(predictions_->header.stamp)).seconds();
      if (age_s > prediction_timeout_s_) {
        publish_diagnostics(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "预测列表已 " + std::to_string(age_s) +
            " s 没有更新 —— 预测链路死了？本周期不发轨迹（下游会刹停）",
          {});
        return;
      }
    } else if (expect_prediction_) {
      publish_diagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "expect_prediction=true 但 /prediction/trajectories 从未到达 —— 链路没接上？"
        "本周期不发轨迹（下游会刹停）",
        {});
      return;
    }

    const ads_common::Pose2D ego_pose{
      ego.transform.translation.x, ego.transform.translation.y,
      tf2::getYaw(ego.transform.rotation)};

    PlanResult result;
    try {
      // 起点用**后轴**位姿：lattice 的起点是车辆本体状态。
      // （控制侧的 Stanley 才用前轴 —— 那是控制律的推导对象，两者不是一回事。）
      // ---- 局部搜索的 hint 回写（2026-08-12 复检修复）--------------------
      //
      // ⚠️ projection_hint_ 原来**从未被赋值**：声明与注释都写着「正确性
      //    要求」，to_frenet 却每周期都拿着 nullopt 做全局最近点搜索 ——
      //    恰是 reference_line.hpp 与 planning.md §3.1 点名的错误
      //    （环线上自车前后有几何相近的两段，全局搜索会在两者间跳；
      //    control_node 为同一问题每拍回写 hint，本节点漏了）。
      //    可追溯的失效：目标在同车道后方时路线首末共线，末端过冲米级
      //    即跳回 s≈0 重发全程轨迹；或越出车道 >1.75 m 时投影跳到对向
      //    车道、每拍抛异常、永久「规划失败」。
      //
      // 先显式投影一次拿到段号，再把同一个 hint 交给 to_frenet ——
      // 同一函数、同一 hint，两次结果一致，多花一次窗口搜索（微秒级）。
      const ads_common::PathProjection projection =
        reference_line_->project(ego_pose, projection_hint_, projection_search_window_);
      projection_hint_ = projection.index;
      const FrenetState start = to_frenet(*reference_line_, ego_pose, projection_hint_);

      // ---- 行为仲裁（P7-S3）------------------------------------------------
      // 时间标注吃**参考线全线**的无约束剖面：只依赖几何，每周期重算一遍
      // ~0.1 ms（600 点）。不必缓存 —— 缓存要多管一个「参考线换了没」的状态。
      decision_ = run_behavior(start);
      result = plan(
        *reference_line_, start, current_obstacles(), params_, previous_offset_m_,
        &decision_->constraint);
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
    // tick 的所有路径都以本函数收尾，所以在这里结算周期耗时恰好覆盖全部分支。
    if (tick_started_) {
      last_cycle_ms_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - *tick_started_)
          .count();
      tick_started_.reset();
    }
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
    add("cycle_ms", std::to_string(last_cycle_ms_));
    add("candidate_count", std::to_string(result.candidate_count));
    add("blocked_count", std::to_string(result.blocked_count));
    add("lateral_offset_m", std::to_string(result.lateral_offset_m));
    add("point_count", std::to_string(result.points.size()));
    if (result.status == PlanStatus::kStopping) {
      add("stop_clearance_m", std::to_string(result.stop_clearance_m));
    }
    // ---- 行为状态（P7-S3）。S4 的判据 ⑨（状态不振荡）直接数这个字段。 ----
    if (decision_.has_value()) {
      add("behavior_state", BehaviorStateName(decision_->state));
      add(
        "behavior_follow_id",
        decision_->follow.has_value() ? std::to_string(decision_->follow->id) : "-");
      add("behavior_crossing_count", std::to_string(decision_->crossings.size()));
      // 横穿冲突的目标 id 列表（S4 排查振荡用；平时也留着 —— 「YIELD 对着谁」
      // 是现场第一问，没有它只能靠猜）。
      std::string crossing_ids;
      for (const auto & crossing : decision_->crossings) {
        crossing_ids += (crossing_ids.empty() ? "" : ",") + std::to_string(crossing.id) + "@" +
                        std::to_string(crossing.s_lo_m);
      }
      add("behavior_crossing_ids", crossing_ids.empty() ? "-" : crossing_ids);
      add(
        "behavior_stop_at_s_m", decision_->constraint.stop_at_s_m.has_value()
                                  ? std::to_string(*decision_->constraint.stop_at_s_m)
                                  : "-");
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
  ads_msgs::msg::PredictedTrajectoryArray::SharedPtr predictions_;
  rclcpp::Subscription<ads_msgs::msg::PredictedTrajectoryArray>::SharedPtr prediction_sub_;
  std::unique_ptr<BehaviorArbiter> arbiter_;
  /// 最近一次仲裁结果（诊断用）。tick 早退分支里它保持上一周期的值 ——
  /// 那些分支不发轨迹，行为字段只是旁证，不做安全判断。
  std::optional<BehaviorArbiter::Decision> decision_;
  bool expect_perception_{false};
  bool expect_prediction_{false};
  double prediction_timeout_s_{1.0};
  double obstacle_timeout_s_{1.0};
  std::optional<std::chrono::steady_clock::time_point> tick_started_;
  double last_cycle_ms_{0.0};
  /// 上一周期的横向选择，喂给代价函数的一致性项。换路径时必须清空。
  std::optional<double> previous_offset_m_;
  /// 最近点投影的局部搜索提示。**不是性能优化是正确性要求** ——
  /// 环线上自车前后必然有几何距离相近的两段路径，全局最近点会在两者间跳。
  std::optional<std::size_t> projection_hint_;
  std::size_t projection_search_window_{30};
};

}  // namespace ads_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ads_planning::PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
