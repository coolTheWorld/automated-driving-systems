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
//  map_node —— ads_map 的 ROS 包装层（SPEC §3.3 的 node/ 那一半）
//
//  发布  /map/lane_graph   visualization_msgs/MarkerArray  map 系，**transient_local**
//  发布  /route/path       nav_msgs/Path                   map 系
//  订阅  /goal_pose        geometry_msgs/PoseStamped       RViz 的 2D Goal Pose
//  依赖  TF   map → base_link（起点位姿）
//
//  本文件里**没有任何算法**：解析、建图、最近车道、Dijkstra 全在 lib/ 里，
//  已经被 27 个不需要 ROS 的 L1 用例覆盖过了。这里只做四件事：
//  读参数、把 ROS 消息翻译成 lib 的调用、把结果翻译回 ROS 消息、把失败讲清楚。
//
//  「把失败讲清楚」不是客套话。这条链路上每一环失败的**表象都是同一个**：
//  RViz 里没有路径。TF 没起来、自车在草地上、目标点点太远、两点不可达 ——
//  四种原因，一种症状。所以每一环都单独判、单独报，且报的是**这一环**的原因。
// =============================================================================

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ads_map/lane_graph.hpp"
#include "ads_map/opendrive_parser.hpp"
#include "ads_map/routing.hpp"

namespace ads_map
{

namespace
{

/// 标记的颜色。纯显示量，不是物理参数，所以直接定在这里而不进配置文件。
struct Rgba
{
  float r, g, b, a;
};

/// 常规道路的车道：青蓝。
constexpr Rgba kNormalLaneColor{0.25F, 0.70F, 1.00F, 0.85F};
/// 路口里的连接道路：橙。与常规道路分色，是为了**肉眼能数出路口的连接数** ——
/// 一个 T 型路口应当有 6 条橙线，少了就是 laneLink 漏了。
constexpr Rgba kJunctionLaneColor{1.00F, 0.60F, 0.10F, 0.85F};
/// 行驶方向箭头：白，压在车道线上。
constexpr Rgba kDirectionArrowColor{1.00F, 1.00F, 1.00F, 0.9F};

/// 车道中心线的显示线宽，单位 m。取 0.15 与地图里的道路中心标线同宽，
/// 这样它盖在路面贴图上时看着像标线而不是一条悬空的粗管子。
constexpr double kLaneLineWidthM = 0.15;

/// 方向箭头的长度，单位 m。短车道（连接道路最短 15 m）上也要放得下，
/// 所以还会再夹一次「不超过车道长度的 1/3」。
constexpr double kDirectionArrowLengthM = 3.0;
constexpr double kArrowShaftDiameterM = 0.20;
constexpr double kArrowHeadDiameterM = 0.50;
constexpr double kArrowHeadLengthM = 0.80;

/// 抬高量，单位 m。地图里的路面在 z=0.005、厚 0.010，直接画在 z=0 会和路面
/// **深度冲突**（z-fighting），表现是线条随视角闪烁。
/// 路径要压在车道图之上，所以抬得更高一点。
constexpr double kLaneGraphElevationM = 0.05;
constexpr double kRoutePathElevationM = 0.10;

/// 区间短于这个值就只出一个采样点，单位 m。
/// 1 µm 远小于任何有意义的路径段，但足以避免「起点终点重合时输出两个同样的点」。
constexpr double kDegenerateSpanM = 1e-6;

}  // namespace

/// @brief 加载 OpenDRIVE 地图，发布车道图，并对 /goal_pose 做全局路由。
class MapNode : public rclcpp::Node
{
public:
  MapNode() : Node("map_node")
  {
    // ---- 参数 ----
    // 地图默认从**本包的 share** 里找，而不是假设工作区挂在 /workspace。
    // 与 gazebo_bridge 装 vehicle_params.yaml 是同一个理由：ROS 包安装后必须
    // 自包含，云端那侧的路径不一样。装进去的是同一个文件（symlink-install），
    // 所以 SPEC §4.1 的地图单一来源仍然成立。
    const std::string default_map_file =
      ament_index_cpp::get_package_share_directory("ads_map") + "/maps/campus.xodr";
    map_file_ = declare_parameter<std::string>("map_file", default_map_file);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // 采样步长。**按参考线弧长 s 走，不是按车道中心线弧长**，所以弯道上
    // 实际点距会比这个值大最多 14.6%（见 Road::lane_arc_length 的说明）。
    // 显示用途下无所谓；真要等距采样应当在 P3 的规划里做，不是在这里。
    // 调大：点更少、RViz 更流畅，但弯道会显出折线感。
    // 调小：线更顺，但一条 250 m 的车道点数线性上升。
    marker_step_m_ = declare_parameter<double>("marker_sample_step_m", 1.0);
    path_step_m_ = declare_parameter<double>("path_sample_step_m", 0.5);

    // 起点容差：自车离最近车道多远还算「在路上」。
    // 依据是半车道宽 1.75 m + 半车宽 ≈ 0.9 m + 余量。
    // 调大：自车停在草地上也会被静默吸附到某条车道，路径看起来正常但起点是错的。
    // 调小：正常的横向误差就会让规划失败，用户只会觉得「点了没反应」。
    max_start_distance_m_ = declare_parameter<double>("max_start_distance_m", 5.0);
    // 终点容差更松：点击是人手操作。但 10 m 已接近三个车道宽，
    // 再大就该怀疑用户点的根本不是路，而不是替他猜。
    max_goal_distance_m_ = declare_parameter<double>("max_goal_distance_m", 10.0);

    // 是否用 2D Goal Pose 拖出来的朝向来约束终点车道。
    // 默认关：多数时候用户只是想「指个地方」，随手一点的朝向是噪声，
    // 拿它当硬约束会让点击频繁失败。想指定「从哪个方向到达」时再打开。
    // ⚠️ 起点的朝向**永远**参与匹配，那不是可选项 —— 见 on_goal() 里的说明。
    use_goal_heading_ = declare_parameter<bool>("use_goal_heading", false);

    // ---- 加载地图 ----
    // 失败直接让异常穿出构造函数，由 main 捕获后**非零退出**。
    // 不 catch 住继续跑：一个加载不了地图的 map_node 活着的唯一效果，
    // 就是让「话题列表里有它」这件事掩盖住真正的故障。
    graph_ = std::make_unique<LaneGraph>(load_opendrive(map_file_));
    RCLCPP_INFO(
      get_logger(), "地图已加载：%s（%zu 条车道，%zu 条连接）", map_file_.c_str(),
      graph_->node_count(), graph_->edge_count());

    // ---- TF ----
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- 话题 ----
    // transient_local 是**必需的**，不是优化：车道图是「发一次就不变」的数据，
    // 用默认的 volatile QoS，RViz 只要晚于本节点启动就永远收不到，
    // 症状是「话题确实在、RViz 一片空白」。P0a 已经踩过一次同类 QoS 坑。
    lane_graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/map/lane_graph", rclcpp::QoS(1).transient_local().reliable());
    route_pub_ = create_publisher<nav_msgs::msg::Path>("/route/path", rclcpp::QoS(1).reliable());
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(1),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr goal) { on_goal(*goal); });

    lane_graph_pub_->publish(build_lane_graph_markers());
    RCLCPP_INFO(get_logger(), "车道图已发布到 /map/lane_graph，等待 /goal_pose");
  }

private:
  // ---------------------------------------------------------------------------
  //  采样：把 (车道, s 区间) 变成一串**行驶方向**上的位姿
  // ---------------------------------------------------------------------------
  /// @brief 沿车道中心线采样。
  /// @param lane 车道。
  /// @param from_s_m 起点的参考线弧长。
  /// @param to_s_m 终点的参考线弧长。**可以小于 from_s_m** —— 正编号车道逆 s 行驶。
  /// @param step_m 采样步长（参考线 s 上的）。
  /// @return 位姿序列，按**行驶顺序**排列，朝向已翻转为行驶朝向。
  std::vector<Pose2D> sample_lane(
    const LaneId & lane, double from_s_m, double to_s_m, double step_m) const
  {
    const Road & road = graph_->road_map().road(lane.road_id);
    const double span_m = std::fabs(to_s_m - from_s_m);
    const double direction = (to_s_m >= from_s_m) ? 1.0 : -1.0;

    const auto pose_at = [&road, &lane, from_s_m, direction](double offset_m) {
      Pose2D pose = road.lane_center_pose_at(lane.lane_id, from_s_m + direction * offset_m);
      if (lane.lane_id > 0) {
        pose.heading_rad += M_PI;  // 正编号车道逆 s 行驶，行驶朝向要翻 180°
      }
      return pose;
    };

    std::vector<Pose2D> poses;
    if (span_m <= kDegenerateSpanM) {
      poses.push_back(pose_at(0.0));  // 退化区间：一个点，不是两个重合的点
      return poses;
    }

    // 把区间**等分**成 count 段，而不是「按步长累加、最后一步夹到端点」。
    //
    // 累加+夹取的写法有一个只在特定输入下才现形的 bug：span/step 恰好是整数时，
    // 浮点上它可能是 80.00000000000001，ceil 多算一步，于是最后**两个**采样点
    // 都被夹到 span —— 一对重合点。RViz 里完全看不出来，下游按弧长参数化时
    // 却会除以零。本项目实测踩到过：nearest_lane 的三分法把终点细化成
    // s = 40.000000000000007，正好触发。
    //
    // 等分写法天然没有这个问题：i=0 精确是起点，i=count 精确是终点，
    // 段长恒为 span/count ≤ step，也不需要任何夹取。
    const int count = static_cast<int>(std::ceil(span_m / step_m));
    poses.reserve(static_cast<std::size_t>(count) + 1);
    for (int i = 0; i <= count; ++i) {
      poses.push_back(pose_at(span_m * static_cast<double>(i) / static_cast<double>(count)));
    }
    return poses;
  }

  static geometry_msgs::msg::Point to_point(const Pose2D & pose, double z_m)
  {
    geometry_msgs::msg::Point point;
    point.x = pose.x_m;
    point.y = pose.y_m;
    point.z = z_m;
    return point;
  }

  // ---------------------------------------------------------------------------
  //  车道图的 MarkerArray
  // ---------------------------------------------------------------------------
  visualization_msgs::msg::MarkerArray build_lane_graph_markers() const
  {
    visualization_msgs::msg::MarkerArray markers;
    const rclcpp::Time stamp = now();

    for (std::size_t i = 0; i < graph_->node_count(); ++i) {
      const LaneNode & lane_node = graph_->node(i);
      const Road & road = graph_->road_map().road(lane_node.id.road_id);
      const Rgba color = (road.junction_id < 0) ? kNormalLaneColor : kJunctionLaneColor;
      const std::vector<Pose2D> poses =
        sample_lane(lane_node.id, lane_node.entry_s_m, lane_node.exit_s_m, marker_step_m_);

      visualization_msgs::msg::Marker line;
      line.header.frame_id = map_frame_;
      line.header.stamp = stamp;
      line.ns = "lane_centerline";
      line.id = static_cast<int>(i);
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      // 必须显式给单位四元数：默认构造出来的是全 0，RViz 会刷
      // "Uninitialized quaternion, assuming identity" 的警告刷到看不见真正的日志。
      line.pose.orientation.w = 1.0;
      line.scale.x = kLaneLineWidthM;
      line.color.r = color.r;
      line.color.g = color.g;
      line.color.b = color.b;
      line.color.a = color.a;
      for (const Pose2D & pose : poses) {
        line.points.push_back(to_point(pose, kLaneGraphElevationM));
      }
      markers.markers.push_back(std::move(line));

      // 方向箭头。**没有它就看不出这是一张有向图** —— 而「有向」正是 S3 花
      // 最大力气保证的性质。一条没有箭头的车道线，无向图画出来一模一样。
      const std::size_t middle = poses.size() / 2;
      const double arrow_length_m = std::fmin(kDirectionArrowLengthM, lane_node.length_m / 3.0);
      const Pose2D & base = poses[middle];
      Pose2D tip = base;
      tip.x_m += arrow_length_m * std::cos(base.heading_rad);
      tip.y_m += arrow_length_m * std::sin(base.heading_rad);

      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = map_frame_;
      arrow.header.stamp = stamp;
      arrow.ns = "lane_direction";
      arrow.id = static_cast<int>(i);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.orientation.w = 1.0;
      // ARROW 用两点表示时：scale.x = 杆径，scale.y = 头径，scale.z = 头长。
      arrow.scale.x = kArrowShaftDiameterM;
      arrow.scale.y = kArrowHeadDiameterM;
      arrow.scale.z = kArrowHeadLengthM;
      arrow.color.r = kDirectionArrowColor.r;
      arrow.color.g = kDirectionArrowColor.g;
      arrow.color.b = kDirectionArrowColor.b;
      arrow.color.a = kDirectionArrowColor.a;
      arrow.points.push_back(to_point(base, kLaneGraphElevationM));
      arrow.points.push_back(to_point(tip, kLaneGraphElevationM));
      markers.markers.push_back(std::move(arrow));
    }
    return markers;
  }

  // ---------------------------------------------------------------------------
  //  路径
  // ---------------------------------------------------------------------------
  nav_msgs::msg::Path build_route_path(const Route & route) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();

    for (const RouteStep & step : route.steps) {
      std::vector<Pose2D> poses =
        sample_lane(step.lane, step.entry_s_m, step.exit_s_m, path_step_m_);
      // 相邻两段在路口处共享同一个点，跳过后一段的第一个，免得路径里出现
      // 一串重复点。P3 拿它做弧长参数化时，重复点会让相邻点距为 0。
      const std::size_t first = path.poses.empty() ? 0 : 1;
      for (std::size_t i = first; i < poses.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose.position = to_point(poses[i], kRoutePathElevationM);
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, poses[i].heading_rad);
        pose_stamped.pose.orientation = tf2::toMsg(quaternion);
        path.poses.push_back(std::move(pose_stamped));
      }
    }
    return path;
  }

  /// @brief 发布一条空路径，把 RViz 上的旧路径清掉。
  ///
  /// ⚠️ 这里的「空」和 find_route() 返回 nullopt 是两回事，别混起来看：
  ///    库那一层**绝不**用空路径表示失败（见 routing.hpp）；
  ///    节点这一层清显示是个显式动作，而且**总是伴随一条 WARN 日志**。
  ///    不清的话，用户点了一个到不了的目标，屏幕上还留着上一条路径 ——
  ///    那比什么都不显示更容易误导。
  void clear_route()
  {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = map_frame_;
    empty.header.stamp = now();
    route_pub_->publish(empty);
  }

  // ---------------------------------------------------------------------------
  //  目标点回调
  // ---------------------------------------------------------------------------
  void on_goal(const geometry_msgs::msg::PoseStamped & goal_in)
  {
    const auto started_at = std::chrono::steady_clock::now();

    // 目标点统一换到地图系。RViz 发的是它当前 Fixed Frame 下的位姿，
    // 而 Fixed Frame 是用户可以随手改的 —— 假设它一定是 map 就会在某天
    // 变成「点了没反应」，且没人会想到去看 RViz 的 Fixed Frame。
    geometry_msgs::msg::PoseStamped goal;
    try {
      goal = tf_buffer_->transform(goal_in, map_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(
        get_logger(), "目标点在 %s 系，换不到 %s 系：%s", goal_in.header.frame_id.c_str(),
        map_frame_.c_str(), error.what());
      return;
    }

    // 起点取 TF map → base_link。取不到就**明确说取不到**，不要退化成
    // 「用上一次的位姿」或「用原点」—— 那会规划出一条从别处出发的路径，
    // 而它看起来完全正常。
    geometry_msgs::msg::TransformStamped ego;
    try {
      ego = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(
        get_logger(), "拿不到 %s → %s 的 TF：%s。仿真起来了吗？没有起点就无法规划",
        map_frame_.c_str(), base_frame_.c_str(), error.what());
      clear_route();
      return;
    }

    // 起点匹配**必须带朝向**。不带的话，自车稍微偏左一点就会被判到对向车道，
    // 于是路由第一步就要求它掉头 —— 而那条路径在 RViz 里平滑正常，看不出问题。
    const double ego_yaw_rad = tf2::getYaw(ego.transform.rotation);
    const auto start =
      graph_->nearest_lane(ego.transform.translation.x, ego.transform.translation.y, ego_yaw_rad);
    if (!start.has_value()) {
      RCLCPP_WARN(
        get_logger(), "自车朝向 %.1f° 与所有车道的行驶方向都差 90° 以上，找不到起点车道",
        ego_yaw_rad * 180.0 / M_PI);
      clear_route();
      return;
    }
    if (start->distance_m > max_start_distance_m_) {
      RCLCPP_WARN(
        get_logger(), "自车离最近车道 %.2f m，超过 %.2f m —— 车不在路上，不替它猜",
        start->distance_m, max_start_distance_m_);
      clear_route();
      return;
    }

    const std::optional<double> goal_heading_rad =
      use_goal_heading_ ? std::optional<double>(tf2::getYaw(goal.pose.orientation)) : std::nullopt;
    const auto target =
      graph_->nearest_lane(goal.pose.position.x, goal.pose.position.y, goal_heading_rad);
    if (!target.has_value() || target->distance_m > max_goal_distance_m_) {
      RCLCPP_WARN(
        get_logger(), "目标点 (%.1f, %.1f) 附近没有可用车道（最近 %.2f m，上限 %.2f m）",
        goal.pose.position.x, goal.pose.position.y, target.has_value() ? target->distance_m : -1.0,
        max_goal_distance_m_);
      clear_route();
      return;
    }

    const auto route = find_route(*graph_, start->lane, start->s_m, target->lane, target->s_m);
    if (!route.has_value()) {
      // 图上确实不连通。本园区地图是强连通的，所以这一支正常情况下不会走到 ——
      // 真走到了，先怀疑车道图，别怀疑用户点错了。
      RCLCPP_WARN(
        get_logger(), "从 road %d lane %d 到 road %d lane %d 不可达", start->lane.road_id,
        start->lane.lane_id, target->lane.road_id, target->lane.lane_id);
      clear_route();
      return;
    }

    const nav_msgs::msg::Path path = build_route_path(*route);
    route_pub_->publish(path);

    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started_at)
        .count();
    RCLCPP_INFO(
      get_logger(), "路径已发布：%zu 段车道、%zu 个点、%.2f m，耗时 %.2f ms", route->steps.size(),
      path.poses.size(), route->length_m, elapsed_ms);
  }

  std::string map_file_;
  std::string map_frame_;
  std::string base_frame_;
  double marker_step_m_{1.0};
  double path_step_m_{0.5};
  double max_start_distance_m_{5.0};
  double max_goal_distance_m_{10.0};
  bool use_goal_heading_{false};

  std::unique_ptr<LaneGraph> graph_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_graph_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

}  // namespace ads_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ads_map::MapNode>());
  } catch (const std::exception & error) {
    // 构造函数里加载地图失败会走到这里。**非零退出**是关键：
    // launch 会把它报出来，而一个"活着但什么都不发"的节点不会。
    RCLCPP_FATAL(rclcpp::get_logger("map_node"), "map_node 启动失败：%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
