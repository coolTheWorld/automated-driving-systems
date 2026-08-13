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
//  静态障碍物**真值**发布器 —— /perception/obstacles（P3-S5）
//
//  这是 SPEC §10 那句「先用仿真器提供的真值，把下游打通，再回头做感知」的
//  又一个实例（前一个是 /ego_pose_gt）。P5 接上真感知之后，
//  **换掉本节点即可，planning_node 一行不用改** —— 话题名与消息类型
//  在 P0a 就定死了（ads_msgs/ObstacleArray），这正是提前定接口的意义。
//
//  ⚠️ **数据来自 config/obstacles.yaml，不是从 Gazebo 里读位姿。**
//     对**静态**障碍物这两者恒等：Gazebo 里的模型本来就是从同一个 YAML
//     生成的（scripts/gen_obstacles.py），而静态模型不会动。
//     好处是根本不存在第二份数据，也就不可能漂移。
//
//     代价（**要知道**）：有人在 Gazebo GUI 里手动拖动障碍物，本节点不会知道。
//     那时"车看到的"和"车会撞上的"就分家了。自动化验收里不会发生，
//     手动调试时请记得这条 —— 想验"感知与真实不一致"的场景，
//     得等 P5 的真感知，本节点结构上做不到。
//
//  ## P5-S1 起它还发**动态**目标的真值
//
//  静态障碍物读 YAML（不会动，两者恒等）；动态目标只能**订阅它们自己的
//  `/model/<name>/pose_gt`**（Gazebo 的 OdometryPublisher 插件发的）。
//
//  ⚠️⚠️ **twist 是车体系，必须转到 map 系再填。**（2026-08-11 实测确认）
//     探针：NPC 车 yaw=π（车头朝 −x），发车体系 linear.x=+4，
//     实测它在世界系里往 −x 走，而 `pose_gt` 的 twist.linear.x 报 **+4**。
//     也就是说 twist 在 `child_frame_id` 系 —— 符合 nav_msgs/Odometry 的规范，
//     但**必须真的去转**，因为 Obstacle.msg 的 velocity_mps 要的是
//     `header.frame_id`（= map）系。
//
//     不转的后果是**对向行驶的目标速度符号完全反**（+4 vs −4）。
//     而感知会正确输出 −4，与错误的真值一比误差 8 m/s，判据红 ——
//     于是人去查 EKF，而错在真值这一侧。
//     **CP-P5-B 第 4 条「速度符号错要单独判」量的正是这个量。**
//
//  ## P5-S1 起它发**两个**话题
//
//      /perception/obstacles_gt   始终发，**评测基准**，算法节点禁止订阅
//      /perception/obstacles      `publish_as_perception` 为 true 时才发
//
//  为什么不是直接改名：`perception_node` 要到 P5-S5 才有。现在就改名的话，
//  S1–S4 期间 `planning_node` 没有障碍物输入，CP-P3-B 直接跑不了。
//  所以默认两个都发；S5 接上真感知时 launch 把 `publish_as_perception` 设 false，
//  两个发布者就不会同时发 `/perception/obstacles`（SPEC §3.3 那条约束）。
// =============================================================================

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ads_msgs/msg/obstacle.hpp"
#include "ads_msgs/msg/obstacle_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace gazebo_bridge
{

class ObstacleTruthNode : public rclcpp::Node
{
public:
  ObstacleTruthNode() : Node("obstacle_truth_node")
  {
    // 障碍物由 launch 从 config/obstacles.yaml 搬运成**平行数组**参数。
    // 用平行数组而不是"每个障碍物一组带下标的参数"，是因为 ROS 2 的参数
    // 没有数组的数组；平行数组的代价是要自己校验长度一致 —— 下面就校验了，
    // 而且长度对不上是**拒绝启动**，不是取最短的那个继续跑。
    // 取最短的话，少配一个尺寸就会让某个障碍物静默变成 0×0（= 一个点），
    // 而规划器会愉快地从它"旁边"开过去。
    const auto center_x =
      declare_parameter<std::vector<double>>("obstacles.center_x_m", std::vector<double>{});
    const auto center_y =
      declare_parameter<std::vector<double>>("obstacles.center_y_m", std::vector<double>{});
    const auto yaw =
      declare_parameter<std::vector<double>>("obstacles.yaw_rad", std::vector<double>{});
    const auto length =
      declare_parameter<std::vector<double>>("obstacles.length_m", std::vector<double>{});
    const auto width =
      declare_parameter<std::vector<double>>("obstacles.width_m", std::vector<double>{});
    const auto height =
      declare_parameter<std::vector<double>>("obstacles.height_m", std::vector<double>{});

    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const double rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);

    const std::size_t count = center_x.size();
    if (
      center_y.size() != count || yaw.size() != count || length.size() != count ||
      width.size() != count || height.size() != count) {
      throw std::invalid_argument(
        "obstacle_truth_node: obstacles.* 六个参数数组长度不一致 —— "
        "少配一项会让某个障碍物静默变成零尺寸，而规划器会从它旁边开过去。");
    }

    // 一次性把消息拼好，之后每周期只更新时间戳。
    // 静态障碍物每拍重算一遍是白费，而且给了"某一拍算错"的机会。
    message_.obstacles.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      ads_msgs::msg::Obstacle obstacle;
      obstacle.id = static_cast<std::uint32_t>(i);
      // CLASSIFICATION_STATIC：本节点只发静态障碍物。
      // 分类不是装饰 —— P3 的 lattice 是纯路径的，表达不了"等它过去"
      // （planning.md §4.3），所以下游把动态障碍物当静态处理是**不正确**的，
      // 只是保守。如实标成 STATIC，免得 P6 有人以为这条链路已经支持动态了。
      obstacle.classification = ads_msgs::msg::Obstacle::CLASSIFICATION_STATIC;
      obstacle.pose.position.x = center_x[i];
      obstacle.pose.position.y = center_y[i];
      // z 取盒子中心高度，与 Gazebo 里的模型一致（生成器也是这么放的）。
      obstacle.pose.position.z = 0.5 * height[i];
      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, yaw[i]);
      obstacle.pose.orientation = tf2::toMsg(quaternion);
      obstacle.size_m.x = length[i];
      obstacle.size_m.y = width[i];
      obstacle.size_m.z = height[i];
      // 真值，所以存在概率是 1.0。P5 的真感知会给出小于 1 的值，
      // 那时下游若要按它筛选，判据得在那时定 —— 现在不预设。
      obstacle.existence_probability = 1.0F;
      // 真值的朝向可信（P6-S3 加的语义标志，见 Obstacle.msg）。
      obstacle.heading_resolved = true;
      message_.obstacles.push_back(obstacle);
    }

    // ---- 动态目标（P5-S1）----------------------------------------------
    // 同样用平行数组，理由与上面一样；长度不一致同样是**拒绝启动**。
    const auto names =
      declare_parameter<std::vector<std::string>>("dynamic.names", std::vector<std::string>{});
    const auto d_length =
      declare_parameter<std::vector<double>>("dynamic.length_m", std::vector<double>{});
    const auto d_width =
      declare_parameter<std::vector<double>>("dynamic.width_m", std::vector<double>{});
    const auto d_height =
      declare_parameter<std::vector<double>>("dynamic.height_m", std::vector<double>{});
    // 模型原点 → 包围盒中心的偏移，**车体系**。
    // ⚠️ 这个偏移不是可选的：NPC 车的模型原点在**后轴中心地面**（与自车同一个
    //    约定），而包围盒中心在它前方 1.35 m、上方 0.75 m。不换算的话，
    //    真值会比实际位置**恒定偏后 1.35 m**，而感知输出的是包围盒中心 ——
    //    于是所有检测都带着一个 1.35 m 的系统偏差，超过 CP-P5-B 第 3 条的
    //    0.5 m 判据。症状是「感知位置全都偏」，人会去查聚类的质心算法。
    const auto d_off_x =
      declare_parameter<std::vector<double>>("dynamic.offset_x_m", std::vector<double>{});
    const auto d_off_z =
      declare_parameter<std::vector<double>>("dynamic.offset_z_m", std::vector<double>{});
    const auto d_class =
      declare_parameter<std::vector<int64_t>>("dynamic.classification", std::vector<int64_t>{});

    const std::size_t dyn_count = names.size();
    if (
      d_length.size() != dyn_count || d_width.size() != dyn_count || d_height.size() != dyn_count ||
      d_off_x.size() != dyn_count || d_off_z.size() != dyn_count || d_class.size() != dyn_count) {
      throw std::invalid_argument(
        "obstacle_truth_node: dynamic.* 七个参数数组长度不一致 —— "
        "少配一项会让某个动态目标静默变成零尺寸或位置偏移错。");
    }

    dynamic_.resize(dyn_count);
    for (std::size_t i = 0; i < dyn_count; ++i) {
      auto & spec = dynamic_[i];
      spec.name = names[i];
      spec.length_m = d_length[i];
      spec.width_m = d_width[i];
      spec.height_m = d_height[i];
      spec.offset_x_m = d_off_x[i];
      spec.offset_z_m = d_off_z[i];
      spec.classification = static_cast<std::uint8_t>(d_class[i]);
      // ID 接在静态障碍物之后，两者不冲突。
      spec.id = static_cast<std::uint32_t>(count + i);

      // ⚠️ 捕获下标而不是引用 —— dynamic_ 在构造期间还可能因 resize 搬家。
      subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        "/model/" + spec.name + "/pose_gt", rclcpp::QoS(10),
        [this, i](nav_msgs::msg::Odometry::SharedPtr msg) { OnTruth(i, std::move(msg)); }));
    }

    // ⚠️ **两个话题**，理由见文件头。默认两个都发。
    publish_as_perception_ = declare_parameter<bool>("publish_as_perception", true);
    truth_publisher_ =
      create_publisher<ads_msgs::msg::ObstacleArray>("/perception/obstacles_gt", rclcpp::QoS(10));
    if (publish_as_perception_) {
      publisher_ =
        create_publisher<ads_msgs::msg::ObstacleArray>("/perception/obstacles", rclcpp::QoS(10));
    }

    // ⚠️ create_timer + 节点时钟，不是 create_wall_timer（SPEC §5）。
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz), [this]() { publish(); });

    RCLCPP_INFO(
      get_logger(),
      "障碍物真值发布器已启动：%zu 个静态障碍物 + %zu 个动态目标，%.1f Hz，frame_id = %s，"
      "%s /perception/obstacles",
      count, dyn_count, rate_hz, frame_id_.c_str(), publish_as_perception_ ? "同时发" : "**不发**");
  }

private:
  /// 一个动态目标的静态属性 + 最近一帧真值。
  struct DynamicSpec
  {
    std::string name;
    double length_m{0.0};
    double width_m{0.0};
    double height_m{0.0};
    /// 模型原点 → 包围盒中心，**车体系**。见构造函数里的说明。
    double offset_x_m{0.0};
    double offset_z_m{0.0};
    std::uint8_t classification{0};
    std::uint32_t id{0};
    bool valid{false};
    ads_msgs::msg::Obstacle obstacle;
  };

  void OnTruth(std::size_t index, nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto & spec = dynamic_[index];
    const auto & q = msg->pose.pose.orientation;
    const double yaw_rad =
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double cos_yaw = std::cos(yaw_rad);
    const double sin_yaw = std::sin(yaw_rad);

    auto & obstacle = spec.obstacle;
    obstacle.id = spec.id;
    obstacle.classification = spec.classification;

    // ---- 模型原点 → 包围盒中心（车体系偏移，绕 z 旋转）----
    // z 的偏移不受 yaw 影响（绕 z 转不动 z），所以直接加。
    obstacle.pose.position.x = msg->pose.pose.position.x + cos_yaw * spec.offset_x_m;
    obstacle.pose.position.y = msg->pose.pose.position.y + sin_yaw * spec.offset_x_m;
    obstacle.pose.position.z = msg->pose.pose.position.z + spec.offset_z_m;
    obstacle.pose.orientation = q;

    obstacle.size_m.x = spec.length_m;
    obstacle.size_m.y = spec.width_m;
    obstacle.size_m.z = spec.height_m;

    // ---- 速度：**车体系 → map 系**（实测确认 twist 在 child_frame_id 系）----
    // ⚠️ 不转的话对向行驶的目标速度符号完全反。详见文件头。
    const double vx_body = msg->twist.twist.linear.x;
    const double vy_body = msg->twist.twist.linear.y;
    obstacle.velocity_mps.x = cos_yaw * vx_body - sin_yaw * vy_body;
    obstacle.velocity_mps.y = sin_yaw * vx_body + cos_yaw * vy_body;
    obstacle.velocity_mps.z = msg->twist.twist.linear.z;

    obstacle.existence_probability = 1.0F;
    // 真值的朝向可信（P6-S3 加的语义标志，见 Obstacle.msg）。
    obstacle.heading_resolved = true;
    spec.valid = true;
  }

  void publish()
  {
    message_.header.stamp = now();
    message_.header.frame_id = frame_id_;
    for (auto & obstacle : message_.obstacles) {
      obstacle.header = message_.header;
    }

    // 每拍在静态数组的基础上追加**当前有效**的动态目标。
    // ⚠️ 还没收到真值的目标**不追加**，而不是补一个零位姿的占位 ——
    //    ObstacleArray 的契约是「数组为空 = 本帧确认没有障碍物」，
    //    补占位等于告诉下游「有个目标在原点」，那是假的。
    ads_msgs::msg::ObstacleArray output = message_;
    for (const auto & spec : dynamic_) {
      if (!spec.valid) {
        continue;
      }
      output.obstacles.push_back(spec.obstacle);
      output.obstacles.back().header = message_.header;
    }

    truth_publisher_->publish(output);
    if (publisher_) {
      publisher_->publish(output);
    }
  }

  std::string frame_id_;
  bool publish_as_perception_{true};
  ads_msgs::msg::ObstacleArray message_;
  std::vector<DynamicSpec> dynamic_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subscriptions_;
  /// **评测基准**，始终发。算法节点禁止订阅。
  rclcpp::Publisher<ads_msgs::msg::ObstacleArray>::SharedPtr truth_publisher_;
  /// 真感知接上之前，同一份数据也当 /perception/obstacles 发。可关。
  rclcpp::Publisher<ads_msgs::msg::ObstacleArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gazebo_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gazebo_bridge::ObstacleTruthNode>());
  rclcpp::shutdown();
  return 0;
}
