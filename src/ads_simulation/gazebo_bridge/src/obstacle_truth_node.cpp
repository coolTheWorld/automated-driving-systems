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
// =============================================================================

#include <string>
#include <vector>

#include "ads_msgs/msg/obstacle.hpp"
#include "ads_msgs/msg/obstacle_array.hpp"
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
      message_.obstacles.push_back(obstacle);
    }

    publisher_ =
      create_publisher<ads_msgs::msg::ObstacleArray>("/perception/obstacles", rclcpp::QoS(10));

    // ⚠️ create_timer + 节点时钟，不是 create_wall_timer（SPEC §5）。
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz), [this]() { publish(); });

    RCLCPP_INFO(
      get_logger(), "障碍物真值发布器已启动：%zu 个静态障碍物，%.1f Hz，frame_id = %s", count,
      rate_hz, frame_id_.c_str());
  }

private:
  void publish()
  {
    message_.header.stamp = now();
    message_.header.frame_id = frame_id_;
    for (auto & obstacle : message_.obstacles) {
      obstacle.header = message_.header;
    }
    publisher_->publish(message_);
  }

  std::string frame_id_;
  ads_msgs::msg::ObstacleArray message_;
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
