// =============================================================================
//  pointcloud_to_base_link —— 把激光雷达点云从 lidar_link 系变换到 base_link 系
//
//  为什么需要这个节点
//  ------------------
//  雷达装在车顶（距 base_link 前 1.35 m、高 1.60 m），Gazebo 发出来的点自然
//  是在 lidar_link 系下的。而 SPEC §4.1 规定 /lidar/points 必须在 base_link 系。
//
//  「让 Gazebo 直接把 frame_id 写成 base_link」是能让验收判据变绿的最省事做法，
//  但那是**谎报坐标系**：点的数值一个没变，却被声称在另一个系里。
//  后果是下游拿到的每个点都带着固定的 1.35 / 1.60 m 偏移 ——
//  地面点会浮在 1.6 m 高处，做地面分割时怎么调阈值都不对；
//  而且从头到尾没有任何报错，只有「算法效果就是不好」。
//
//  所以这里老老实实查 TF、真的把每个点乘一遍变换矩阵。
//
//  为什么放在 gazebo_bridge 里
//  ---------------------------
//  它是 bridge 层职责的一部分：把仿真器原生数据翻译成 SPEC 规定的规范格式。
//  P0b 的 carla_bridge 会做同样的事（CARLA 的雷达同样装在车顶）。
//  等到第二个使用者真的出现时，再把它抽成共用组件 —— 现在抽是过早抽象。
// =============================================================================

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace ads_simulation
{

/// @brief 订阅 lidar_link 系点云，发布 base_link 系点云。
///
/// 输入：sensor_msgs/PointCloud2，frame_id 通常为 lidar_link
/// 输出：sensor_msgs/PointCloud2，frame_id == target_frame（默认 base_link）
/// 依赖：TF 中存在 target_frame → 输入 frame_id 的变换
///       （由 robot_state_publisher 读 URDF 后作为静态 TF 发布）
class PointCloudToBaseLink : public rclcpp::Node
{
public:
  PointCloudToBaseLink()
  : Node("pointcloud_to_base_link")
  {
    // ---- 参数 ----
    // 话题名做成参数而不是写死，是为了 P0b 的 carla_bridge 能复用同一个节点。
    // 默认值直接写 SPEC §4.1 的规范名，让契约在代码里看得见。
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar/points_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lidar/points");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");

    // 查 TF 的等待时长。base_link→lidar_link 是**静态**变换（雷达焊死在车上），
    // 正常情况下一进来就能查到，这个超时只在启动瞬间 TF 还没发出来时起作用。
    // 调大：启动更稳，但真出问题时要等更久才报出来。
    // 调小到 0：启动头几帧必然丢失。
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.05);

    // 单帧变换的耗时预算。CLAUDE.md 规定回调里超过 10 ms 的工作要挪到独立线程。
    // 57600 个点做一次刚体变换实测在 1-2 ms 量级，留 10 ms 余量足够；
    // 一旦超了要立刻知道（比如以后把线数调到 128 线），所以这里主动告警而不是沉默。
    transform_budget_ms_ = declare_parameter<double>("transform_budget_ms", 10.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- QoS：这里**故意不用** SensorDataQoS，理由值得写清楚 ----
    //
    // ROS 的惯例是传感器数据走 SensorDataQoS（best-effort + 深度 5）：
    // 传感器源源不断地产出，重传一帧过期的数据没有意义，而 reliable
    // 在消费者卡顿时会积压出延迟。这个惯例针对的是**真车**。
    //
    // 但在本仿真链路里实测下来，best-effort 是丢帧的主因，不是保护措施：
    //
    //     一帧 32×1800 点 = 1.8 MB。链路是
    //     Gazebo → gz-transport → ros_gz_bridge → 本节点 → 下游，
    //     每一跳都有一个深度 5 的队列。消费者稍慢一点就溢出丢帧，
    //     实测 /lidar/points 只有标称 10 Hz 的 35%（3.58 Hz）。
    //     而丢帧是**静默**的：没有任何日志，只是频率变低。
    //
    // 关键证据：Gazebo 侧的帧间隔要么正好 100 ms 要么正好 200 ms，
    // 没有中间值 —— 传感器一直在准点出帧，少掉的那些是被队列丢的，
    // 不是没生成。所以该修的是传输，不是降传感器规格。
    //
    // 改成 reliable 之后 DDS 会重传 + 施加背压，本机链路上代价可以接受。
    // 深度给 10（1 秒的量）：再深只会在消费者长时间卡顿时积压出更大延迟，
    // 而那时候丢帧反而是更好的选择。
    //
    // ⚠️ P7 上实车时应改回 best-effort：那时链路里有真实网络，
    //    reliable 的重传会让延迟不可控，而控制回路对延迟极其敏感。
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&PointCloudToBaseLink::onCloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "点云坐标变换已启动：%s → %s（目标系 %s）",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // 已经在目标系里就直接转发，省掉一次全点云拷贝。
    // 这条分支在 carla_bridge 复用本节点时可能真的会走到。
    if (msg->header.frame_id == target_frame_) {
      pub_->publish(*msg);
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
    } catch (const tf2::TransformException & ex) {
      // 节流打印：TF 没起来的时候这里会每帧都失败，不节流会把日志刷爆，
      // 反而看不见真正有用的信息。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "查不到 %s → %s 的变换，丢弃本帧点云：%s",
        target_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    const auto t_start = std::chrono::steady_clock::now();

    sensor_msgs::msg::PointCloud2 out;
    tf2::doTransform(*msg, out, tf);

    // ⚠️ doTransform 会把输出的 header 覆盖成**变换的** header，
    // 而静态变换的时间戳未必等于点云的采集时刻。
    // 不修回来的话，下游按时间戳做多传感器同步时会拿到错误的时刻，
    // 而点云数据本身完全正常 —— 这类 bug 极难定位，所以这里显式改回去。
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = target_frame_;

    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_start).count();
    if (elapsed_ms > transform_budget_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "点云变换耗时 %.1f ms，超过预算 %.1f ms —— "
        "回调阻塞会拖慢整条链路，考虑降低线数或挪到独立线程",
        elapsed_ms, transform_budget_ms_);
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double tf_timeout_s_{0.05};
  double transform_budget_ms_{10.0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace ads_simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ads_simulation::PointCloudToBaseLink>());
  rclcpp::shutdown();
  return 0;
}
