// =============================================================================
//  lidar_preprocessor —— 把仿真器原始点云整理成 SPEC §4.1 的 /lidar/points
//
//  做三件事，顺序不能换：
//     1. 坐标变换  lidar_link → base_link
//     2. 自车滤除  剔掉打在自己车身上的点
//     3. 无效点滤除 剔掉 ±inf / NaN
//
//  为什么这三件事放在一个节点里
//  ----------------------------
//  不是为了省事，是实测出来的：一帧点云 1.8 MB，每多一次跨进程传递就多一轮
//  序列化 + 一个可能溢出的队列。S3 时实测 best-effort 下每跳都在静默丢帧
//  （详见 tasks/todo.md「排查记录 1」）。所以能在一趟里做完的就不拆成三个节点。
//
//  为什么放在 gazebo_bridge 而不是感知包
//  --------------------------------------
//  这三件事都是「把仿真器原生数据整理成 SPEC 规定的规范格式」，是 bridge 层的
//  职责。P0b 的 carla_bridge 要做**完全相同**的三件事 —— CARLA 的雷达同样装在
//  车顶、同样会打到自车、同样会返回无效点。等第二个使用者真的出现时再抽成
//  共用组件，现在抽是过早抽象。
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace ads_simulation
{

/// @brief 自车包围盒，base_link 系，单位 m。落在盒内的点会被剔除。
struct EgoBox
{
  double x_min, x_max, y_min, y_max, z_min, z_max;

  /// @brief 点是否落在盒内（闭区间）。
  bool contains(float x, float y, float z) const
  {
    return x >= x_min && x <= x_max &&
           y >= y_min && y <= y_max &&
           z >= z_min && z <= z_max;
  }
};

/// @brief 订阅仿真器原始点云，发布 base_link 系、已滤除自车与无效点的点云。
///
/// 输入：sensor_msgs/PointCloud2，frame_id 通常为 lidar_link，有序（32×1800）
/// 输出：sensor_msgs/PointCloud2，frame_id == base_link，**无序且 is_dense=true**
/// 依赖：TF 中存在 base_link → 输入 frame_id 的变换
///       （由 robot_state_publisher 读 URDF 后作为静态 TF 发布）
class LidarPreprocessor : public rclcpp::Node
{
public:
  LidarPreprocessor()
  : Node("lidar_preprocessor")
  {
    // ---- 话题 ----
    // 做成参数而不是写死，是为了 P0b 的 carla_bridge 能复用同一个节点。
    // 默认值直接写 SPEC §4.1 的规范名，让契约在代码里看得见。
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar/points_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lidar/points");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");

    // ---- 自车包围盒 ----
    // ⚠️ 这六个数由 launch 从 config/vehicle_params.yaml 读出来传进来，
    //    **不允许在这里写默认车身尺寸** —— 那就等于把车辆参数写了第二份
    //    （SPEC §4.1 车辆参数单一来源）。默认值故意给成一个退化的空盒子：
    //    launch 忘了传参时滤不掉任何点，是「功能失效但数据无损」，
    //    比默认一个错误尺寸把真实障碍物误删要安全。
    ego_.x_min = declare_parameter<double>("ego_box.x_min", 0.0);
    ego_.x_max = declare_parameter<double>("ego_box.x_max", 0.0);
    ego_.y_min = declare_parameter<double>("ego_box.y_min", 0.0);
    ego_.y_max = declare_parameter<double>("ego_box.y_max", 0.0);
    ego_.z_min = declare_parameter<double>("ego_box.z_min", 0.0);
    ego_.z_max = declare_parameter<double>("ego_box.z_max", 0.0);

    // 包围盒各面向外扩张的余量。
    // 调大：自车残留点更干净，但会把紧贴车身的真实障碍物也吃掉 ——
    //       低速场景下贴身障碍是要报的，所以不能给太大。
    // 调小到 0：车身噪声点（测距噪声 σ=1 cm）会漏出来，被感知当成零距离障碍物。
    // 5 cm ≈ 5σ，足够覆盖噪声又不至于吃掉真实目标。
    margin_m_ = declare_parameter<double>("ego_box.margin_m", 0.05);

    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.05);
    transform_budget_ms_ = declare_parameter<double>("transform_budget_ms", 10.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- QoS：这里**故意不用** SensorDataQoS，理由值得写清楚 ----
    //
    // ROS 惯例是传感器数据走 SensorDataQoS（best-effort + 深度 5）：
    // 传感器源源不断产出，重传过期数据没有意义，reliable 在消费者卡顿时
    // 会积压延迟。这个惯例针对的是**真车**。
    //
    // 但在本仿真链路里实测下来，best-effort 是丢帧的主因，不是保护措施：
    // 一帧 1.8 MB，链路上每一跳都有深度 5 的队列，消费者稍慢就溢出，
    // 实测 /lidar/points 只有标称 10 Hz 的 35%（3.58 Hz），且**完全没有日志**。
    //
    // 关键证据：Gazebo 侧帧间隔要么正好 100 ms 要么正好 200 ms，没有中间值 ——
    // 传感器一直准点出帧，少掉的是被队列丢的。所以该修传输，不是降传感器规格。
    //
    // ⚠️ P7 上实车时应改回 best-effort：真实网络下 reliable 的重传会让延迟
    //    不可控，而控制回路对延迟极其敏感。
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&LidarPreprocessor::onCloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "点云预处理已启动：%s → %s（目标系 %s）",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "自车包围盒 x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]  余量 %.3f m",
      ego_.x_min, ego_.x_max, ego_.y_min, ego_.y_max, ego_.z_min, ego_.z_max,
      margin_m_);
    if (ego_.x_max <= ego_.x_min) {
      RCLCPP_WARN(
        get_logger(),
        "自车包围盒是空的 —— launch 没传 ego_box.* 参数，自车反射点不会被滤除。"
        "感知会把自己的车顶当成零距离障碍物。");
    }
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
    } catch (const tf2::TransformException & ex) {
      // 节流打印：TF 没起来时这里每帧都失败，不节流会把日志刷爆，
      // 反而看不见真正有用的信息。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "查不到 %s → %s 的变换，丢弃本帧点云：%s",
        target_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    const auto t_start = std::chrono::steady_clock::now();

    // ---- 第一步：坐标变换 ----
    // 用 tf2::doTransform 而不是自己乘矩阵 —— SPEC §11 明令禁止手写变换矩阵。
    // 它一次变换整帧，包括后面要被滤掉的点。看似浪费，但实测整帧变换在 ms 量级，
    // 而为了省这点开销去手写「边变换边判断」的单趟循环，要自己实现旋转平移，
    // 正是最容易写错且错了不报错的地方。
    sensor_msgs::msg::PointCloud2 transformed;
    tf2::doTransform(*msg, transformed, tf);

    // ⚠️ doTransform 会把输出的 header 覆盖成**变换的** header，
    // 而静态变换的时间戳未必等于点云的采集时刻。不修回来的话，下游按时间戳
    // 做多传感器同步时会拿到错误的时刻，而点云数据本身完全正常 ——
    // 这类 bug 极难定位，所以这里显式改回去。
    transformed.header.stamp = msg->header.stamp;
    transformed.header.frame_id = target_frame_;

    // ---- 第二、三步：滤除自车反射点与无效点 ----
    const auto stats = filterInPlace(transformed);

    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_start).count();
    if (elapsed_ms > transform_budget_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "点云预处理耗时 %.1f ms，超过预算 %.1f ms —— "
        "回调阻塞会拖慢整条链路，考虑降低线数或挪到独立线程",
        elapsed_ms, transform_budget_ms_);
    }

    // 每 10 秒报一次滤除比例。这不是调试残留：自车滤除比例突然变化，
    // 意味着车身尺寸参数或雷达外参被改动了，是个值得注意的信号。
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "点云 %zu → %zu（自车 %zu，无效 %zu），耗时 %.1f ms",
      stats.total, stats.kept, stats.self_hits, stats.invalid, elapsed_ms);

    pub_->publish(transformed);
  }

  struct FilterStats
  {
    size_t total{0};
    size_t kept{0};
    size_t self_hits{0};
    size_t invalid{0};
  };

  /// @brief 原地压缩点云：保留「有限 且 不在自车盒内」的点。
  ///
  /// 输出**不再是有序点云**（height 变成 1）。
  ///
  /// 为什么可以接受失去有序性：Gazebo 的 gpu_lidar 输出带 `ring` 字段（线束编号），
  /// 基于线束的地面分割等算法靠 ring 就能恢复线号，不依赖网格排布。
  /// 而收益是实打实的：一帧从 57600 点降到约 19000 点，消息从 1.8 MB 降到 0.6 MB，
  /// 直接缓解 S3 实测的丢帧问题。
  ///
  /// 为什么不改成「把滤掉的点置为 NaN」（那样能保住有序性）：
  /// 消息大小一点不降，而下游仍然要逐点跳过 NaN —— 两头的代价都付了。
  FilterStats filterInPlace(sensor_msgs::msg::PointCloud2 & cloud) const
  {
    FilterStats st;
    st.total = static_cast<size_t>(cloud.width) * cloud.height;
    if (st.total == 0) {
      return st;
    }

    // 把余量算进盒子，避免在逐点循环里反复做加减法
    const EgoBox box{
      ego_.x_min - margin_m_, ego_.x_max + margin_m_,
      ego_.y_min - margin_m_, ego_.y_max + margin_m_,
      ego_.z_min - margin_m_, ego_.z_max + margin_m_};
    const bool box_valid = box.x_max > box.x_min;

    const uint32_t step = cloud.point_step;
    uint8_t * const base = cloud.data.data();

    // 用整点 memcpy 搬运，而不是逐字段拷贝：这样 intensity、ring 以及任何
    // 我们不认识的字段都会原样保留。换个仿真器多出几个字段也不用改这里。
    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

    size_t kept = 0;
    for (size_t i = 0; i < st.total; ++i, ++it_x, ++it_y, ++it_z) {
      const float x = *it_x, y = *it_y, z = *it_z;

      // ⚠️ 必须同时挡住 NaN 和 ±inf。
      // Gazebo 的 gpu_lidar 对没有回波的射线（打向天空、超出量程）返回的是
      // **±inf 而不是 NaN**，所以只判 isnan 会漏掉一半。std::isfinite 两者都挡。
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++st.invalid;
        continue;
      }
      if (box_valid && box.contains(x, y, z)) {
        ++st.self_hits;
        continue;
      }
      // 保留点向前压缩。kept <= i 恒成立，所以源和目标区间不会重叠出错。
      if (kept != i) {
        std::memcpy(base + kept * step, base + i * step, step);
      }
      ++kept;
    }

    st.kept = kept;
    cloud.height = 1;                       // 压缩后不再有序
    cloud.width = static_cast<uint32_t>(kept);
    cloud.row_step = static_cast<uint32_t>(kept * step);
    cloud.data.resize(kept * step);
    cloud.is_dense = true;                  // 已保证无 NaN / inf
    return st;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  EgoBox ego_{};
  double margin_m_{0.05};
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
  rclcpp::spin(std::make_shared<ads_simulation::LidarPreprocessor>());
  rclcpp::shutdown();
  return 0;
}
