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
//  keyboard_teleop —— 键盘 → /vehicle_cmd
//
//  P0a 阶段唯一的"驾驶员"。它存在的意义不是好开，而是**让整条链路被人手驱动一遍**：
//  按键 → /vehicle_cmd → gazebo_bridge → Gazebo → 传感器 → 回到 RViz。
//  这条闭环通了，P2 的控制器只要接管 /vehicle_cmd 就能立刻看到车在动。
//
//  为什么单独成包（ads_teleop）而不是塞进 gazebo_bridge
//  ----------------------------------------------------
//  它发布的是 SPEC §4.1 的规范话题 /vehicle_cmd，**与仿真器无关**。
//  P0b 接 CARLA 时同一个节点原样复用 —— 放进 gazebo_bridge 就绑死在 Gazebo 上了。
//
//  ⚠️ 两个物理事实，不是 bug
//  -------------------------
//  1. **静止时打方向车不会转。** 下游把转角换算成横摆角速度 ω = v·tan(δ)/L，
//     v=0 时 ω 恒为 0。真车原地打方向车也不动，阿克曼转向就是这样。
//     要转弯必须先有速度。
//  2. **不支持倒车。** ads_msgs/VehicleCmd 只有转角和加速度，**没有挡位字段**，
//     负加速度只能理解成"减速"，无法与"倒车"区分。真实栈（如 Autoware）
//     用独立的 GearCommand 解决这个歧义。
// =============================================================================

// include 必须分三段、段间留空行：C 标准库 → C++ 标准库 → 第三方。
// 这是 cpplint 的硬性要求；.clang-format 里设了 IncludeBlocks: Preserve，
// clang-format 只在**段内**排序，不会把三段合并 —— 合并了 cpplint 就报错。
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

#include <ads_msgs/msg/vehicle_cmd.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ads_teleop
{

/// @brief 把 stdin 切成「raw + 非阻塞」，析构时还原。
///
/// 为什么必须用 RAII：raw 模式下终端不回显、不按行缓冲。程序如果异常退出而
/// 没还原，用户的 shell 会变成"打字看不见、回车不生效"的状态，只能靠
/// `reset` 命令救回来。析构函数保证无论正常退出还是 Ctrl-C 都能还原。
///
/// ⚠️ 为什么光靠 termios 的 VMIN/VTIME 不够 —— 这是实际踩过的坑
/// -----------------------------------------------------------
/// `VMIN=0 / VTIME=0` 让 read() 立即返回，但它**只对终端生效**。
/// 在 `ros2 launch` 下，子进程的 stdin 是**管道**而不是终端：
/// tcgetattr 失败 → raw 模式没设上 → read() 在管道上**永久阻塞** →
/// 定时器回调卡死 → 节点发布者存在但一条消息都发不出来。
///
/// 症状极具迷惑性：进程活着（状态 S）、启动日志正常打印、
/// `ros2 topic info` 也能看到 publisher，就是一条数据都没有。
///
/// 所以必须额外给 fd 加 O_NONBLOCK —— 它对管道、终端、/dev/null 一视同仁。
class StdinRawMode
{
public:
  StdinRawMode()
  {
    // 先加 O_NONBLOCK。这一步与 stdin 是什么类型无关，必须成功。
    old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (old_flags_ != -1) {
      fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
    }

    if (tcgetattr(STDIN_FILENO, &saved_) != 0) {
      return;  // 不是终端：非阻塞已生效，读不到按键但不会卡死
    }
    is_tty_ = true;
    struct termios raw = saved_;
    // ICANON 关掉 = 不等回车，逐字符可读；ECHO 关掉 = 按键不回显到屏幕
    raw.c_lflag &= ~(static_cast<tcflag_t>(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  ~StdinRawMode()
  {
    if (is_tty_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
    if (old_flags_ != -1) {
      fcntl(STDIN_FILENO, F_SETFL, old_flags_);
    }
  }

  bool is_tty() const { return is_tty_; }

  StdinRawMode(const StdinRawMode &) = delete;
  StdinRawMode & operator=(const StdinRawMode &) = delete;

private:
  // 写 `termios` 而不是 `struct termios`：后者会让 clang-format 把 `{}` 误认成
  // 结构体定义体，格式化成三行、看着像在定义一个叫 saved_ 的结构体。
  // C++ 不需要这个 struct 前缀（C 才需要），去掉即可。
  termios saved_{};
  int old_flags_{-1};
  bool is_tty_{false};
};

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop() : Node("keyboard_teleop")
  {
    // ⚠️ 限值由 launch 从 config/vehicle_params.yaml 读出来传进来，
    //    这里**不写任何车辆物理参数**（SPEC §4.1 单一来源）。
    //    默认值给 0：launch 忘了传参时车根本不会动，是"功能失效但行为安全"，
    //    比默认一个大数导致车按错误的限值飞出去要好。
    max_steer_rad_ = declare_parameter<double>("limits.max_steer_angle_rad", 0.0);
    max_accel_ = declare_parameter<double>("limits.max_accel_mps2", 0.0);
    max_decel_ = declare_parameter<double>("limits.max_decel_mps2", 0.0);
    emergency_decel_ = declare_parameter<double>("limits.emergency_decel_mps2", 0.0);

    // 每次按键的增量 = 限值的 1/5，也就是连按 5 次到底。
    // 调大：响应快但难精调；调小：好精调但要狂按键。
    // 取 1/5 是因为键盘只有"按/不按"两种状态，没有踏板那样的连续行程，
    // 档位太多反而不好用。
    const int kSteps = 5;
    steer_step_ = max_steer_rad_ / kSteps;
    accel_step_ = max_accel_ / kSteps;
    decel_step_ = max_decel_ / kSteps;

    // 发布频率。下游 vehicle_cmd_bridge 有看门狗（默认 500 ms 没收到指令就刹车），
    // 所以这里必须持续发，而不是只在按键时发一次。
    // 20 Hz 远高于看门狗阈值，掉几帧也不会误触发刹车。
    const double rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);

    pub_ = create_publisher<ads_msgs::msg::VehicleCmd>("/vehicle_cmd", 10);

    // ⚠️ 这里用 create_wall_timer（墙上时钟），而下游 vehicle_cmd_bridge 用的是
    //    create_timer + 节点时钟（仿真钟）。**两者不一致是有意的**，但有一个
    //    成立条件，写在这里免得将来有人踩：
    //
    //    用墙钟的理由：本节点的驱动源是**人按键盘**，而人是按真实时间反应的。
    //    改用仿真钟的话，RTF 掉到 0.1 时轮询频率也跟着掉到 2 Hz（墙上时间），
    //    手感会变成按一下等半秒才响应。下游 bridge 则相反 —— 它做的是
    //    v += a·dt 的积分，必须走仿真钟，否则 RTF 偏离 1 时积出来的速度就是错的。
    //
    //    成立条件：看门狗按**仿真时间**判超时（阈值 cmd_timeout_s = 0.5 仿真秒），
    //    而本节点的发布间隔换算成仿真时间是 (1/rate_hz) × RTF。于是
    //
    //        (1 / 20) × RTF > 0.5   ⟹   RTF > 10   → 看门狗持续误触发，车永远刹停
    //
    //    当前实测 RTF = 1.000，余量 10 倍；而且人也不可能在 10 倍速下开车，
    //    所以现状安全。**但这是个隐含假设**：如果将来有人把本节点拿去做
    //    加速回放下的指令注入（RTF 远大于 1），就会撞上它，
    //    症状是「指令明明在发，车却一直被看门狗刹住」。
    //    届时的正确改法是把这里换成节点时钟，而不是去调大看门狗阈值 ——
    //    调大阈值等于削弱失联保护，那是安全逻辑（SPEC §11）。
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), std::bind(&KeyboardTeleop::on_timer, this));

    print_help();

    // stdin 不是终端时明确告警，而不是安静地"按键没反应"。
    // 最常见的触发方式是用 `ros2 launch` 起本节点 —— launch 会接管子进程的
    // stdio，键盘输入根本到不了这里。交互驾驶必须用 `ros2 run`。
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_WARN(
        get_logger(),
        "stdin 不是终端，读不到键盘。节点会继续以零指令发布（下游看门狗因此不会误触发），"
        "但车不会动。交互驾驶请用：ros2 run ads_teleop keyboard_teleop --ros-args ...");
    }
  }

private:
  void print_help() const
  {
    // 用 \r\n 而不是 \n：raw 模式下终端不做 NL→CRNL 转换，
    // 只发 \n 的话光标只下移不回到行首，输出会显示成阶梯状。
    std::printf("\r\n");
    std::printf("  ================ 键盘驾驶 ================\r\n");
    std::printf(
      "     w / s    加速 / 减速（每次 %.2f / %.2f m/s^2）\r\n", accel_step_, decel_step_);
    std::printf("     a / d    左转 / 右转（每次 %.3f rad）\r\n", steer_step_);
    std::printf("     空格     松油门 + 方向回正\r\n");
    std::printf("     b        紧急制动（%.1f m/s^2）\r\n", emergency_decel_);
    std::printf("     q        退出\r\n");
    std::printf("  ------------------------------------------\r\n");
    std::printf("   注意：静止时打方向车不会转，阿克曼转向要先有速度\r\n");
    std::printf("  ==========================================\r\n\r\n");
    std::fflush(stdout);
  }

  /// @brief 把值限制到 [lo, hi]。
  static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

  void on_timer()
  {
    // 一次把缓冲区里堆积的按键全读掉。
    // 只读一个的话，用户快速连按时按键会在缓冲区里排队，
    // 表现为"手松开了车还在继续加速"。
    char c = 0;
    while (::read(STDIN_FILENO, &c, 1) == 1) {
      handle_key(c);
    }

    ads_msgs::msg::VehicleCmd msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.steer_angle_rad = steer_rad_;
    msg.accel_mps2 = accel_;
    pub_->publish(msg);

    // 状态行原地刷新（\r 回到行首，不换行）。
    //
    // ⚠️ 只在指令**变化时**才打印，不是每拍都打。
    // 每拍都打（20 Hz × 60 字符 ≈ 1.2 KB/s）的两个实际后果：
    //   1. 在 ros2 launch 下会把其他节点的日志全部淹掉；
    //   2. 如果 stdout 接的是没人读的管道/伪终端，缓冲区填满后 printf 会
    //      **阻塞**，定时器回调卡死，节点从此不再响应按键 ——
    //      这个问题在写 4.1 的自动化测试时真的踩到了。
    if (steer_rad_ != last_printed_steer_ || accel_ != last_printed_accel_) {
      last_printed_steer_ = steer_rad_;
      last_printed_accel_ = accel_;
      std::printf(
        "\r  转角 %+.3f rad (%+5.1f deg)   加速度 %+.2f m/s^2      ", steer_rad_,
        steer_rad_ * 180.0 / M_PI, accel_);
      std::fflush(stdout);
    }
  }

  void handle_key(char c)
  {
    switch (c) {
      case 'w':
        // 上限是 max_accel，下限是 -max_decel：
        // 「减速」在本消息里就是负加速度，不需要单独的刹车字段。
        accel_ = clamp(accel_ + accel_step_, -max_decel_, max_accel_);
        break;
      case 's':
        accel_ = clamp(accel_ - decel_step_, -max_decel_, max_accel_);
        break;
      case 'a':
        steer_rad_ = clamp(steer_rad_ + steer_step_, -max_steer_rad_, max_steer_rad_);
        break;
      case 'd':
        steer_rad_ = clamp(steer_rad_ - steer_step_, -max_steer_rad_, max_steer_rad_);
        break;
      case ' ':
        accel_ = 0.0;
        steer_rad_ = 0.0;
        break;
      case 'b':
        // 紧急制动是唯一允许突破 max_decel 的路径（SPEC §4.1）。
        // 真实系统里它由安全模块触发，P0a 先挂在键盘上，
        // 目的是让下游的限幅逻辑有东西可测。
        accel_ = -emergency_decel_;
        steer_rad_ = 0.0;
        break;
      case 'q':
        std::printf("\r\n  退出。\r\n");
        std::fflush(stdout);
        rclcpp::shutdown();
        break;
      default:
        break;  // 其他键忽略
    }
  }

  // 左转为正（ROS 右手系绕 z 轴），与 ads_msgs/VehicleCmd 的约定一致
  double steer_rad_{0.0};
  double accel_{0.0};

  // 上次打印过的值。初值设成 NaN，保证第一拍一定会打印一次，
  // 用户启动后立刻能看到状态行而不是一片空白。
  double last_printed_steer_{std::nan("")};
  double last_printed_accel_{std::nan("")};

  double max_steer_rad_{0.0};
  double max_accel_{0.0};
  double max_decel_{0.0};
  double emergency_decel_{0.0};
  double steer_step_{0.0};
  double accel_step_{0.0};
  double decel_step_{0.0};

  rclcpp::Publisher<ads_msgs::msg::VehicleCmd>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ads_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    // raw + 非阻塞的作用域包住 spin：spin 返回后（Ctrl-C 或按 q）
    // 析构函数立刻还原终端，不会把用户的 shell 留在不回显状态。
    //
    // ⚠️ try/catch 不是装饰（2026-08-12 复检补上）：未捕获异常下 GCC
    //    **不保证栈回退**，StdinRawMode 的析构不执行，终端被留在
    //    raw + 无回显 + O_NONBLOCK —— 用户的 shell 看起来"死了"，
    //    只能盲敲 reset。参数覆盖类型错误就足以触发。
    ads_teleop::StdinRawMode raw_mode;
    rclcpp::spin(std::make_shared<ads_teleop::KeyboardTeleop>());
  } catch (const std::exception & e) {
    // raw_mode 已随作用域退出还原终端，这里才能安全打印。
    fprintf(stderr, "keyboard_teleop 异常退出：%s\n", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
