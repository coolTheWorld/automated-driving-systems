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

#ifndef ADS_CONTROL__STANLEY_HPP_
#define ADS_CONTROL__STANLEY_HPP_

// =============================================================================
//  横向控制：Stanley —— 纯 C++17，**不依赖 ROS**
//
//      δ = ψ − arctan( k_e · e / (k_soft + v) )
//
//  完整推导见 docs/modules/control.md §3。这里只重复**做错了不会报错**的四条，
//  它们各自的症状都是「车能开，就是开得不太对」：
//
//  1. **必须用前轴。** 本项目 base_link 在**后轴**中心（Autoware 惯例），
//     而 Stanley 的推导对象是前轮的横向运动。漏掉换算的症状是：低速几乎看不出来，
//     速度越高越明显地持续外偏，弯道比直道严重。而人的第一反应是「增益不够」，
//     调大 k_e 能压住一部分稳态偏差，代价是低速开始震荡 —— 于是陷入
//     「快了偏、慢了晃」的死循环，两头都调不好，因为控制律的输入点就是错的。
//     用 front_axle_pose() 换算，别自己写那两行。
//
//  2. **本项目是减号。** 教科书写加号，因为教科书的 e 右正；本项目 e 左正
//     （与地图的横向偏移 t、ROS 的 y 轴、转角 δ 一致）。符号写反是**正反馈**，
//     车会立刻冲出路径 —— 这反而是这四条里最不用担心的，因为它藏不住。
//
//  3. **不要加曲率前馈。** 「曲率都算出来了，顺手加个 δ_ff = arcsin(L·κ)
//     让它入弯更快」是个看起来毫无争议的改进，实际是**双倍补偿**：
//     §3.4 证明定曲率上 ψ ≡ δ 是几何恒等式，航向项已经把曲率补掉了。
//     加了之后 R=8 m 弯道上稳态误差 1.42 m（车道越界线才 0.85 m），
//     而它在直路上毫无影响、所有直线用例全绿。
//     由 test_stanley.cpp 的 CurvatureFeedforwardBreaksTheSteadyState 盯着。
//
//  4. **转向速率限幅必须由控制器实现。** gazebo_bridge 只截断转角、且是**静默**
//     截断；转向速率在 Gazebo 侧根本没生效（SDF 的 AckermannSteering 插件
//     自己管转向速度，URDF 里那个 max_steer_rate 只进了关节限位）。
//     所以这一层不实现它，整条链路上就没人实现它。
//
//     ⚠️ 但它**不是**一个大的相位滞后 —— 这一点实测推翻了立项时的估计。
//     "0.500 rad/s 意味着零到满舵要 1.2 s"这句话本身没错，错在拿它当入弯的
//     工况：入弯时 e ≈ 0 于是 δ ≈ ψ，而 ψ 以路径切向的转速 **v/R** 增长，
//     根本不是阶跃。于是限幅起作用的临界车速是 `v* = R · 速率上限`，
//     本地图最急的弯上 = 8 × 0.5 = **4.0 m/s**，而曲率限速给的是 3.033
//     （P3-S5 起 a_lat_max = 1.15；S2 写这段时是 1.5 → 3.464）—— 差 24%。
//     实测入弯峰值只有 1.6 cm。由 TheSteeringRateLimitOnlyBindsAboveRadiusTimesRateLimit
//     钉住这条边界，它同时也是交给 S3 的约束（a_lat_max 调到 2.0 就顶在临界值上）。
// =============================================================================

#include "ads_common/reference_line.hpp"

namespace ads_control
{

/// @brief 把 `base_link`（后轴中心）位姿换算成**前轴中心**位姿。
///
/// @param rear_axle_pose 后轴中心位姿（= ROS 的 `base_link`），map 系。
/// @param wheelbase_m    轴距，单位 m。来自 `config/vehicle_params.yaml`
///                       的 `geometry.wheelbase_m`（2.700），**不得在控制器里写死**。
/// @return 前轴中心位姿：位置前移一个轴距，**朝向不变**（刚体，车头方向就是车身方向）。
/// @throw std::invalid_argument 轴距非有限或非正；或位姿含非有限值。
///
/// @note 换算本身只有两行 `x + L·cosθ` —— 之所以做成一个带校验的函数而不是
///       让调用方现写，是因为**漏掉这一步是 Stanley 最经典的实现错误**（见文件头第 1 条），
///       而漏掉它不会报错。有了这个函数，S4 的节点里就会出现一处显眼的
///       `front_axle_pose(...)`，代码审查一眼能看见它在不在。
///
/// @note 轴距为 0 会让前轴 = 后轴，也就是**静默退化成那个经典错误**。
///       所以这里宁可抛异常：`wheelbase_m` 传错（比如读配置读到了默认值 0）
///       的后果太隐蔽，不值得为省一次比较而放过。
ads_common::Pose2D front_axle_pose(const ads_common::Pose2D & rear_axle_pose, double wheelbase_m);

/// @brief Stanley 控制器的全部参数。
///
/// 分两类来源，**不要混**：
///   * `gain_inv_s` / `soft_speed_mps` 是**控制器自己的**调参，来自
///     `config/control_params.yaml`（S4 新建）。
///   * `max_steer_angle_rad` / `max_steer_rate_rad_s` 是**车辆能力**，
///     来自 `config/vehicle_params.yaml`，控制器**不得重新定义**（SPEC §4.1）。
///     在两个地方各写一份是"行为漂移"的头号成因。
///
/// @note 结构体**故意不给默认值**。给了默认值的话，S4 忘了从 YAML 读某一项时
///       会拿到一个"看起来合理"的数继续跑；不给的话聚合初始化会把漏掉的项填 0，
///       而 0 会被下面构造函数的校验拦下并明确报出是哪一项。
///       **让错误在启动时喊出来，而不是在弯道上表现为"控制器不行"。**
struct StanleyParams
{
  /// 横向误差增益 `k_e`，量纲 **1/s**（不是无量纲！）。
  ///
  /// 物理含义：直线上小误差时闭环退化成 `ė ≈ −k_e·e`，时间常数 `1/k_e`。
  /// **收敛速度与车速无关**（前提 `k_soft ≪ v`），这是 Stanley 相对纯追踪
  /// 最大的优点 —— 纯追踪的前视距离必须随速度调。
  ///
  /// 初值 1.0（时间常数 1 s，从 1.0 m 收到 0.05 m 理论 ln(20) = 3.0 s）。
  /// 调大（→ 3.0）→ 收敛快，但与 0.5 rad/s 的速率限幅耦合后易震荡；
  /// 调小（→ 0.5）→ 收敛慢，入弯暂态偏差变大。经验范围 0.5–3.0。
  double gain_inv_s;

  /// 低速软化项 `k_soft`，单位 **m/s**。
  ///
  /// 存在的唯一理由：`v → 0` 时 `k_e·e/v → ∞`，`arctan` 饱和到 ±π/2，
  /// **转角瞬间打死**。而低速正是起步和停车的工况，每次运行都必然经过。
  ///
  /// 初值 0.5：巡航 5.556 m/s 下打折 8.3%（几乎无感），
  /// 0.5 m/s 的爬行速度下打折 50%（保守，正确）。
  /// 调大（→ 2.0）→ 低速平稳，但**高速也在打折**（5.556 下收敛慢 26%）；
  /// 调小（→ 0）→ 低速修正积极但接近停车时转角剧烈抖动，等于 0 时打死。
  double soft_speed_mps;

  /// 前轮最大转角，单位 rad。**来自 `vehicle_params.yaml`**（0.600）。
  ///
  /// ⚠️ R=8 m 的路口转弯车道上，光是稳态转角就要 0.3443 rad = **57%**，
  ///    只剩 43% 留给误差修正。把地图的 `turn_radius_m` 从 8 改到 5，
  ///    `arcsin(2.7/5) = 0.5686` 直接吃掉 95% —— **车将无法修正任何误差**，
  ///    而症状看起来完全像"控制器不行"。这是一条改**地图**时会踩的约束。
  double max_steer_angle_rad;

  /// 转向执行机构的角速度上限，单位 rad/s。**来自 `vehicle_params.yaml`**（0.500）。
  ///
  /// 调大 → 响应快，但会下发真车执行机构跟不上的指令，仿真里好用、真车上
  /// 变成"指令与实际转角持续有差"；调小 → 入弯跟不上路径切向的转速。
  ///
  /// 判断它会不会成为瓶颈有个现成的公式：入弯所需的转向速率就是 **v/R**，
  /// 所以临界车速 `v* = R · max_steer_rate_rad_s`。本地图 R=8 → 4.0 m/s，
  /// 而曲率限速给 3.033 m/s（a_lat_max = 1.15），**低 24%，限幅碰不到**
  /// （见文件头第 4 条）。
  double max_steer_rate_rad_s;
};

/// @brief Stanley 横向控制器。
///
/// **有状态**：只存一个量 —— 上一拍的转角输出，用于转向速率限幅。
/// 换路径、重新接管控制时必须 `reset()`，否则速率限幅会从一个陈旧的转角出发。
class StanleyController
{
public:
  /// @param params 见 StanleyParams。
  /// @throw std::invalid_argument 任一参数非有限或非正。
  ///
  /// @note 四项**都要求严格为正**。`soft_speed_mps == 0` 时车速为零会得到
  ///       `atan2(0, 0)` = 0，看着"没事"，但只要横向误差非零就是 `atan2(±x, 0)`
  ///       = ±π/2 —— 停车时方向盘打死。零值不是"关掉这个功能"，是奇点。
  explicit StanleyController(const StanleyParams & params);

  /// @brief 纯控制律：`δ = ψ − arctan(k_e·e / (k_soft + v))`。**无状态、不限幅。**
  ///
  /// @param heading_error_rad 航向误差 `ψ = normalize(θ_path − θ_vehicle)`，
  ///                          取自 `PathProjection::heading_error_rad`。
  /// @param lateral_error_m   **前轴**横向误差，**左正**，
  ///                          取自 `PathProjection::lateral_error_m`。
  /// @param speed_mps         车速，m/s。见下面关于负值的说明。
  /// @param params            控制器参数。
  /// @return 未限幅的转角指令，rad，左正。
  ///
  /// @note 做成 `static` 是为了让 L1 能**逐点比对手算值**而不必构造有状态对象，
  ///       也让"控制律"和"限幅"两件事在测试里可以分开验。
  ///
  /// @note 实现用 `atan2(k_e·e, k_soft + v)` 而不是 `atan(k_e·e/(k_soft+v))`：
  ///       两者在分母恒正时完全等价，但前者根本不做除法 ——
  ///       少一处将来有人改坏 `k_soft` 校验后就会炸的地方。
  ///
  /// @note **车速为负时按 0 处理。** 本项目不支持倒车（control.md §7），
  ///       而负车速会让分母 `k_soft + v` 变号甚至过零 —— 那是**正反馈**，
  ///       车会一头扎出路径。夹到 0 只是数值保护，**不是**倒车支持。
  ///       实践中 `/odom` 在停车时抖出 −0.001 m/s 很常见，为此抛异常会
  ///       在最不该出事的时候把节点搞死，所以这里选择夹取。
  ///       ⚠️ 相应地，**S4 的节点必须对持续为负的车速打限流告警** ——
  ///       否则真的挂了倒挡时这一层会安静地把它当成静止。
  static double raw_steering_rad(
    double heading_error_rad, double lateral_error_m, double speed_mps,
    const StanleyParams & params);

  /// @brief 走一拍：控制律 → 转角限幅 → **转向速率**限幅。
  ///
  /// @param heading_error_rad 同 raw_steering_rad()。
  /// @param lateral_error_m   同上。
  /// @param speed_mps         同上。
  /// @param dt_s              距上一拍的时间间隔，单位 s，**必须用节点时钟
  ///                          （仿真时间），不是墙钟**（SPEC §3.3）。
  /// @return 限幅后的转角指令，rad，左正。同时被记为"上一拍的转角"。
  /// @throw std::invalid_argument 任一入参非有限；或 `dt_s < 0`。
  ///
  /// @note `dt_s == 0` **不是**错误：速率限幅允许的变化量恰好是 0，
  ///       于是原样返回上一拍的转角。这从公式里自然掉出来，不需要特判。
  ///       而 `dt_s < 0` 是错误 —— 它意味着时钟倒流（本仓库见过：
  ///       两套仿真并存时 TF 刷 "jump back in time"），把它当成正常值
  ///       会让限幅方向反过来。
  ///
  /// @note **非有限入参一律抛异常，不夹取。** NaN 会污染 `steering_rad_`，
  ///       而那是**持久状态** —— 一拍的坏数据会让控制器**永久**输出 NaN，
  ///       之后即使输入恢复正常也回不来（NaN 参与任何运算仍是 NaN）。
  ///       现场表现是「车自己停了」（下游 isfinite 挡下 + 看门狗刹停），
  ///       于是所有人去查控制器 —— 而错在上游。本仓库已经吃过两次同源的亏，
  ///       见 CLAUDE.md 陷阱表「用比较去拦非有限值」。
  double update(double heading_error_rad, double lateral_error_m, double speed_mps, double dt_s);

  /// @brief 重置速率限幅的状态。
  ///
  /// @param steering_rad 认为"当前"的转角，rad。默认 0 = 方向盘回正，
  ///                     这与 `vehicle_cmd_bridge` 启动时的状态一致。
  /// @throw std::invalid_argument `steering_rad` 非有限。
  ///
  /// @note 值会被夹到 `±max_steer_angle_rad`，保证任何时候
  ///       `|steering_rad()| ≤ max_steer_angle_rad` 都成立 ——
  ///       这条不变量是 `update()` 里"限幅后仍在范围内"推理的前提。
  void reset(double steering_rad = 0.0);

  /// @brief 上一拍的转角输出，rad。恒满足 `|·| ≤ max_steer_angle_rad`。
  double steering_rad() const noexcept { return steering_rad_; }

  /// @brief 构造时传入的参数。
  const StanleyParams & params() const noexcept { return params_; }

private:
  StanleyParams params_;
  /// 上一拍的转角输出。**唯一的状态**，只服务于速率限幅。
  double steering_rad_{0.0};
};

}  // namespace ads_control

#endif  // ADS_CONTROL__STANLEY_HPP_
