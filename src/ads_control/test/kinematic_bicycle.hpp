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

#ifndef KINEMATIC_BICYCLE_HPP_
#define KINEMATIC_BICYCLE_HPP_

// =============================================================================
//  运动学自行车模型 —— **测试夹具，不是产品代码**
//
//      ẋ = v·cos θ        ẏ = v·sin θ
//      θ̇ = v·tan(δ) / L   v̇ = a
//
//  存在的理由：CP-P2-A 要在一个**不需要 ROS、不需要 Gazebo、毫秒级**的闭环里
//  证明控制律收敛。在 Gazebo 里验同一件事要几十秒、要 GPU、还进不了 CI。
//
//  ⚠️ **它证不了车在 Gazebo 里能开。** Gazebo 是 dartsim 的真物理：有质量、
//     惯量、轮胎摩擦、悬架、执行机构延迟。两者的差异正是 CP-P2-B 要验的东西。
//     三层各验各的：L1 验**控制律**、假车闭环验**节点接线**、CP-P2-B 验**真物理**。
//     把这里的绿灯当成"控制做完了"，正是 SPEC §8「没崩溃不等于对了」防的那件事。
//
//  ⚠️ 模型里**没有轮胎**，所以不存在"侧滑/飘出去"这回事 —— 定速过弯不会甩尾，
//     再大的横向加速度也照样贴着路径走。这曾经把一条判据的理由写错过
//     （见 tasks/plan.md §P2-1 决策二的更正块）。
//
//  放在 test/ 而不是 src/：SPEC §3.3 的 lib/ 是**产品**算法。
//  P2-S5 会把这个模型包成一个 ROS 假车节点做不需要 GPU 的闭环回归（L3-G，进 CI），
//  届时它要发 /odom + TF + /clock —— 那时它才成为产品代码。
// =============================================================================

#include <cmath>

#include "ads_common/angles.hpp"
#include "ads_common/reference_line.hpp"

namespace ads_common_test
{

/// @brief 运动学自行车模型的一辆车。状态量全部公开 —— 这是夹具，不是接口。
struct KinematicBicycle
{
  /// 轴距，m。来自 `config/vehicle_params.yaml`，由调用方传进来。
  double wheelbase_m{2.700};

  /// **后轴中心**位置（= ROS 的 `base_link`），map 系。
  double x_m{0.0};
  double y_m{0.0};
  /// 车身朝向，rad，落在 [−π, π]（每步归一化，与真实 ROS 四元数的值域一致）。
  double heading_rad{0.0};
  /// 纵向车速，m/s。
  double speed_mps{0.0};

  /// @brief 前向欧拉积分一步。
  ///
  /// @param steering_rad 前轮转角，rad，左正。
  /// @param accel_mps2   纵向加速度，m/s²。
  /// @param dt_s         步长，s。
  ///
  /// @note **RK1（前向欧拉）足够**：步长 0.02 s、闭环时间常数 1 s，比值 50，
  ///       欧拉的相对误差在 1% 量级，远小于本文所有判据的余量。
  ///       用更高阶的积分器不会让判据变紧，只会让"实测值为什么是这个数"
  ///       更难解释 —— 而这个模型存在的意义就是**每个数字都能手推**。
  ///
  /// @note 所有导数都取**更新前**的状态（这才是显式欧拉）。边算边覆盖会变成半隐式。
  ///
  ///       ⚠️ 这里**曾经**写着"半隐式会让圆的半径系统性偏小"。**那是想当然** ——
  ///       故障注入实测：改成半隐式后 27 个用例**一条都不红**。原因是两种写法
  ///       画出的是**同一个正多边形**，只差起始朝向的半步相位，外接圆半径完全相同。
  ///
  ///       所以这条的真实要求只是「同一步里的所有导数取自**一致的**状态」，
  ///       而不是"显式比半隐式准"。混用（位置用旧朝向、朝向又用新速度之类）
  ///       才是真会出事的写法，而那个测试套件同样看不出来 ——
  ///       **记下来是为了不让下一个人以为这里有测试在保护。**
  void step(double steering_rad, double accel_mps2, double dt_s)
  {
    const double speed_before_mps = speed_mps;
    const double heading_before_rad = heading_rad;

    x_m += speed_before_mps * std::cos(heading_before_rad) * dt_s;
    y_m += speed_before_mps * std::sin(heading_before_rad) * dt_s;
    // θ̇ = v·tan δ / L。tan 在 δ → ±π/2 时发散，但转角早已被限幅到 0.600 rad
    // （tan = 0.684），离奇点很远。
    heading_rad = ads_common::normalize_angle(
      heading_before_rad + speed_before_mps * std::tan(steering_rad) / wheelbase_m * dt_s);
    speed_mps += accel_mps2 * dt_s;
  }

  /// @brief 后轴中心位姿（= `base_link`）。这是定位/里程计会给出的东西。
  ads_common::Pose2D rear_axle_pose() const { return {x_m, y_m, heading_rad}; }

  /// @brief 前轴中心位姿 —— **Stanley 要的就是它**，见 control.md §3.2。
  ///
  /// 这里刻意不复用产品代码的 `ads_control::front_axle_pose()`：
  /// 用被测函数去构造被测函数的期望值，等于什么都没验。
  ads_common::Pose2D front_axle_pose() const
  {
    return {
      x_m + wheelbase_m * std::cos(heading_rad), y_m + wheelbase_m * std::sin(heading_rad),
      heading_rad};
  }
};

}  // namespace ads_common_test

#endif  // KINEMATIC_BICYCLE_HPP_
