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

#ifndef ADS_COMMON__ANGLES_HPP_
#define ADS_COMMON__ANGLES_HPP_

// =============================================================================
//  角度工具 —— 纯 C++17，**不依赖 ROS**
//
//  为什么单独抽出来，而不是在用到的地方写一行
//  ------------------------------------------
//  角度是**周期量**：359° 和 -1° 指向同一个方向，但两个 double 相差 360。
//  控制器算航向误差时如果不归一化，车会为了"少转 1°"而选择转 359° ——
//  方向盘直接打死。这是自动驾驶里最经典的一类 bug，而且：
//
//    * 不会崩溃、不会报错，只是车行为诡异
//    * 平时看不出来，只在角度跨越 ±π 那一瞬间发作
//    * 每个模块都要用（Stanley、纯追踪、定位、预测），各写一份必然有人写错
//
//  所以它属于 ads_common：**一处写对，处处受益**。
//
//  为什么这个包不能依赖 ROS（SPEC §3.3 / §8）
//  -------------------------------------------
//  L1 单元测试要保持**毫秒级**。一旦链接 rclcpp，每个测试都要初始化 ROS 上下文，
//  单个测试从 0.001 s 变成 0.5 s 起步。测试一慢，人就不跑了，测试就白写了。
//  纯函数还有个好处：没有隐藏状态，测试用例写多少都不会互相干扰。
// =============================================================================

namespace ads_common
{

/// @brief 把任意角度归一化到 (-π, π] 之外的等价表示，落入 [-π, π]。
///
/// @param angle_rad 输入角度，单位**弧度**。必须是有限值（见下方说明）。
/// @return 与输入等价、且落在 [-π, π] 闭区间内的角度，单位弧度。
///
/// @note 区间是**闭**的，±π 两个端点都可能出现，而且落在边界时
///       **符号不做保证**：它由 IEEE 754 的「就近取偶」规则决定。
///       实测 normalize_angle(π) 得 +π，而 normalize_angle(3π) 得 −π ——
///       两者数学上是同一个方向（相差整整一圈）。
///       行为被 test_angles.cpp 的 BoundarySignIsNotGuaranteed 锁定。
///
///       为什么不加特判把符号统一：±π 意味着「正对反方向」，此时左转右转
///       完全等价，是个真正的奇点。加个 if 只是把不确定性从数据挪进代码，
///       还在热路径上多一个分支，并没有消除它。
///       **调用方不应依赖边界处的符号**；真需要确定符号，说明上层逻辑
///       在这个奇点上有歧义，该在上层解决。
///
/// @warning 输入 NaN 返回 NaN；输入 ±inf 返回 NaN（inf 没有等价的有限角度）。
///          调用方若数据来自传感器，**必须先自己判 isfinite** ——
///          本函数不做检查是为了不在热路径上加分支。
///          本项目已经吃过一次亏：gpu_lidar 的无回波射线返回 ±inf 而非 NaN。
double normalize_angle(double angle_rad);

/// @brief 从 from 转到 to 的**最短**角度差。
///
/// @param from_rad 起始角，单位弧度
/// @param to_rad   目标角，单位弧度
/// @return 落在 [-π, π] 的角度差，单位弧度。正值表示逆时针（左转，
///         与 REP-103 的 z 轴向上、右手系一致）。
///
/// @note 这才是控制器该用的"误差"。直接写 `to - from` 在跨越 ±π 时会得到
///       接近 ±2π 的值，控制器会据此打满方向盘。
double angle_diff(double from_rad, double to_rad);

}  // namespace ads_common

#endif  // ADS_COMMON__ANGLES_HPP_
