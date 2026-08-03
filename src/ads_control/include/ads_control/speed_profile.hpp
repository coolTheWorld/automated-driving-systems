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

#ifndef ADS_CONTROL__SPEED_PROFILE_HPP_
#define ADS_CONTROL__SPEED_PROFILE_HPP_

// =============================================================================
//  速度剖面：路径上每一点该跑多快 —— 纯 C++17，**不依赖 ROS**
//
//  三步，缺一不可（推导见 docs/modules/control.md §4.1–§4.3）：
//
//    ① 曲率限速   v[i] = min(v_cruise, √(a_lat_max / |κ[i]|))
//    ② 终点归零 + 后向扫描   v[i] = min(v[i], √(v[i+1]² + 2·a_dec·Δs))
//    ③ 前向扫描              v[i] = min(v[i], √(v[i−1]² + 2·a_acc·Δs))
//
//  **它是纯几何量**：只依赖路径的曲率和长度，不依赖车现在在哪、跑多快。
//  所以路径来的时候算一次就够，控制回调里只查表（SPEC §7「不允许在回调中做重计算」）。
//
//  ⚠️ **代价要说清楚**：剖面不含当前车速，所以它**不是**"从此刻开始的可行速度曲线"。
//     车当前速度远高于剖面值时（比如刚被 teleop 开到 8 m/s 然后切自动驾驶），
//     速度环会给出一个被限幅到 −3.0 的减速指令 —— 行为正确，但减速过程会
//     **短暂超出剖面**。这是可接受的：剖面是期望，速度环负责收敛。
//
//  ⚠️ 这一层是**纯几何**的：只看曲率和终点，不看障碍物、不看前车、不看红灯。
//     P3 的规划器接管后它退化成兜底。**不要往这里加避障** —— 那会让
//     "安全相关逻辑"藏进一个按几何算出来的表里（SPEC §11 禁止项）。
// =============================================================================

#include <cstddef>
#include <vector>

#include "ads_control/path_tracking.hpp"

namespace ads_control
{

/// @brief 速度剖面的全部参数。
///
/// 与 `StanleyParams` 一样**故意不给默认值**：漏读一项时聚合初始化填 0，
/// 而 0 会被构造函数指名报错。给了默认值反而会让"漏读配置"变成
/// 一个跑得起来、只是开得不对的系统。
struct SpeedProfileParams
{
  /// 巡航速度上限，m/s。**来自 `vehicle_params.yaml`** 的 `limits.cruise_speed_mps`（5.556）。
  ///
  /// 剖面的上界。直路上处处等于它。
  double cruise_speed_mps;

  /// 过弯的横向加速度上限，m/s²。来自 `control_params.yaml` 的
  /// `profile.max_lateral_accel_mps2`（1.5）。
  ///
  /// 调大 → 过弯快（2.0 时 R=8 处限速从 3.464 升到 4.000）；调小 → 全程慢。
  /// 舒适区 1.5–2.0，乘用车物理极限 8–9 —— 但**运动学模型不体现轮胎打滑**，
  /// 调到 5 以上仿真里"能过"而真车会失控。
  ///
  /// ⚠️ **它与转向速率限幅耦合**：入弯所需的转向速率是 `v/R`，
  ///    临界车速 `v* = R · max_steer_rate`。本地图 R=8 → `v*` = 4.0 m/s，
  ///    而 `a_lat_max = 2.0` 给出的限速**恰好就是 4.0**。
  ///    也就是说这个"只关舒适性"的参数调到 2.0 就顶在了另一个模块的限值上。
  ///    见 `docs/modules/control.md` §3.7。
  double max_lateral_accel_mps2;

  /// 加速度上限，m/s²，**正数**。来自 `vehicle_params.yaml` 的 `limits.max_accel_mps2`（1.5）。
  double max_accel_mps2;

  /// 减速度上限，m/s²，**正数**（前向扫描里用的是它的绝对值）。
  /// 来自 `vehicle_params.yaml` 的 `limits.max_decel_mps2`（3.0）。
  ///
  /// ⚠️ **不要用 `emergency_decel_mps2`（5.0）**。那是车辆的物理能力，
  ///    只有安全模块可以下发。常规剖面用它等于把每次到站都开成紧急制动。
  double max_decel_mps2;
};

/// @brief 一条路径对应的速度剖面。**不可变**，构造时一次算完。
///
/// @note 构造时**复制**了路径的弧长数组，之后不再持有 `TrackedPath` 的引用。
///       多存 n 个 double 换掉一个悬垂引用的可能 —— 而"路径对象先于剖面析构"
///       在 S4 换路径时是完全可能发生的（新剖面算好了，旧路径先被换掉）。
class SpeedProfile
{
public:
  /// @brief 从路径与参数算出剖面。
  ///
  /// @param path   已预处理的路径（提供逐点弧长与曲率）。
  /// @param params 见 SpeedProfileParams。
  /// @throw std::invalid_argument 任一参数非有限或非正。
  SpeedProfile(const TrackedPath & path, const SpeedProfileParams & params);

  /// @brief 按最近点投影查剖面速度，**O(1)**。控制回调里用这个。
  ///
  /// @param projection `TrackedPath::project()` 的返回值。
  /// @return 期望速度，m/s。
  /// @throw std::out_of_range 投影索引超出本剖面的范围（说明剖面与路径对不上了）。
  ///
  /// @note **索引对不上时抛异常而不是夹取。** 夹取的话，
  ///       "剖面还是上一条路径的"这种错误会表现为"车速莫名其妙"，
  ///       而不是一条指名道姓的报错。S4 换路径时必须同时换剖面。
  double speed_at(const PathProjection & projection) const;

  /// @brief 同上，直接给线段索引与段内比例。
  double speed_at(std::size_t index, double ratio) const;

  /// @brief 剖面**自身要求的加速度** `a_ref = ½·d(v²)/ds`，单位 m/s²。给速度环做前馈。
  ///
  /// @param projection `TrackedPath::project()` 的返回值。
  /// @return 期望加速度，m/s²。负 = 剖面要求减速；巡航段为 0。
  /// @throw std::out_of_range 同 `speed_at`。
  ///
  /// @note **为什么必须有这个东西**：`v_ref` 沿路径是一条**斜坡**（入弯前和
  ///       终点前都按 `√(2a·Δs)` 下降），而纯 P 控制器跟踪斜坡有稳态误差
  ///       `= 斜率 / K_p`。本项目里那是 `3.0 / 1.0 = 3.0 m/s` —— S4 实测
  ///       车到终点时还有 4.36 m/s、冲过去 4.26 m，
  ///       同一个原因还把最大横向加速度顶到 2.113（入弯超速 0.85 m/s）。
  ///       **调大 `K_p` 只能按比例减小它，而前馈能把它消掉**（推导见
  ///       `docs/modules/control.md` §4.4）。
  ///
  /// @note **不需要车速参数，这一点值得说清楚**（初版收了一个，后来去掉了）：
  ///       严格的链式法则是 `dv_ref/dt = (dv_ref/ds)·(ds/dt)`，而 `ds/dt` 是
  ///       **实际**车速。但前馈的定义就是"车跟在剖面上时需要多大加速度"，
  ///       那时 `ds/dt = v_ref`，于是它化简成 `½·d(v²)/ds` —— 一个只依赖剖面的量。
  ///       车偏离剖面带来的那一份差额是**反馈该管的**，塞进前馈只会让两者
  ///       职责混淆，而且在 `v_ref → 0`（终点）时还要处理除零。
  double target_accel_at(const PathProjection & projection) const;

  /// @brief 同上，直接给线段索引与段内比例。
  ///
  /// @note `ratio` 不影响结果（`v²` 在段内线性，斜率是常数），但仍然收下它 ——
  ///       将来若改成高阶插值，调用方不必跟着改签名。
  double target_accel_at(std::size_t index, double ratio) const;

  /// @brief 逐点的剖面速度，m/s。与路径点一一对应。
  const std::vector<double> & speeds_mps() const noexcept { return speeds_mps_; }

  /// @brief 路径总长度，m。
  double length_m() const noexcept { return arc_lengths_m_.back(); }

private:
  std::vector<double> speeds_mps_;
  /// 路径逐点弧长的副本。见类注释：不留 TrackedPath 的引用。
  std::vector<double> arc_lengths_m_;
};

}  // namespace ads_control

#endif  // ADS_CONTROL__SPEED_PROFILE_HPP_
