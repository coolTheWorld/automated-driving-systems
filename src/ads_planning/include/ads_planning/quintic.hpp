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

#ifndef ADS_PLANNING__QUINTIC_HPP_
#define ADS_PLANNING__QUINTIC_HPP_

// =============================================================================
//  五次多项式 —— 纯 C++17，**不依赖 ROS**
//
//  横向 lattice 的每一条候选都是一条 d(s)：从自车当前的横向状态出发，
//  在 S 米之后达到某个目标横向偏移，且**平行于车道中心线、曲率一致**。
//
//  **为什么次数是 5 而不是 3 或 7**（planning.md §4.1）：
//  要固定的边界条件有六个（起点的 d、d′、d″ 和终点的三个），
//  六个条件唯一确定六个系数 ⟹ 五次。**次数是被边界条件数定死的，不是调出来的。**
//
//  为什么起点必须约束到二阶：
//    只约束 d   → 位置连续、**朝向跳变** → 转角指令跳变
//    约束到 d′  → 朝向连续、**曲率跳变** → 转角**速率**指令跳变
//    约束到 d″  → 曲率连续 → 转角速率有界
//  P2 已经量到过「被控对象比控制器慢」会怎样（τ=1.198 s ⟹ 弯道误差 0.801 m）。
//  往这样一个执行机构嘴里塞不连续的指令，症状是同一类的：**车能开完全程，只是抖**。
//
//  ⚠️ 自变量是**弧长 s，不是时间 t**。见 planning.md §4.3 ——
//     对静态障碍物这是正确的，对动态障碍物是**表达能力不足**（表达不了"等它过去"）。
// =============================================================================

#include <array>

namespace ads_planning
{

/// @brief 由两端各三个边界条件唯一确定的五次多项式。
///
/// 自变量记作 `x`，在本模块里它是**沿参考线的弧长增量**（单位 m），
/// 因变量是横向偏移 `d`（单位 m）。于是一阶导无量纲、二阶导单位 1/m。
class QuinticPolynomial
{
public:
  /// @brief 从六个边界条件构造。
  ///
  /// @param start_value             `d(0)`
  /// @param start_first_derivative  `d′(0)`
  /// @param start_second_derivative `d″(0)`
  /// @param end_value               `d(span)`
  /// @param end_first_derivative    `d′(span)`
  /// @param end_second_derivative   `d″(span)`
  /// @param span_m                  `S`，必须为**有限正数**
  /// @throw std::invalid_argument `span_m` 非正或非有限，或任一边界条件非有限。
  ///
  /// @note **本模块的调用方总是传 `end_first_derivative = end_second_derivative = 0`**
  ///       —— 绕完要回到与车道中心线平行、曲率一致。那为什么还留成参数？
  ///       因为把这个**策略**写死在多项式里，就没人再想起它是个选择了。
  ///       P7 做借道/换道时终点不一定平行，那时它是参数才改得动。
  ///       一个参数的代价，换的是「这条约定在调用点可见」。
  ///
  /// @note **非有限值单独判**：NaN 参与任何比较都返回 false，`span_m > 0`
  ///       对它恒为假会原样放行，然后每个系数都是 NaN 而不报错 ——
  ///       下游只会看到"车不动了"。要拦 ±inf 不只是 NaN。
  ///       见 CLAUDE.md 陷阱表「用比较去拦非有限值」。
  QuinticPolynomial(
    double start_value, double start_first_derivative, double start_second_derivative,
    double end_value, double end_first_derivative, double end_second_derivative, double span_m);

  /// @brief 求值 `d(x)`。
  ///
  /// @note **不做定义域检查**。`x > span_m` 时多项式照样有值，但那是**外推**，
  ///       五次式在区间外发散得很快（本模块的典型系数下，1.5·S 处已偏出米级）。
  ///       调用方必须自己把 x 限制在 [0, span]。不在这里检查是因为它在最内层循环里，
  ///       每条候选每个采样点都会调一次。
  double value_at(double x) const;

  /// @brief 一阶导 `d′(x)`，无量纲。
  double first_derivative_at(double x) const;

  /// @brief 二阶导 `d″(x)`，单位 1/m。
  double second_derivative_at(double x) const;

  /// @brief 六个系数，`coefficients()[i]` 是 `x^i` 的系数。测试与调试用。
  const std::array<double, 6> & coefficients() const noexcept { return coefficients_; }

private:
  std::array<double, 6> coefficients_{};
};

}  // namespace ads_planning

#endif  // ADS_PLANNING__QUINTIC_HPP_
