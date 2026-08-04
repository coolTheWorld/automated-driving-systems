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
//  L1：五次多项式（P3-S3）
//
//  这个类的正确性判据非常硬：**六个边界条件必须被逐项精确复现**。
//  它不是"差不多就行"的东西 —— 系数是线性方程组的唯一解，对了就是浮点级对。
//  所以这里的容差一律是 1e-9 量级，任何需要放宽的地方都说明推导错了。
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "ads_planning/quintic.hpp"

namespace
{

using ads_planning::QuinticPolynomial;

TEST(Quintic, ReproducesAllSixBoundaryConditionsExactly)
{
  // 六个条件全部取互不相同的非零值 —— 全零或对称的输入会让"某一项漏乘"
  // 这类错误相互抵消。
  constexpr double kStartValue = 0.37;
  constexpr double kStartFirst = -0.21;
  constexpr double kStartSecond = 0.043;
  constexpr double kEndValue = -0.62;
  constexpr double kEndFirst = 0.11;
  constexpr double kEndSecond = -0.017;
  constexpr double kSpanM = 23.5;

  const QuinticPolynomial polynomial(
    kStartValue, kStartFirst, kStartSecond, kEndValue, kEndFirst, kEndSecond, kSpanM);

  EXPECT_NEAR(polynomial.value_at(0.0), kStartValue, 1e-12);
  EXPECT_NEAR(polynomial.first_derivative_at(0.0), kStartFirst, 1e-12);
  EXPECT_NEAR(polynomial.second_derivative_at(0.0), kStartSecond, 1e-12);

  EXPECT_NEAR(polynomial.value_at(kSpanM), kEndValue, 1e-9);
  EXPECT_NEAR(polynomial.first_derivative_at(kSpanM), kEndFirst, 1e-9);
  EXPECT_NEAR(polynomial.second_derivative_at(kSpanM), kEndSecond, 1e-9);
}

TEST(Quintic, MatchesTheKnownUnitStepClosedForm)
{
  // planning.md §4.2 的自检：全零起点 → 单位终点、跨度 1 ⟹ d(s) = 10s³ − 15s⁴ + 6s⁵。
  // 这是最小 jerk 五次式最广为人知的一个特例，拿它当"外部真值"。
  const QuinticPolynomial polynomial(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);
  const auto & coefficients = polynomial.coefficients();

  EXPECT_NEAR(coefficients[0], 0.0, 1e-12);
  EXPECT_NEAR(coefficients[1], 0.0, 1e-12);
  EXPECT_NEAR(coefficients[2], 0.0, 1e-12);
  EXPECT_NEAR(coefficients[3], 10.0, 1e-12);
  EXPECT_NEAR(coefficients[4], -15.0, 1e-12);
  EXPECT_NEAR(coefficients[5], 6.0, 1e-12);

  // 中点值 = 0.5：五次式对称，这一条顺便验了 Horner 求值没写错。
  EXPECT_NEAR(polynomial.value_at(0.5), 0.5, 1e-12);
}

TEST(Quintic, StayingAtTheSameOffsetIsIdenticallyConstant)
{
  // 起点终点同值、导数全零 ⟹ 多项式恒等于那个常数。
  // 这条守的是「本来不该动的候选被算出了一条起伏的曲线」——
  // 在 lattice 里它就是「车已经在 d = 0.4 上、目标也是 0.4」的那条候选。
  constexpr double kOffsetM = 0.4;
  const QuinticPolynomial polynomial(kOffsetM, 0.0, 0.0, kOffsetM, 0.0, 0.0, 30.0);

  for (double x = 0.0; x <= 30.0; x += 1.3) {
    EXPECT_NEAR(polynomial.value_at(x), kOffsetM, 1e-12) << "x = " << x;
    EXPECT_NEAR(polynomial.first_derivative_at(x), 0.0, 1e-12) << "x = " << x;
    EXPECT_NEAR(polynomial.second_derivative_at(x), 0.0, 1e-12) << "x = " << x;
  }
}

TEST(Quintic, DerivativesAgreeWithNumericalDifferentiation)
{
  // 解析导数 vs 中心差分。这条抓的是「求导时某一项的系数写错」——
  // 上面那条边界条件用例只验两个端点，端点之间写错了它抓不住。
  const QuinticPolynomial polynomial(0.1, 0.05, -0.01, 0.8, 0.0, 0.0, 20.0);

  constexpr double kStepM = 1e-5;
  for (double x = 2.0; x < 18.0; x += 1.7) {
    const double numeric_first =
      (polynomial.value_at(x + kStepM) - polynomial.value_at(x - kStepM)) / (2.0 * kStepM);
    const double numeric_second = (polynomial.value_at(x + kStepM) - 2.0 * polynomial.value_at(x) +
                                   polynomial.value_at(x - kStepM)) /
                                  (kStepM * kStepM);

    // 一阶中心差分的截断误差是 O(h²) = 1e-10，取 1e-7 有三个数量级余量。
    EXPECT_NEAR(polynomial.first_derivative_at(x), numeric_first, 1e-7) << "x = " << x;
    // 二阶差分要除 h²，浮点相消把有效位吃掉大半，所以容差必须松得多 ——
    // **这是数值方法的限制，不是实现精度不够**。
    EXPECT_NEAR(polynomial.second_derivative_at(x), numeric_second, 1e-3) << "x = " << x;
  }
}

TEST(Quintic, NonZeroTerminalDerivativesAreHonored)
{
  // 本模块的调用方总是传终点导数 = 0，但接口是通用的（planning.md §4.2 的 note）。
  // 留这条用例是为了：将来 P7 做借道时真的传了非零值，它已经被验过。
  // 不测的话，那个"留给未来"的自由度实际上从没工作过。
  const QuinticPolynomial polynomial(0.0, 0.0, 0.0, 1.0, 0.3, -0.05, 10.0);
  EXPECT_NEAR(polynomial.value_at(10.0), 1.0, 1e-9);
  EXPECT_NEAR(polynomial.first_derivative_at(10.0), 0.3, 1e-9);
  EXPECT_NEAR(polynomial.second_derivative_at(10.0), -0.05, 1e-9);
}

TEST(Quintic, ThrowsOnNonPositiveOrNonFiniteSpan)
{
  EXPECT_THROW(QuinticPolynomial(0, 0, 0, 1, 0, 0, 0.0), std::invalid_argument);
  EXPECT_THROW(QuinticPolynomial(0, 0, 0, 1, 0, 0, -5.0), std::invalid_argument);
  EXPECT_THROW(QuinticPolynomial(0, 0, 0, 1, 0, 0, std::nan("")), std::invalid_argument);
  EXPECT_THROW(
    QuinticPolynomial(0, 0, 0, 1, 0, 0, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

TEST(Quintic, ThrowsOnNonFiniteBoundaryConditions)
{
  // ⚠️ 这条不是形式主义。NaN 不判的话，`span_m > 0` 对它恒为假会**原样放行**，
  //    六个系数全是 NaN，求值全是 NaN，然后 lattice 里每一次距离比较都返回 false
  //    —— 表现为「所有候选都没碰撞」，车直接开过去。**不报错、只给假结果。**
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(QuinticPolynomial(kNaN, 0, 0, 1, 0, 0, 10.0), std::invalid_argument);
  EXPECT_THROW(QuinticPolynomial(0, kNaN, 0, 1, 0, 0, 10.0), std::invalid_argument);
  EXPECT_THROW(QuinticPolynomial(0, 0, kNaN, 1, 0, 0, 10.0), std::invalid_argument);
  EXPECT_THROW(QuinticPolynomial(0, 0, 0, kNaN, 0, 0, 10.0), std::invalid_argument);
  EXPECT_THROW(QuinticPolynomial(0, 0, 0, 1, kNaN, 0, 10.0), std::invalid_argument);
  EXPECT_THROW(QuinticPolynomial(0, 0, 0, 1, 0, kNaN, 10.0), std::invalid_argument);
  EXPECT_THROW(QuinticPolynomial(-kInf, 0, 0, 1, 0, 0, 10.0), std::invalid_argument);
}

}  // namespace
