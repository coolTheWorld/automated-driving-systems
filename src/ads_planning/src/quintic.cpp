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

#include "ads_planning/quintic.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace ads_planning
{

namespace
{

void require_finite(double value, const char * name)
{
  // 单独判 isfinite 而不是靠后面的比较：NaN 参与任何比较都返回 false，
  // 于是 `span_m > 0` 这类检查对它恒为假、会原样放行。而且要拦 ±inf 不只是 NaN。
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string("QuinticPolynomial: ") + name + " 不是有限值");
  }
}

}  // namespace

QuinticPolynomial::QuinticPolynomial(
  double start_value, double start_first_derivative, double start_second_derivative,
  double end_value, double end_first_derivative, double end_second_derivative, double span_m)
{
  require_finite(start_value, "start_value");
  require_finite(start_first_derivative, "start_first_derivative");
  require_finite(start_second_derivative, "start_second_derivative");
  require_finite(end_value, "end_value");
  require_finite(end_first_derivative, "end_first_derivative");
  require_finite(end_second_derivative, "end_second_derivative");
  require_finite(span_m, "span_m");

  if (span_m <= 0.0) {
    throw std::invalid_argument(
      "QuinticPolynomial: span_m = " + std::to_string(span_m) + "，必须为正数");
  }

  // ---------------------------------------------------------------------------
  //  前三个系数：把 x = 0 代进 d、d′、d″ 直接读出来
  // ---------------------------------------------------------------------------
  //   d (0) = a₀            ⟹ a₀ = d₀
  //   d′(0) = a₁            ⟹ a₁ = d₀′
  //   d″(0) = 2a₂           ⟹ a₂ = d₀″/2
  coefficients_[0] = start_value;
  coefficients_[1] = start_first_derivative;
  coefficients_[2] = 0.5 * start_second_derivative;

  // ---------------------------------------------------------------------------
  //  后三个系数：终点三个条件构成的 3×3 线性方程组的闭式解
  // ---------------------------------------------------------------------------
  // 这就是标准的 minimum-jerk 五次式。**闭式，不要去数值求解** ——
  // 每条候选都要构造一次，最内层循环里做 3×3 消元既慢又给了数值噪声可乘之机。
  // 推导与自检见 docs/modules/planning.md §4.2。
  const double span_squared = span_m * span_m;
  const double span_cubed = span_squared * span_m;
  const double span_fourth = span_cubed * span_m;
  const double span_fifth = span_fourth * span_m;

  const double delta = end_value - start_value;

  coefficients_[3] =
    (20.0 * delta - (8.0 * end_first_derivative + 12.0 * start_first_derivative) * span_m -
     (3.0 * start_second_derivative - end_second_derivative) * span_squared) /
    (2.0 * span_cubed);

  coefficients_[4] =
    (-30.0 * delta + (14.0 * end_first_derivative + 16.0 * start_first_derivative) * span_m +
     (3.0 * start_second_derivative - 2.0 * end_second_derivative) * span_squared) /
    (2.0 * span_fourth);

  coefficients_[5] =
    (12.0 * delta - 6.0 * (end_first_derivative + start_first_derivative) * span_m -
     (start_second_derivative - end_second_derivative) * span_squared) /
    (2.0 * span_fifth);
}

// 三个求值函数一律用 Horner 嵌套而不是 pow：
// 少一半乘法，而且避免 pow 在小指数上的精度损失（pow(x,5) 走的是 exp(5·log x)）。
double QuinticPolynomial::value_at(double x) const
{
  return coefficients_[0] +
         x * (coefficients_[1] +
              x * (coefficients_[2] +
                   x * (coefficients_[3] + x * (coefficients_[4] + x * coefficients_[5]))));
}

double QuinticPolynomial::first_derivative_at(double x) const
{
  return coefficients_[1] + x * (2.0 * coefficients_[2] +
                                 x * (3.0 * coefficients_[3] +
                                      x * (4.0 * coefficients_[4] + x * 5.0 * coefficients_[5])));
}

double QuinticPolynomial::second_derivative_at(double x) const
{
  return 2.0 * coefficients_[2] +
         x * (6.0 * coefficients_[3] + x * (12.0 * coefficients_[4] + x * 20.0 * coefficients_[5]));
}

}  // namespace ads_planning
