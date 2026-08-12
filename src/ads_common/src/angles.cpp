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

#include "ads_common/angles.hpp"

#include <cmath>

namespace ads_common
{

namespace
{
/// 2π。写成 constexpr 而不是宏，也不用 M_PI（M_PI 不是标准 C++，
/// 在某些编译器上要先 define _USE_MATH_DEFINES 才有）。
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
}  // namespace

double normalize_angle(double angle_rad)
{
  // 用 std::remainder 而不是常见的三种写法，理由值得写下来：
  //
  //   1) while (a > π) a -= 2π;  —— 输入很大时要循环很多次。更糟的是
  //      浮点减法反复做会累积误差，而角度累加（比如里程计积分航向）
  //      恰恰是最容易出现大角度的地方。
  //
  //   2) std::fmod(a, 2π)  —— 结果符号跟随**被除数**，范围是 (-2π, 2π)，
  //      不是我们要的 [-π, π]，还得再补一次判断。漏补是常见 bug。
  //
  //   3) std::atan2(std::sin(a), std::cos(a))  —— 结果对，但要算三个
  //      超越函数，且 sin/cos 在大角度上本身就有精度损失。
  //
  // std::remainder(x, y) 返回 x − n·y，其中 n 是**最接近** x/y 的整数
  // （C++11 起标准保证，半数情况取偶）。所以商落在 [-0.5, 0.5]，
  // 余数天然落在 [-y/2, y/2] = [-π, π]。一次调用，无循环，无额外分支。
  //
  // 边界：±π 时商恰为 ±0.5，"取偶"规则让 n = 0，于是原样返回 ±π。
  //       这就是头文件里说的"闭区间、不把 -π 折成 +π"。
  return std::remainder(angle_rad, kTwoPi);
}

double angle_diff(double from_rad, double to_rad)
{
  // 先做减法再归一化，不能反过来。
  // 反过来（先各自归一化再相减）在跨越 ±π 时照样得到接近 ±2π 的结果 ——
  // 例如 from = 179°、to = -179°：两者都已在 [-π, π] 内，
  // 直接相减 to − from = -358°，而正确答案是 **+2°**（从 179° 左转 2° 跨过 π
  // 就到 -179°；本函数约定左转为正）。
  //
  // ⚠️ 这段注释曾把两个符号都写反（写成"相减得 358°、正确答案 -2°"——
  //    那是 from − to 的算法）。代码一直是对的、测试锁定的也是对的，
  //    错的只有注释 —— 而注释错比没注释更糟：下一个人会拿它当依据
  //    去"修"一段没坏的代码。2026-08-12 复检发现并改正。
  return normalize_angle(to_rad - from_rad);
}

}  // namespace ads_common
