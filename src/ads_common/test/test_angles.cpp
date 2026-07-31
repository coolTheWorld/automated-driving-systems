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
//  L1 单元测试样板（SPEC §8）
//
//  这个文件是**后续所有模块的模板**，比它测的两个函数本身更重要。
//  三条规矩，每条都有代价换来的理由：
//
//  1. **不链接 ROS。** 整个文件跑完是毫秒级。一旦引入 rclcpp，每个测试都要
//     初始化 ROS 上下文，从 0.001 s 变成 0.5 s 起步 —— 测试一慢人就不跑了。
//
//  2. **每个用例说明它在防哪个 bug**，而不是复述函数在算什么。
//     「行覆盖率 80%」是手段不是目的；一个不知道在防什么的用例，
//     改代码时没人敢删，坏了也没人知道意味着什么。
//
//  3. **浮点一律用 EXPECT_NEAR**，绝不用 EXPECT_EQ 去比算出来的值。
//     角度经过加减必然带舍入误差，用 == 的测试要么碰巧过，要么随机失败。
//
//  测试名用英文：gtest 会把 TEST(A, B) 拼成 C++ 类名 A_B_Test，
//  中文标识符虽然 GCC 能编，但会出现在 xunit 报告和 --gtest_filter 里，
//  不是所有工具链都处理得好。**解释放在注释里，注释才是给人看的地方。**
//
//  跑法：
//      colcon test --packages-select ads_common      # 走完整 CTest（含 lint）
//      ./build/ads_common/test_angles                # 直接跑，快一个数量级
// =============================================================================

// gtest 放在最前面：它是 <gtest/gtest.h>，cpplint 按 .h 后缀把它归成
// 「C 系统头」，而 C 系统头必须排在 C++ 标准库之前。这也是 ROS 2 生态里
// 测试文件的惯例写法。段间空行不能省，否则 clang-format 会跨段合并排序。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "ads_common/angles.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

/// 容差。std::remainder 本身是精确运算（IEEE 754 保证无舍入），
/// 误差只可能来自用例里的加减法，1e-12 对 double 而言已经很严。
/// 调大会让真正的错误漏网；调到 0 则会被浮点噪声打成随机失败。
constexpr double kEps = 1e-12;

/// 度转弧度。测试里用度数写更直观 —— 「179° 转到 -179°」一眼能看懂，
/// 写成 3.12414 就得停下来算。**只在测试里这么做**，
/// 产品代码一律用弧度（SPEC §7）。
constexpr double deg(double degrees) { return degrees * kPi / 180.0; }

}  // namespace

// -----------------------------------------------------------------------------
// normalize_angle
// -----------------------------------------------------------------------------

// 防「过度归一化」：已经在 [-π, π] 里的值不该被动。
TEST(NormalizeAngle, InRangeValuesUnchanged)
{
  EXPECT_NEAR(ads_common::normalize_angle(0.0), 0.0, kEps);
  EXPECT_NEAR(ads_common::normalize_angle(kPi / 2), kPi / 2, kEps);
  EXPECT_NEAR(ads_common::normalize_angle(-kPi / 2), -kPi / 2, kEps);
}

// 区间是闭的：±π 本身在范围内，不会被再折一次。
TEST(NormalizeAngle, PiBoundariesStayInRange)
{
  EXPECT_NEAR(ads_common::normalize_angle(kPi), kPi, kEps);
  EXPECT_NEAR(ads_common::normalize_angle(-kPi), -kPi, kEps);
}

// **这个用例是写测试时发现文档与实现不符才加的，值得细看。**
//
// 本以为 normalize_angle(3π) 会得到 +π（毕竟 3π 和 π 差整整一圈），
// 实测是 −π。原因是 std::remainder 按 IEEE 754 的「就近取偶」选商：
//
//     π  / 2π = 0.5  → 最近的整数 0 和 1，取偶得 0 → π − 0  = +π
//     3π / 2π = 1.5  → 最近的整数 1 和 2，取偶得 2 → 3π − 4π = −π
//
// 数学上 +π 和 −π 是同一个方向，两个答案都对。错的是原先头文件里
// 「±π 返回 ±π」那句承诺 —— 它只在输入恰好是 ±π 时成立。
//
// 所以这里锁定的不是某个具体符号，而是「**边界符号不保证**」这件事本身，
// 以及它的绝对值必须是 π。谁要是加特判把符号统一了，这个用例会失败，
// 那时应当连头文件的说明一起改，而不是只改测试。
TEST(NormalizeAngle, BoundarySignIsNotGuaranteed)
{
  EXPECT_NEAR(std::abs(ads_common::normalize_angle(3 * kPi)), kPi, kEps);
  EXPECT_NEAR(std::abs(ads_common::normalize_angle(-3 * kPi)), kPi, kEps);
  // 记录当前实测行为，便于将来比对（不是承诺，是快照）。
  EXPECT_NEAR(ads_common::normalize_angle(3 * kPi), -kPi, kEps);
}

TEST(NormalizeAngle, FullTurnsAreRemoved)
{
  EXPECT_NEAR(ads_common::normalize_angle(2 * kPi), 0.0, kEps);
  EXPECT_NEAR(ads_common::normalize_angle(-2 * kPi), 0.0, kEps);
  EXPECT_NEAR(ads_common::normalize_angle(2 * kPi + kPi / 4), kPi / 4, kEps);
  EXPECT_NEAR(ads_common::normalize_angle(-2 * kPi - kPi / 4), -kPi / 4, kEps);
}

// 整个函数存在的理由就是这一条：359° 和 -1° 指向同一个方向，
// 数值上却差 360 —— 控制器拿 359 当误差就会打死方向盘。
TEST(NormalizeAngle, Deg359EqualsDegMinus1)
{
  EXPECT_NEAR(ads_common::normalize_angle(deg(359.0)), deg(-1.0), kEps);
  EXPECT_NEAR(ads_common::normalize_angle(deg(-359.0)), deg(1.0), kEps);
}

// 防 `while (a > π) a -= 2π;` 那种写法：循环次数多，浮点减法反复做会累积
// 误差。而航向角由角速度积分得来，长时间运行后累计到几十圈很正常。
// 100π + 0.5 转了 50 整圈，归一化后应当精确回到 0.5。
TEST(NormalizeAngle, LargeAnglesKeepPrecision)
{
  EXPECT_NEAR(ads_common::normalize_angle(100 * kPi + 0.5), 0.5, kEps);
  EXPECT_NEAR(ads_common::normalize_angle(-100 * kPi - 0.5), -0.5, kEps);
}

// 不是在鼓励传 NaN 进来 —— 头文件已写明调用方必须先判 isfinite。
// 这条锁定的是**失败方式**：出问题时返回 NaN（会显式传播、比较全为 false、
// 很快暴露），而不是返回某个看着正常的角度值。
// 本项目吃过一次亏：gpu_lidar 的无回波射线返回 ±inf 而非 NaN，
// skip_nans 滤不掉，结果 min/max 直接变成 ±inf。
TEST(NormalizeAngle, NonFiniteInputReturnsNaN)
{
  EXPECT_TRUE(std::isnan(ads_common::normalize_angle(std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(std::isnan(ads_common::normalize_angle(std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(ads_common::normalize_angle(-std::numeric_limits<double>::infinity())));
}

// -----------------------------------------------------------------------------
// angle_diff
// -----------------------------------------------------------------------------

TEST(AngleDiff, SameDirectionGivesZero)
{
  EXPECT_NEAR(ads_common::angle_diff(0.0, 0.0), 0.0, kEps);
  EXPECT_NEAR(ads_common::angle_diff(kPi / 3, kPi / 3), 0.0, kEps);
  // 0 和 2π 是同一个方向，差也必须是 0。
  EXPECT_NEAR(ads_common::angle_diff(0.0, 2 * kPi), 0.0, kEps);
}

// 符号约定来自 REP-103：z 轴向上、右手系，所以逆时针（左转）为正。
// 这条一旦反了，车会朝误差的反方向修正 —— 越修越偏，直接发散。
TEST(AngleDiff, PositiveMeansLeftTurn)
{
  EXPECT_NEAR(ads_common::angle_diff(0.0, kPi / 2), kPi / 2, kEps);
  EXPECT_NEAR(ads_common::angle_diff(0.0, -kPi / 2), -kPi / 2, kEps);
}

// **本函数最核心的用例。**
// 179° → -179°：直接相减得 -358°，控制器会试图右转近一整圈；
// 正确答案是左转 2°（这两个方向实际只差 2°）。
TEST(AngleDiff, TakesShortWayAcrossPi)
{
  EXPECT_NEAR(ads_common::angle_diff(deg(179.0), deg(-179.0)), deg(2.0), kEps);
  EXPECT_NEAR(ads_common::angle_diff(deg(-179.0), deg(179.0)), deg(-2.0), kEps);
}

// 锁定实现顺序：必须「先减后归一化」。两个输入本身**都已经在** [-π, π] 内，
// 所以「先各自归一化」那一步是无效操作，错误照样发生（179 - (-179) = 358）。
// 实现若被改成 normalize(to) - normalize(from)，这个用例会失败。
TEST(AngleDiff, SubtractsBeforeNormalizing)
{
  const double from = deg(179.0);
  const double to = deg(-179.0);
  EXPECT_NEAR(ads_common::normalize_angle(from), from, kEps);  // 前提：本就在区间内
  EXPECT_NEAR(ads_common::normalize_angle(to), to, kEps);
  EXPECT_NEAR(ads_common::angle_diff(from, to), deg(2.0), kEps);
}

// 180° 的差没有「更短的一边」，左转右转完全等价 —— 这是个真正的奇点。
// 所以只断言**绝对值**是 π。符号取决于 normalize_angle 的边界行为，
// 而那个行为明确"不保证"（见 BoundarySignIsNotGuaranteed）。
//
// 控制器碰到这个输入时无论选哪边都合理；真需要确定方向，说明上层
// （比如"倒车还是掉头"）有歧义没解决，不该指望这个函数替它决定。
TEST(AngleDiff, OppositeDirectionGivesMagnitudePi)
{
  EXPECT_NEAR(std::abs(ads_common::angle_diff(0.0, kPi)), kPi, kEps);
  EXPECT_NEAR(std::abs(ads_common::angle_diff(kPi, 0.0)), kPi, kEps);
}
