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
//  路径预处理的 L1 测试
//
//  全部与**解析解**比对，不读任何文件、不起 ROS。构造的路径都是圆弧和直线，
//  因为它们的弧长、曲率、切向都能手推 —— 判据不依赖任何实现细节。
//
//  这一层写错的话，Stanley 和速度规划都会跟着错，而症状会出现在很远的地方
//  （「车在弯道上开得慢」的根因可能是这里的一个 hypot 被写成了乘法）。
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "ads_common/angles.hpp"
#include "ads_common/reference_line.hpp"

namespace
{

using ads_common::PathPoint;
using ads_common::PathProjection;
using ads_common::Pose2D;
using ads_common::ReferenceLine;

/// 逆时针（左转）圆弧，圆心在原点，半径 radius_m。
/// 参数角 φ 从 start_rad 起、步进 step_rad、共 count 个点。
/// 点 = R·(cos φ, sin φ)，切向 = φ + π/2（逆时针行驶），曲率 = +1/R。
///
/// 朝向经过 normalize_angle —— 真实路径来自 ROS 四元数，值域必然是 (−π, π]，
/// 构造时不归一化的话，跨 ±π 的用例就测不到该测的东西。
std::vector<Pose2D> MakeLeftArc(double radius_m, double start_rad, double step_rad, int count)
{
  std::vector<Pose2D> poses;
  poses.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double phi_rad = start_rad + step_rad * i;
    poses.push_back(
      {radius_m * std::cos(phi_rad), radius_m * std::sin(phi_rad),
       ads_common::normalize_angle(phi_rad + M_PI_2)});
  }
  return poses;
}

/// 沿 +x 的直线，起点 (0, 0)，逐段间距由 spacings 给出（**故意不等距**）。
std::vector<Pose2D> MakeStraightWithSpacings(const std::vector<double> & spacings_m)
{
  std::vector<Pose2D> poses{{0.0, 0.0, 0.0}};
  double x_m = 0.0;
  for (const double spacing_m : spacings_m) {
    x_m += spacing_m;
    poses.push_back({x_m, 0.0, 0.0});
  }
  return poses;
}

/// 发夹弯：直行 (0,0)→(20,0) → 半径 1 m 的半圆 → 反向直行 (20,2)→(0,2)。
///
/// 两条腿相距只有 2 m，所以路径上**几何距离相近的点分属两段**——
/// 这正是环线上刚驶出路口时的真实情形，也是全局最近点会跳变的场景。
std::vector<Pose2D> MakeHairpin()
{
  constexpr double kSpacingM = 0.5;
  constexpr int kLegPoints = 40;  // 每条腿 40 段 = 20 m
  constexpr int kArcSegments = 12;

  std::vector<Pose2D> poses;
  // 第一条腿：朝 +x
  for (int i = 0; i <= kLegPoints; ++i) {
    poses.push_back({kSpacingM * i, 0.0, 0.0});
  }
  // 半圆：圆心 (20, 1)，半径 1，φ 从 −π/2 转到 +π/2（逆时针）。
  // i 从 1 起，避免与第一条腿的终点 (20, 0) 重合 —— 重合点会被构造函数拒绝。
  for (int i = 1; i <= kArcSegments; ++i) {
    const double phi_rad = -M_PI_2 + M_PI * i / kArcSegments;
    poses.push_back(
      {20.0 + std::cos(phi_rad), 1.0 + std::sin(phi_rad),
       ads_common::normalize_angle(phi_rad + M_PI_2)});
  }
  // 第二条腿：朝 −x，y = 2
  for (int i = 1; i <= kLegPoints; ++i) {
    poses.push_back({20.0 - kSpacingM * i, 2.0, M_PI});
  }
  return poses;
}

// ---------------------------------------------------------------------------
//  构造与校验
// ---------------------------------------------------------------------------

TEST(TrackedPathConstruction, RejectsPathsWithFewerThanTwoPoints)
{
  // 少于两个点连切线都定义不出来。返回一个"空路径"让下游判空，
  // 等于把错误往下游推一层，而下游的表现是「车不动，没有任何日志」。
  EXPECT_THROW(ReferenceLine(std::vector<Pose2D>{}), std::invalid_argument);
  EXPECT_THROW(ReferenceLine(std::vector<Pose2D>{{0.0, 0.0, 0.0}}), std::invalid_argument);
}

TEST(TrackedPathConstruction, RejectsCoincidentPoints)
{
  // P1 真出过这个 bug：ceil(span/step) 让路径最后两个点重合，RViz 里完全看不出来。
  // 静默跳过重合点的话，那个上游 bug 会被永久掩盖。
  const std::vector<Pose2D> poses{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  EXPECT_THROW(ReferenceLine{poses}, std::invalid_argument);
}

// ---------------------------------------------------------------------------
//  弧长
// ---------------------------------------------------------------------------

TEST(TrackedPathArcLength, AccumulatesActualSpacingNotASampleStep)
{
  // 故意不等距。用「序号 × 步长」的实现在这里必红，而在等距路径上不会。
  const ReferenceLine path(MakeStraightWithSpacings({0.5, 1.25, 0.25, 2.0}));

  ASSERT_EQ(path.points().size(), 5u);
  EXPECT_DOUBLE_EQ(path.points()[0].s_m, 0.00);
  EXPECT_DOUBLE_EQ(path.points()[1].s_m, 0.50);
  EXPECT_DOUBLE_EQ(path.points()[2].s_m, 1.75);
  EXPECT_DOUBLE_EQ(path.points()[3].s_m, 2.00);
  EXPECT_DOUBLE_EQ(path.points()[4].s_m, 4.00);
  EXPECT_DOUBLE_EQ(path.length_m(), 4.00);
}

TEST(TrackedPathArcLength, MatchesTheRealSpacingOnACurveNotTheSamplingStep)
{
  // 这条直接钉住 docs/modules/control.md §2.1 那个 14.58% 的账。
  //
  // map_node 按**参考线**弧长每 0.5 m 采样，而路径是**车道中心线**：
  // 环线四角（参考线 R=12、车道外偏 1.75 m）上实际点距是 0.5 × (1 + 1.75/12)
  // = 0.5729 m。用 0.5 的实现会在这里差 14.58%。
  constexpr double kLaneRadiusM = 13.75;      // = 12 + 1.75，外侧车道中心线半径
  constexpr double kActualSpacingM = 0.5729;  // 实际点距
  const double step_rad = kActualSpacingM / kLaneRadiusM;

  const ReferenceLine path(MakeLeftArc(kLaneRadiusM, 0.0, step_rad, 20));

  // 弦长（实现算的）略小于弧长（解析值），相对差 (Δφ/2)²/6 ≈ 1.5e-4。
  const double chord_m = 2.0 * kLaneRadiusM * std::sin(step_rad / 2.0);
  EXPECT_NEAR(path.points()[1].s_m, chord_m, 1e-12);
  EXPECT_NEAR(path.points()[19].s_m, 19.0 * chord_m, 1e-9);

  // 与「序号 × 采样步长」的差距是 14.58%，远大于任何数值误差 ——
  // 这一行的意义是：如果哪天有人把实现改回乘法，上面两条也许还能蒙混过关，
  // 但这条一定红。
  const double naive_s_m = 19.0 * 0.5;
  EXPECT_GT(path.points()[19].s_m / naive_s_m, 1.14);
}

// ---------------------------------------------------------------------------
//  曲率
// ---------------------------------------------------------------------------

TEST(TrackedPathCurvature, MatchesAnalyticCircleAndIsPositiveForALeftTurn)
{
  constexpr double kRadiusM = 8.0;  // 路口转弯车道中心线半径
  constexpr double kExpectedCurvature = 1.0 / kRadiusM;
  const double step_rad = 0.5729 / kRadiusM;

  const ReferenceLine path(MakeLeftArc(kRadiusM, 0.3, step_rad, 20));

  // 内点用中心差分。误差只来自「弧长用弦长近似」，相对量 (Δφ/2)²/6 ≈ 2.1e-4。
  for (std::size_t i = 1; i + 1 < path.points().size(); ++i) {
    EXPECT_NEAR(path.points()[i].curvature_inv_m, kExpectedCurvature, 1e-4)
      << "内点 " << i << " 的曲率不对";
  }
  // 逆时针 = 左转 = 曲率为正。符号错了 Stanley 的速度规划会在弯道给错限速，
  // 但**限速是取绝对值的**，所以符号错在 P2 不发作 —— 会留到 P3 规划器用它做
  // 横向偏移时才炸。所以要在这里就钉死。
  EXPECT_GT(path.points()[5].curvature_inv_m, 0.0);

  // 两端只能单侧差分，精度低一档，判据也放宽一档。
  EXPECT_NEAR(path.points().front().curvature_inv_m, kExpectedCurvature, 1e-3);
  EXPECT_NEAR(path.points().back().curvature_inv_m, kExpectedCurvature, 1e-3);
}

TEST(TrackedPathCurvature, IsNegativeForARightTurn)
{
  // 顺时针 = 右转。把左转圆弧的点序反过来走即可：位置不变，切向反向。
  std::vector<Pose2D> poses = MakeLeftArc(8.0, 0.0, 0.5729 / 8.0, 20);
  std::vector<Pose2D> reversed;
  for (auto it = poses.rbegin(); it != poses.rend(); ++it) {
    reversed.push_back({it->x_m, it->y_m, ads_common::normalize_angle(it->heading_rad + M_PI)});
  }
  const ReferenceLine path(reversed);
  EXPECT_NEAR(path.points()[5].curvature_inv_m, -1.0 / 8.0, 1e-4);
}

TEST(TrackedPathCurvature, IsZeroOnAStraightPath)
{
  const ReferenceLine path(MakeStraightWithSpacings({0.5, 0.5, 0.5, 0.5}));
  for (const auto & point : path.points()) {
    EXPECT_NEAR(point.curvature_inv_m, 0.0, 1e-12);
  }
}

TEST(TrackedPathCurvature, SurvivesTheHeadingWrapAtPi)
{
  // 本地图上**所有朝西行驶的路段**都会经过这里：朝向从 +π 翻到 −π。
  //
  // 裸写 θ[i+1] − θ[i−1] 会得到 ≈ −2π，除以 2 m 弧长得到曲率 ≈ −3.14 1/m
  // （半径 32 cm）。速度剖面立刻把限速压到 √(1.5/3.14) = 0.69 m/s ——
  // **车在一条几乎笔直的路上爬行**，而没有任何一层报错。
  constexpr double kRadiusM = 500.0;  // 近乎直线的大半径
  constexpr double kExpectedCurvature = 1.0 / kRadiusM;
  const double step_rad = 1.0 / kRadiusM;  // 点距 1 m

  // φ = π/2 时切向恰为 π。让 φ 扫过 π/2 两侧，保证朝向真的翻过 ±π。
  const ReferenceLine path(MakeLeftArc(kRadiusM, M_PI_2 - 10.0 * step_rad, step_rad, 21));

  // 先确认这条路径确实跨过了 ±π —— 否则这个用例什么都没测到，
  // 而它会安静地绿着。**测试自己也要被测**。
  bool crossed = false;
  for (std::size_t i = 1; i < path.points().size(); ++i) {
    if (
      path.points()[i - 1].heading_rad * path.points()[i].heading_rad < 0.0 &&
      std::abs(path.points()[i].heading_rad) > 3.0) {
      crossed = true;
    }
  }
  ASSERT_TRUE(crossed) << "构造的路径没有跨过 ±π，这个用例形同虚设";

  for (std::size_t i = 1; i + 1 < path.points().size(); ++i) {
    EXPECT_NEAR(path.points()[i].curvature_inv_m, kExpectedCurvature, 1e-6)
      << "内点 " << i << " 处朝向跨 ±π 时曲率算错了";
  }
}

// ---------------------------------------------------------------------------
//  投影：横向误差与航向误差
// ---------------------------------------------------------------------------

TEST(TrackedPathProjection, LateralErrorIsPositiveOnTheLeftOfThePath)
{
  // 路径沿 +x，左侧就是 +y。符号写反的话 Stanley 变成正反馈，车立刻冲出去 ——
  // 所以这不是一个"静默"的错误，但它是整条控制律的地基，值得单独钉住。
  const ReferenceLine path(MakeStraightWithSpacings({1.0, 1.0, 1.0, 1.0}));

  const PathProjection left = path.project({2.0, +1.5, 0.0});
  EXPECT_NEAR(left.lateral_error_m, +1.5, 1e-12);

  const PathProjection right = path.project({2.0, -0.75, 0.0});
  EXPECT_NEAR(right.lateral_error_m, -0.75, 1e-12);
}

TEST(TrackedPathProjection, HeadingErrorIsPositiveWhenThePathTurnsLeftOfTheVehicle)
{
  const ReferenceLine path(MakeStraightWithSpacings({1.0, 1.0, 1.0, 1.0}));

  // 车头偏右 30°（θ = −30°），路径朝 0° → 路径在车头左侧 → ψ = +30°。
  const PathProjection projection = path.project({2.0, 0.0, -M_PI / 6.0});
  EXPECT_NEAR(projection.heading_error_rad, +M_PI / 6.0, 1e-12);
}

TEST(TrackedPathProjection, InterpolatesInsideTheSegmentRatherThanSnappingToAVertex)
{
  // 采样点间距 0.57 m，取最近**采样点**会带来最多 0.29 m 的量化误差 ——
  // 那正是 CP-P2-B 横向误差判据（0.30 m）的量级，会把判据吃光。
  const ReferenceLine path(MakeStraightWithSpacings({1.0, 1.0, 1.0}));

  // 查询点在 x = 1.5，恰好落在两个采样点正中间。
  const PathProjection projection = path.project({1.5, 0.4, 0.0});
  EXPECT_EQ(projection.index, 1u);
  EXPECT_NEAR(projection.ratio, 0.5, 1e-12);
  EXPECT_NEAR(projection.x_m, 1.5, 1e-12);
  EXPECT_NEAR(projection.s_m, 1.5, 1e-12);
  EXPECT_NEAR(projection.lateral_error_m, 0.4, 1e-12);
}

TEST(TrackedPathProjection, ClampsBeyondBothEnds)
{
  const ReferenceLine path(MakeStraightWithSpacings({1.0, 1.0}));

  const PathProjection before = path.project({-5.0, 0.0, 0.0});
  EXPECT_EQ(before.index, 0u);
  EXPECT_NEAR(before.ratio, 0.0, 1e-12);
  EXPECT_NEAR(before.s_m, 0.0, 1e-12);

  const PathProjection after = path.project({99.0, 0.0, 0.0});
  EXPECT_NEAR(after.ratio, 1.0, 1e-12);
  EXPECT_NEAR(after.s_m, path.length_m(), 1e-12);

  // ⚠️ **这一行是本用例存在的主要理由，不是补充。**
  //
  // 车已经冲过终点 97 m，而横向误差是 **0** —— 因为投影被夹到端点后，
  // lateral_error_m 只剩偏移量的横向分量，纵向那一份被丢掉了。
  // 一个只看横向误差的上层会认为「跟得很好」，而车正在开向天边。
  //
  // 所以 S4 的「到达终点 / 冲过终点」判定必须用 s_m 和 ratio，
  // **不能用 lateral_error_m**。把这个行为钉在测试里，
  // 是为了让写 S4 的人一定会读到它。
  EXPECT_NEAR(after.lateral_error_m, 0.0, 1e-12);
  EXPECT_EQ(after.index, path.points().size() - 2) << "夹到的应当是最后一段";
}

TEST(TrackedPathConstruction, RejectsNonFiniteValues)
{
  // NaN 参与任何比较都返回 false —— 所以点距检查 `spacing_m < kMinSpacingM`
  // 对 NaN **恒为假**，不单独判的话会原样放行。
  // 本仓库在 vehicle_cmd_bridge 上吃过一次同源的亏（CLAUDE.md 陷阱表）。
  //
  // 放行的后果不是崩溃，是**误诊**：NaN 一路传到转角，被 bridge 的 isfinite
  // 挡下并触发看门狗刹停，现场变成「车自己停了，日志说收到非有限指令」，
  // 于是所有人去查控制器 —— 而错在路径里。
  const double nan_v = std::numeric_limits<double>::quiet_NaN();
  const double inf_v = std::numeric_limits<double>::infinity();

  EXPECT_THROW(
    ReferenceLine(std::vector<Pose2D>{{0.0, 0.0, 0.0}, {nan_v, 0.0, 0.0}}), std::invalid_argument);
  EXPECT_THROW(
    ReferenceLine(std::vector<Pose2D>{{0.0, 0.0, 0.0}, {1.0, nan_v, 0.0}}), std::invalid_argument);
  // 朝向是 NaN 时点距完全正常，**只有单独判朝向才能抓住**。
  EXPECT_THROW(
    ReferenceLine(std::vector<Pose2D>{{0.0, 0.0, 0.0}, {1.0, 0.0, nan_v}}), std::invalid_argument);
  // ±inf 同样要拦。本项目的 gpu_lidar 无回波射线返回的就是 ±inf 而不是 NaN
  // （CLAUDE.md 陷阱表），所以"只判 NaN"是不够的。
  EXPECT_THROW(
    ReferenceLine(std::vector<Pose2D>{{0.0, 0.0, 0.0}, {inf_v, 0.0, 0.0}}), std::invalid_argument);
}

// ---------------------------------------------------------------------------
//  局部搜索
// ---------------------------------------------------------------------------

TEST(TrackedPathProjection, LocalSearchAgreesWithGlobalWhenThereIsNoAmbiguity)
{
  // 没有歧义时两者必须完全一致 —— 否则局部搜索就成了另一套语义，
  // 而不是同一个查询的加速版本。
  const ReferenceLine path(MakeLeftArc(8.0, 0.0, 0.5729 / 8.0, 40));

  for (std::size_t i = 1; i + 1 < path.points().size(); ++i) {
    const Pose2D query{path.points()[i].x_m + 0.3, path.points()[i].y_m, 0.0};
    const PathProjection global = path.project(query);
    const PathProjection local = path.project(query, i);
    EXPECT_EQ(global.index, local.index) << "第 " << i << " 点处两种搜索给出不同的段";
    EXPECT_NEAR(global.lateral_error_m, local.lateral_error_m, 1e-12);
  }
}

TEST(TrackedPathProjection, LocalSearchStaysOnTheCurrentLegOfAHairpin)
{
  // **这是局部搜索存在的全部理由。**
  //
  // 发夹弯的两条腿只相距 2 m。车在第一条腿上、位置 (10, 1.2)：
  //   到第一条腿（y=0）的距离 1.2 m
  //   到第二条腿（y=2）的距离 0.8 m   ← 更近
  // 全局最近点会**跳到还没走到的第二条腿上**，而那一段的行驶方向是反的，
  // 于是航向误差瞬间变成 180°、转角打死。
  //
  // 环线上刚驶出路口时就是这个情形，不是构造出来的边界情况。
  const ReferenceLine path(MakeHairpin());

  const Pose2D query{10.0, 1.2, 0.0};

  // 全局搜索：跳到第二条腿（朝向 π）。
  const PathProjection global = path.project(query);
  EXPECT_NEAR(std::abs(global.heading_rad), M_PI, 1e-9)
    << "这个用例的前提是全局搜索确实会跳段；不跳的话它什么都没测到";
  EXPECT_NEAR(global.lateral_error_m, +0.8, 1e-9);

  // 局部搜索：hint 指向第一条腿上 x=10 处（索引 20），窗口 30 点覆盖 ±15 m，
  // 够不着第二条腿（在索引 53 之后）。
  const PathProjection local = path.project(query, 20u);
  EXPECT_NEAR(local.heading_rad, 0.0, 1e-9);
  EXPECT_NEAR(local.lateral_error_m, +1.2, 1e-9);
  EXPECT_LE(local.index, 40u) << "局部搜索不该越过第一条腿的末端（索引 40）";
}

TEST(TrackedPathProjection, HintBeyondTheLastSegmentIsClampedRatherThanUndefined)
{
  // 路径换掉之后上一拍的索引可能已经越界。夹住而不是崩，
  // 因为「路径更新」和「控制回调」是两个不同频率的事件，这一定会发生。
  const ReferenceLine path(MakeStraightWithSpacings({1.0, 1.0}));
  const PathProjection projection = path.project({1.5, 0.0, 0.0}, 999u);
  EXPECT_LE(projection.index, 1u);
}

// ---------------------------------------------------------------------------
//  at()：按弧长求值（P3-S1 新增，规划器的 Frenet→笛卡尔要用）
// ---------------------------------------------------------------------------

TEST(ReferenceLineAt, AgreesWithProjectionOfPointsThatLieOnTheLine)
{
  // 线上的点投影回来就是它自己，所以 at(projection.s_m) 必须给出同一个点。
  // 这条把 at() 和 project() 拴在一起 —— 两者共用 interpolate()，
  // 拆成两份实现的话，症状是同一个几何位置从两个接口拿到两个不同的朝向。
  const ReferenceLine line(MakeLeftArc(20.0, 0.0, 0.025, 63));

  for (double ratio = 0.05; ratio < 0.95; ratio += 0.11) {
    const double s_m = ratio * line.length_m();
    const PathPoint point = line.at(s_m);
    const PathProjection projection = line.project({point.x_m, point.y_m, point.heading_rad});

    EXPECT_NEAR(projection.s_m, s_m, 1e-9) << "s = " << s_m;
    EXPECT_NEAR(projection.x_m, point.x_m, 1e-9) << "s = " << s_m;
    EXPECT_NEAR(projection.y_m, point.y_m, 1e-9) << "s = " << s_m;
    EXPECT_NEAR(projection.heading_rad, point.heading_rad, 1e-9) << "s = " << s_m;
    EXPECT_NEAR(projection.curvature_inv_m, point.curvature_inv_m, 1e-9) << "s = " << s_m;
    EXPECT_NEAR(projection.lateral_error_m, 0.0, 1e-9) << "s = " << s_m;
  }
}

TEST(ReferenceLineAt, ReturnsTheRequestedArcLength)
{
  const ReferenceLine line(MakeStraightWithSpacings(std::vector<double>(40, 0.5)));
  for (const double s_m : {0.0, 0.25, 7.3, 19.99}) {
    EXPECT_NEAR(line.at(s_m).s_m, s_m, 1e-12) << "s = " << s_m;
  }
}

TEST(ReferenceLineAt, ThrowsBeyondEitherEndInsteadOfClampingLikeProject)
{
  // ⚠️ **与 project() 有意不同**：project() 越界夹到端点是对的（"最近的点"确实是端点），
  //    at() 越界抛异常也是对的（弧长 s 处根本不存在那样一个点）。
  //    P2-S4 已经因为「投影夹到端点」误诊过一次 —— 冲过终点 4 m 与恰好停住
  //    给出一模一样的数。所以这里让调用方显式处理，不替它做决定。
  const ReferenceLine line(MakeStraightWithSpacings(std::vector<double>(40, 0.5)));

  EXPECT_THROW(line.at(line.length_m() + 1.0), std::out_of_range);
  EXPECT_THROW(line.at(-1.0), std::out_of_range);
  EXPECT_THROW(line.at(std::nan("")), std::out_of_range);
  EXPECT_THROW(line.at(std::numeric_limits<double>::infinity()), std::out_of_range);
  EXPECT_THROW(line.at(-std::numeric_limits<double>::infinity()), std::out_of_range);

  // 而 project() 在同样的越界查询下**不抛**，照样返回夹到端点的结果。
  EXPECT_NO_THROW(line.project({100.0, 0.0, 0.0}));
}

TEST(ReferenceLineAt, ToleratesFloatingPointNoiseAtTheEndpoints)
{
  // 调用方常写 at(length_m())，而 length_m() 本身是浮点累加的结果，
  // 上游再做一次乘除（比如把前视距离截断成 min(horizon, length)）就可能溢出几个 ulb。
  // 不给容差的话，规划器每一拍在路径末端都会抛异常。
  const ReferenceLine line(MakeStraightWithSpacings(std::vector<double>(40, 0.5)));
  const double length_m = line.length_m();

  EXPECT_NO_THROW(line.at(length_m));
  EXPECT_NO_THROW(line.at(length_m + 1e-9));
  EXPECT_NO_THROW(line.at(-1e-9));
  // 但超过 kMinSpacingM（1 mm）就不再是浮点噪声，而是逻辑错误，必须抛。
  EXPECT_THROW(line.at(length_m + 2.0 * ReferenceLine::kMinSpacingM), std::out_of_range);
}

TEST(ReferenceLineAt, InterpolatesHeadingAcrossThePiBoundaryInsteadOfFlippingIt)
{
  // 朝向跨 ±π 时线性插值会插到**反方向**去（例如 π−0.05 与 −π+0.05 之间
  // 裸插得到 0，即正好掉头）。at() 与 project() 共用的 interpolate() 过 angle_diff，
  // 所以这里必须始终在 ±π 附近。
  //
  // 构造：MakeLeftArc 的朝向 = φ + π/2，令 φ 扫过 π/2 即让朝向扫过 π。
  const ReferenceLine line(MakeLeftArc(20.0, M_PI_2 - 0.3, 0.025, 25));

  // ⚠️ 采样步长必须与**段长不可通约**，否则可能只采到段的端点。
  //    初稿用「总长的 1/20」= 0.6 m 步进，而段长恰好 0.5 m，两者在 s = 6.0 处重合 ——
  //    跨 ±π 的那一段偏偏就从 s = 6.0 开始，于是只被采到 ratio = 0 的端点，
  //    而端点上朴素线性插值与正确插值给出**同一个值**。
  //    故障注入（把 angle_diff 换成裸减）当时**一条用例都没红** —— 这条用例
  //    看着在测跨 ±π，实际什么都没测。0.037 是随手取的非整除步长，只要不整除 0.5 即可。
  for (double s_m = 0.0; s_m < line.length_m(); s_m += 0.037) {
    const double heading_rad = line.at(s_m).heading_rad;
    // 全程朝向都该在 π 附近（|heading| > π − 0.4），绝不该出现接近 0 的值。
    EXPECT_GT(std::abs(heading_rad), M_PI - 0.4)
      << "s = " << s_m << " 处朝向 " << heading_rad << " 掉到了反方向";
  }
}

}  // namespace
