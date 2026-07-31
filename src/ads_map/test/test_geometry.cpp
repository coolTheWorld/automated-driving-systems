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
//  参考线几何的 L1 测试
//
//  全部与**解析解**比对，不读任何地图文件。这一层写错的话，
//  上面所有依赖它的东西（车道中心线、路由、可视化）都是错的，
//  但症状会出现在很远的地方 —— 所以这里的用例要尽可能直白、能手推。
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>

#include "ads_map/geometry.hpp"

namespace
{

/// 位置比对容差。这里比的是纯解析式计算，残差只来自浮点舍入，
/// 实测在 1e-14 量级；取 1e-12 留足余量，同时远小于任何有物理意义的偏差。
constexpr double kPosTol = 1e-12;
constexpr double kAngTol = 1e-12;

}  // namespace

TEST(Geometry, LineAdvancesAlongItsHeadingAndKeepsIt)
{
  const double hdg = 30.0 * M_PI / 180.0;
  const ads_map::Geometry g{0.0, 3.0, -4.0, hdg, 10.0, 0.0};

  const ads_map::Pose2D p = g.pose_at(10.0);
  EXPECT_NEAR(p.x_m, 3.0 + 10.0 * std::cos(hdg), kPosTol);
  EXPECT_NEAR(p.y_m, -4.0 + 10.0 * std::sin(hdg), kPosTol);
  EXPECT_NEAR(p.heading_rad, hdg, kAngTol);
}

TEST(Geometry, LineIsExactOverLongDistances)
{
  // 本项目最长的一段直线是 76 m。这条用例锁的是「直线单独一支」这个实现决定：
  // 如果哪天有人把直线改成「用一个很小的曲率去近似」，误差会随长度放大，
  // 在 76 m 上就足以让车道中心线错位。
  const ads_map::Geometry g{0.0, 90.0, -38.0, M_PI_2, 76.0, 0.0};
  const ads_map::Pose2D p = g.end_pose();
  EXPECT_NEAR(p.x_m, 90.0, kPosTol);
  EXPECT_NEAR(p.y_m, 38.0, kPosTol);
}

TEST(Geometry, QuarterArcTurningLeftLandsOnTheCircleAxis)
{
  // 从原点朝 +x 出发、曲率 +1/R、走过 R·π/2 → 终点 (R, +R)，朝向 +90°。
  // 这是圆弧闭式解最容易手推的特例。
  const double radius = 12.0;
  const ads_map::Geometry g{0.0, 0.0, 0.0, 0.0, radius * M_PI_2, 1.0 / radius};

  const ads_map::Pose2D p = g.end_pose();
  EXPECT_NEAR(p.x_m, radius, 1e-9);
  EXPECT_NEAR(p.y_m, radius, 1e-9);
  EXPECT_NEAR(p.heading_rad, M_PI_2, kAngTol);
}

TEST(Geometry, QuarterArcTurningRightLandsOnTheOppositeSide)
{
  const double radius = 12.0;
  const ads_map::Geometry g{0.0, 0.0, 0.0, 0.0, radius * M_PI_2, -1.0 / radius};

  const ads_map::Pose2D p = g.end_pose();
  EXPECT_NEAR(p.x_m, radius, 1e-9);
  EXPECT_NEAR(p.y_m, -radius, 1e-9);
  EXPECT_NEAR(p.heading_rad, -M_PI_2, kAngTol);
}

TEST(Geometry, EveryPointOfAnArcStaysOnItsCircle)
{
  // 比端点比对更强：端点对了不代表中间也对。
  // 圆心在起点左侧（左转）半径处，弧上任意点到圆心的距离都应恒等于半径。
  const double radius = 8.0;
  const double hdg0 = 37.0 * M_PI / 180.0;
  const ads_map::Geometry g{0.0, 1.0, 2.0, hdg0, radius * M_PI_2, 1.0 / radius};

  const double cx = 1.0 + radius * std::cos(hdg0 + M_PI_2);
  const double cy = 2.0 + radius * std::sin(hdg0 + M_PI_2);

  for (int i = 0; i <= 20; ++i) {
    const ads_map::Pose2D p = g.pose_at(g.length_m * i / 20.0);
    EXPECT_NEAR(std::hypot(p.x_m - cx, p.y_m - cy), radius, 1e-9)
      << "第 " << i << " 个采样点偏离了圆";
  }
}

TEST(Geometry, ArcHeadingIsNotNormalised)
{
  // 锁定 geometry.hpp 里写明的契约：航向沿参考线**连续累加**，不归一化。
  //
  // 这是有意的。归一化会让航向在跨越 ±π 时跳变，而下游算「两点之间转了多少」
  // 时会因此得到差 2π 的结果。需要归一化的地方（比如与车辆当前航向求误差）
  // 应当由调用方显式调 ads_common::normalize_angle。
  const double radius = 10.0;
  const ads_map::Geometry g{0.0, 0.0, 0.0, 0.0, radius * 1.5 * M_PI, 1.0 / radius};

  const ads_map::Pose2D p = g.end_pose();
  EXPECT_NEAR(p.heading_rad, 1.5 * M_PI, kAngTol)
    << "航向被归一化到了 " << p.heading_rad << "，这会破坏下游的连续性假设";
}

TEST(Geometry, EndPoseEqualsPoseAtFullLength)
{
  const ads_map::Geometry g{5.0, 1.0, 2.0, 0.3, 7.0, 0.05};
  const ads_map::Pose2D a = g.end_pose();
  const ads_map::Pose2D b = g.pose_at(7.0);
  EXPECT_DOUBLE_EQ(a.x_m, b.x_m);
  EXPECT_DOUBLE_EQ(a.y_m, b.y_m);
  EXPECT_DOUBLE_EQ(a.heading_rad, b.heading_rad);
}
