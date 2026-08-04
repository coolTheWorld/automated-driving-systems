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
//  L1：OBB 相交与间距（P3-S3）
//
//  SPEC §8 L1 明确点名要覆盖「多边形碰撞检测的边界情况（相切、包含、退化）」，
//  三种各有一条用例。另外两条是本实现特有的：
//
//    - 旋转矩形：只有**对方的**边法向能分开它们，验证 SAT 真的检了四条轴
//      而不是退化成 AABB；
//    - 顶点对顶点：SAT 的最大间隙**低估**真实距离，验证 distance_m() 走的是
//      精确的顶点-边遍历而不是拿那个间隙当距离。
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>

#include "ads_planning/collision.hpp"

namespace
{

using ads_planning::corners_of;
using ads_planning::distance_m;
using ads_planning::overlaps;
using ads_planning::Rectangle;

/// 边长 2、中心可指定的轴对齐正方形（角点在 中心 ± 1）。
Rectangle UnitSquareAt(double center_x_m, double center_y_m, double heading_rad = 0.0)
{
  return Rectangle{center_x_m, center_y_m, heading_rad, 2.0, 2.0};
}

TEST(Collision, IdenticalRectanglesOverlapAndHaveZeroDistance)
{
  const Rectangle a = UnitSquareAt(3.0, -1.0);
  EXPECT_TRUE(overlaps(a, a));
  EXPECT_EQ(distance_m(a, a), 0.0);
}

TEST(Collision, AxisAlignedSeparationDistanceIsExact)
{
  // 沿 x 分开：间隙 = 中心距 − 两个半长 = 5 − 1 − 1 = 3。
  const Rectangle a = UnitSquareAt(0.0, 0.0);
  const Rectangle b = UnitSquareAt(5.0, 0.0);
  EXPECT_FALSE(overlaps(a, b));
  EXPECT_NEAR(distance_m(a, b), 3.0, 1e-12);
  // 对称性：距离与参数顺序无关。写错了 min_vertex_to_edge 的单向遍历就会不对称。
  EXPECT_NEAR(distance_m(b, a), 3.0, 1e-12);
}

TEST(Collision, TangentRectanglesCountAsOverlapping)
{
  // ⚠️ 相切（投影区间恰好接触，间隙 = 0）判为**相交**。
  //    数学上闭矩形共享边界点确实相交；安全语义上"恰好贴上"也必须算撞上。
  //    实现里那句 `gap > 0.0` 写成 `>= 0.0` 就会翻过来，
  //    而随机位形几乎采不到恰好相切 —— 所以必须有这一条专门钉住它。
  const Rectangle a = UnitSquareAt(0.0, 0.0);
  const Rectangle b = UnitSquareAt(2.0, 0.0);  // 右边缘 x=1 贴左边缘 x=1
  EXPECT_TRUE(overlaps(a, b));
  EXPECT_EQ(distance_m(a, b), 0.0);
}

TEST(Collision, FullyContainedRectangleOverlaps)
{
  // ⚠️ 包含关系下**不存在任何分离轴**。只检查"某条轴上是否分离"的实现
  //    在这里必须返回 true —— 但一个写成"必须找到重叠证据"的实现会漏判。
  const Rectangle big = Rectangle{0.0, 0.0, 0.3, 10.0, 8.0};
  const Rectangle small = Rectangle{0.5, -0.4, -1.1, 0.6, 0.4};
  EXPECT_TRUE(overlaps(big, small));
  EXPECT_TRUE(overlaps(small, big));
  EXPECT_EQ(distance_m(big, small), 0.0);
}

TEST(Collision, DegenerateZeroSizeRectangleBehavesAsAPoint)
{
  // 上游给出零尺寸障碍物通常意味着感知有 bug，但本层**不能崩也不能给随机结果**。
  const Rectangle point = Rectangle{4.0, 0.0, 0.0, 0.0, 0.0};
  const Rectangle square = UnitSquareAt(0.0, 0.0);

  EXPECT_FALSE(overlaps(square, point));
  EXPECT_NEAR(distance_m(square, point), 3.0, 1e-12);  // 4 − 1

  // 点落在矩形内部 ⟹ 相交。
  const Rectangle inside = Rectangle{0.5, 0.5, 0.0, 0.0, 0.0};
  EXPECT_TRUE(overlaps(square, inside));

  // 零宽但有长（退化成线段）也要正确。
  const Rectangle segment = Rectangle{4.0, 0.0, M_PI_2, 6.0, 0.0};  // x=4，y∈[−3,3]
  EXPECT_FALSE(overlaps(square, segment));
  EXPECT_NEAR(distance_m(square, segment), 3.0, 1e-12);
}

TEST(Collision, RotatedRectangleSeparatedOnlyByTheOtherRectanglesAxis)
{
  // A：边长 2 的轴对齐方块，占 [−1,1]²。
  // B：边长 2、转 45° 的方块（菱形），中心 (2,2)，顶点在 (0.586,2) 和 (2,0.586)。
  //
  // 关键：**两者的 AABB 是重叠的**（都覆盖 x∈[0.586,1], y∈[0.586,1]），
  // 所以退化成 AABB 的实现会误报相交。只有 B 自己的 45° 边法向能把它们分开 ——
  // 这条用例证明 SAT 真的检了四条轴。
  const Rectangle a = UnitSquareAt(0.0, 0.0);
  const Rectangle b = UnitSquareAt(2.0, 2.0, M_PI_4);

  EXPECT_FALSE(overlaps(a, b));

  // 最近点对：A 的角 (1,1) 到 B 的边（直线 x + y = 2 + √2）。
  // 距离 = |1 + 1 − (2 + √2)| / √2 = √2 − 1 ≈ 0.41421356
  EXPECT_NEAR(distance_m(a, b), std::sqrt(2.0) - 1.0, 1e-12);
}

TEST(Collision, VertexToVertexDistanceIsExactNotTheSeparatingAxisLowerBound)
{
  // ⚠️ 这条是 distance_m() 存在的理由。
  //    两个轴对齐方块斜着放：A 占 [−1,1]²，B 中心 (4,3) 占 x∈[3,5]、y∈[2,4]。
  //    最近点是 A 的角 (1,1) 与 B 的角 (3,2)，真实距离 = √(2² + 1²) = √5 ≈ 2.2360680。
  //
  //    而**分离轴的最大间隙只有 2.0**（x 轴：4−1−1；y 轴：3−1−1=1）——
  //    顶点对顶点时最近点连线不是任何一条边的法向，SAT 会低估。
  //    低估是保守的（把安全的判成不安全），但会让本来能绕的候选被误杀，
  //    而 §6 的可行区间本就只有零点几米宽。
  const Rectangle a = UnitSquareAt(0.0, 0.0);
  const Rectangle b = UnitSquareAt(4.0, 3.0);

  EXPECT_FALSE(overlaps(a, b));
  EXPECT_NEAR(distance_m(a, b), std::sqrt(5.0), 1e-12);
  // 显式钉住"它必须严格大于 SAT 的那个下界"，免得实现哪天退回去用间隙当距离。
  EXPECT_GT(distance_m(a, b), 2.0 + 1e-6);
}

TEST(Collision, CornersAreInWindingOrderSoAdjacentPairsAreRealEdges)
{
  // corners_of() 的顺序必须是**绕行**的：相邻两个是一条真实的边。
  // 顺序写成对角交叉的话，distance_m() 里 (i, i+1) 取到的是对角线，
  // 算出来的距离会**偏小**（对角线离外部点更近）—— 又是一个偏向"更保守"
  // 因而不会引起怀疑的错误。
  const Rectangle rectangle = Rectangle{0.0, 0.0, 0.0, 4.0, 2.0};
  const auto corners = corners_of(rectangle);

  // 边长应交替为 width(2) 和 length(4)。
  const double expected[4] = {2.0, 4.0, 2.0, 4.0};
  for (std::size_t i = 0; i < corners.size(); ++i) {
    const auto & from = corners[i];
    const auto & to = corners[(i + 1) % corners.size()];
    EXPECT_NEAR(std::hypot(to[0] - from[0], to[1] - from[1]), expected[i], 1e-12) << "边 " << i;
  }
}

TEST(Collision, HeadingIsRespectedWhenPlacingCorners)
{
  // 长 4 宽 2 的矩形转 90° 之后，沿 x 的半宽应变成 1（原来的半宽）。
  const Rectangle rotated = Rectangle{0.0, 0.0, M_PI_2, 4.0, 2.0};
  const Rectangle probe = UnitSquareAt(3.0, 0.0);  // 左边缘 x = 2

  // 旋转后矩形沿 x 只占 [−1, 1] ⟹ 间隙 = 2 − 1 = 1。
  // 没转的话它沿 x 占 [−2, 2]，会与 probe 相交。
  EXPECT_FALSE(overlaps(rotated, probe));
  EXPECT_NEAR(distance_m(rotated, probe), 1.0, 1e-12);
}

}  // namespace
