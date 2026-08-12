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
//  L-Shape 拟合 + 尺寸分类的 L1 判据
//
//  ⚠️ 点云按**只看得见两个面**的方式造 —— 那是 L-Shape 之所以叫这个名字的
//     原因，也是它与"最小外接矩形"的区别所在。用四面都有点的合成数据测，
//     两种准则给出一样的结果，判据就没有区分力了
//     （与地面分割那组"必须带坡度"是同一类问题）。
//
//  ## 故障注入实测（2026-08-11，写完立刻做的）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 长短轴判反（`extent_1 >= extent_2` 改成 `<`） | **红 1 条** |
//  | closeness 换成「最小外接面积」准则 | **红 2 条** |
//  | **假装解决二义性**（让 yaw 输出 `[0, 2π)`，按中心位置猜车头） | **红 1 条** |
//  | 分类器去掉 `isfinite` 前置检查（改成 `!(x >= a)`） | **红 1 条** |
//
//  第三条是本文件最要紧的那一条：它证明「不许假装解决 180° 二义性」这件事
//  **真的被守住了**。猜错的概率是 50%，而错了之后 P6 会预测出一条逆行轨迹。
//
//  第四条证明 `isfinite` 前置检查不是装饰：去掉之后把某个分支写成
//  `!(x >= a)` 的形式，NaN 立刻命中它（NaN 参与任何比较都返回 false）。
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "ads_perception/lshape_fit.hpp"
#include "ads_perception/size_classifier.hpp"

namespace
{

using ads_perception::ClassifyBySize;
using ads_perception::FitLShape;
using ads_perception::LShapeBox;
using ads_perception::LShapeFitParams;
using ads_perception::ObjectClass;
using ads_perception::ObjectClassName;
using ads_perception::SizeClassifierParams;

/// 造一辆「只看得见两个面」的车（L 形点云）。
///
/// @param yaw_rad 车的真实朝向（长轴方向）
/// @param sensor  雷达位置 —— 决定哪两个面看得见
std::vector<Eigen::Vector3d> MakeLShapedCar(
  const Eigen::Vector2d & center, double yaw_rad, double length, double width, double height,
  const Eigen::Vector2d & sensor, double spacing = 0.06, double noise_stddev = 0.01)
{
  std::vector<Eigen::Vector3d> points;
  std::mt19937 rng(4321);
  std::normal_distribution<double> noise(0.0, noise_stddev);
  const Eigen::Vector2d axis_long(std::cos(yaw_rad), std::sin(yaw_rad));
  const Eigen::Vector2d axis_short(-std::sin(yaw_rad), std::cos(yaw_rad));

  // 四条边各自的外法向；只保留**背对雷达为假**（即朝向雷达）的那两条。
  const double half_l = length / 2.0;
  const double half_w = width / 2.0;
  struct Edge
  {
    Eigen::Vector2d normal;
    Eigen::Vector2d anchor;
    Eigen::Vector2d direction;
    double extent;
  };
  const Edge edges[4] = {
    {axis_long, center + axis_long * half_l, axis_short, half_w},
    {-axis_long, center - axis_long * half_l, axis_short, half_w},
    {axis_short, center + axis_short * half_w, axis_long, half_l},
    {-axis_short, center - axis_short * half_w, axis_long, half_l},
  };

  for (const Edge & edge : edges) {
    // 边的外法向若背对雷达（法向与"指向雷达"的向量夹角 > 90°），雷达看不到它。
    if (edge.normal.dot((sensor - center).normalized()) <= 0.0) {
      continue;
    }
    for (double t = -edge.extent; t <= edge.extent; t += spacing) {
      const Eigen::Vector2d on_edge = edge.anchor + edge.direction * t;
      for (double z = 0.2; z <= height; z += 0.25) {
        points.emplace_back(on_edge.x() + noise(rng), on_edge.y() + noise(rng), z + noise(rng));
      }
    }
  }
  return points;
}

/// 把 yaw 归一化到 [0, π)，再取与真值的最短差（考虑 π 周期）。
double AxisAngleErrorRad(double fitted_rad, double truth_rad)
{
  double diff = std::fmod(fitted_rad - truth_rad, M_PI);
  if (diff < 0.0) {
    diff += M_PI;
  }
  return std::min(diff, M_PI - diff);
}

}  // namespace

// ---------------------------------------------------------------------------
//  朝向拟合精度
// ---------------------------------------------------------------------------
TEST(LShapeFit, RecoversTheAxisOfACarAtManyOrientations)
{
  // 扫一圈朝向。**不能只测一个角度** —— 搜索范围是 [0, π/2)，
  // 只测 0° 的话，"长短轴弄反了"（extent_1 vs extent_2 判反）那个 bug
  // 恰好不会暴露，因为 0° 时长轴就是 e1。
  const Eigen::Vector2d center(15.0, 2.0);
  const Eigen::Vector2d sensor(0.0, 0.0);
  double worst_deg = 0.0;
  for (const double degrees : {0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 100.0, 135.0, 170.0}) {
    const double yaw = degrees * M_PI / 180.0;
    const auto points = MakeLShapedCar(center, yaw, 4.4, 1.8, 1.5, sensor);
    ASSERT_GE(points.size(), 20U) << degrees << "° 时点太少，场景造错了";

    const LShapeBox box = FitLShape(points, LShapeFitParams{});
    ASSERT_TRUE(box.valid);
    const double error_deg = AxisAngleErrorRad(box.yaw_rad, yaw) * 180.0 / M_PI;
    worst_deg = std::max(worst_deg, error_deg);
    printf(
      "[          ] 真值 %5.1f° → 拟合 %5.1f°（轴向误差 %4.2f°），"
      "尺寸 %.2f × %.2f × %.2f，%zu 点\n",
      degrees, box.yaw_rad * 180.0 / M_PI, error_deg, box.length_m, box.width_m, box.height_m,
      points.size());

    // 判据：朝向（轴向）误差 < 10°。
    EXPECT_LT(error_deg, 10.0) << degrees << "° 处朝向拟合偏了";
    // 长轴必须是较长的那一条 —— 否则尺寸分类会把车当成"横着的东西"。
    EXPECT_GE(box.length_m, box.width_m);
    EXPECT_NEAR(box.length_m, 4.4, 0.4) << degrees << "° 处长度不对";
    EXPECT_NEAR(box.width_m, 1.8, 0.4) << degrees << "° 处宽度不对";
  }
  printf("[          ] 最差轴向误差 %.2f°\n", worst_deg);
}

// ---------------------------------------------------------------------------
//  ⚠️⚠️ 180° 二义性：本层**解决不了**，用例要证明它没有假装解决
// ---------------------------------------------------------------------------
TEST(LShapeFit, DoesNotPretendToResolveTheOneEightyDegreeAmbiguity)
{
  // 同一辆车，朝向差 180° —— 点云**完全一样**（把车头车尾对调，
  // 从雷达看过去的两个面还是那两个面）。所以拟合结果必须也一样。
  //
  // ⚠️ 如果哪天有人"顺手"让它输出 [0, 2π) 的朝向（比如"取靠近雷达的
  //    那一端当车头"），这条用例会立刻红 —— 那正是它存在的理由。
  //    猜错的概率是 50%，而错了之后 P6 会预测出一条**逆行**轨迹，
  //    P7 据此判断"对方要过来"。**症状出现在两个模块之外，且无人报错。**
  const Eigen::Vector2d center(18.0, -3.0);
  const Eigen::Vector2d sensor(0.0, 0.0);
  const double yaw = 40.0 * M_PI / 180.0;

  const LShapeBox forward = FitLShape(MakeLShapedCar(center, yaw, 4.4, 1.8, 1.5, sensor), {});
  const LShapeBox backward =
    FitLShape(MakeLShapedCar(center, yaw + M_PI, 4.4, 1.8, 1.5, sensor), {});
  ASSERT_TRUE(forward.valid);
  ASSERT_TRUE(backward.valid);

  printf(
    "[          ] 朝向 %.0f° 与 %.0f°（差 180°）→ 拟合 %.2f° 与 %.2f°\n", yaw * 180 / M_PI,
    (yaw + M_PI) * 180 / M_PI, forward.yaw_rad * 180 / M_PI, backward.yaw_rad * 180 / M_PI);

  // 值域必须是 [0, π)。
  EXPECT_GE(forward.yaw_rad, 0.0);
  EXPECT_LT(forward.yaw_rad, M_PI);
  // 两者必须给出**同一个**轴向 —— 差 180° 的车在点云上无法区分。
  EXPECT_NEAR(AxisAngleErrorRad(forward.yaw_rad, backward.yaw_rad), 0.0, 1e-9)
    << "差 180° 的两辆车拟合出了不同的轴向 —— 那说明它在猜车头朝向";
}

// ---------------------------------------------------------------------------
//  ⚠️ 与轴对齐包围盒的对比 —— 这是 L-Shape 存在的理由
// ---------------------------------------------------------------------------
TEST(LShapeFit, BeatsTheAxisAlignedBoxOnADiagonallyParkedCar)
{
  // 斜停 45° 的车：AABB 比车本身大得多，而那部分"面积"在规划的碰撞检查里
  // 是**凭空多出来的障碍**，症状是「车绕一个不存在的东西」。
  const Eigen::Vector2d center(15.0, 0.0);
  const auto points = MakeLShapedCar(center, M_PI / 4.0, 4.4, 1.8, 1.5, {0.0, 0.0});
  const LShapeBox box = FitLShape(points, LShapeFitParams{});
  ASSERT_TRUE(box.valid);

  double x_min = points.front().x();
  double x_max = x_min;
  double y_min = points.front().y();
  double y_max = y_min;
  for (const auto & point : points) {
    x_min = std::min(x_min, point.x());
    x_max = std::max(x_max, point.x());
    y_min = std::min(y_min, point.y());
    y_max = std::max(y_max, point.y());
  }
  const double aabb_area = (x_max - x_min) * (y_max - y_min);
  const double lshape_area = box.length_m * box.width_m;
  printf(
    "[          ] 45° 斜停：AABB %.2f m²，L-Shape %.2f m²（真值 %.2f m²），"
    "省掉 %.0f%%\n",
    aabb_area, lshape_area, 4.4 * 1.8, 100.0 * (1.0 - lshape_area / aabb_area));

  EXPECT_LT(lshape_area, aabb_area * 0.8) << "L-Shape 没有比 AABB 更贴合 —— 那它就白写了";
  EXPECT_NEAR(lshape_area, 4.4 * 1.8, 1.5);
}

// ---------------------------------------------------------------------------
//  边界与防御
// ---------------------------------------------------------------------------
TEST(LShapeFit, RefusesToFitTooFewPoints)
{
  // ⚠️ 三个点必定共面，任何朝向都能"完美贴合" —— 拟合出来的角度是噪声。
  //    此时必须报 valid = false，而不是给一个看起来正常的朝向。
  std::vector<Eigen::Vector3d> few = {{1, 0, 0}, {1, 1, 0}, {2, 1, 0}};
  EXPECT_FALSE(FitLShape(few, LShapeFitParams{}).valid);
  EXPECT_FALSE(FitLShape({}, LShapeFitParams{}).valid);
}

TEST(LShapeFit, ThrowsOnNonFiniteInput)
{
  auto points = MakeLShapedCar({15.0, 0.0}, 0.3, 4.4, 1.8, 1.5, {0.0, 0.0});
  for (const double poison : {std::numeric_limits<double>::infinity(), std::nan("")}) {
    auto polluted = points;
    polluted[5].x() = poison;
    EXPECT_THROW(FitLShape(polluted, LShapeFitParams{}), std::invalid_argument);
  }
}

TEST(LShapeFit, KeepsTheSparseTargetsThatSOneMeasured)
{
  // S1 体检实测：锥桶与行人在 20–25 m 只有 7 点。min_points = 4 必须放它们过去
  // —— 朝向不可靠时下游可以退回用 AABB，而目标整个消失就没得救了。
  const auto sparse = MakeLShapedCar({22.0, 0.0}, 0.0, 0.5, 0.5, 0.8, {0.0, 0.0}, 0.25, 0.01);
  printf(
    "[          ] 稀疏锥桶 %zu 点 → valid = %d\n", sparse.size(),
    static_cast<int>(FitLShape(sparse, LShapeFitParams{}).valid));
  EXPECT_TRUE(FitLShape(sparse, LShapeFitParams{}).valid);
}

// ---------------------------------------------------------------------------
//  尺寸分类
// ---------------------------------------------------------------------------
TEST(SizeClassifier, ClassifiesTheThreeTargetsThisProjectActuallyHas)
{
  const SizeClassifierParams params;
  // 尺寸取**实测包围盒**的量级，不是标称值（雷达看到的系统性偏小）。
  EXPECT_EQ(ClassifyBySize(4.4, 1.8, 1.45, params), ObjectClass::kVehicle);
  EXPECT_EQ(ClassifyBySize(0.42, 0.40, 1.52, params), ObjectClass::kPedestrian);
  EXPECT_EQ(ClassifyBySize(0.50, 0.50, 0.72, params), ObjectClass::kStatic) << "锥桶";
  EXPECT_EQ(ClassifyBySize(1.8, 0.6, 1.4, params), ObjectClass::kBicycle);
}

TEST(SizeClassifier, SaysUnknownRatherThanGuessing)
{
  // ⚠️ UNKNOWN 是**正常输出**，不是失败。落到这一档的东西下游会保守处理
  //    （当静态障碍物做碰撞检查、不做运动预测）。
  const SizeClassifierParams params;
  // 一堵 8 m 的矮墙 —— 长度超出车辆上限。
  EXPECT_EQ(ClassifyBySize(8.0, 0.3, 2.0, params), ObjectClass::kUnknown);
  // 两辆车被并成一簇（S2 的 tolerance 调大就会这样）。
  EXPECT_EQ(ClassifyBySize(9.0, 1.9, 1.5, params), ObjectClass::kUnknown);
}

TEST(SizeClassifier, ShowsWhatItCannotDistinguish)
{
  // ⚠️ 这条用例**不是**在测试功能，而是把能力边界钉住：
  //    尺寸分类是「猜」不是「识别」，下面这些误判是**设计使然**。
  //    P6 会按 classification 选运动模型，所以这份清单必须显式存在 ——
  //    否则下游会以为拿到了语义。
  const SizeClassifierParams params;

  // 一根 1.6 m 高的路灯杆底段 → 被判成行人。
  EXPECT_EQ(ClassifyBySize(0.3, 0.3, 1.6, params), ObjectClass::kPedestrian)
    << "路灯杆被判成行人 —— 这是预期的误判，见 size_classifier.hpp";
  // 一堵 4 m 长、1.5 m 高的矮墙 → 被判成车辆。
  EXPECT_EQ(ClassifyBySize(4.0, 1.5, 1.5, params), ObjectClass::kVehicle)
    << "矮墙被判成车辆 —— 这是预期的误判";

  printf(
    "[          ] 能力边界：路灯杆→%s，矮墙→%s（**都是预期的误判**）\n",
    ObjectClassName(ClassifyBySize(0.3, 0.3, 1.6, params)),
    ObjectClassName(ClassifyBySize(4.0, 1.5, 1.5, params)));
}

TEST(SizeClassifier, HandlesNonFiniteInputExplicitly)
{
  // ⚠️ NaN 参与任何比较都返回 false，于是所有 if 全不成立、落到 UNKNOWN ——
  //    那个结果**碰巧**安全，但它是"恰好对了"不是"处理了"。
  //    一旦有人把某个分支改成 `if (!(x > a))`，NaN 就会命中它。
  const SizeClassifierParams params;
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(ClassifyBySize(std::nan(""), 1.8, 1.5, params), ObjectClass::kUnknown);
  EXPECT_EQ(ClassifyBySize(4.4, inf, 1.5, params), ObjectClass::kUnknown);
  EXPECT_EQ(ClassifyBySize(4.4, 1.8, std::nan(""), params), ObjectClass::kUnknown);
}
