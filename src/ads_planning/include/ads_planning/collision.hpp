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

#ifndef ADS_PLANNING__COLLISION_HPP_
#define ADS_PLANNING__COLLISION_HPP_

// =============================================================================
//  矩形（OBB）相交与间距 —— 纯 C++17，**不依赖 ROS**
//
//  **为什么在笛卡尔系做，不在 Frenet 系做**（planning.md §5）：
//  障碍物是笛卡尔系里的矩形，变到 Frenet 之后**不再是矩形** ——
//  弯道上它会被弯成扇形。用 Frenet 下的轴对齐盒去近似它，误差在弯道上最大，
//  而弯道恰恰是最需要精确的地方。
//
//  两个函数，职责分开：
//    overlaps()   —— 布尔判定，用**分离轴定理**，对凸多边形是精确的
//    distance_m() —— 不相交时的最小间距，用于 SPEC §8 S04 的「侧向间距 > 0.5 m」
//
//  ⚠️ **间距是准入条件，不是代价项。** 不满足直接淘汰候选，不允许"够便宜就擦着过"
//     —— 那等于把安全逻辑放进一个可被权重绕过的分支，CLAUDE.md 明令禁止。
// =============================================================================

#include <array>

namespace ads_planning
{

/// @brief 有向包围盒（OBB）。
///
/// @note 车体和障碍物用同一个类型。车体的 `length_m`/`width_m` 取自
///       `vehicle_params.yaml` 的 `geometry`，障碍物的取自 `ads_msgs/Obstacle::size_m`。
///       **两者都不许硬编码。**
struct Rectangle
{
  /// 矩形中心，`map` 系，单位 m。**注意是几何中心，不是 `base_link`（后轴中心）** ——
  /// 车体矩形要先把后轴位姿沿车头方向前移 `length/2 − rear_overhang` 才对。
  /// 这一步在调用方做，本层只认几何中心。
  double center_x_m{0.0};
  double center_y_m{0.0};
  /// 长边方向，单位弧度。
  double heading_rad{0.0};
  /// 沿 `heading_rad` 的全长，单位 m。
  double length_m{0.0};
  /// 垂直于 `heading_rad` 的全宽，单位 m。
  double width_m{0.0};
};

/// @brief 四个角点，顺序为「左前、右前、右后、左后」（车体坐标下的象限顺序）。
std::array<std::array<double, 2>, 4> corners_of(const Rectangle & rectangle);

/// @brief 两个矩形是否相交（含相切）。
///
/// 用**分离轴定理**：两个凸多边形不相交 ⟺ 存在一条轴，两者在其上的投影区间不重叠。
/// 矩形只需检验四条轴（各自的两个边法向），因为对边平行、法向重复。
///
/// @note **相切（投影区间恰好接触）判为相交。** 闭矩形共享一个边界点在数学上
///       确实是相交；而在安全语义上，"恰好贴上"必须算撞上。
///       这条由 `TangentRectanglesCountAsOverlapping` 用例钉住 —— `>` 写成 `>=`
///       就会翻过来，而随机测试几乎不可能采到恰好相切的位形。
///
/// @note 零宽或零长的退化矩形不会崩：它的投影区间退化成一个点，SAT 照常成立。
///       上游给出这种障碍物通常意味着感知有 bug，但**本层不替它做判断** ——
///       校验在 `plan_lateral()` 入口做一次，见 lattice.hpp。
bool overlaps(const Rectangle & a, const Rectangle & b);

/// @brief 两个矩形之间的最小距离，单位 m。相交时返回 0。
///
/// @note **相交时返回 0 而不是负的穿透深度**，这是有意的：
///       没有任何调用方需要穿透深度（相交的候选是直接淘汰的，不参与打分），
///       而算穿透深度是另一套算法。返回 0 是诚实的 ——「贴上了或更糟」。
///
/// @note **不用分离轴的最大间隙当距离。** 那个值只是下界：顶点对顶点的位形下
///       最近点连线不是任何一条边的法向，SAT 会低估真实距离。
///       这里改成"遍历所有（顶点, 边）对取最小"，对**不相交的凸多边形是精确的**
///       —— 因为最近点对中至少有一个必在顶点上。
///       4 个顶点 × 4 条边 × 2 个方向 = 32 次点到线段距离，代价可以忽略。
double distance_m(const Rectangle & a, const Rectangle & b);

}  // namespace ads_planning

#endif  // ADS_PLANNING__COLLISION_HPP_
