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

#ifndef ADS_MAP__GEOMETRY_HPP_
#define ADS_MAP__GEOMETRY_HPP_

// =============================================================================
//  参考线几何 —— 纯 C++17，**不依赖 ROS，也不依赖 XML**
//
//  这一层只回答一个问题：「沿着这段线走 ds 米，人在哪、朝哪」。
//  它不知道 OpenDRIVE、不知道车道、不知道地图，所以它的单元测试
//  可以直接跟解析解比对，不需要造任何地图。
//
//  为什么只支持直线和圆弧
//  ----------------------
//  OpenDRIVE 规范定义了五种参考线原语：line、arc、spiral（回旋线）、
//  poly3、paramPoly3。本项目**有意只实现前两种**，因为地图由
//  scripts/gen_map.py 自己生成，用到哪些原语完全可控。
//
//  ⚠️ 代价是读不了 CARLA 官方的 Town 地图（它们含 spiral）。
//     这是清醒的取舍，不是疏漏。真要读的时候，扩展点就在这个文件里，
//     而 opendrive_parser 遇到不支持的原语会**显式抛异常**——
//     静默跳过的症状是路网少了一段而无人知晓。
// =============================================================================

namespace ads_map
{

/// @brief 平面位姿。坐标系为地图系（ENU：x 东、y 北、z 上），与 ROS 的 `map` 一致。
struct Pose2D
{
  double x_m{0.0};
  double y_m{0.0};
  /// 航向角，单位**弧度**。x 轴正方向为 0，逆时针为正（右手系，与 REP-103 一致）。
  /// 注意本结构体**不保证**它已归一化到 [-π, π]：沿参考线累加时它会连续增长，
  /// 那正是我们想要的（跨越 ±π 时不跳变）。需要归一化请显式调用 ads_common。
  double heading_rad{0.0};
};

/// @brief 参考线上的一段几何。字段与 OpenDRIVE 的 `<geometry>` 元素一一对应。
///
/// 一条道路的参考线是若干段 Geometry 首尾相接而成，各段按 s0_m 递增排列。
struct Geometry
{
  /// 该段起点在**整条参考线**上的弧长坐标，单位 m。
  double s0_m{0.0};
  /// 该段起点的世界坐标，单位 m。
  double x_m{0.0};
  double y_m{0.0};
  /// 该段起点的航向，单位弧度。
  double heading_rad{0.0};
  /// 该段弧长，单位 m。必须为正。
  double length_m{0.0};
  /// 曲率 1/R，单位 1/m。**0 表示直线**；正值为左转（逆时针），负值为右转。
  double curvature_inv_m{0.0};

  /// @brief 求该段起点之后 ds_m 处的位姿。
  ///
  /// @param ds_m 相对**本段起点**的弧长，单位 m。
  ///             有效范围 [0, length_m]，但本函数**不做范围检查** ——
  ///             它是热路径上的底层原语，越界由上层 Road::reference_pose_at 拦。
  ///             越界求值在数学上仍有定义（沿同一条直线/圆继续延伸）。
  /// @return 该处的位姿，坐标为地图系。
  ///
  /// @note 圆弧的闭式解由 dx/ds = cos(h₀ + k·s)、dy/ds = sin(h₀ + k·s) 积分得到：
  ///           x(ds) = x₀ + [sin(h₀ + k·ds) − sin(h₀)] / k
  ///           y(ds) = y₀ − [cos(h₀ + k·ds) − cos(h₀)] / k
  ///           h(ds) = h₀ + k·ds
  ///       对 k 的正负都成立，不需要分左右转两种写法。
  ///
  /// @warning k → 0 时上式是 0/0 型，所以**直线必须单独走一支**。
  ///          靠「取一个很小的 k 来近似直线」会在 s 大时产生肉眼可见的偏差。
  Pose2D pose_at(double ds_m) const;

  /// @brief 该段终点的位姿。等价于 pose_at(length_m)。
  Pose2D end_pose() const;
};

}  // namespace ads_map

#endif  // ADS_MAP__GEOMETRY_HPP_
