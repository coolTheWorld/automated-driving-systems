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

#ifndef ADS_PERCEPTION__SIZE_CLASSIFIER_HPP_
#define ADS_PERCEPTION__SIZE_CLASSIFIER_HPP_

// =============================================================================
//  尺寸分类：按包围盒的长/宽/高猜一个类别
//
//  ## ⚠️⚠️ 它是「尺寸猜测」，不是「识别」。这句话必须一直挂在这里
//
//  SPEC §3.2 的第一版方案就是尺寸分类（升级路径才是 PointPillars/CenterPoint，
//  那属于「先问后做」）。所以本模块的能力边界必须写死在接口上，
//  否则下游会误以为拿到了语义：
//
//      判为 PEDESTRIAN 的，可能是一根路灯杆 + 一丛灌木
//      判为 VEHICLE   的，可能是一堵 4 m 的矮墙、一排并在一起的锥桶
//
//  **为什么这件事必须说清楚**：P6 预测会按 classification 选运动模型 ——
//  车走"地图约束下的车道跟随"，人走"恒速 + 不确定椭圆"。选错的后果是
//  预测轨迹荒谬（一堵墙被预测成沿车道行驶），而那时人会去查预测模块。
//  **症状出现在两个模块之外，这是本项目反复踩到的那一类。**
//
//  ## ⚠️ 阈值必须按**实测的**包围盒定，不是按目标的标称尺寸
//
//  雷达看到的包围盒**系统性偏小**，有两个独立的原因：
//    ① 只打得到朝向自己的面，背面没有点 → 进深方向偏小（S2 实测：
//       正对雷达的 4.4 m 车，可见面到包围盒中心差约 2.2 m）；
//    ② 贴地那一圈被地面分割吸收 → 高度偏小（S2 实测 0.15 m 阈值下，
//       离地 0.25 m 以内的点大部分被算成地面）。
//
//  所以拿"行人高 1.7 m"去定阈值下限，实测的 1.5 m 会**落空**。
//  阈值区间要足够宽，并且**宁可判 UNKNOWN 也不要判错** —— UNKNOWN 下游
//  会保守处理，而判错会让 P6 用错模型。
// =============================================================================

#include <cstdint>

namespace ads_perception
{

/// 类别。取值与 `ads_msgs/Obstacle` 的 `CLASSIFICATION_*` 常量**逐一对应** ——
/// 这里不用 ROS 类型（本包零 ROS 依赖），但数值必须一致，
/// 否则节点层做一次映射就多一处可以写错的地方。
enum class ObjectClass : std::uint8_t
{
  kUnknown = 0,
  kPedestrian = 1,
  kBicycle = 2,
  kVehicle = 3,
  kStatic = 4,
};

/// 分类阈值。全部按**实测包围盒**定，不是标称尺寸，见文件头。
struct SizeClassifierParams
{
  /// 行人：底面小、细高。
  ///
  /// 底面上限 0.9 m —— 成年人肩宽约 0.5，加上手臂摆动与点云噪声留余量。
  /// 调大到 1.5 → 一丛灌木、一个消防栓都会被判成行人。
  double pedestrian_max_footprint_m{0.9};
  /// 高度区间。**下限 1.2 而不是 1.5**：贴地一圈被地面分割吸收后，
  /// 1.7 m 的行人实测只有 1.5 左右，再远一点更矮。
  /// 调高下限 → 远处行人掉进 UNKNOWN；调低到 0.8 → 锥桶被判成行人。
  double pedestrian_min_height_m{1.2};
  double pedestrian_max_height_m{2.1};

  /// 自行车/摩托：细长、比行人矮一点。
  double bicycle_min_length_m{1.2};
  double bicycle_max_length_m{2.5};
  double bicycle_max_width_m{1.0};
  double bicycle_max_height_m{2.0};

  /// 车辆。
  ///
  /// 长度下限 2.5 —— 比自行车上限大，两类不重叠。
  /// ⚠️ **上限 7.0 不是"卡车也算"**，而是给"只看得见一部分的车 + 噪声"留余量；
  ///    真出现 7 m 以上的簇，多半是两辆车被并成了一簇（S2 的 tolerance
  ///    调大就会这样），那时判 UNKNOWN 比判 VEHICLE 诚实。
  double vehicle_min_length_m{2.5};
  double vehicle_max_length_m{7.0};
  double vehicle_min_width_m{1.2};
  double vehicle_max_width_m{2.8};
  double vehicle_min_height_m{1.0};
  double vehicle_max_height_m{2.6};

  /// 比这个矮的一律 STATIC（锥桶、路缘、减速带）。
  ///
  /// ⚠️ STATIC **不是"可以忽略"** —— SPEC §11 明确禁止拿分类去过滤
  ///    碰撞检查。它只是告诉 P6「别给这个东西编一条运动轨迹」。
  double static_max_height_m{1.0};
};

/// 按尺寸猜一个类别。
///
/// @param length_m 长（沿长轴），来自 L-Shape 拟合。
/// @param width_m  宽。
/// @param height_m 高（**实测极差，系统性偏小**，见文件头）。
/// @param params   阈值。
/// @return 类别；任何一档都不匹配时返回 `kUnknown`。
///
/// ⚠️ 返回 `kUnknown` 是**正常且有用的输出**，不是失败。下游对未知目标
/// 应当保守处理（当成静态障碍物做碰撞检查、不做运动预测）。
/// 把它当失败去"兜底成某个类"，等于把猜错的风险藏起来。
ObjectClass ClassifyBySize(
  double length_m, double width_m, double height_m, const SizeClassifierParams & params);

/// 类别名，供日志与诊断用。
const char * ObjectClassName(ObjectClass value);

}  // namespace ads_perception

#endif  // ADS_PERCEPTION__SIZE_CLASSIFIER_HPP_
