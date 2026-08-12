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

#include "ads_perception/size_classifier.hpp"

#include <cmath>

namespace ads_perception
{

ObjectClass ClassifyBySize(
  double length_m, double width_m, double height_m, const SizeClassifierParams & params)
{
  // ⚠️ 非有限值**必须先拦**，不能靠后面的比较。
  //    NaN 参与任何比较都返回 false，于是所有 if 全不成立、落到 kUnknown ——
  //    那个结果**碰巧**是安全的，但它是"恰好对了"不是"处理了"。
  //    一旦有人把某个分支改成 `if (!(x > a))` 的形式，NaN 就会命中它。
  //    这个坑本仓库咬过两次（vehicle_cmd_bridge 的限幅、ads_control 的点距校验）。
  if (!std::isfinite(length_m) || !std::isfinite(width_m) || !std::isfinite(height_m)) {
    return ObjectClass::kUnknown;
  }

  // ---- 矮的一律 STATIC，**先判** ----------------------------------------
  // 放在最前面是因为它与其它几档在长宽上会重叠（一排锥桶的包围盒长度
  // 可能落进车辆区间），而高度是最可靠的区分量。
  if (height_m < params.static_max_height_m) {
    return ObjectClass::kStatic;
  }

  const double footprint = std::max(length_m, width_m);

  // ---- 行人：底面小、细高 ------------------------------------------------
  if (
    footprint <= params.pedestrian_max_footprint_m && height_m >= params.pedestrian_min_height_m &&
    height_m <= params.pedestrian_max_height_m) {
    return ObjectClass::kPedestrian;
  }

  // ---- 自行车：细长 ------------------------------------------------------
  if (
    length_m >= params.bicycle_min_length_m && length_m <= params.bicycle_max_length_m &&
    width_m <= params.bicycle_max_width_m && height_m <= params.bicycle_max_height_m) {
    return ObjectClass::kBicycle;
  }

  // ---- 车辆 --------------------------------------------------------------
  if (
    length_m >= params.vehicle_min_length_m && length_m <= params.vehicle_max_length_m &&
    width_m >= params.vehicle_min_width_m && width_m <= params.vehicle_max_width_m &&
    height_m >= params.vehicle_min_height_m && height_m <= params.vehicle_max_height_m) {
    return ObjectClass::kVehicle;
  }

  // ⚠️ 落到这里是**正常输出**，不是失败。下游对 UNKNOWN 应当保守处理
  //    （当静态障碍物做碰撞检查、不做运动预测）。
  //    "兜底成某个类"等于把猜错的风险藏起来。
  return ObjectClass::kUnknown;
}

const char * ObjectClassName(ObjectClass value)
{
  switch (value) {
    case ObjectClass::kUnknown:
      return "UNKNOWN";
    case ObjectClass::kPedestrian:
      return "PEDESTRIAN";
    case ObjectClass::kBicycle:
      return "BICYCLE";
    case ObjectClass::kVehicle:
      return "VEHICLE";
    case ObjectClass::kStatic:
      return "STATIC";
  }
  return "UNKNOWN";
}

}  // namespace ads_perception
