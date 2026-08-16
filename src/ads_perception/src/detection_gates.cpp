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

#include "ads_perception/detection_gates.hpp"

#include <algorithm>

#include "ads_common/numeric_checks.hpp"

namespace ads_perception
{

Admission AdmitDetection(
  double length_m, double width_m, double height_m, double bottom_above_ground_m,
  const AdmissionParams & params)
{
  ads_common::RequireFinite(length_m, "AdmitDetection", "length_m");
  ads_common::RequireFinite(width_m, "AdmitDetection", "width_m");
  ads_common::RequireFinite(height_m, "AdmitDetection", "height_m");
  ads_common::RequireFinite(bottom_above_ground_m, "AdmitDetection", "bottom_above_ground_m");

  // ① 剃刀条：薄 且 矮。边界：严格小于（min(l,w) 恰好等于 0.1 的不算薄）。
  if (
    std::min(length_m, width_m) < params.razor_min_extent_m &&
    height_m < params.razor_max_height_m) {
    return Admission::kRazorStrip;
  }
  // ② 浮空碎片：底离地严格高于门 且 延展严格低于门。
  //    落地目标 30 m 内最低一环离地 ≤ 地面阈值 0.2 + 线间距 0.59 = 0.8 < 1.0，够不着；
  //    墙后只露上半身的行人（底 1.0、延展 0.7）在延展条件上被放行。
  if (
    bottom_above_ground_m > params.floating_min_bottom_m &&
    height_m < params.floating_max_height_m) {
    return Admission::kFloatingFragment;
  }
  return Admission::kAccepted;
}

}  // namespace ads_perception
