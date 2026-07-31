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

#include "ads_map/geometry.hpp"

#include <cmath>

namespace ads_map
{

Pose2D Geometry::pose_at(double ds_m) const
{
  // 直线单独一支：下面圆弧的闭式解在 k = 0 时是 0/0 型。
  // 用「很小的 k」去近似直线会在 s 大的时候产生肉眼可见的偏差 ——
  // 本项目最长的一段直线 76 m，那个偏差足以让车道中心线错位。
  if (curvature_inv_m == 0.0) {
    return Pose2D{
      x_m + ds_m * std::cos(heading_rad), y_m + ds_m * std::sin(heading_rad), heading_rad};
  }

  const double k = curvature_inv_m;
  const double h1 = heading_rad + k * ds_m;
  return Pose2D{
    x_m + (std::sin(h1) - std::sin(heading_rad)) / k,
    y_m - (std::cos(h1) - std::cos(heading_rad)) / k, h1};
}

Pose2D Geometry::end_pose() const { return pose_at(length_m); }

}  // namespace ads_map
