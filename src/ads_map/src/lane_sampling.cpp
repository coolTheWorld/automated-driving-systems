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

#include "ads_map/lane_sampling.hpp"

#include <cmath>
#include <vector>

namespace ads_map
{

namespace
{
/// 区间短于它就当成一个点。1e-6 m = 微米级，比任何真实几何都小两个量级。
constexpr double kDegenerateSpanM = 1e-6;
}  // namespace

std::vector<Pose2D> sample_lane_centerline(
  const RoadMap & map, const LaneId & lane, double from_s_m, double to_s_m, double step_m)
{
  const Road & road = map.road(lane.road_id);
  const double span_m = std::fabs(to_s_m - from_s_m);
  const double direction = (to_s_m >= from_s_m) ? 1.0 : -1.0;

  const auto pose_at = [&road, &lane, from_s_m, direction](double offset_m) {
    Pose2D pose = road.lane_center_pose_at(lane.lane_id, from_s_m + direction * offset_m);
    if (lane.lane_id > 0) {
      pose.heading_rad += M_PI;  // 正编号车道逆 s 行驶，行驶朝向要翻 180°
    }
    return pose;
  };

  std::vector<Pose2D> poses;
  if (span_m <= kDegenerateSpanM) {
    poses.push_back(pose_at(0.0));  // 退化区间：一个点，不是两个重合的点
    return poses;
  }

  // 等分而不是「步长累加 + 末点夹取」，理由见头文件的坑 ②：
  // i=0 精确是起点，i=count 精确是终点，段长恒为 span/count ≤ step，无需夹取。
  const int count = static_cast<int>(std::ceil(span_m / step_m));
  poses.reserve(static_cast<std::size_t>(count) + 1);
  for (int i = 0; i <= count; ++i) {
    poses.push_back(pose_at(span_m * static_cast<double>(i) / static_cast<double>(count)));
  }
  return poses;
}

}  // namespace ads_map
