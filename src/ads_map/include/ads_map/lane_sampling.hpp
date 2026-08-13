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

#ifndef ADS_MAP__LANE_SAMPLING_HPP_
#define ADS_MAP__LANE_SAMPLING_HPP_

// =============================================================================
//  沿车道中心线按**行驶方向**采样
//
//  P6-S1 从 map_node 的私有方法下沉至此（决策二）：这段逻辑此前只有一个
//  消费者（map_node 画 /route/path 与 MarkerArray），P6 预测的车道跟随
//  外推是第二个 —— 按 CLAUDE.md 的判据「有第二个消费者、且它有逻辑」，
//  必须共用，不许各抄一份。
//
//  ⚠️ 它守着两个「做错了不报错、只给出一条看起来正常的路」的坑：
//
//  ① **正编号车道逆 s 行驶，行驶朝向要翻 180°**（OpenDRIVE 约定，
//     见 road_map.hpp 顶部）。漏翻的话采出来的位姿位置全对、朝向全反 ——
//     预测拿它外推就是一条**逆行**轨迹，而 RViz 里那条线看起来完全正常。
//
//  ② **区间等分，不是「步长累加 + 末点夹取」**。span/step 恰好整除时浮点
//     上可能是 80.00000000000001，ceil 多算一步，最后两个采样点重合 ——
//     RViz 里看不出来，下游按弧长参数化时除以零。本项目实测踩过
//     （nearest_lane 三分法给出 s=40.000000000000007 触发）。
// =============================================================================

#include <vector>

#include "ads_map/lane_graph.hpp"
#include "ads_map/road_map.hpp"

namespace ads_map
{

/// @brief 沿车道中心线采样一串**行驶方向**上的位姿。
/// @param map 路网。
/// @param lane 车道。
/// @param from_s_m 起点的参考线弧长 [m]。
/// @param to_s_m 终点的参考线弧长 [m]。**可以小于 from_s_m** ——
///        正编号车道逆 s 行驶（LaneNode 的 entry_s_m > exit_s_m）。
/// @param step_m 采样步长（参考线 s 上的）[m]，必须 > 0。
/// @return 位姿序列，按**行驶顺序**排列，朝向已翻转为行驶朝向；
///         区间退化（|to−from| ≤ 1e-6）时返回单个位姿。
/// @throw std::out_of_range s 越界（沿用 Road::lane_center_pose_at 的约定：
///        显式抛出而不是静默截断 —— 调用方要自己经 successors 换段）。
std::vector<Pose2D> sample_lane_centerline(
  const RoadMap & map, const LaneId & lane, double from_s_m, double to_s_m, double step_m);

}  // namespace ads_map

#endif  // ADS_MAP__LANE_SAMPLING_HPP_
