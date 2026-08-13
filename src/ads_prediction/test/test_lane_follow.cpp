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
//  车道跟随预测的 L1 判据（CP-P6-A ②③④⑤⑦）
//
//  弧线判据跑在**真园区地图**上（ADS_MAP_XODR_PATH，与 ads_map 的测试同一份），
//  期望值来自**独立的解析推导**（campus_map.yaml 的数字手推，不从被测代码取）：
//  SE 弯参考线 R=12 圆心 (78,−38)，内圈车道中心 = r = 12 − 1.75 = 10.25 的圆。
//  这套数字已被 S1 实测背书（curve_car 沿它跑，对分析中心线偏差 max 0.21 m）。
//
//  截断/逆行/正编号翻转跑在**程序化迷你地图**上（一条 40 m 直路、双向两车道、
//  无后继）—— 这三条要的是"没有后继"和"可控的车道方向"，真地图给不了。
//
//  ## 故障注入实测（2026-08-12，跑完回填 —— 第一行的预写就是错的，如实改）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 正编号车道不翻 180° | **红 4 条**：`PositiveLane…` `TracksTheCornerArc…`
//  |   | `Diverges…` `Branches…`。预写猜"红 2 条：本文件 + ads_map 的采样
//  |   | 对账"—— 后者**不存在**（ads_map 没有采样单测），而弧线/分叉用例
//  |   | 跑的车道恰是正编号，全被翻转波及。红得比预想广是好事：藏不住 |
//  | 截断改成"夹到末点" | **红 1 条**：`TruncatesAtLaneEnd…`（末尾堆重合点被抓） |
//  | 分叉激励点放进路口区（x=6 < 11） | **红 1 条**（开发中实测）：连接道路
//  |   | 单后继，分叉不发生 —— 判据的激励点错了什么都测不到 |
//  | 去掉 30° 方向门限 | 第一版**六条全绿 —— 门限无人守着**（逆行由
//  |   | nearest_lane 的 90° 过滤兜住，门限的独立职责是斜穿）。补
//  |   | `RefusesACrossingTarget`（60° 斜穿）后重注入 → **红 1 条**且只红它 |
// =============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "ads_map/lane_graph.hpp"
#include "ads_map/opendrive_parser.hpp"
#include "ads_prediction/lane_follow.hpp"
#include "ads_prediction/motion_model.hpp"

namespace
{

using ads_map::Geometry;
using ads_map::Lane;
using ads_map::LaneGraph;
using ads_map::LaneSection;
using ads_map::LaneWidth;
using ads_map::Road;
using ads_map::RoadMap;
using ads_prediction::LaneFollowParams;
using ads_prediction::MotionModelParams;
using ads_prediction::PredictConstantVelocity;
using ads_prediction::PredictedPath;
using ads_prediction::PredictLaneFollow;
using ads_prediction::TargetSnapshot;

// SE 弯的独立解析推导（campus_map.yaml 手推，理由见文件头）。
constexpr double kCornerCenterX = 78.0;
constexpr double kCornerCenterY = -38.0;
constexpr double kInnerLaneRadiusM = 10.25;
constexpr double kArcEntryX = 88.25;
constexpr double kArcEntryY = -38.0;

const LaneGraph & CampusGraph()
{
  static const LaneGraph graph{ads_map::load_opendrive(ADS_MAP_XODR_PATH)};
  return graph;
}

/// 迷你地图：一条 40 m 直路（+x），双向两车道，无后继。
LaneGraph MiniGraph()
{
  Road road;
  road.id = 1;
  road.length_m = 40.0;
  Geometry geometry;
  geometry.length_m = 40.0;
  road.geometries = {geometry};

  LaneWidth width;
  width.a = 3.5;
  Lane right;
  right.id = -1;
  right.type = "driving";
  right.widths = {width};
  Lane left;
  left.id = 1;
  left.type = "driving";
  left.widths = {width};
  LaneSection section;
  section.right = {right};
  section.left = {left};
  road.lane_sections = {section};

  RoadMap map;
  map.roads[1] = road;
  return LaneGraph(std::move(map));
}

TargetSnapshot MakeTarget(double x, double y, double vx, double vy)
{
  TargetSnapshot target;
  target.id = 11;
  target.position = {x, y};
  target.velocity = {vx, vy};
  target.length_m = 4.4;
  target.width_m = 1.8;
  target.net_displacement_1s_m = std::hypot(vx, vy);
  return target;
}

}  // namespace

// ---------------------------------------------------------------------------
//  CP-P6-A ②：沿 R=10.25 弧的预测 vs 解析圆
// ---------------------------------------------------------------------------
TEST(LaneFollow, TracksTheCornerArcAgainstTheAnalyticCircle)
{
  MotionModelParams motion;
  LaneFollowParams params;
  // 弧入口，顺时针（南向）进弯。
  const TargetSnapshot target = MakeTarget(kArcEntryX, kArcEntryY, 0.0, -4.0);
  const auto paths = PredictLaneFollow(target, CampusGraph(), motion, params);
  ASSERT_EQ(paths.size(), 1U) << "SE 弯中段没有路口，应当只有一条假设";

  double worst = 0.0;
  for (const auto & point : paths[0].points) {
    const double phi = 4.0 * point.t_s / kInnerLaneRadiusM;  // 顺时针扫过的角
    const Eigen::Vector2d expected(
      kCornerCenterX + kInnerLaneRadiusM * std::cos(-phi),
      kCornerCenterY + kInnerLaneRadiusM * std::sin(-phi));
    worst = std::max(worst, (point.position - expected).norm());
  }
  printf("[          ] 弧线跟随 vs 解析圆最大偏差 %.4f m（判据 0.01）\n", worst);
  EXPECT_LT(worst, 0.01) << "弧上的车道跟随不贴解析圆 —— 采样/参数化有错";
}

// ---------------------------------------------------------------------------
//  CP-P6-A ⑤：分歧解析对账 —— 判据区分力的 L1 版本
// ---------------------------------------------------------------------------
TEST(LaneFollow, DivergesFromConstantVelocityByTheAnalyticAmount)
{
  MotionModelParams motion;
  LaneFollowParams params;
  const TargetSnapshot target = MakeTarget(kArcEntryX, kArcEntryY, 0.0, -4.0);
  const auto lane_paths = PredictLaneFollow(target, CampusGraph(), motion, params);
  ASSERT_EQ(lane_paths.size(), 1U);
  const PredictedPath cv_path =
    PredictConstantVelocity(target, motion, motion.vehicle_lateral_accel_mps2);

  // 解析值：φ = v·T/r = 12/10.25，末点差 = |(v·T − r·sinφ, r(1−cosφ))|
  //        = |(2.5623, 6.2510)| = 6.756 m。
  // ⚠️ plan.md CP-P6-A ⑤ 原写 5.83 —— 那是**参考线** R=12 的数；车沿的是
  //    车道弧 r=10.25。判据修正为 6.76，推导就在这两行（不是改到能过：
  //    数字变大了，分得更开）。
  const double gap = (lane_paths[0].points.back().position - cv_path.points.back().position).norm();
  printf("[          ] 3 s 末点分歧 %.3f m（解析 6.756）—— 两模型分得开\n", gap);
  EXPECT_NEAR(gap, 6.756, 0.05) << "分歧与解析值不符 —— 要么弧没跟上，要么恒速不直";
}

// ---------------------------------------------------------------------------
//  CP-P6-A ③：跨路口的多假设 + 逐点连续性
// ---------------------------------------------------------------------------
TEST(LaneFollow, BranchesAtTheJunctionWithContinuousPaths)
{
  MotionModelParams motion;
  LaneFollowParams params;
  // 南直道西行（对向车道 y=−48.25），x=14 在路口腿断开处（|x|=11，
  // cutback = 8 + 1.75 + 1.25）以东 3 m —— 3 s 视界（12 m）必然进路口，
  // 那里有「直行 + 右转北上」两条连接道路（test_lane_graph：常规车道每条
  // 恰好 2 条出边）。⚠️ 放进 |x|<11 的话目标已在**连接道路**上（单后继），
  // 分叉根本不会发生 —— 第一版就踩了这个，激励点错了测不出分叉。
  const TargetSnapshot target = MakeTarget(14.0, -48.25, -4.0, 0.0);
  const auto paths = PredictLaneFollow(target, CampusGraph(), motion, params);
  ASSERT_GE(paths.size(), 2U) << "路口没有分叉出多假设";

  double probability_sum = 0.0;
  for (const auto & path : paths) {
    probability_sum += path.probability;
    ASSERT_GE(path.points.size(), 2U);
    for (std::size_t i = 1; i < path.points.size(); ++i) {
      const double spacing = (path.points[i].position - path.points[i - 1].position).norm();
      // 相邻点距 = v·step = 0.8 m。跨车道边界的跳变是米级 —— 1.5× 就抓得住。
      EXPECT_LT(spacing, 1.5 * 4.0 * motion.step_s)
        << "第 " << i << " 点跳了 " << spacing << " m —— 跨 successor 不连续";
      const double turn = std::fabs(
        std::remainder(path.points[i].heading_rad - path.points[i - 1].heading_rad, 2.0 * M_PI));
      // 路口转弯 R=8：0.8 m 走 0.1 rad；0.35 rad 只有约定级错误才到得了。
      EXPECT_LT(turn, 0.35) << "第 " << i << " 点朝向跳了 " << turn << " rad";
    }
  }
  printf("[          ] 路口分出 %zu 条假设，概率和 %.3f\n", paths.size(), probability_sum);
  EXPECT_NEAR(probability_sum, 1.0, 1e-9) << "多假设的概率不归一";
}

// ---------------------------------------------------------------------------
//  CP-P6-A ④：车道走到头且无后继 —— 如实截断
// ---------------------------------------------------------------------------
TEST(LaneFollow, TruncatesAtLaneEndInsteadOfExtrapolating)
{
  const LaneGraph graph = MiniGraph();
  MotionModelParams motion;
  LaneFollowParams params;
  // 距路末 10 m，3 s 要走 12 m —— 走不完。
  const TargetSnapshot target = MakeTarget(30.0, -1.75, 4.0, 0.0);
  const auto paths = PredictLaneFollow(target, graph, motion, params);
  ASSERT_EQ(paths.size(), 1U);
  const auto & points = paths[0].points;
  ASSERT_GE(points.size(), 2U);
  printf(
    "[          ] 末点 x=%.2f（路末 40），点数 %zu/16 —— 如实截断\n", points.back().position.x(),
    points.size());
  EXPECT_LT(points.size(), 16U) << "没截断 —— 预测越过了地图的尽头";
  EXPECT_LE(points.back().position.x(), 40.0 + 0.01);
  // 截断不是"夹到末点"：不许出现一堆挤在末点上的重合点
  // （path_remaining_m 那条陷阱的预测版）。
  for (std::size_t i = 1; i < points.size(); ++i) {
    EXPECT_GT((points[i].position - points[i - 1].position).norm(), 0.1)
      << "第 " << i << " 点与前一点重合 —— 截断写成了夹取";
  }
}

// ---------------------------------------------------------------------------
//  正编号车道：逆 s 行驶，位姿必须翻 180°
// ---------------------------------------------------------------------------
TEST(LaneFollow, PositiveLaneRunsAgainstSAndHeadsBackwards)
{
  const LaneGraph graph = MiniGraph();
  MotionModelParams motion;
  LaneFollowParams params;
  // +1 车道（y=+1.75）逆 s 行驶 = 朝 −x。
  const TargetSnapshot target = MakeTarget(30.0, 1.75, -4.0, 0.0);
  const auto paths = PredictLaneFollow(target, graph, motion, params);
  ASSERT_EQ(paths.size(), 1U);
  const auto & points = paths[0].points;
  printf(
    "[          ] +1 车道：末点 x=%.2f（起点 30），朝向 %.3f（应 ±π）\n",
    points.back().position.x(), points.back().heading_rad);
  EXPECT_LT(points.back().position.x(), 30.0 - 8.0) << "正编号车道没朝 −s 走";
  EXPECT_NEAR(std::fabs(std::remainder(points.back().heading_rad, 2.0 * M_PI)), M_PI, 1e-6)
    << "正编号车道的行驶朝向没翻 180° —— 预测逆行（lane_sampling 的坑 ①）";
}

// ---------------------------------------------------------------------------
//  CP-P6-A ⑦：逆行目标 —— 车道跟随拒绝，交回调用方退恒速
// ---------------------------------------------------------------------------
TEST(LaneFollow, RefusesAWrongWayTarget)
{
  const LaneGraph graph = MiniGraph();
  MotionModelParams motion;
  LaneFollowParams params;
  // 在 −1 车道（沿 s = +x 行驶）上却朝 −x 开 —— NPC 折返段的真实工况。
  // ⚠️ 这一条 nearest_lane 自己的 90° 过滤就拦得住（180° > 90°）——
  //    它守的是「逆行不给车道预测」这个**行为**，不区分哪道闸拦的。
  const TargetSnapshot target = MakeTarget(30.0, -1.75, -4.0, 0.0);
  const auto paths = PredictLaneFollow(target, graph, motion, params);
  printf("[          ] 逆行目标：车道跟随给出 %zu 条假设（应 0）\n", paths.size());
  EXPECT_TRUE(paths.empty()) << "给逆行目标编了沿车道的轨迹 —— 方向门限失效，预测与实际反向 180°";
}

TEST(LaneFollow, RefusesACrossingTarget)
{
  const LaneGraph graph = MiniGraph();
  MotionModelParams motion;
  LaneFollowParams params;
  // 60° 斜穿车道：nearest_lane 的 90° 过滤拦不住（60 < 90），
  // 只有 30° 方向门限在守 —— 这是那道门限**唯一**的独立判据
  // （第一版没有它，注入"去掉门限"六条用例全绿 —— 门限无人守着）。
  // 真实工况：横穿路口 / 掉头中段的目标，速度方向与车道斜交。
  const TargetSnapshot target = MakeTarget(30.0, -1.75, 2.0, 3.4641);
  const auto paths = PredictLaneFollow(target, graph, motion, params);
  printf("[          ] 60° 斜穿目标：车道跟随给出 %zu 条假设（应 0）\n", paths.size());
  EXPECT_TRUE(paths.empty()) << "斜穿目标被按\"沿车道\"预测 —— 方向差 60° 的轨迹会把规划带偏";
}
