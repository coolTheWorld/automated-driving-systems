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
//  时空冲突计算的 L1 测试（CP-P7-A ①⑥⑦ + 时间标注）
//
//  参考线取一条沿 +x 的直线（s ≡ x、d ≡ y），于是每一条判据都有闭式解：
//    * 匀速横穿目标进出走廊的时刻    t = (∓(half+2σ) − y₀) / v_y
//    * 恒速剖面的到达时刻            t(s) = s / v
//    * 匀加速段内的到达时刻          t = (v(s) − v₀) / a
//  判据来自运动学，不来自实现 —— 改实现不用改判据。
//
//  ## 故障注入实测（2026-08-13，写完立刻做的）
//
//  | 注入 | 结果 |
//  |---|---|
//  | follow 判定加 `length_m < 1.0 → 跳过`（复原「行人不算前车」类特判）
//    | **红** 1 例：StaticPedestrianInLaneIsAFollowConflict。预写的表以为 ⑦
//    也会红 —— 错了，⑦ 用的是 4.4 m 的车，不吃小目标特判。
//    **又一次**预写注入表不准，第三回了 —— 表必须写实测 |
//  | crossing 的膨胀去掉 2σ（只用走廊半宽）
//    | **红** 1 例：InflationWidensTheWindow（σ=0 的用例不受影响，
//    正是要它们分开红） |
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "ads_common/reference_line.hpp"
#include "ads_planning/conflict.hpp"

namespace
{

using ads_common::Pose2D;
using ads_common::ReferenceLine;
using ads_planning::annotate_times;
using ads_planning::BehaviorParams;
using ads_planning::find_crossing_conflicts;
using ads_planning::find_follow_conflict;
using ads_planning::PredictedPoint;
using ads_planning::PredictionHypothesis;
using ads_planning::TargetBox;
using ads_planning::time_at;

/// 全部用例共用的参数。数值与 behavior.md §5 的表一致 ——
/// 不是必须一致（L1 不读配置），一致是为了让手推的数直接可比。
BehaviorParams MakeParams()
{
  BehaviorParams params;
  params.corridor_half_m = 1.75;
  params.blocking_half_m = 0.55;  // = 0.9 + 0.5 − 0.85（推导量，见 conflict.hpp）
  params.stand_off_m = 4.0;
  params.yield_margin_m = 4.0;  // S4 实测从 2.0 上调：让行点也要停在盲区外
  params.time_margin_s = 1.0;
  params.sigma_inflation_cap_m = 3.5;
  params.front_offset_m = 3.55;
  return params;
}

/// 沿 +x 的直线参考线，s ≡ x、d ≡ y（左正）。
ReferenceLine MakeStraightLine(double length_m, double spacing_m)
{
  std::vector<Pose2D> poses;
  for (double x = 0.0; x <= length_m + 1e-9; x += spacing_m) {
    poses.push_back({x, 0.0, 0.0});
  }
  return ReferenceLine(std::move(poses));
}

// -----------------------------------------------------------------------------
//  时间标注
// -----------------------------------------------------------------------------

TEST(AnnotateTimes, ConstantSpeedIsLinear)
{
  // 恒速 v=4 下 t(s) = s/4，逐点闭式。
  const std::vector<double> s{0.0, 2.0, 5.0, 10.0};
  const std::vector<double> v{4.0, 4.0, 4.0, 4.0};
  const auto t = annotate_times(s, v);
  for (std::size_t i = 0; i < s.size(); ++i) {
    EXPECT_NEAR(t[i], s[i] / 4.0, 1e-12);
  }
  // 段内任意点同样闭式（time_at 不是线性插值，但恒速下两者相同）。
  EXPECT_NEAR(time_at(s, v, t, 3.5), 3.5 / 4.0, 1e-12);
}

TEST(AnnotateTimes, ConstantAccelerationMatchesKinematics)
{
  // 从 2 到 6 m/s 匀加速跑 8 m：a = (36−4)/16 = 2，全程 t = (6−2)/2 = 2 s。
  // 中途 s=2：v = √(4+2·2·2) = √12，t = (√12−2)/2 —— time_at 必须给出闭式值。
  const std::vector<double> s{0.0, 8.0};
  const std::vector<double> v{2.0, 6.0};
  const auto t = annotate_times(s, v);
  EXPECT_NEAR(t[1], 2.0, 1e-12);
  EXPECT_NEAR(time_at(s, v, t, 2.0), (std::sqrt(12.0) - 2.0) / 2.0, 1e-12);
}

TEST(AnnotateTimes, StoppedSegmentIsInfinityAndPropagates)
{
  // 真实停车剖面的形状是**从停车点起持续为 0**（截断 + terminal=0）。
  // 零速段 v_avg = 0 ⟹ +∞，且要向后传染，不是只标一个点。
  //
  // ⚠️ 单个 v=0 的**点**后面接正速度（{3,0,3}）不属于这个用例：
  //    运动学上从静止匀加速穿过零点用时有限（t = Δs/v_avg，L1 首轮实测
  //    把它误当成了「到不了」）—— 那是 stop-and-go，不是停车。
  const std::vector<double> s{0.0, 5.0, 10.0, 15.0};
  const std::vector<double> v{3.0, 0.0, 0.0, 0.0};
  const auto t = annotate_times(s, v);
  EXPECT_NEAR(t[1], 5.0 / 1.5, 1e-12);  // 减速段 v_avg = 1.5
  EXPECT_TRUE(std::isinf(t[2]));
  EXPECT_TRUE(std::isinf(t[3]));
  EXPECT_TRUE(std::isinf(time_at(s, v, t, 12.0)));
}

// -----------------------------------------------------------------------------
//  自车道类（FOLLOW）
// -----------------------------------------------------------------------------

TEST(FollowConflict, NearestInCorridorWinsAndEdgesAreUsed)
{
  const auto line = MakeStraightLine(60.0, 0.5);
  const auto params = MakeParams();
  // 两个都在走廊里：近的赢。近边 = 中心 − 半长。
  const std::vector<TargetBox> targets{
    {7u, 40.0, 0.3, 4.4, 1.8},
    {9u, 25.0, -0.5, 4.4, 1.8},
  };
  const auto conflict = find_follow_conflict(line, 5.0, targets, params);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_EQ(conflict->id, 9u);
  EXPECT_NEAR(conflict->near_edge_s_m, 25.0 - 2.2, 1e-9);
}

TEST(FollowConflict, BlockingNotCorridorDecidesWhoIsALead)
{
  // FOLLOW 的判据是**阻挡**（挡死所有横向候选），不是「在车道里」——
  // S3 集成时实测改的：P3 的贴边锥桶（近缘 −0.95）在 1.75 的走廊里，
  // 按走廊判会把「可绕」当成跟停对象，test_closed_loop_obstacle 当场红。
  const auto line = MakeStraightLine(60.0, 0.5);
  const auto params = MakeParams();
  // CP-P3-B avoid 场景的真实锥桶：d=−1.2、宽 0.5 ⟹ [−1.45, −0.95]，
  // t_hi = −0.95 < −0.55 ⟹ 不阻挡 —— 归 lattice 绕行，不触发跟停。
  const std::vector<TargetBox> avoidable{{1u, 30.0, -1.2, 0.5, 0.5}};
  EXPECT_FALSE(find_follow_conflict(line, 0.0, avoidable, params).has_value());
  // 同一个锥桶往中心挪到 d=−0.75 ⟹ [−1.0, −0.5]，t_hi = −0.5 ≥ −0.55
  // ⟹ 它伸进了「无论怎么偏都必须占用」的带 —— 阻挡，是前车。
  const std::vector<TargetBox> blocking{{2u, 30.0, -0.75, 0.5, 0.5}};
  EXPECT_TRUE(find_follow_conflict(line, 0.0, blocking, params).has_value());
  // 隔壁车道的车（d=3.5）：远在带外，不是前车。
  const std::vector<TargetBox> next_lane{{3u, 30.0, 3.5, 4.4, 1.8}};
  EXPECT_FALSE(find_follow_conflict(line, 0.0, next_lane, params).has_value());
}

TEST(FollowConflict, BehindEgoIsIgnored)
{
  const auto line = MakeStraightLine(60.0, 0.5);
  const std::vector<TargetBox> targets{{3u, 10.0, 0.0, 4.4, 1.8}};
  // 目标近边 7.8 < ego 20：车尾已越过的目标不是前车。
  EXPECT_FALSE(find_follow_conflict(line, 20.0, targets, MakeParams()).has_value());
}

TEST(FollowConflict, StaticPedestrianInLaneIsAFollowConflict)
{
  // CP-P7-A ⑥ 红线的结构性验证：判定**不看运动状态、不看分类** ——
  // 一个 0.4×0.4 的静止行人站在走廊里，与一辆车触发同一条逻辑。
  const auto line = MakeStraightLine(60.0, 0.5);
  const std::vector<TargetBox> targets{{4u, 30.0, 0.0, 0.4, 0.4}};
  const auto conflict = find_follow_conflict(line, 5.0, targets, MakeParams());
  ASSERT_TRUE(conflict.has_value());
  EXPECT_NEAR(conflict->near_edge_s_m, 29.8, 1e-9);
}

// -----------------------------------------------------------------------------
//  横穿类（YIELD）—— CP-P7-A ① 的闭式对账
// -----------------------------------------------------------------------------

/// 匀速横穿的预测假设：从 (x₀, y₀) 以 v_y 向 +y 走，采样步长 dt。
PredictionHypothesis MakeCrossing(
  std::uint32_t id, double x0, double y0, double vy, double dt_s, double horizon_s, double sigma)
{
  PredictionHypothesis hypothesis;
  hypothesis.obstacle_id = id;
  for (double t = 0.0; t <= horizon_s + 1e-9; t += dt_s) {
    hypothesis.points.push_back({t, x0, y0 + vy * t, sigma});
  }
  return hypothesis;
}

TEST(CrossingConflict, WindowMatchesClosedFormExactly)
{
  // CP-P7-A ①：目标从 y₀=−4.75 以 v_y=1.5 北上，x=30。σ=0，走廊半宽 1.75。
  // 进出时刻闭式：t_in = (−1.75+4.75)/1.5 = 2.0，t_out = (1.75+4.75)/1.5 ≈ 4.333。
  // 采样 dt=0.25 恰好落在 t_in 上（8×0.25）—— 窗口下界必须**精确**等于闭式值。
  // ⚠️ 步长与边界可通约是**故意的**（这里要的就是精确对账）；
  //    「采样与结构不可通约」那条纪律适用于**扫描找 bug** 的用例，不适用这里。
  const auto line = MakeStraightLine(60.0, 0.5);
  auto params = MakeParams();
  const double ego_s = 0.0;
  // ego 恒速 5：t_ego(30) = 6.0，窗口 [2.0, 4.33] + τ=1 ⟹ [1.0, 5.33] 不含 6.0 → 不冲突。
  // 把 τ 放大到 2 ⟹ [0, 6.33] 含 6.0 → 冲突。两侧都对账。
  std::vector<double> s, v;
  for (double x = 0.0; x <= 60.0 + 1e-9; x += 0.5) {
    s.push_back(x);
    v.push_back(5.0);
  }
  const auto t = annotate_times(s, v);

  const std::vector<PredictionHypothesis> hyp{MakeCrossing(11u, 30.0, -4.75, 1.5, 0.25, 6.0, 0.0)};
  auto conflicts = find_crossing_conflicts(line, ego_s, s, v, t, hyp, params);
  EXPECT_TRUE(conflicts.empty()) << "τ=1 时 ego 晚于窗口 0.67 s 到达，不该触发";

  params.time_margin_s = 2.0;
  conflicts = find_crossing_conflicts(line, ego_s, s, v, t, hyp, params);
  ASSERT_EQ(conflicts.size(), 1u);
  EXPECT_NEAR(conflicts[0].t_lo_s, 2.0, 1e-9) << "进入时刻必须等于闭式值 (1.75+…)/v_y";
  EXPECT_NEAR(conflicts[0].t_hi_s, 4.25, 1e-9)
    << "离开时刻是最后一个仍在走廊内的采样点（4.333 的前一个 0.25 格 = 4.25）";
  EXPECT_NEAR(conflicts[0].s_lo_m, 30.0, 1e-9);
  EXPECT_NEAR(conflicts[0].s_hi_m, 30.0, 1e-9);
}

TEST(CrossingConflict, StaticHypothesisInCorridorAlwaysConflicts)
{
  // lib 层的机制事实：点不动的假设站在走廊里 ⟹ t 窗 = [0, 视界]，恒重叠。
  //
  // ⚠️ **系统层面 STATIC 模型不走这条路**（S4 实测改的，见 planning_node 的
  //    过滤与 behavior.md §2.2）：STATIC 的威胁是位置性的，归阻挡/准入管；
  //    起步律椭圆进横穿判定会把路侧静物变成永久让行。本用例保留是因为
  //    lib 不认识「模型」—— 它钉住的是**机制**（恒定假设的窗口语义），
  //    过滤是 node 的职责，两层各测各的。
  const auto line = MakeStraightLine(60.0, 0.5);
  std::vector<double> s, v;
  for (double x = 0.0; x <= 60.0 + 1e-9; x += 0.5) {
    s.push_back(x);
    v.push_back(5.0);
  }
  const auto t = annotate_times(s, v);
  // 目标放 s=15（ego 3.0 s 可达，窗口 [0,3]+τ=1 有 1 s 余量）——
  // 不放 s=20：那会把判据卡在 4.0 ≤ 4.0 的浮点边界上（累加的 4.000…002 会假红）。
  const std::vector<PredictionHypothesis> hyp{MakeCrossing(12u, 15.0, 0.5, 0.0, 0.25, 3.0, 0.0)};
  const auto conflicts = find_crossing_conflicts(line, 0.0, s, v, t, hyp, MakeParams());
  ASSERT_EQ(conflicts.size(), 1u);
  EXPECT_NEAR(conflicts[0].t_lo_s, 0.0, 1e-12);
  EXPECT_NEAR(conflicts[0].t_hi_s, 3.0, 1e-12);
}

TEST(CrossingConflict, InflationWidensTheWindow)
{
  // CP-P7-A ⑦ 的横穿半边：σ_cross 参与膨胀。
  // 同一条假设，σ=0 时首个入廊点在 t=2.0（见闭式用例）；σ=0.5 ⟹ 半宽 2.75，
  // t_in = (−2.75+4.75)/1.5 ≈ 1.333 → 首个入廊采样点 1.5。窗口必须变宽。
  const auto line = MakeStraightLine(60.0, 0.5);
  auto params = MakeParams();
  params.time_margin_s = 2.0;
  std::vector<double> s, v;
  for (double x = 0.0; x <= 60.0 + 1e-9; x += 0.5) {
    s.push_back(x);
    v.push_back(5.0);
  }
  const auto t = annotate_times(s, v);
  const std::vector<PredictionHypothesis> hyp{MakeCrossing(13u, 30.0, -4.75, 1.5, 0.25, 6.0, 0.5)};
  const auto conflicts = find_crossing_conflicts(line, 0.0, s, v, t, hyp, params);
  ASSERT_EQ(conflicts.size(), 1u);
  EXPECT_NEAR(conflicts[0].t_lo_s, 1.5, 1e-9) << "2σ 膨胀后窗口提前（1.333 后的首个采样点）";
  EXPECT_GT(conflicts[0].t_hi_s, 4.25) << "离开侧同样变宽";
}

TEST(CrossingConflict, EgoStoppedNeverConflicts)
{
  // ego 被停车剖面钉住（stop 之后 +∞）：够不到的窗口不构成冲突。
  const auto line = MakeStraightLine(60.0, 0.5);
  const std::vector<double> s{0.0, 5.0, 60.0};
  const std::vector<double> v{3.0, 0.0, 0.0};
  const auto t = annotate_times(s, v);
  const std::vector<PredictionHypothesis> hyp{MakeCrossing(14u, 30.0, 0.0, 0.0, 0.25, 3.0, 0.0)};
  EXPECT_TRUE(find_crossing_conflicts(line, 0.0, s, v, t, hyp, MakeParams()).empty());
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(FollowConflict, TargetBeyondTheRouteEndIsNotALead)
{
  // S4 junction 实测咬到的：车流驶出 ego 路线后投影被夹到末端，
  // 行为层把它当成「路线尽头的静止前车」，ego 在离终点 9.75 m 处趴住。
  // 陷阱表「投影越过路径末点后被夹到端点」的行为层落点 —— 用例钉死排除。
  const auto line = MakeStraightLine(60.0, 0.5);
  // 路线只到 x=60；目标在 (80, 0.3) —— 路线之外的世界里，不是前车。
  const std::vector<TargetBox> beyond{{31u, 80.0, 0.3, 4.4, 1.8}};
  EXPECT_FALSE(find_follow_conflict(line, 30.0, beyond, MakeParams()).has_value());
  // 对照：同样的目标放在路线内（x=50）就是前车。
  const std::vector<TargetBox> inside{{32u, 50.0, 0.3, 4.4, 1.8}};
  EXPECT_TRUE(find_follow_conflict(line, 30.0, inside, MakeParams()).has_value());
}

TEST(CrossingConflict, PredictionBeyondTheRouteEndIsNotAConflict)
{
  // 同上，横穿半边：驶出路线末端的预测点不许在末端捏出假冲突窗。
  const auto line = MakeStraightLine(60.0, 0.5);
  std::vector<double> s, v;
  for (double x = 0.0; x <= 60.0 + 1e-9; x += 0.5) {
    s.push_back(x);
    v.push_back(5.0);
  }
  const auto t = annotate_times(s, v);
  const std::vector<PredictionHypothesis> hyp{MakeCrossing(33u, 80.0, 0.0, 0.0, 0.25, 3.0, 0.0)};
  EXPECT_TRUE(find_crossing_conflicts(line, 30.0, s, v, t, hyp, MakeParams()).empty());
}
