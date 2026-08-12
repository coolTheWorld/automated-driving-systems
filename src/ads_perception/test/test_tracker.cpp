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
//  多目标跟踪的 L1 判据
//
//  ⚠️ **判据必须跑在带丢帧的序列上。** 每帧都命中的理想序列上，
//     「累计命中确认」与「连续命中确认」给出完全一样的结果 ——
//     判据就没有区分力了，而那正是 S1 实测推翻教科书写法的地方。
//     （与地面分割"必须带坡度"、聚类"必须按真实角分辨率采样"同一类。）
//
//     S1 体检实测：目标在连续帧之间**闪烁**，命中率只有 33–74%。
//
//  ## 故障注入实测（2026-08-11）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 确认改成「连续命中」（未命中时重置 hits） | **红 1 条**（闪烁那条） |
//  | 给静止目标也猜朝向（去掉速度门限） | **红 1 条** |
//
//  第一条证明「累计命中而非连续命中」这个设计**真的被守住了** ——
//  而那个设计来自 S1 的实测，不是抄来的默认值。
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "ads_perception/tracker.hpp"

namespace
{

using ads_perception::Detection;
using ads_perception::Track;
using ads_perception::Tracker;
using ads_perception::TrackerParams;

constexpr double kDt = 0.1;  // 10 Hz，与雷达一致

Detection MakeDetection(double x, double y, double yaw_rad = 0.0)
{
  Detection detection;
  detection.position = Eigen::Vector2d(x, y);
  detection.yaw_rad = yaw_rad;
  detection.length_m = 4.4;
  detection.width_m = 1.8;
  detection.height_m = 1.5;
  return detection;
}

/// 找指定 id 的航迹。
const Track * FindById(const std::vector<Track> & tracks, std::uint32_t id)
{
  const auto found =
    std::find_if(tracks.begin(), tracks.end(), [id](const Track & t) { return t.id == id; });
  return found == tracks.end() ? nullptr : &*found;
}

}  // namespace

// ---------------------------------------------------------------------------
//  速度估计
// ---------------------------------------------------------------------------
TEST(Tracker, ConvergesToTheTrueVelocity)
{
  Tracker tracker;
  const double truth_vx = 4.0;
  for (int frame = 0; frame < 30; ++frame) {
    tracker.Update({MakeDetection(10.0 + truth_vx * frame * kDt, 0.0)}, kDt);
  }
  const auto tracks = tracker.ConfirmedTracks();
  ASSERT_EQ(tracks.size(), 1U);
  printf(
    "[          ] 真值 vx = %.2f → 估计 (%.3f, %.3f)\n", truth_vx, tracks[0].velocity().x(),
    tracks[0].velocity().y());
  EXPECT_NEAR(tracks[0].velocity().x(), truth_vx, 0.3);
  EXPECT_NEAR(tracks[0].velocity().y(), 0.0, 0.3);
}

TEST(Tracker, GetsTheSignRightForAnOncomingTarget)
{
  // ⚠️ **符号错要单独判**。对向行驶时真值 vx 是负的，而如果哪里把
  //    新息的方向写反了，速度大小照样对、只有符号反 ——
  //    P6 会预测它朝相反方向开走，P7 于是认为"不用让"。
  Tracker tracker;
  for (int frame = 0; frame < 30; ++frame) {
    tracker.Update({MakeDetection(60.0 - 4.0 * frame * kDt, 0.0)}, kDt);
  }
  const auto tracks = tracker.ConfirmedTracks();
  ASSERT_EQ(tracks.size(), 1U);
  printf("[          ] 对向行驶：估计 vx = %.3f（真值 −4.0）\n", tracks[0].velocity().x());
  EXPECT_LT(tracks[0].velocity().x(), 0.0) << "对向目标的速度符号反了";
  EXPECT_NEAR(tracks[0].velocity().x(), -4.0, 0.3);
}

// ---------------------------------------------------------------------------
//  ⚠️ 闪烁：这一组是本文件的核心
// ---------------------------------------------------------------------------
TEST(Tracker, ConfirmsATargetThatFlickersLikeSOneMeasured)
{
  // S1 实测的闪烁形态：命中、命中、丢、命中、丢、丢、命中……
  //
  // ⚠️ 「连续 3 帧命中才确认」在这个序列上**永远确认不了**（没有连续 3 帧）。
  //    而累计命中在第 5 帧就达成。**这条用例就是那个设计决定的判据。**
  const std::vector<bool> hit_pattern = {true, true, false, true, false, false, true, true};
  Tracker tracker;
  double x = 10.0;
  int frame = 0;
  for (const bool hit : hit_pattern) {
    x += 4.0 * kDt;
    tracker.Update(
      hit ? std::vector<Detection>{MakeDetection(x, 0.0)} : std::vector<Detection>{}, kDt);
    ++frame;
    printf(
      "[          ] 第 %d 帧 %s → 航迹 %zu 条，已确认 %zu 条\n", frame, hit ? "命中" : "丢失",
      tracker.tracks().size(), tracker.ConfirmedTracks().size());
  }

  const auto confirmed = tracker.ConfirmedTracks();
  ASSERT_EQ(confirmed.size(), 1U) << "闪烁的目标没能被确认 —— 确认判据用的是连续命中？";
  EXPECT_EQ(confirmed[0].id, 1U) << "ID 变了 —— 航迹被删过又重建";
}

TEST(Tracker, KeepsTheIdAcrossAnOcclusion)
{
  // 目标被遮挡 4 帧后重现（max_misses = 5，所以航迹应当活下来）。
  //
  // ⚠️ ID 跳变的后果是 P6 的历史轨迹被清空 —— 预测退化成"刚看见这个东西"，
  //    而它其实已经跟了两秒。这正是 CP-P5-B 第 6 条要验的东西。
  Tracker tracker;
  double x = 10.0;
  for (int frame = 0; frame < 5; ++frame) {
    x += 4.0 * kDt;
    tracker.Update({MakeDetection(x, 0.0)}, kDt);
  }
  const std::uint32_t original_id = tracker.ConfirmedTracks().at(0).id;

  for (int frame = 0; frame < 4; ++frame) {
    x += 4.0 * kDt;  // 目标继续走，只是看不见
    tracker.Update({}, kDt);
  }
  ASSERT_FALSE(tracker.tracks().empty()) << "遮挡 4 帧航迹就没了 —— max_misses 太小";

  x += 4.0 * kDt;
  tracker.Update({MakeDetection(x, 0.0)}, kDt);
  const auto tracks = tracker.ConfirmedTracks();
  ASSERT_EQ(tracks.size(), 1U);
  printf("[          ] 遮挡 4 帧后重现：ID %u → %u\n", original_id, tracks[0].id);
  EXPECT_EQ(tracks[0].id, original_id) << "遮挡后 ID 变了";
}

TEST(Tracker, DropsATrackWhoseTargetReallyLeft)
{
  // 与上一条相对：真的走了就该删，否则会留下"幽灵航迹"而规划会绕它。
  Tracker tracker;
  for (int frame = 0; frame < 5; ++frame) {
    tracker.Update({MakeDetection(10.0 + 4.0 * frame * kDt, 0.0)}, kDt);
  }
  ASSERT_EQ(tracker.ConfirmedTracks().size(), 1U);
  for (int frame = 0; frame < 6; ++frame) {  // > max_misses = 5
    tracker.Update({}, kDt);
  }
  printf("[          ] 连续丢 6 帧后剩 %zu 条航迹\n", tracker.tracks().size());
  EXPECT_TRUE(tracker.tracks().empty()) << "目标走了却留下幽灵航迹";
}

// ---------------------------------------------------------------------------
//  数据关联
// ---------------------------------------------------------------------------
TEST(Tracker, DoesNotSwapIdsWhenTwoTargetsPassCloseBy)
{
  // 两个目标相向而行、擦身而过 —— 这是关联最容易出错的位形。
  //
  // ⚠️ ID 互换之后，两条航迹的速度估计各自跳到对方的历史上，
  //    P6 会预测出两条交叉的荒谬轨迹。**这正是用匈牙利而不是贪心的理由。**
  Tracker tracker;
  std::uint32_t id_a = 0;
  std::uint32_t id_b = 0;
  for (int frame = 0; frame < 40; ++frame) {
    const double t = frame * kDt;
    // A 沿 +x 走（y = +1.5），B 沿 −x 走（y = −1.5），在 t = 2 s 附近交会。
    const double xa = 10.0 + 3.0 * t;
    const double xb = 22.0 - 3.0 * t;
    tracker.Update({MakeDetection(xa, 1.5), MakeDetection(xb, -1.5)}, kDt);

    if (frame == 5) {
      const auto tracks = tracker.ConfirmedTracks();
      ASSERT_EQ(tracks.size(), 2U);
      // 记下 y > 0 的那条（A）与 y < 0 的那条（B）。
      for (const Track & track : tracks) {
        (track.position().y() > 0.0 ? id_a : id_b) = track.id;
      }
    }
  }

  const auto tracks = tracker.ConfirmedTracks();
  ASSERT_EQ(tracks.size(), 2U);
  const Track * a = FindById(tracks, id_a);
  const Track * b = FindById(tracks, id_b);
  ASSERT_NE(a, nullptr) << "A 的 ID 丢了";
  ASSERT_NE(b, nullptr) << "B 的 ID 丢了";
  printf(
    "[          ] 交会后：ID %u 在 y=%.2f（vx=%.2f），ID %u 在 y=%.2f（vx=%.2f）\n", a->id,
    a->position().y(), a->velocity().x(), b->id, b->position().y(), b->velocity().x());

  // A 始终在 y > 0 且向 +x；B 始终在 y < 0 且向 −x。互换了就会反过来。
  EXPECT_GT(a->position().y(), 0.0) << "ID 互换了";
  EXPECT_GT(a->velocity().x(), 0.0);
  EXPECT_LT(b->position().y(), 0.0) << "ID 互换了";
  EXPECT_LT(b->velocity().x(), 0.0);
}

TEST(Tracker, StartsANewTrackForADetectionThatIsTooFarToAssociate)
{
  // 一个突然出现在 50 m 外的检测不该被关联到已有航迹上 —— 那是新目标。
  Tracker tracker;
  for (int frame = 0; frame < 5; ++frame) {
    tracker.Update({MakeDetection(10.0 + 4.0 * frame * kDt, 0.0)}, kDt);
  }
  const std::size_t before = tracker.tracks().size();
  tracker.Update({MakeDetection(10.0 + 2.0, 0.0), MakeDetection(60.0, 20.0)}, kDt);
  printf("[          ] 远处冒出一个检测：航迹 %zu → %zu 条\n", before, tracker.tracks().size());
  EXPECT_EQ(tracker.tracks().size(), before + 1) << "远处的新目标没有起新航迹";
}

// ---------------------------------------------------------------------------
//  ⚠️ 180° 二义性的消歧 —— 与 S3 接力的地方
// ---------------------------------------------------------------------------
TEST(Tracker, ResolvesHeadingFromVelocityButOnlyWhenMovingFastEnough)
{
  // S3 的 L-Shape 只给**轴向**（[0, π)）。这里用速度把它消歧成车头朝向。
  //
  // 目标沿 −x 行驶，轴向报 0（即 ±x 轴）。真正的车头朝向是 π（−x 方向）。
  Tracker tracker;
  for (int frame = 0; frame < 30; ++frame) {
    tracker.Update({MakeDetection(60.0 - 4.0 * frame * kDt, 0.0, /*yaw=*/0.0)}, kDt);
  }
  const auto tracks = tracker.ConfirmedTracks();
  ASSERT_EQ(tracks.size(), 1U);
  ASSERT_TRUE(tracks[0].heading_resolved) << "4 m/s 的目标应当能消歧朝向";
  printf(
    "[          ] 轴向 0.00 rad + 速度 %.2f m/s → 车头朝向 %.3f rad（应为 ±π）\n",
    tracks[0].velocity().x(), tracks[0].heading_rad);
  // 车头朝 −x，即 ±π。
  EXPECT_NEAR(std::abs(tracks[0].heading_rad), M_PI, 0.2) << "朝向没被消歧到速度方向那一侧";
}

TEST(Tracker, RefusesToGuessHeadingForASlowTarget)
{
  // ⚠️ **静止目标的朝向物理上无解**，不能猜。
  //    猜错的概率 50%，而错了之后 P6 会预测出一条逆行轨迹 ——
  //    症状出现在两个模块之外，且没有任何一层会报错。
  Tracker tracker;
  for (int frame = 0; frame < 30; ++frame) {
    tracker.Update({MakeDetection(20.0, 0.0, 0.3)}, kDt);  // 一动不动
  }
  const auto tracks = tracker.ConfirmedTracks();
  ASSERT_EQ(tracks.size(), 1U);
  printf(
    "[          ] 静止目标：速度 %.4f m/s，heading_resolved = %d\n", tracks[0].velocity().norm(),
    static_cast<int>(tracks[0].heading_resolved));
  EXPECT_FALSE(tracks[0].heading_resolved) << "给静止目标猜了一个朝向";
}

// ---------------------------------------------------------------------------
//  确认门限
// ---------------------------------------------------------------------------
TEST(Tracker, DoesNotPublishUnconfirmedTracks)
{
  // ⚠️ 未确认的航迹可能只是噪点簇。下游（规划）会对每个障碍物做碰撞检查，
  //    虚警的代价是车无故刹停。
  Tracker tracker;
  tracker.Update({MakeDetection(15.0, 0.0)}, kDt);
  EXPECT_EQ(tracker.tracks().size(), 1U);
  EXPECT_TRUE(tracker.ConfirmedTracks().empty()) << "一帧就确认了 —— confirm_hits 没生效";

  tracker.Update({MakeDetection(15.4, 0.0)}, kDt);
  EXPECT_TRUE(tracker.ConfirmedTracks().empty());
  tracker.Update({MakeDetection(15.8, 0.0)}, kDt);
  EXPECT_EQ(tracker.ConfirmedTracks().size(), 1U) << "累计 3 次命中后应当确认";
}

// ---------------------------------------------------------------------------
//  边界与防御
// ---------------------------------------------------------------------------
TEST(Tracker, RejectsBadInput)
{
  Tracker tracker;
  EXPECT_THROW(tracker.Update({}, 0.0), std::invalid_argument) << "dt = 0";
  EXPECT_THROW(tracker.Update({}, -0.1), std::invalid_argument) << "dt < 0";
  EXPECT_THROW(tracker.Update({MakeDetection(std::nan(""), 0.0)}, kDt), std::invalid_argument);
  EXPECT_THROW(
    tracker.Update({MakeDetection(std::numeric_limits<double>::infinity(), 0.0)}, kDt),
    std::invalid_argument);

  TrackerParams bad;
  bad.confirm_hits = 0;
  EXPECT_THROW(Tracker{bad}, std::invalid_argument);
}

TEST(Tracker, HandlesAnEmptyFrame)
{
  // 空帧是**常态**（S1 实测目标闪烁），不是异常。
  Tracker tracker;
  EXPECT_NO_THROW(tracker.Update({}, kDt));
  EXPECT_TRUE(tracker.tracks().empty());
}
