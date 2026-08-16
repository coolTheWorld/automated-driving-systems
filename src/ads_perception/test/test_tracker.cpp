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
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ads_perception/tracker.hpp"

namespace
{

using ads_perception::Detection;
using ads_perception::Track;
using ads_perception::Tracker;
using ads_perception::TrackerParams;

constexpr double kDt = 0.1;  // 10 Hz，与雷达一致

/// 传感器位置。本文件里目标都在 +x 前方，所以自车放原点最自然。
/// ⚠️ 它参与「补全方向」的判断（见 tracker.hpp 的 CompletedCenter）——
///    本文件多数用例的检测尺寸恒定、补全量为零，所以取值不影响它们；
///    专门验补全的用例在文件末尾，那里传感器位置是**判据的一部分**。
const Eigen::Vector2d kSensor(0.0, 0.0);

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
    tracker.Update({MakeDetection(10.0 + truth_vx * frame * kDt, 0.0)}, kDt, kSensor);
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
    tracker.Update({MakeDetection(60.0 - 4.0 * frame * kDt, 0.0)}, kDt, kSensor);
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
      hit ? std::vector<Detection>{MakeDetection(x, 0.0)} : std::vector<Detection>{}, kDt, kSensor);
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
    tracker.Update({MakeDetection(x, 0.0)}, kDt, kSensor);
  }
  const std::uint32_t original_id = tracker.ConfirmedTracks().at(0).id;

  for (int frame = 0; frame < 4; ++frame) {
    x += 4.0 * kDt;  // 目标继续走，只是看不见
    tracker.Update({}, kDt, kSensor);
  }
  ASSERT_FALSE(tracker.tracks().empty()) << "遮挡 4 帧航迹就没了 —— max_misses 太小";

  x += 4.0 * kDt;
  tracker.Update({MakeDetection(x, 0.0)}, kDt, kSensor);
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
    tracker.Update({MakeDetection(10.0 + 4.0 * frame * kDt, 0.0)}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.ConfirmedTracks().size(), 1U);
  for (int frame = 0; frame < 6; ++frame) {  // > max_misses = 5
    tracker.Update({}, kDt, kSensor);
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
    tracker.Update({MakeDetection(xa, 1.5), MakeDetection(xb, -1.5)}, kDt, kSensor);

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
    tracker.Update({MakeDetection(10.0 + 4.0 * frame * kDt, 0.0)}, kDt, kSensor);
  }
  const std::size_t before = tracker.tracks().size();
  tracker.Update({MakeDetection(10.0 + 2.0, 0.0), MakeDetection(60.0, 20.0)}, kDt, kSensor);
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
    tracker.Update({MakeDetection(60.0 - 4.0 * frame * kDt, 0.0, /*yaw=*/0.0)}, kDt, kSensor);
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
    tracker.Update({MakeDetection(20.0, 0.0, 0.3)}, kDt, kSensor);  // 一动不动
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
  tracker.Update({MakeDetection(15.0, 0.0)}, kDt, kSensor);
  EXPECT_EQ(tracker.tracks().size(), 1U);
  EXPECT_TRUE(tracker.ConfirmedTracks().empty()) << "一帧就确认了 —— confirm_hits 没生效";

  tracker.Update({MakeDetection(15.4, 0.0)}, kDt, kSensor);
  EXPECT_TRUE(tracker.ConfirmedTracks().empty());
  tracker.Update({MakeDetection(15.8, 0.0)}, kDt, kSensor);
  EXPECT_EQ(tracker.ConfirmedTracks().size(), 1U) << "累计 3 次命中后应当确认";
}

// ---------------------------------------------------------------------------
//  边界与防御
// ---------------------------------------------------------------------------
TEST(Tracker, RejectsBadInput)
{
  Tracker tracker;
  EXPECT_THROW(tracker.Update({}, 0.0, kSensor), std::invalid_argument) << "dt = 0";
  EXPECT_THROW(tracker.Update({}, -0.1, kSensor), std::invalid_argument) << "dt < 0";
  EXPECT_THROW(
    tracker.Update({MakeDetection(std::nan(""), 0.0)}, kDt, kSensor), std::invalid_argument);
  EXPECT_THROW(
    tracker.Update({MakeDetection(std::numeric_limits<double>::infinity(), 0.0)}, kDt, kSensor),
    std::invalid_argument);

  TrackerParams bad;
  bad.confirm_hits = 0;
  EXPECT_THROW(Tracker{bad}, std::invalid_argument);
}

TEST(Tracker, HandlesAnEmptyFrame)
{
  // 空帧是**常态**（S1 实测目标闪烁），不是异常。
  Tracker tracker;
  EXPECT_NO_THROW(tracker.Update({}, kDt, kSensor));
  EXPECT_TRUE(tracker.tracks().empty());
}

// ---------------------------------------------------------------------------
//  ⚠️ 尺寸记忆 + 中心补全 —— CP-P5-B 实测逼出来的
//
//  这一组守的是 tracker.hpp 文件头那节「为什么直接滤包围盒中心是错的」。
//  实测背景：对向 NPC 车（4.40×1.80），远处只看得见车头那一面，
//  拟出的盒子塌成 1.77 m，中心跟着往自车方向挪 2.2 m —— 而它一跳，
//  卡尔曼就把它读成速度，实测最坏 8.8 m/s（真值 4.0，连符号都反）。
//
//  ## 故障注入实测（2026-08-12）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 补全方向取成"朝向传感器"（`-` 改 `+`） | 红 `CompletesTheBoxAwayFromTheSensor` |
//  | 去掉轴向一致性闸（永远合并尺寸） | 红 `RefusesToMerge…` + `TreatsAxisAndItsOpposite…` |
//  | 关联仍用原始中心（只在 ApplyUpdate 里补全） | 红 `KeepsTheIdWhenTheObservedExtentCollapses` |
//  | 去掉尺寸记忆（长度取当帧值） | 红 `Remembers…` + `KeepsTheId…` |
//
//  ⚠️ **这张表的头两版是错的，而且错得很典型 —— 如实留在这里当教训。**
//     第一版是**预先写上去**的（还没跑就写了"红 2 条"）；真跑之后四条注入
//     里**两条是绿的**。两条的失效原因不同，都不是"判据太松"：
//
//     · 「补全方向取反」绕道看滤波器输出 → 方向反了新息有 3 m，
//       **直接被卡方门限拒掉**，航迹靠惯性原地不动，于是"位置没动"照样成立。
//       判据量到的是"这一帧被拒了"，不是"补全对不对"。
//       改成**直接验 `CompletedCenter`**，并把传感器换到目标另一侧再验一次
//       （把"方向"和"大小"分开量）。
//     · 「关联用原始中心」建模成了尺寸**变大**，而补全只在「观测 < 记忆」
//       时才有缺口可补 —— 变大时 deficit 恒为 0，本来就不该动。
//       实测序列其实是 4.4 →**塌到 1.56**→ 4.405，起作用的是中间那次塌陷。
//       另外 ID 变化要等旧航迹被删（5 帧）+ 新航迹确认（3 帧）才显形，
//       只跑一帧同样什么都测不到。
//
//     教训有两条：**判据要直接量被测的那个量，别绕道**；
//     **注入之前先问"这条用例建模的是不是修复真正起作用的那个工况"**。
// ---------------------------------------------------------------------------
namespace
{
/// 一个可指定尺寸的检测，用来模拟"只看得见一部分"。
Detection MakePartial(double x, double y, double length_m, double width_m, double yaw_rad = 0.0)
{
  Detection detection;
  detection.position = Eigen::Vector2d(x, y);
  detection.yaw_rad = yaw_rad;
  detection.length_m = length_m;
  detection.width_m = width_m;
  detection.height_m = 1.5;
  return detection;
}
}  // namespace

TEST(Tracker, RemembersTheLargestExtentItHasSeen)
{
  Tracker tracker;
  // 先在近处看全（4.4 m），再退远只看得见 1.8 m。
  for (int frame = 0; frame < 4; ++frame) {
    tracker.Update({MakePartial(10.0, 0.0, 4.4, 1.8)}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_NEAR(tracker.tracks()[0].length_m, 4.4, 1e-9);

  tracker.Update({MakePartial(10.0, 0.0, 1.8, 1.8)}, kDt, kSensor);
  printf("[          ] 只看见 1.8 m 之后，航迹记住的长度 = %.3f\n", tracker.tracks()[0].length_m);
  EXPECT_NEAR(tracker.tracks()[0].length_m, 4.4, 1e-9) << "尺寸记忆没生效 —— 退回了当帧值";
}

TEST(Tracker, CompletesTheBoxAwayFromTheSensor)
{
  // ⚠️ **直接验 CompletedCenter，不要绕道看滤波器的输出。**
  //    第一版这条用例是"喂一帧塌陷的检测，看航迹位置动没动"，结果把补全方向
  //    取反注进去**仍然是绿的** —— 因为方向反了之后新息有 3 m，
  //    直接被卡方门限拒了，航迹靠惯性原地不动，于是"位置没动"照样成立。
  //    判据量到的是"这一帧被拒了"，而不是"补全对不对"。
  Tracker tracker;
  Track track;
  track.yaw_rad = 0.0;
  track.length_m = 4.4;
  track.width_m = 1.8;

  // 只看得见车头 1.4 m ⟹ 可见部分的中心比真中心近 (4.4−1.4)/2 = 1.5 m。
  const Detection partial = MakePartial(18.5, 0.0, 1.4, 1.8);

  // 传感器在原点（目标的近侧）：缺口藏在**更远**处，中心应当被推到 20.0。
  const Eigen::Vector2d from_near = tracker.CompletedCenter(partial, track, kSensor);
  printf("[          ] 传感器在近侧 → 补全中心 x = %.3f（应为 20.000）\n", from_near.x());
  EXPECT_NEAR(from_near.x(), 20.0, 1e-9) << "补全方向反了";
  EXPECT_NEAR(from_near.y(), 0.0, 1e-9);

  // 传感器搬到目标另一侧：同一个检测，补全方向**必须跟着翻**到 17.0。
  // 这一半是关键 —— 它把"方向"与"大小"分开验，写死一个方向的实现过不了。
  const Eigen::Vector2d from_far = tracker.CompletedCenter(partial, track, {40.0, 0.0});
  printf("[          ] 传感器在远侧 → 补全中心 x = %.3f（应为 17.000）\n", from_far.x());
  EXPECT_NEAR(from_far.x(), 17.0, 1e-9) << "补全方向没跟着传感器换边";

  // 没有缺口时原样返回 —— 否则每一帧都会被无端挪动。
  const Detection full = MakePartial(20.0, 0.0, 4.4, 1.8);
  EXPECT_NEAR(tracker.CompletedCenter(full, track, kSensor).x(), 20.0, 1e-12);
}

// ⚠️⚠️ 下面两条的场景**必须按「近边固定」构造**，不能图省事把中心钉住。
//    第一版把中心固定在 20 m 再改尺寸 —— 那等于宣称物体朝传感器方向也长大了，
//    观测本身不自洽。后果是重锚算出的位置与观测差 2.15 m、**被卡方门限拒掉**，
//    于是"航迹尺寸没变"这个断言**因为这一帧根本没被采纳而通过**。
//    这是本文件第三次踩同一个陷阱（前两次见文件头）。判据要量的是"合并对不对"，
//    不是"这一帧被拒了没有"。
//
//    真实几何：目标近边不动在 N，
//      只看得见正面（进深 d）→ 中心在 N + d/2，L-Shape 报 (1.8 × d) @ 90°
//      侧面露出来（长 4.4）  → 中心在 N + 2.2，L-Shape 报 (4.4 × 1.8) @ 0°
namespace
{
constexpr double kNearEdgeX = 19.95;  // 目标近边，固定
constexpr double kFaceDepth = 0.1;    // 只看得见正面时的进深
}  // namespace

TEST(Tracker, ResolvesTheAxisFlipInsteadOfGivingUp)
{
  // ⚠️ **这条用例换过一次判据，原因值得记。**
  //    第一版叫 RefusesToMergeExtentsWhenTheAxisFlipped，断言"轴向翻转时
  //    丢掉尺寸记忆"。那是**权宜**：当时怕沿错误的轴外扩 1.3 m，就一刀切地
  //    放弃。代价是目标由远及近、侧面刚露出来的那一帧既不补全也不重锚，
  //    新息含整跳 ⟹ 判不进门 ⟹ **ID 跳变**（实测 17–22 m 反复切）。
  //
  //    正确的做法不是放弃，是**换个说法**：`a×b @ ψ` 与 `b×a @ ψ+90°`
  //    是同一个盒子。AlignedDetection 做这件事，于是记忆照样能合并，
  //    而且合并在**对的轴**上。
  Tracker tracker;
  for (int frame = 0; frame < 4; ++frame) {
    tracker.Update({MakePartial(kNearEdgeX + 2.2, 0.0, 4.4, 1.8, /*yaw=*/0.0)}, kDt, kSensor);
  }
  ASSERT_NEAR(tracker.tracks()[0].length_m, 4.4, 1e-9);
  const int hits_before = tracker.tracks()[0].hits;

  // 目标退远，只剩正面：可见范围 1.8（横向）× 0.1（进深），
  // 而 L-Shape 会把较大的 1.8 叫 length ⟹ 轴向报成 90°。
  tracker.Update(
    {MakePartial(kNearEdgeX + 0.5 * kFaceDepth, 0.0, 1.8, kFaceDepth, /*yaw=*/M_PI / 2.0)}, kDt,
    kSensor);
  const Track & track = tracker.tracks()[0];
  printf(
    "[          ] 轴向报成 90° 之后：尺寸 %.3f × %.3f，航迹 %zu 条，命中 %d → %d\n", track.length_m,
    track.width_m, tracker.tracks().size(), hits_before, track.hits);
  // ⚠️ 先确认这一帧**真的被采纳了** —— 否则下面的断言毫无意义。
  ASSERT_EQ(tracker.tracks().size(), 1U) << "这一帧被拒了，另起了一条航迹";
  ASSERT_EQ(track.hits, hits_before + 1) << "这一帧没被关联上，断言测不到合并";
  EXPECT_NEAR(track.length_m, 4.4, 1e-9) << "把轴向翻转当成了目标变小";
  EXPECT_NEAR(track.width_m, 1.8, 1e-9);
}

TEST(Tracker, NormalizesStoredExtentsSoLengthIsAlwaysTheLongAxis)
{
  // ⚠️ 归一化不是洁癖，它有一个具体的下游：ClassifyBySize 拿 `length` 去比
  //    vehicle_min_length = 2.5。远处只看得见车头时记忆是 1.8 × 0.1，
  //    等侧面露出来"宽"长到 4.4 —— 不交换的话 length 仍是 1.8，
  //    一辆量得完全正确的车会被判成 UNKNOWN。
  //    ResolveHeading 同样假定 yaw 是长轴。
  Tracker tracker;
  for (int frame = 0; frame < 4; ++frame) {
    tracker.Update(
      {MakePartial(kNearEdgeX + 0.5 * kFaceDepth, 0.0, 1.8, kFaceDepth, /*yaw=*/M_PI / 2.0)}, kDt,
      kSensor);
  }
  ASSERT_NEAR(tracker.tracks()[0].length_m, 1.8, 1e-9);
  const int hits_before = tracker.tracks()[0].hits;

  // 侧面露出来：近边没动，中心因此从 N+0.05 挪到 N+2.2 —— 那是**重新锚定**。
  tracker.Update({MakePartial(kNearEdgeX + 2.2, 0.0, 4.4, 1.8, /*yaw=*/0.0)}, kDt, kSensor);
  const Track & track = tracker.tracks()[0];
  printf(
    "[          ] 侧面露出后：尺寸 %.3f × %.3f，轴向 %.3f rad，位置 x=%.3f（应 %.2f）\n",
    track.length_m, track.width_m, track.yaw_rad, track.position().x(), kNearEdgeX + 2.2);
  ASSERT_EQ(tracker.tracks().size(), 1U) << "这一帧被拒了，另起了一条航迹";
  ASSERT_EQ(track.hits, hits_before + 1) << "这一帧没被关联上";
  EXPECT_GE(track.length_m, track.width_m) << "length 不再是长轴 —— 分类会把车判成 UNKNOWN";
  EXPECT_NEAR(track.length_m, 4.4, 1e-9);
  EXPECT_NEAR(track.width_m, 1.8, 1e-9);
  EXPECT_TRUE(tracker.AxesConsistent(track.yaw_rad, 0.0)) << "轴向没跟着长宽一起换";
  // 位置必须落到**整车中心**上，而不是可见正面的中心。
  EXPECT_NEAR(track.position().x(), kNearEdgeX + 2.2, 0.3) << "重新锚定没生效";
}

TEST(Tracker, StillDropsTheMemoryWhenTheTargetGenuinelyTurns)
{
  // 换算只化解 90° 的**命名**翻转；目标**真的**转了 30° 时两条轴对不上，
  // 记忆必须丢掉重来 —— 否则会把转弯前的尺寸按转弯后的轴外扩。
  Tracker tracker;
  for (int frame = 0; frame < 4; ++frame) {
    tracker.Update({MakePartial(20.0, 0.0, 4.4, 1.8, /*yaw=*/0.0)}, kDt, kSensor);
  }
  tracker.Update({MakePartial(20.0, 0.0, 2.0, 1.8, /*yaw=*/0.52)}, kDt, kSensor);  // 30°
  printf(
    "[          ] 目标真的转了 30° 之后，航迹尺寸 = %.3f × %.3f（应当是当帧的 2.0 × 1.8）\n",
    tracker.tracks()[0].length_m, tracker.tracks()[0].width_m);
  EXPECT_NEAR(tracker.tracks()[0].length_m, 2.0, 1e-9) << "目标转弯时仍然沿旧轴合并了尺寸";
}

TEST(Tracker, TreatsAxisAndItsOppositeAsTheSameAxis)
{
  // 轴向的等价类是 π：0.01 与 3.13 是同一条轴，不是差 3.12。
  Tracker tracker;
  EXPECT_TRUE(tracker.AxesConsistent(0.01, M_PI - 0.01)) << "没把 a 与 a+π 当同一条轴";
  EXPECT_TRUE(tracker.AxesConsistent(0.1, 0.2));
  EXPECT_FALSE(tracker.AxesConsistent(0.0, M_PI / 2.0)) << "90° 必须判成不同的轴";
}

TEST(Tracker, KeepsTheIdWhenTheObservedExtentCollapses)
{
  // ⚠️ 实测场景（CP-P5-B，车贴身而过）：感知长度在 4.41 与 1.56 之间跳，
  //    可见部分的中心跟着朝自车挪 1.425 m。若关联仍用**原始**中心，
  //    这一跳超出卡方门限 ⟹ 关联失败 ⟹ 旧航迹连续未命中被删、
  //    新航迹确认 ⟹ **ID 跳变**。实测那里连着跳了两次。
  //
  // ⚠️ **必须建模「塌陷」而不是「增长」。** 第一版写的是尺寸变大，
  //    而补全只在「观测 < 记忆」时才有缺口可补 —— 变大时 deficit 恒为 0，
  //    补全本来就不该动，于是注入什么都不红。**判据建模错了方向。**
  //
  // ⚠️ 也**必须跑满 max_misses + confirm_hits 帧**：ID 变化要等旧航迹被删
  //    （5 帧）且新航迹确认（3 次命中）才显形。只跑一帧同样什么都测不到。
  Tracker tracker;
  double x = 12.0;
  for (int frame = 0; frame < 4; ++frame, x -= 0.4) {
    tracker.Update({MakePartial(x, 0.0, 4.41, 1.8)}, kDt, kSensor);
  }
  ASSERT_FALSE(tracker.ConfirmedTracks().empty());
  const std::uint32_t id = tracker.ConfirmedTracks()[0].id;

  // 此后只看得见车头 1.56 m：可见中心朝自车（原点方向）挪 (4.41−1.56)/2。
  for (int frame = 0; frame < 8; ++frame, x -= 0.4) {
    tracker.Update({MakePartial(x - 1.425, 0.0, 1.56, 1.8)}, kDt, kSensor);
  }
  ASSERT_FALSE(tracker.ConfirmedTracks().empty()) << "航迹全被删了";
  printf(
    "[          ] 尺寸 4.41 → 1.56 之后跑 8 帧，ID %u → %u（航迹 %zu 条）\n", id,
    tracker.ConfirmedTracks()[0].id, tracker.tracks().size());
  EXPECT_EQ(tracker.ConfirmedTracks()[0].id, id) << "尺寸一塌 ID 就变了 —— 关联没用补全后的位置";
  EXPECT_EQ(tracker.tracks().size(), 1U) << "多出了一条重复航迹";
}

TEST(Tracker, DoesNotReadRevealedExtentAsVelocity)
{
  // ⚠️ 与 CompletedCenter 对称的另一半：目标**由远及近**时侧面逐渐露出来，
  //    盒子变长而近边没动 —— 中心因此往远处挪，这不是运动，是**重新锚定**。
  //    不补的话卡尔曼把它读成速度。实测一帧长 0.87 m ⟹ 4.4 m/s（真值 4.0）。
  //
  // ⚠️ **增长必须是渐进的。** 一次涨 1.7 m 的话新息会超出卡方门限、
  //    这一帧被直接拒掉，航迹靠惯性保持 v≈0 —— 于是不修也"通过"。
  //    那测到的是"门限拒了它"，不是"锚定对了"。这个陷阱本文件已经踩过两次。
  //    每帧 +0.3 m ⟹ 中心挪 0.15 m ⟹ 马氏距离 0.125，稳稳进门。
  Tracker tracker;
  const double near_edge_x = 10.0;  // 目标**静止**，近边固定在 10 m
  double length = 1.0;

  for (int frame = 0; frame < 4; ++frame) {
    tracker.Update({MakePartial(near_edge_x + 0.5 * length, 0.0, length, 1.8)}, kDt, kSensor);
  }
  ASSERT_FALSE(tracker.ConfirmedTracks().empty());

  for (int frame = 0; frame < 11; ++frame) {
    length += 0.3;  // 侧面一点点露出来，一直到 4.3 m
    tracker.Update({MakePartial(near_edge_x + 0.5 * length, 0.0, length, 1.8)}, kDt, kSensor);
  }
  ASSERT_FALSE(tracker.ConfirmedTracks().empty()) << "航迹被删了";
  const Track & track = tracker.ConfirmedTracks()[0];
  printf(
    "[          ] 盒子 1.0 → %.1f m（近边不动、目标静止），估计速度 %.3f m/s\n", length,
    track.velocity().norm());
  EXPECT_LT(track.velocity().norm(), 0.3)
    << "把「露出来的部分」读成了速度 —— 观测变大时没有重新锚定航迹";
  // 位置应当落在**整个目标**的中心上，而不是可见部分的中心。
  EXPECT_NEAR(track.position().x(), near_edge_x + 0.5 * length, 0.35);
}

// ---------------------------------------------------------------------------
//  ⚠️ 重复航迹合并 —— CP-P5-B 实测出来的独立缺陷
//
//  症状是「ID 在两个值之间来回跳」（338↔369、390↔391），而它**不是**
//  "航迹死了重建"，是两条航迹**并存**、评测逐帧在它们之间摇摆。
//  实测：31/678 帧次的真值目标 1.0 m 内有 2 个以上感知目标，
//  第二近者中位 **0.07 m**、最大 0.44 m。
//
//  重复本身就是缺陷：规划会把一个目标当成两个障碍物去做碰撞检查。
// ---------------------------------------------------------------------------
TEST(Tracker, MergesADuplicateTrackOntoTheOlderOne)
{
  // ⚠️ 目标必须是**运动**的。第一版用静止目标，于是两条航迹速度都是 0，
  //    "速度门限只对已确认的航迹生效"这条注进去也不红 —— 门限根本没被走到。
  //    真实的重复恰恰是"新航迹初速 0 vs 老航迹 −4 m/s"，静止目标测不到它。
  Tracker tracker;
  double x = 20.0;
  for (int frame = 0; frame < 5; ++frame, x += 4.0 * kDt) {
    tracker.Update({MakeDetection(x, 0.0)}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.ConfirmedTracks().size(), 1U);
  const std::uint32_t original_id = tracker.ConfirmedTracks()[0].id;
  ASSERT_GT(tracker.ConfirmedTracks()[0].velocity().norm(), 2.0) << "老航迹得先有速度";

  // 同一帧多出一个只差 0.2 m 的检测 —— 一对一关联下它配不上，于是新建一条
  // **初速为 0** 的航迹。合并要能把它收掉，而且留下更老的那条。
  x += 4.0 * kDt;
  tracker.Update({MakeDetection(x, 0.0), MakeDetection(x + 0.2, 0.0)}, kDt, kSensor);
  printf(
    "[          ] 多一个 0.2 m 外的检测之后：航迹 %zu 条，ID %u → %u\n", tracker.tracks().size(),
    original_id, tracker.tracks().empty() ? 0 : tracker.tracks()[0].id);
  EXPECT_EQ(tracker.tracks().size(), 1U) << "重复航迹没被合并 —— 规划会看到两个障碍物";
  EXPECT_EQ(tracker.tracks()[0].id, original_id) << "合并时留错了那条（应当留更老的）";
}

TEST(Tracker, KeepsTheOlderIdWhenTwoDuplicatesHaveTheSameHitCount)
{
  // ⚠️ 专门走**打平**那条分支。上一条用例里两条航迹命中数是 6 vs 1，
  //    打平的分支从没被执行过 —— 把"打平留更新的"注进去照样绿。
  //    同一帧生出的两条航迹命中数都是 1，这时才轮到 id 决定去留。
  Tracker tracker;
  tracker.Update({MakeDetection(20.0, 0.0), MakeDetection(20.2, 0.0)}, kDt, kSensor);
  ASSERT_EQ(tracker.tracks().size(), 1U) << "同一帧生出的两条重复航迹没被合并";
  printf("[          ] 同一帧两条重复航迹（命中数都是 1）→ 留下 ID %u\n", tracker.tracks()[0].id);
  EXPECT_EQ(tracker.tracks()[0].id, 1U) << "打平时没留更老（id 更小）的那条";
}

TEST(Tracker, MergesAYoungDuplicateBeforeItsVelocityConverges)
{
  // P9-S5c 实测：车身碎片在正主 0.15–0.5 m 外起一条新航迹，凑够 3 帧确认那一刻
  // 它的 KF 速度还差正主 2 m/s —— 按「都已确认就比速度」永远合不掉，两条并存
  // 1–2 帧，评测在两者间摆 ⟹ 每次记 2 次 ID 切换。年轻的重复要只按距离并。
  Tracker tracker;
  double x = 30.0;
  for (int frame = 0; frame < 15; ++frame, x -= 0.4) {
    tracker.Update({MakeDetection(x, 0.0)}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.ConfirmedTracks().size(), 1U);
  const std::uint32_t main_id = tracker.ConfirmedTracks()[0].id;
  // 复刻实测形态：碎片航迹出生在正主中心 1.5 m 外（合并半径之外 —— 实测里是
  // 「年轻航迹锚在车头面、正主锚在整车中心」差出来的 2 m），第 3 帧（确认那帧）
  // 检测跳到正主 0.3 m 内（实测里是它自己被先验锚定、中心一步推回整车中心）。
  // 跳变让它的 KF 速度瞬间偏离正主好几 m/s —— 按「已确认就比速度」永远不并。
  std::size_t max_confirmed = 0;
  for (int frame = 0; frame < 5; ++frame, x -= 0.4) {
    const double fragment_y = frame < 2 ? 1.5 : 0.3;
    tracker.Update({MakeDetection(x, 0.0), MakePartial(x, fragment_y, 0.6, 0.4)}, kDt, kSensor);
    max_confirmed = std::max(max_confirmed, tracker.ConfirmedTracks().size());
  }
  const std::vector<Track> confirmed = tracker.ConfirmedTracks();
  printf(
    "[          ] 碎片伴行 5 帧内确认航迹峰值 %zu 条，最后 %zu 条，正主 ID %s\n", max_confirmed,
    confirmed.size(), FindById(confirmed, main_id) ? "保持" : "丢失");
  EXPECT_EQ(max_confirmed, 1U) << "年轻的重复航迹确认后与正主并存了 —— 速度门限把它挡在合并之外";
  ASSERT_EQ(confirmed.size(), 1U);
  EXPECT_NE(FindById(confirmed, main_id), nullptr) << "合并留下了年轻的那条 —— 该留命中多的";
}

TEST(Tracker, DoesNotMergeTwoTargetsThatMerelyPassCloseBy)
{
  // ⚠️ 合并的反面风险比重复更严重：把车和行人并成一个目标 = **漏掉一个人**。
  //    位置近但**速度截然不同**就是两个目标，不是重复。
  //    实测本场景两个目标速度差 5.2 m/s（车 −4.0、行人 +1.2）。
  //
  // ⚠️ **必须断言 ID 还在，不能只数条数。** 第一版只断言"最后还有 2 条"，
  //    而交会时并掉一条之后、分开后又会重建一条 —— 数量照样回到 2，
  //    于是"去掉速度门限"注进去也不红。数量守恒 ≠ 身份守恒。
  //
  //    横向只差 0.9 m（< merge_distance 1.0），所以交会时**一定**进入合并的
  //    距离判据 —— 唯一拦住它的就是速度门限。
  Tracker tracker;
  double xa = 10.0;
  double xb = 24.0;
  std::uint32_t id_a = 0;
  std::uint32_t id_b = 0;
  double closest = 1e9;
  for (int frame = 0; frame < 40; ++frame) {
    xa += 3.0 * kDt;
    xb -= 3.0 * kDt;
    tracker.Update({MakeDetection(xa, 0.45), MakeDetection(xb, -0.45)}, kDt, kSensor);
    closest = std::min(closest, std::hypot(xa - xb, 0.9));
    if (frame == 6) {
      const auto tracks = tracker.ConfirmedTracks();
      ASSERT_EQ(tracks.size(), 2U);
      for (const Track & track : tracks) {
        (track.velocity().x() > 0.0 ? id_a : id_b) = track.id;
      }
    }
  }
  const auto tracks = tracker.ConfirmedTracks();
  printf(
    "[          ] 两个目标最近曾靠到 %.2f m（合并阈值 1.0），最后剩 %zu 条航迹，"
    "原 ID %u/%u 是否都还在：%d/%d\n",
    closest, tracks.size(), id_a, id_b, FindById(tracks, id_a) != nullptr,
    FindById(tracks, id_b) != nullptr);
  ASSERT_LT(closest, 1.0) << "两者从没靠到合并阈值以内 —— 这条用例什么都没测";
  EXPECT_EQ(tracks.size(), 2U) << "把两个擦身而过的目标并成了一个 —— 会漏掉一个目标";
  EXPECT_NE(FindById(tracks, id_a), nullptr) << "A 的 ID 在交会时被并掉了";
  EXPECT_NE(FindById(tracks, id_b), nullptr) << "B 的 ID 在交会时被并掉了";
}

// ---------------------------------------------------------------------------
//  ⚠️ 遮挡感知的航迹保持 —— CP-P5-B 第 6 条与生命周期冲突的解（2026-08-12）
//
//  实测：遮挡持续 0.5–1.8 s（车 4.4 m 扫过视线），而 max_misses = 0.5 s ——
//  不做这条规则的话，任何真实遮挡都会删航迹换 ID，判据第 6 条物理上无法过。
//  解法是 MOT 的标准做法：预测位置被另一条已确认航迹挡住视线时，
//  未命中不计入删除计数（有上限）。
//
//  ## 故障注入实测（2026-08-12，跑完立刻回填）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 去掉遮挡分支（未命中一律计数） | 红 SurvivesAnOcclusionLongerThanMaxMisses |
//  | 去掉滑行上限 | 红 OcclusionCoastingHasACap |
// ---------------------------------------------------------------------------
TEST(Tracker, SurvivesAnOcclusionLongerThanMaxMisses)
{
  // 场景（照实测搬）：车停在传感器与行人航迹之间，行人被挡 15 帧（1.5 s，
  // 是 max_misses 窗口的 3 倍），随后重现 —— ID 必须保持。
  Tracker tracker;
  // 车：在 (10, 0)，轴向沿 y（横在视线上），4.4 × 1.8。
  Detection car;
  car.position = {10.0, 0.0};
  car.yaw_rad = M_PI / 2.0;
  car.length_m = 4.4;
  car.width_m = 1.8;
  car.height_m = 1.5;
  // 行人：在车后面 (20, 0)，以 1.2 m/s 沿 +y 走。
  auto ped = [](double y) { return MakePartial(20.0, y, 0.4, 0.4); };

  // ⚠️ 遮挡前先跟 12 帧（≥ mature_hits 10）：滑行只给速度已收敛的成熟航迹
  //    （P9-S5c，见 TrackerParams::mature_hits）；真实场景里行人被车挡住之前
  //    已被跟踪好几秒。
  double y = 0.0;
  for (int frame = 0; frame < 12; ++frame, y += 0.12) {
    tracker.Update({car, ped(y)}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.ConfirmedTracks().size(), 2U);
  std::uint32_t ped_id = 0;
  for (const Track & track : tracker.ConfirmedTracks()) {
    if (track.width_m < 1.0) {
      ped_id = track.id;
    }
  }
  ASSERT_NE(ped_id, 0U);

  // 遮挡 15 帧：只有车的检测，行人消失（被挡住）。
  for (int frame = 0; frame < 15; ++frame, y += 0.12) {
    tracker.Update({car}, kDt, kSensor);
  }
  ASSERT_NE(FindById(tracker.ConfirmedTracks(), ped_id), nullptr)
    << "遮挡 1.5 s 后行人航迹被删了 —— 遮挡滑行没生效";

  // 重现：行人从车后走出来（滑行预测的位置附近）。
  // ⚠️ ConfirmedTracks() **按值返回**：先落成本地副本再取指针（2026-08-16 修：此前
  //    指针指着已析构的临时 vector，printf 出对的数只是运气）。
  const std::vector<Track> before_reappear = tracker.ConfirmedTracks();
  const Track * coasted = FindById(before_reappear, ped_id);
  ASSERT_NE(coasted, nullptr);
  const double coasted_y = coasted->position().y();
  tracker.Update({car, ped(y)}, kDt, kSensor);
  const std::vector<Track> after_reappear = tracker.ConfirmedTracks();
  const Track * after = FindById(after_reappear, ped_id);
  printf(
    "[          ] 遮挡 15 帧后滑行位置 y=%.2f（真值 %.2f），重现后 ID %s\n", coasted_y, y,
    after ? "保持" : "丢失");
  ASSERT_NE(after, nullptr) << "重现后配不上滑行航迹 —— ID 换了";
  EXPECT_EQ(tracker.ConfirmedTracks().size(), 2U);
}

TEST(Tracker, OcclusionCoastingHasACap)
{
  // 滑行必须有上限：目标真的在遮挡后面离开时，幽灵不能永生 ——
  // 与 max_misses 防幽灵同一个理由。
  TrackerParams params;
  params.max_occluded_misses = 8;  // 收紧到 0.8 s，让用例快
  Tracker tracker(params);
  Detection car;
  car.position = {10.0, 0.0};
  car.yaw_rad = M_PI / 2.0;
  car.length_m = 4.4;
  car.width_m = 1.8;
  car.height_m = 1.5;
  for (int frame = 0; frame < 12; ++frame) {  // ≥ mature_hits，否则根本不滑行、测不到上限
    tracker.Update({car, MakePartial(20.0, 0.0, 0.4, 0.4)}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.ConfirmedTracks().size(), 2U);

  // 遮挡远超上限（8 + max_misses 5 + 余量）。
  for (int frame = 0; frame < 20; ++frame) {
    tracker.Update({car}, kDt, kSensor);
  }
  printf(
    "[          ] 遮挡 20 帧（上限 8）后剩 %zu 条确认航迹\n", tracker.ConfirmedTracks().size());
  EXPECT_EQ(tracker.ConfirmedTracks().size(), 1U) << "滑行没有上限 —— 幽灵永生";
}

// ---------------------------------------------------------------------------
//  ⚠️ 两道物理闸 —— P6-S0 车道内幻影的修复（2026-08-12）
//
//  实测因果链（复现轮 rosbag 逐帧钉死，见 tasks/todo.md 的 P6-S0 记录）：
//  墙沿/杆件碎片的 L-Shape 拟合帧间剧变（0.03 ↔ 6 m）→ 重锚位移 3 m
//  把数米外**另一个**碎片"拉进"卡方门限 → 两个静止碎片被焊成一条带
//  9–12 m/s 假速度的航迹 → 遮挡滑行让它免死最多 3 s、按假速度飞越
//  35 m 横穿地图 → 滑进自车车道 30 m 带内，成为 6.00×N 的幻影虚警。
//  两道闸各断一环：位移上限断"焊接"，速度准入断"飞行"。
//
//  ## 故障注入实测（2026-08-12，跑完立刻回填）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 位移闸条件改恒假（去掉闸） | **红 1 条**：`RejectsTheAnchorWormhole…`，
//  |   | 「4.5 m 外的碎片被焊进原航迹」断言失败（航迹数 1 ≠ 2），其余 26 条全绿 |
//  | 去掉滑行的速度准入 | **红 1 条**：`DoesNotCoastAnImplausiblyFastTrack`
//  |   | （幽灵靠滑行存活）；对照用例 `SurvivesAnOcclusion…` 保持绿 ——
//  |   | 证明红的是速度准入，不是遮挡判定 |
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//  U 转鬼影：遮挡滑行 × 重锚跳变（P9-S3 Gazebo 实测，P9-S5c 修）
//
//  Gazebo `both` 场景 NPC 车在航点原地掉头：L-Shape 轴向翻转 + 车辆先验锚定让
//  框中心一步跳 ~2 m ⟹ 旧航迹配不上（卡方门 9.21，2 m 跳变 d² ≈ 40）⟹ 新航迹；
//  旧航迹按恒速继续外推，预测位置恰在新航迹盒子**后面**⟹ §6.5 判"被已确认
//  航迹遮挡"⟹ 滑行最多 3 s —— 一个 4.4×1.8 的鬼影贴车道带边走，记成车道内虚警。
//
//  区分"重锚"与"真遮挡"的签名：遮挡者是**刚出生**（≤ reanchor_max_age_frames）
//  且出生时离**正在丢失的**旧航迹预测位置 ≤ anchor_shift_max（同一目标露出
//  另一面能造成的最大中心位移）—— 那是同一个目标换了个框，不是另一个目标挡住了它。
//  判定生效后旧航迹按普通未命中倒计时（0.5 s 内消失），不给 3 s。
//
//  ## 故障注入实测（2026-08-16）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 去掉重锚判定（superseded 恒 false） | 红：掉头 0.8 s 后确认航迹 2 条，旧航迹还在
//    @x≈13.4（真值 ≈8.6）；其余 30 条含三条遮挡用例绿 |
// ---------------------------------------------------------------------------
TEST(Tracker, DoesNotCoastAGhostBehindItsOwnReanchoredTrack)
{
  // 车沿 +x 远离传感器，2 m/s，10 帧确认并收敛速度。
  Tracker tracker;
  auto car = [](double x) { return MakeDetection(x, 0.0); };
  double x = 10.0;
  for (int frame = 0; frame < 10; ++frame, x += 0.2) {
    tracker.Update({car(x)}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.ConfirmedTracks().size(), 1U);
  const std::uint32_t old_id = tracker.ConfirmedTracks()[0].id;

  // 掉头：框中心一步跳回 2.0 m（轴向翻转 + 先验锚定的实测量级），随后以 2 m/s
  // 朝传感器回来。旧航迹的预测位置（继续 +x）落在新盒子后面 —— 鬼影配方。
  x -= 2.0;
  for (int frame = 0; frame < 8; ++frame, x -= 0.2) {
    tracker.Update({car(x)}, kDt, kSensor);
  }
  // 跳变后 0.8 s：新航迹早已确认；旧航迹若被当成"被遮挡"会滑行 3 s。
  // 没有滑行的话它在 max_misses（0.5 s）内就该消失。
  // ⚠️ ConfirmedTracks() **按值返回**：指针/引用要绑在本地副本上，绑临时量就是悬空。
  const std::vector<Track> confirmed = tracker.ConfirmedTracks();
  const Track * ghost = FindById(confirmed, old_id);
  printf(
    "[          ] 掉头 0.8 s 后确认航迹 %zu 条，旧航迹 %s%s\n", confirmed.size(),
    ghost ? "还在（鬼影）@x=" : "已删除",
    ghost ? std::to_string(ghost->position().x()).c_str() : "");
  EXPECT_EQ(ghost, nullptr) << "旧航迹借新航迹的盒子当遮挡在滑行 —— 那是它自己的重锚，不是遮挡";
  ASSERT_EQ(confirmed.size(), 1U) << "掉头后应只剩一条航迹（新的那条）";
  EXPECT_NEAR(confirmed[0].position().x(), x + 0.2, 0.6);
}

// ---------------------------------------------------------------------------
//  由远及近、侧面刚露出来那几帧不许换 ID（P9-S5c，bag 回放钉死的缺陷）
//
//  实测序列（Gazebo both 场景，2026-08-16 bag，自车静止 (30,−51.75)，NPC 车沿
//  邻车道 y=−48.25 以 4 m/s 驶来）：11.0–13.3 s 只见车头（1.8 × 0.05–0.85，
//  轴向 90°），13.4 s 起侧面露出（深度 1.77 → 2.05 → 2.97 → 3.4 → 4.15，
//  轴向翻到 0°/180°）。旧版先验只在深度 < 1.0 时沿视线推 (4.4 − 深度)/2 ——
//  13.4 s 那帧推量从 1.7 m 跳到 0，加上按记忆宽度增量的横向重锚 0.46 m，
//  新息 1.3 m 判不进门 ⟹ 8 → 30 → 31 换两次 ID。**每轮都在 18–19 m 处复现**。
//  修法：先验沿**车头方向**把观测框补到 4.4 × 1.8 的先验盒（推量随侧面露出
//  连续收缩），已锚定的航迹不再按记忆重锚。
//
//  ## 故障注入实测（2026-08-16）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 已锚定的航迹也按「观测深度 ≥ 1.0 不推」（复原旧版的跳变） | 红：ID 集合 3
//    （8→30→31 同实测）；基线 1 个 ID、中心误差最大 0.184 m |
// ---------------------------------------------------------------------------
TEST(Tracker, KeepsTheIdWhenTheSideOfAnApproachingCarComesIntoView)
{
  struct Row
  {
    double t, x, y, yaw, l, w, h;
  };
  // 原样照抄 bag（perception/detections 里那辆车的框，map 系）。
  static const Row kRows[] = {
    {11.0, 57.96, -48.24, 1.571, 1.70, 0.05, 0.54}, {11.1, 57.56, -48.25, 1.571, 1.77, 0.04, 0.54},
    {11.2, 57.29, -48.24, 1.588, 1.81, 0.30, 0.53}, {11.3, 56.76, -48.26, 1.571, 1.72, 0.04, 0.52},
    {11.4, 56.40, -48.27, 1.588, 1.77, 0.12, 0.51}, {11.5, 55.96, -48.24, 1.571, 1.76, 0.05, 0.50},
    {11.6, 55.57, -48.25, 1.570, 1.82, 0.05, 0.50}, {11.7, 55.35, -48.29, 1.570, 1.75, 0.41, 0.98},
    {11.8, 54.76, -48.28, 1.571, 1.76, 0.05, 0.95}, {11.9, 54.52, -48.28, 1.570, 1.77, 0.36, 0.94},
    {12.0, 53.96, -48.28, 1.570, 1.79, 0.06, 0.92}, {12.1, 53.73, -48.28, 1.571, 1.79, 0.38, 0.91},
    {12.2, 53.17, -48.29, 1.571, 1.80, 0.05, 0.89}, {12.3, 52.96, -48.29, 1.571, 1.80, 0.43, 0.89},
    {12.4, 52.41, -48.22, 1.571, 1.80, 0.14, 0.86}, {12.5, 52.21, -48.22, 1.571, 1.80, 0.54, 0.86},
    {12.6, 51.69, -48.23, 1.571, 1.78, 0.30, 0.83}, {12.7, 51.17, -48.24, 1.606, 1.78, 0.08, 0.82},
    {12.8, 50.97, -48.25, 1.588, 1.77, 0.47, 0.81}, {12.9, 50.50, -48.27, 1.571, 1.75, 0.31, 1.17},
    {13.0, 50.03, -48.25, 1.571, 1.80, 0.18, 1.15}, {13.1, 49.56, -48.27, 1.571, 1.78, 0.05, 1.13},
    {13.2, 49.37, -48.28, 1.570, 1.75, 0.47, 1.11}, {13.3, 49.15, -48.26, 1.588, 1.80, 0.85, 1.13},
    {13.4, 49.22, -48.28, 1.588, 1.77, 1.77, 1.20}, {13.5, 48.78, -48.29, 1.553, 1.78, 1.68, 1.16},
    {13.6, 48.57, -48.30, 0.000, 2.05, 1.75, 1.18}, {13.7, 48.63, -48.30, 3.141, 2.97, 1.76, 1.31},
    {13.8, 48.43, -48.27, 0.000, 3.37, 1.82, 1.31}, {13.9, 48.22, -48.27, 3.141, 3.77, 1.82, 1.31},
    {14.0, 48.01, -48.26, 0.017, 4.15, 1.86, 1.31}, {14.1, 47.56, -48.26, 0.017, 4.06, 1.85, 1.31},
    {14.2, 47.12, -48.26, 3.141, 3.96, 1.84, 1.31}, {14.3, 46.93, -48.27, 0.000, 4.38, 1.82, 1.32},
    {14.4, 46.49, -48.26, 0.017, 4.30, 1.90, 1.31}, {14.5, 46.08, -48.28, 3.141, 4.29, 1.80, 1.31},
    {14.6, 45.67, -48.27, 3.141, 4.28, 1.81, 1.31}, {14.7, 45.27, -48.28, 3.141, 4.27, 1.80, 1.32},
    {14.8, 44.88, -48.26, 0.000, 4.28, 1.83, 1.32}, {14.9, 44.50, -48.27, 3.141, 4.33, 1.81, 1.29},
    {15.0, 44.12, -48.26, 0.017, 4.38, 1.83, 1.30},
  };
  const Eigen::Vector2d sensor(30.0, -51.75);
  Tracker tracker;
  std::uint32_t id_at_12s = 0;
  std::set<std::uint32_t> ids_seen;
  double worst_center_error_m = 0.0;
  for (const Row & row : kRows) {
    Detection d;
    d.position = {row.x, row.y};
    d.yaw_rad = row.yaw;
    d.length_m = row.l;
    d.width_m = row.w;
    d.height_m = row.h;
    tracker.Update({d}, kDt, sensor);
    if (row.t < 12.0) {
      continue;  // 头几帧在确认 + 先验锚定
    }
    // ⚠️ ConfirmedTracks() **按值返回**：引用要绑在本地副本上，绑临时量就是悬空。
    const std::vector<Track> confirmed = tracker.ConfirmedTracks();
    ASSERT_EQ(confirmed.size(), 1U) << "t=" << row.t << " 确认航迹不是 1 条";
    const Track & track = confirmed[0];
    if (id_at_12s == 0) {
      id_at_12s = track.id;
    }
    ids_seen.insert(track.id);
    // 真值：13.4 s 时中心 (50.64, −48.25)，沿 −x 4 m/s。
    const Eigen::Vector2d truth(50.64 - 4.0 * (row.t - 13.4), -48.25);
    worst_center_error_m = std::max(worst_center_error_m, (track.position() - truth).norm());
  }
  printf(
    "[          ] 12–15 s 见到的 ID 集合大小 %zu（首个 %u），中心误差最大 %.3f m\n",
    ids_seen.size(), id_at_12s, worst_center_error_m);
  EXPECT_EQ(ids_seen.size(), 1U) << "侧面露出来那几帧换了 ID —— 先验补全在深度过 1 m 时跳变了？";
  EXPECT_LT(worst_center_error_m, 0.6) << "先验补全的中心离真值太远（沿视线推 vs 沿车头推？）";
}

// ---------------------------------------------------------------------------
//  U 转实测序列：掉头前后不许有鬼影（P9-S5c 收口的回归夹具，2026-08-16 复审时加）
//
//  Gazebo both 场景 bag（浮空碎片门已开的那一轮），20.6–23.0 s 车周 14–28 m 内的全部
//  检测框 + 真值中心原样内嵌。道具在 x≈19.5 处原地 ~300°/s 转身，真值中心每帧跳 0.5–0.7 m。
//  修前的病理：旧航迹配不上跳变检测 ⟹ 新航迹三帧确认 ⟹ 旧航迹躲在新盒子后面按 −4 m/s
//  滑行 1.4 s（对向车道里 3–5 m 外的鬼影）。修后：重锚取代 + 成熟门槛，旧航迹 0.5 s 内消失。
//  判据：每一帧每条确认航迹的发布位置离真值中心 ≤ 2.5 m（鬼影是 3–5 m 且越漂越远）；
//  整段出现的确认 ID ≤ 2（残余那一次切换是道具运动学，plan/§6.7 记录）。
//
//  ## 故障注入实测（2026-08-16）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 重锚取代关掉（superseded 恒 false） | 红：最远 8.71 m @22.4（旧航迹躲在新盒子后滑行、
//    越漂越远）；另红 DoesNotCoastAGhostBehindItsOwnReanchoredTrack |
//  基线：确认 ID 2 个，最远 2.24 m @21.5 —— 那是**成熟旧航迹丢第 2 帧时的普通预测**（道具那两帧
//  真值中心跳了 1.2 m），不是鬼影；鬼影从 3.7 m 起且越漂越远。2.5 的门就卡在两者之间。
// ---------------------------------------------------------------------------
TEST(Tracker, UTurnSequenceLeavesNoGhostBehind)
{
  struct Row
  {
    double t, x, y, yaw, l, w, h;
  };
  static const Row kDetections[] = {
    {20.6, 21.78, -48.27, 0.000, 4.43, 1.80, 1.31}, {20.7, 21.47, -48.26, 0.000, 4.25, 1.82, 1.30},
    {20.8, 21.04, -48.26, 0.000, 4.29, 1.82, 1.30}, {20.9, 20.60, -48.27, 0.000, 4.38, 1.81, 1.30},
    {21.0, 20.28, -48.27, 3.141, 4.24, 1.81, 1.29}, {21.1, 19.80, -48.27, 0.000, 4.37, 1.80, 1.30},
    {21.2, 19.46, -48.27, 0.000, 4.27, 1.81, 1.30}, {21.3, 19.51, -48.92, 0.523, 4.40, 1.83, 1.31},
    {21.4, 19.99, -49.42, 1.030, 4.39, 1.81, 1.31}, {21.5, 21.12, -49.67, 1.448, 4.35, 0.56, 1.25},
    {21.6, 20.98, -49.59, 1.780, 4.41, 1.74, 1.31}, {21.7, 21.35, -49.56, 2.042, 4.38, 1.77, 1.31},
    {21.8, 21.67, -49.56, 2.269, 4.40, 1.84, 1.31}, {21.9, 22.06, -49.56, 2.443, 4.33, 1.81, 1.31},
    {22.0, 22.43, -49.56, 2.600, 4.29, 1.81, 1.31}, {22.1, 23.79, -49.97, 2.740, 2.10, 1.79, 1.31},
    {22.1, 22.69, -48.53, 0.000, 0.03, 0.01, 1.12}, {22.1, 21.20, -47.89, 0.000, 0.01, 0.00, 1.09},
    {22.1, 21.99, -48.23, 0.000, 0.02, 0.01, 1.00}, {22.2, 24.90, -50.02, 1.274, 1.81, 0.66, 1.30},
    {22.3, 25.61, -49.85, 1.379, 1.79, 0.06, 1.26}, {22.4, 24.26, -49.44, 3.037, 3.58, 1.81, 1.31},
    {22.4, 21.88, -50.08, 0.000, 0.04, 0.01, 1.16}, {22.5, 24.22, -49.33, 3.106, 4.45, 1.81, 1.31},
    {22.6, 24.64, -49.26, 0.017, 4.36, 1.82, 1.31}, {22.7, 25.04, -49.17, 0.070, 4.35, 1.83, 1.31},
    {22.8, 25.39, -49.11, 0.105, 4.42, 1.84, 1.31}, {22.9, 25.80, -49.01, 0.122, 4.37, 1.84, 1.31},
    {23.0, 26.18, -48.94, 0.140, 4.40, 1.82, 1.31},
  };
  struct Truth
  {
    double t, x, y;
  };
  static const Truth kTruth[] = {
    {20.6, 21.84, -48.25}, {20.7, 21.44, -48.25}, {20.8, 21.04, -48.25}, {20.9, 20.64, -48.25},
    {21.0, 20.24, -48.25}, {21.1, 19.84, -48.25}, {21.2, 19.44, -48.25}, {21.3, 19.41, -48.78},
    {21.4, 19.85, -49.34}, {21.5, 20.38, -49.58}, {21.6, 20.84, -49.59}, {21.7, 21.22, -49.56},
    {21.8, 21.57, -49.54}, {21.9, 21.92, -49.54}, {22.0, 22.27, -49.53}, {22.1, 22.63, -49.51},
    {22.2, 23.00, -49.48}, {22.3, 23.38, -49.44}, {22.4, 23.75, -49.39}, {22.5, 24.14, -49.33},
    {22.6, 24.52, -49.26}, {22.7, 24.91, -49.18}, {22.8, 25.30, -49.11}, {22.9, 25.69, -49.03},
    {23.0, 26.08, -48.95},
  };
  const Eigen::Vector2d sensor(30.0, -51.75);  // 自车静止在起点，与实测一致
  Tracker tracker;
  // 前滚 12 帧直行（把航迹养成熟 —— 实测里它在掉头前已被跟了好几秒）。
  for (int frame = 0; frame < 12; ++frame) {
    Detection d;
    d.position = {26.6 - 0.4 * frame, -48.26};
    d.yaw_rad = 0.0;
    d.length_m = 4.4;
    d.width_m = 1.8;
    d.height_m = 1.3;
    tracker.Update({d}, kDt, sensor);
  }
  std::map<double, std::vector<Detection>> frames;
  for (const Row & row : kDetections) {
    Detection d;
    d.position = {row.x, row.y};
    d.yaw_rad = row.yaw;
    d.length_m = row.l;
    d.width_m = row.w;
    d.height_m = row.h;
    frames[row.t].push_back(d);
  }
  std::set<std::uint32_t> ids;
  double worst_offset_m = 0.0;
  double worst_t = 0.0;
  for (const auto & [t, dets] : frames) {
    tracker.Update(dets, kDt, sensor);
    const Truth * truth = nullptr;
    for (const Truth & candidate : kTruth) {
      if (std::abs(candidate.t - t) < 0.01) {
        truth = &candidate;
      }
    }
    ASSERT_NE(truth, nullptr);
    const std::vector<Track> published = tracker.ConfirmedTracks();  // 按值返回，先落地
    for (const Track & track : published) {
      ids.insert(track.id);
      const double offset_m =
        (track.published_position(10) - Eigen::Vector2d(truth->x, truth->y)).norm();
      if (offset_m > worst_offset_m) {
        worst_offset_m = offset_m;
        worst_t = t;
      }
    }
  }
  printf(
    "[          ] U 转序列：确认 ID %zu 个，发布位置离真值最远 %.2f m @t=%.1f\n", ids.size(),
    worst_offset_m, worst_t);
  EXPECT_LE(ids.size(), 2U) << "U 转前后 ID 数超过 2 —— 航迹在碎裂";
  EXPECT_LT(worst_offset_m, 2.5) << "有确认航迹离真值 ≥2.5 m 还在发 —— 掉头鬼影回来了";
}

TEST(Tracker, ConfirmHitsOfOneDoesNotBreakTheReanchorScan)
{
  // 复审（2026-08-16）发现的潜伏越界：confirm_hits=1 时新航迹出生即确认，同一帧后建的
  // 新航迹扫描重锚候选时若拿 tracks_.size() 做上界，会读 assignment[t] 越界。
  // 用例：一帧里两个互不关联的检测同时出生 —— 修前是 UB，修后各起一条、互不为候选。
  TrackerParams params;
  params.confirm_hits = 1;
  params.mature_hits = 1;
  Tracker tracker(params);
  tracker.Update({MakeDetection(10.0, 0.0), MakeDetection(10.0, 6.0)}, kDt, kSensor);
  ASSERT_EQ(tracker.tracks().size(), 2U);
  for (const Track & track : tracker.tracks()) {
    EXPECT_TRUE(track.confirmed);
    EXPECT_EQ(track.reanchor_of_id, 0U) << "同帧出生的航迹不该互为重锚候选";
  }
  // 再喂几帧，两条都该稳定存在（没有被彼此"取代"）。
  for (int frame = 1; frame < 6; ++frame) {
    tracker.Update(
      {MakeDetection(10.0 + 0.1 * frame, 0.0), MakeDetection(10.0 + 0.1 * frame, 6.0)}, kDt,
      kSensor);
  }
  EXPECT_EQ(tracker.ConfirmedTracks().size(), 2U);
}

TEST(Tracker, ReanchorScanIgnoresTracksErasedThisFrame)
{
  // 复审（2026-08-16）两位审阅者各自用探针坐实的下标错位：删除到期航迹之后 tracks_ 左移，
  // 重锚候选扫描却仍按 erase 前的 assignment[t] 读 —— 一条**本帧命中**的确认航迹会被
  // 记成新检测的重锚候选，随后它闪烁两帧、新航迹确认 ⟹ 它被错误 superseded、从发布里消失。
  // 夹具：噪声航迹 E 先出生（下标 0），随后连丢 5 帧、恰在第 5 帧被删；同一帧正主 C 命中，
  // 旁边 1.5 m 处出生一个碎片检测。
  Tracker tracker;
  tracker.Update({MakeDetection(100.0, 50.0)}, kDt, kSensor);  // E
  for (int frame = 0; frame < 4; ++frame) {
    tracker.Update({MakeDetection(20.0, 0.0)}, kDt, kSensor);  // C 命中，E 丢 1–4
  }
  // 第 5 帧：E 到期删除；C 命中；碎片 F 出生在 C 旁 1.5 m。
  tracker.Update({MakeDetection(20.0, 0.0), MakePartial(20.0, 1.5, 0.6, 0.4)}, kDt, kSensor);
  ASSERT_EQ(tracker.tracks().size(), 2U) << "E 该在这一帧被删";
  std::uint32_t c_id = 0;
  for (const Track & track : tracker.tracks()) {
    if (track.width_m > 1.0) {
      c_id = track.id;
    } else {
      EXPECT_EQ(track.reanchor_of_id, 0U)
        << "碎片把本帧**命中**的 C 记成了重锚候选 —— assignment 下标在 erase 后错位";
    }
  }
  ASSERT_NE(c_id, 0U);
  // C 闪烁两帧、碎片连续命中确认：C 必须一直在发布里（它没丢过 —— 它是正主）。
  for (int frame = 0; frame < 3; ++frame) {
    tracker.Update({MakePartial(20.0, 1.5, 0.6, 0.4)}, kDt, kSensor);
    const std::vector<Track> published = tracker.ConfirmedTracks();
    EXPECT_NE(FindById(published, c_id), nullptr) << "第 " << frame << " 帧 C 从发布里消失了";
  }
}

TEST(Tracker, UTurnGhostIsStillCaughtWhenAnUnrelatedTrackDiesThatFrame)
{
  // 同一处错位的另一个方向：U 转帧恰有一条无关航迹到期删除、且旧车前面还有一条本帧命中的
  // 航迹 ⟹ 新航迹读到别人的配对结果、reanchor_of_id 落空 ⟹ 旧车不被取代、鬼影回来。
  // 几何照抄 DoesNotCoastAGhostBehindItsOwnReanchoredTrack，另加：静止目标 S（先出生、恒命中）、
  // 无关碎片 E（U 转帧恰好被删）。
  Tracker tracker;
  auto car = [](double x) { return MakeDetection(x, 0.0); };
  const Detection still = MakePartial(5.0, 8.0, 0.5, 0.5);  // S：始终命中，排在旧车前面
  // 下标顺序是本用例的要害：E 必须排在 S 前面（E 删除后 S/车整体左移一格，车读到 S 的配对）。
  tracker.Update({MakePartial(60.0, -30.0, 0.5, 0.5)}, kDt, kSensor);  // E 出生（下标 0）
  tracker.Update({MakePartial(60.0, -30.0, 0.5, 0.5), still}, kDt, kSensor);  // S 出生（下标 1）
  double x = 10.0;
  // 车要养到成熟（hits ≥ mature_hits 10），否则它根本没资格滑行、用例测不到鬼影。
  for (int frame = 0; frame < 8; ++frame, x += 0.2) {
    tracker.Update({MakePartial(60.0, -30.0, 0.5, 0.5), still, car(x)}, kDt, kSensor);
  }
  for (int frame = 0; frame < 4; ++frame, x += 0.2) {  // E 丢 1–4（车 hits 到 12）
    tracker.Update({still, car(x)}, kDt, kSensor);
  }
  std::uint32_t old_id = 0;
  for (const Track & track : tracker.ConfirmedTracks()) {
    if (track.length_m > 4.0) {
      old_id = track.id;
    }
  }
  ASSERT_NE(old_id, 0U);
  // 掉头帧：E 恰在此帧到期删除；车框跳 2.0 m 回来。
  x -= 2.0;
  for (int frame = 0; frame < 8; ++frame, x -= 0.2) {
    tracker.Update({still, car(x)}, kDt, kSensor);
  }
  const std::vector<Track> published = tracker.ConfirmedTracks();
  const Track * ghost = FindById(published, old_id);
  printf(
    "[          ] 同帧删除 + 前置命中航迹：掉头 0.8 s 后旧车航迹 %s\n",
    ghost ? "还在发布（鬼影）" : "已消失");
  EXPECT_EQ(ghost, nullptr) << "U 转鬼影漏网 —— 新航迹的重锚候选被下标错位吃掉了";
}

TEST(Tracker, OneHitConfirmedTrackNeverPublishesTheOrigin)
{
  // 复审探针：confirm_hits=1、mature_hits 默认 10 —— 年轻航迹第一个未命中帧按
  // published_position() 发 last_observed_position，而它出生时没被写 ⟹ 发 map 原点 (0,0)。
  TrackerParams params;
  params.confirm_hits = 1;
  Tracker tracker(params);
  tracker.Update({MakeDetection(15.0, 3.0)}, kDt, kSensor);
  tracker.Update({}, kDt, kSensor);
  const std::vector<Track> published = tracker.ConfirmedTracks();
  ASSERT_EQ(published.size(), 1U);
  const Eigen::Vector2d pos = published[0].published_position(params.mature_hits);
  printf("[          ] 出生即确认的航迹丢一帧后发布位置 (%.2f, %.2f)\n", pos.x(), pos.y());
  EXPECT_LT((pos - Eigen::Vector2d(15.0, 3.0)).norm(), 1.0)
    << "发到原点去了 —— 出生时没写最近观测位置";
}

TEST(Tracker, RejectsMatureHitsBelowConfirmHits)
{
  // 三条规则都假定 成熟 ⟹ 已确认；反着配会让年轻/成熟的分野与确认门槛交叉。
  TrackerParams params;
  params.confirm_hits = 5;
  params.mature_hits = 3;
  EXPECT_THROW(Tracker{params}, std::invalid_argument);
  params.mature_hits = 5;
  EXPECT_NO_THROW(Tracker{params});
}

// ---------------------------------------------------------------------------
//  回放仪器（默认不跑）：把 bag 里抠出来的逐帧检测喂给 Tracker，逐帧打印每条
//  航迹的内部状态（记忆尺寸 / 先验锚定 / 命中未命中 / 速度）。
//  用法：ADS_TRACKER_REPLAY_CSV=<csv> ADS_TRACKER_REPLAY_SENSOR="30,-51.75" \
//        ./build/ads_perception/test_tracker --gtest_also_run_disabled_tests \
//        --gtest_filter='*ReplayFromCsv*'
//  csv 列：t,x,y,yaw,l,w,h（map 系）。只打印 |x-sx|<25 且 |y-sy|<10 的航迹。
// ---------------------------------------------------------------------------
TEST(Tracker, DISABLED_ReplayFromCsv)
{
  const char * path = std::getenv("ADS_TRACKER_REPLAY_CSV");
  ASSERT_NE(path, nullptr) << "设 ADS_TRACKER_REPLAY_CSV";
  Eigen::Vector2d sensor(30.0, -51.75);
  if (const char * s = std::getenv("ADS_TRACKER_REPLAY_SENSOR")) {
    std::sscanf(s, "%lf,%lf", &sensor.x(), &sensor.y());
  }
  std::ifstream in(path);
  ASSERT_TRUE(in.is_open());
  std::string line;
  std::getline(in, line);  // header
  std::map<double, std::vector<Detection>> frames;
  while (std::getline(in, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    double t = 0.0;
    Detection d;
    ss >> t >> d.position.x() >> d.position.y() >> d.yaw_rad >> d.length_m >> d.width_m >>
      d.height_m;
    frames[std::round(t * 100.0) / 100.0].push_back(d);
  }
  Tracker tracker;
  double last_t = -1.0;
  for (const auto & [t, dets] : frames) {
    const double dt = last_t < 0.0 ? kDt : t - last_t;
    last_t = t;
    tracker.Update(dets, dt, sensor);
    printf("--- t=%.2f  %zu dets\n", t, dets.size());
    for (const Detection & d : dets) {
      if (
        std::abs(d.position.x() - sensor.x()) < 25.0 &&
        std::abs(d.position.y() - sensor.y()) < 10.0 && d.height_m < 3.0) {
        printf(
          "   det (%6.2f,%6.2f) yaw %6.1f  %.2fx%.2fx%.2f\n", d.position.x(), d.position.y(),
          d.yaw_rad * 180.0 / M_PI, d.length_m, d.width_m, d.height_m);
      }
    }
    for (const Track & tr : tracker.tracks()) {
      if (
        std::abs(tr.position().x() - sensor.x()) < 25.0 &&
        std::abs(tr.position().y() - sensor.y()) < 10.0 && tr.height_m < 3.0) {
        printf(
          "   trk %3u %s (%6.2f,%6.2f) v=(%5.2f,%5.2f) yaw %6.1f mem %.2fx%.2f hits %d miss %d/%d "
          "prior %d sup %d P=%.3f\n",
          tr.id, tr.confirmed ? "C" : "-", tr.position().x(), tr.position().y(), tr.velocity().x(),
          tr.velocity().y(), tr.yaw_rad * 180.0 / M_PI, tr.length_m, tr.width_m, tr.hits,
          tr.consecutive_misses, tr.occluded_misses, tr.vehicle_prior_anchored ? 1 : 0,
          tr.superseded ? 1 : 0, tr.covariance(0, 0));
      }
    }
  }
}

TEST(Tracker, DoesNotCoastAYoungTrackBehindAnOccluder)
{
  // P9-S5c 实测：车身碎片 3 帧确认，KF 速度还是噪声（5.5 m/s 朝向自车车道），
  // 躲在本车框后面按 §6.5 滑行 —— 3 帧虚警滑进车道。滑行是**外推速度**，
  // 只准给速度已收敛的成熟航迹（hits ≥ mature_hits）；年轻的按普通未命中倒计时。
  Tracker tracker;
  Detection car;
  car.position = {10.0, 0.0};
  car.yaw_rad = M_PI / 2.0;
  car.length_m = 4.4;
  car.width_m = 1.8;
  car.height_m = 1.5;
  for (int frame = 0; frame < 12; ++frame) {
    tracker.Update({car}, kDt, kSensor);
  }
  // 车后 3 帧冒出一个碎片（位置每帧跳 0.5 m —— 碎片的典型形态），刚够确认。
  for (int frame = 0; frame < 3; ++frame) {
    tracker.Update({car, MakePartial(20.0, 0.5 * frame, 0.6, 0.4)}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.ConfirmedTracks().size(), 2U) << "碎片没确认 —— 用例前提没建立起来";
  // 碎片消失（被车挡住的几何成立）。年轻航迹不许滑行：max_misses 5 帧内必须删掉。
  for (int frame = 0; frame < 6; ++frame) {
    tracker.Update({car}, kDt, kSensor);
  }
  const std::vector<Track> confirmed = tracker.ConfirmedTracks();
  printf("[          ] 年轻碎片航迹消失 0.6 s 后确认航迹 %zu 条（应只剩车）\n", confirmed.size());
  EXPECT_EQ(confirmed.size(), 1U) << "3 帧确认的年轻航迹在车后滑行了 —— 在外推一个还没收敛的速度";
}

TEST(Tracker, HoldsAYoungTrackAtItsLastObservationOnMissedFrames)
{
  // P9-S5c：年轻航迹（hits < mature_hits）的未命中帧照发（连续性），但发布位置钉在
  // 最近一次观测、不外推 —— Gazebo 实测那只碎片 3 帧确认后连丢 3 帧，预测位置按
  // 5 m/s 的噪声速度滑进自车车道。成熟航迹的未命中帧照发预测。
  Tracker tracker;
  double x = 20.0;
  for (int frame = 0; frame < 12; ++frame, x -= 0.4) {  // 成熟目标
    tracker.Update({MakeDetection(x, 0.0)}, kDt, kSensor);
  }
  // 年轻目标：3 帧刚确认，位置每帧跳 0.5 m（碎片形态 ⟹ KF 速度是 5 m/s 的噪声）。
  for (int frame = 0; frame < 3; ++frame, x -= 0.4) {
    tracker.Update(
      {MakeDetection(x, 0.0), MakePartial(30.0, 5.0 + 0.5 * frame, 0.6, 0.4)}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.ConfirmedTracks().size(), 2U) << "年轻目标没确认 —— 用例前提没建立起来";
  const double last_seen_y = 6.0;
  const double last_hit_x = x + 0.4;  // 循环末尾多减了一次
  // 两个都丢三帧：都照发；成熟的按预测走，年轻的钉在最近观测。
  for (int frame = 0; frame < 3; ++frame) {
    tracker.Update({}, kDt, kSensor);
  }
  x = last_hit_x - 3 * 0.4;  // 成熟目标丢帧期间的真值位置
  const std::vector<Track> published = tracker.ConfirmedTracks();
  ASSERT_EQ(published.size(), 2U) << "未命中帧不该把航迹从发布里拿掉（连续性）";
  const Track * young = nullptr;
  const Track * mature = nullptr;
  for (const Track & track : published) {
    (track.hits < 10 ? young : mature) = &track;
  }
  ASSERT_NE(young, nullptr);
  ASSERT_NE(mature, nullptr);
  const Eigen::Vector2d young_pub = young->published_position(10);
  const Eigen::Vector2d mature_pub = mature->published_position(10);
  printf(
    "[          ] 丢 3 帧后：年轻航迹 KF 位置 y=%.2f、发布位置 y=%.2f（最近观测 %.1f）；"
    "成熟航迹发布位置 x=%.2f（真值 %.2f）\n",
    young->position().y(), young_pub.y(), last_seen_y, mature_pub.x(), x);
  // 0.15：最近观测位置是 KF **更新后**的状态（滤波值落后原始量测一点），不是量测本身。
  EXPECT_NEAR(young_pub.y(), last_seen_y, 0.15) << "年轻航迹的未命中帧被外推了 —— 那是噪声速度";
  EXPECT_GT(young->position().y(), last_seen_y + 0.8) << "用例前提：KF 内部确实在按噪声速度外推";
  EXPECT_NEAR(mature_pub.x(), x, 0.3) << "成熟航迹的未命中帧该按预测发";
}

TEST(Tracker, RejectsTheAnchorWormholeBetweenStructureFragments)
{
  // 复刻实测工况：帧 1 一个微小碎片（0.03×0.01，L-Shape 对杆件/墙沿的
  // 典型输出），帧 2 同一轴向上 4.5 m 外一个 6.0×0.25 的长碎片。
  // 4.5 m/0.1 s = 45 m/s —— 物理上不可能是同一个目标在运动，
  // 唯一能把它拉进门限的是"尺寸差 5.97 → 重锚位移 2.99"这个虫洞。
  Tracker tracker;
  tracker.Update({MakePartial(20.0, 0.0, 0.03, 0.01)}, kDt, kSensor);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  const std::uint32_t fragment_id = tracker.tracks()[0].id;

  tracker.Update({MakePartial(24.5, 0.0, 6.0, 0.25)}, kDt, kSensor);

  ASSERT_EQ(tracker.tracks().size(), 2U) << "4.5 m 外的碎片被焊进了原航迹 —— 重锚位移闸没生效";
  const Track * fragment = FindById(tracker.tracks(), fragment_id);
  ASSERT_NE(fragment, nullptr);
  printf(
    "[          ] 碎片航迹位置 (%.2f, %.2f)、速度 %.2f m/s（虫洞打开时是 8.8 量级）\n",
    fragment->position().x(), fragment->position().y(), fragment->velocity().norm());
  EXPECT_LT((fragment->position() - Eigen::Vector2d(20.0, 0.0)).norm(), 0.6)
    << "原碎片航迹被拖走了";
  EXPECT_LT(fragment->velocity().norm(), 0.5)
    << "静止碎片得到了数米每秒的假速度 —— 这正是幻影的出生证";
}

TEST(Tracker, DoesNotCoastAnImplausiblyFastTrack)
{
  // 几何与 SurvivesAnOcclusionLongerThanMaxMisses **完全相同**（车横在
  // (10,0) 挡视线、目标在 (20+,0) 的车后），那条用例证明这个几何下
  // 1.2 m/s 的行人**能**滑行 —— 所以本用例里目标死掉只能是速度准入拦的，
  // 不是遮挡判定坏了。两条用例互为对照。
  //
  // 10 m/s 超出 ODD 上限 8.33：园区里没有这么快的东西，这条航迹的状态
  // 必然是坏的（实测里它就是被碎片焊出来的）。外推坏状态 = 制造幽灵。
  Tracker tracker;
  Detection car;
  car.position = {10.0, 0.0};
  car.yaw_rad = M_PI / 2.0;
  car.length_m = 4.4;
  car.width_m = 1.8;
  car.height_m = 1.5;
  for (int frame = 0; frame < 12; ++frame) {  // ≥ mature_hits，否则是「年轻」不是「太快」在拦
    tracker.Update({car, MakePartial(20.0 + frame, 0.0, 4.4, 1.8)}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.ConfirmedTracks().size(), 2U);
  std::uint32_t ghost_id = 0;
  double ghost_speed = 0.0;
  for (const Track & track : tracker.ConfirmedTracks()) {
    if (track.velocity().norm() > ghost_speed) {
      ghost_speed = track.velocity().norm();
      ghost_id = track.id;
    }
  }
  ASSERT_GE(ghost_speed, 9.0) << "快目标的速度还没收敛 —— 用例前提没建立起来";

  // 消失在车后。速度合法的目标在这里会滑行（对照用例），10 m/s 的不许。
  for (int frame = 0; frame < 6; ++frame) {
    tracker.Update({car}, kDt, kSensor);
  }
  printf(
    "[          ] %.1f m/s 的航迹在遮挡后 %zu 条确认航迹存活（应只剩车）\n", ghost_speed,
    tracker.ConfirmedTracks().size());
  EXPECT_EQ(FindById(tracker.ConfirmedTracks(), ghost_id), nullptr)
    << "物理上不可能的状态被滑行外推 —— 幽灵会带着假速度横穿地图";
  EXPECT_EQ(tracker.ConfirmedTracks().size(), 1U);
}

// =============================================================================
//  结构物档 + 车辆形状先验（P8-S2b）
//
//  ## 故障注入实测（2026-08-13，写完立刻做的）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 去掉结构物冻速度（is_structure 不清 tail） | **红** 1 例：StructureTrackFreezesVelocity |
//  | 去掉车辆先验（memory_length 不取 max） | **红** 1 例：FrontalVehiclePriorCompletesTheCenter |
// =============================================================================

TEST(Tracker, StructureTrackFreezesVelocity)
{
  // 建筑片段：帧帧 6.0 大框，中心随自车视角连续小步滑移（每帧 0.5 m
  // < anchor_shift_max 2.2 —— 焊接闸拦不住的**真实**测量位移）。
  // P8-S2b 画像：这种航迹 KF 积出 9–15 m/s 假速度，86 条。
  // 结构物档（连续 5 帧观测超 5.5）后速度必须恒为零。
  Tracker tracker;
  for (int frame = 0; frame < 20; ++frame) {
    Detection wall = MakeDetection(30.0 + 0.5 * frame, 5.0);
    wall.length_m = 6.0;
    wall.width_m = 6.0;
    tracker.Update({wall}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.tracks().size(), 1u)
    << "冻结若发生在状态层，滑移观测会把一条墙断成两条（新息累积判不进门 —— 实测过）";
  const Track & track = tracker.tracks()[0];
  EXPECT_TRUE(track.is_structure) << "连续 20 帧 6.0 框还没进结构物档";
  // ⚠️ 断言**报告层**不是状态层：内部速度是关联要用的（跟滑移的框），
  //    「不许出门」的是 reported_velocity —— 发布出口以它为准（perception_node）。
  EXPECT_LT(track.reported_velocity().norm(), 1e-9)
    << "结构物对外的『速度』必须恒为零 —— 内部滑移速度出了门就是幻影横穿";
  EXPECT_GT(track.velocity().norm(), 3.0)
    << "内部速度反而应当在跟滑移（关联健康的证据）—— 它要是零，关联靠什么跟框？";
}

TEST(Tracker, MomentaryUndersegmentationDoesNotBrandAVehicle)
{
  // 真车 + 一次 2 帧的瞬间并簇（欠分割，P5 实测存在）：
  // 观测长边 6.2 只持续 2 帧 < structure_confirm_frames(5) —— 不许把正主
  // 标成结构物（标了速度就没了，跟踪等于失效）。
  Tracker tracker;
  for (int frame = 0; frame < 20; ++frame) {
    Detection car = MakeDetection(10.0 + 4.0 * frame * kDt, 0.0);
    if (frame == 8 || frame == 9) {
      car.length_m = 6.2;  // 并簇瞬间
    }
    tracker.Update({car}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.tracks().size(), 1u);
  const Track & track = tracker.tracks()[0];
  EXPECT_FALSE(track.is_structure) << "2 帧并簇就永久判死一辆真车";
  EXPECT_NEAR(track.velocity().norm(), 4.0, 0.5);
}

TEST(Tracker, FrontalVehiclePriorCompletesTheCenter)
{
  // 正对驶来的车：从头到尾只露尾面（观测 1.8 宽 × 0.4 深），记忆里永远
  // 没有「长」—— P8-S2b 画像：中心停在尾面，沿视线偏 1.3–2.1 m。
  // 速度（4 m/s）+ 宽度（1.8 ≥ 1.4）双门控 ⟹ 按 ODD 车长 4.4 先验补全：
  // 中心应被推到离尾面 ~半车长（背离传感器一侧）。
  //
  // 几何：真车几何中心从 x=60 向传感器（原点）驶来，速度 −4；
  // 可见尾面在中心 −2.2（朝传感器一侧），观测框中心 ≈ 尾面 +0.2 深。
  Tracker tracker;
  double center_x = 60.0;
  const Track * track = nullptr;
  for (int frame = 0; frame < 30; ++frame) {
    center_x -= 4.0 * kDt;
    Detection tail_face = MakeDetection(center_x - 2.2 + 0.2, 0.0);
    tail_face.length_m = 1.8;  // L-Shape 在正对时给的"长"是车宽
    tail_face.width_m = 0.4;   // 只有尾面的深度
    tracker.Update({tail_face}, kDt, kSensor);
  }
  ASSERT_EQ(tracker.tracks().size(), 1u);
  track = &tracker.tracks()[0];
  // 无先验时估计中心 ≈ 观测中心（x = 真中心 − 2.0）；有先验时应回到真中心
  // 附近。判据取中点 1.0：先验修正 ≥ 一半即视为生效（KF 收敛余量）。
  const double error_m = std::abs(track->position().x() - center_x);
  EXPECT_LT(error_m, 1.0) << "正对中心仍偏 " << error_m
                          << " m —— 形状先验没有生效（无先验时 ≈2.0）";
}
