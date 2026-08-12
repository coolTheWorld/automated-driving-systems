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
