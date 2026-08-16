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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "ads_perception/detection_gates.hpp"

// =============================================================================
//  准入门的 L1：每个阈值的两侧各一个实测形态（数字来自 P9 的 bag / 裸测）。
//
//  ## 故障注入实测（2026-08-16）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 剃刀门去掉「矮」条件（只按薄剃） | 红 FrontalFacesAreThinButTallAndPass（车尾面/行人侧面被吞）
//    + BoundariesAreStrict |
//  | 浮空门去掉「扁」条件（只按高剃） | 红 OccludedUpperBodyIsHighButTallAndPasses
//    + BoundariesAreStrict |
// =============================================================================

namespace
{

using ads_perception::Admission;
using ads_perception::AdmissionParams;
using ads_perception::AdmitDetection;

const AdmissionParams kParams{};  // yaml 现值：0.1 / 0.3 / 1.0 / 0.3

TEST(DetectionGates, RazorStripIsThinAndLow)
{
  // CARLA 路面接缝单环残留：0.03 宽、1–2.5 m 长、竖向延展 ≈ 0（P5 实测 374 帧虚警主体）。
  EXPECT_EQ(AdmitDetection(2.0, 0.03, 0.05, 0.02, kParams), Admission::kRazorStrip);
  // Gazebo 剃刀条 0.04 宽两环残留（竖向 0.15）：也剃。
  EXPECT_EQ(AdmitDetection(1.2, 0.04, 0.15, 0.03, kParams), Admission::kRazorStrip);
}

TEST(DetectionGates, FrontalFacesAreThinButTallAndPass)
{
  // 正对的车尾面：L-Shape 深度只剩雷达噪声（min(l,w) p50 0.047），但 1.3 m 高 —— 放行。
  // 这就是 2026-08-15 A/B 里门 0.1（单条件）让跟停 −5.19 m 撞车的那批框。
  EXPECT_EQ(AdmitDetection(1.8, 0.05, 1.3, 0.2, kParams), Admission::kAccepted);
  // 行人侧面 0.08 深、1.5 高：放行。
  EXPECT_EQ(AdmitDetection(0.4, 0.08, 1.5, 0.2, kParams), Admission::kAccepted);
}

TEST(DetectionGates, RoofFragmentIsHighAndFlat)
{
  // 车顶远端环碎片（Gazebo 实测 0.9×0.5×0.03，底离地 1.48）：浮空，剃。
  EXPECT_EQ(AdmitDetection(0.9, 0.5, 0.03, 1.48, kParams), Admission::kFloatingFragment);
  // 近端边缘小片 0.4×0.11×0.02 底 1.49：同上。
  EXPECT_EQ(AdmitDetection(0.4, 0.11, 0.02, 1.49, kParams), Admission::kFloatingFragment);
}

TEST(DetectionGates, OccludedUpperBodyIsHighButTallAndPasses)
{
  // 墙后只露上半身的行人：底 1.0、延展 0.7 —— 高但不扁，放行（这是浮空门第二个条件的意义）。
  EXPECT_EQ(AdmitDetection(0.4, 0.4, 0.7, 1.05, kParams), Admission::kAccepted);
  // 记录在案的代价：只露头顶 0.2 m（底 1.5）不成检测。
  EXPECT_EQ(AdmitDetection(0.3, 0.3, 0.2, 1.5, kParams), Admission::kFloatingFragment);
}

TEST(DetectionGates, GroundedTargetsNeverLookFloating)
{
  // 30 m 处车 / 行人的最低一环离地 ≤ 0.8（地面阈值 0.2 + 线间距 0.59）：底 0.79、延展 0.6 放行。
  EXPECT_EQ(AdmitDetection(1.8, 0.4, 0.6, 0.79, kParams), Admission::kAccepted);
}

TEST(DetectionGates, BoundariesAreStrict)
{
  // 剃刀门：min(l,w) 恰等于 0.1 不算薄；延展恰等于 0.3 不算矮。
  EXPECT_EQ(AdmitDetection(2.0, 0.1, 0.05, 0.02, kParams), Admission::kAccepted);
  EXPECT_EQ(AdmitDetection(2.0, 0.05, 0.3, 0.02, kParams), Admission::kAccepted);
  // 浮空门：底恰等于 1.0 不算高；延展恰等于 0.3 不算扁。
  EXPECT_EQ(AdmitDetection(0.9, 0.5, 0.03, 1.0, kParams), Admission::kAccepted);
  EXPECT_EQ(AdmitDetection(0.9, 0.5, 0.3, 1.48, kParams), Admission::kAccepted);
}

TEST(DetectionGates, RazorWinsOverFloatingWhenBothApply)
{
  // 又薄又矮又高（浮在 1.2 m 的一根细条）：诊断计数归剃刀门 —— 与 node 里原来的顺序一致。
  EXPECT_EQ(AdmitDetection(1.0, 0.03, 0.05, 1.2, kParams), Admission::kRazorStrip);
}

TEST(DetectionGates, ThrowsOnNonFiniteInput)
{
  // NaN 参与比较恒假 ⟹ 两道门都会**静默放行**一个 NaN 框；所以必须显式拦。
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(AdmitDetection(nan, 0.5, 0.03, 1.48, kParams), std::invalid_argument);
  EXPECT_THROW(AdmitDetection(0.9, 0.5, 0.03, inf, kParams), std::invalid_argument);
}

}  // namespace
