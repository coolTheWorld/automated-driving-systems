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
//  模型选择的 L1 判据（P6-1 决策五 + S1 位移一致性）
//
//  每条规则一个用例、两侧都卡（该选的选上、不该选的拦下）。
//  被拦的用例全部来自**实测过的病理**，不是构造的稻草人：
//    墙沿假速度 15.5 / 5.5 m/s（S0/S1 实测）、原地摆的 24.5%（S1 体检）。
//
//  ## 故障注入实测（2026-08-12，跑完回填）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 位移一致性整段跳过（规则 3） | **红 3 条**：原地摆、无历史、比例门限三条全红 |
//  | 去掉 ODD 上限（规则 2） | **红 1 条**：`RefusesImplausiblySpeedyTrack` |
//
//  ⚠️ 规则 3 的注入必须**整段**跳过：只把 has_value 检查改恒假的话，
//     下一行对空 optional 解引用是 UB —— 注入自己制造了未定义行为，
//     红绿都不可信（第一轮注入实测踩到，用例"侥幸"全绿）。
// =============================================================================

#include <gtest/gtest.h>

#include "ads_prediction/model_selector.hpp"

namespace
{

using ads_prediction::ModelKind;
using ads_prediction::SelectModel;
using ads_prediction::SelectorParams;
using ads_prediction::TargetSnapshot;

TargetSnapshot Vehicle(double speed, double net_displacement)
{
  TargetSnapshot target;
  target.velocity = {speed, 0.0};
  target.length_m = 4.4;
  target.width_m = 1.8;
  target.net_displacement_1s_m = net_displacement;
  return target;
}

}  // namespace

TEST(ModelSelector, SlowTargetIsStatic)
{
  // 新航迹初速恒 0（tracker 的设计），0.3 m/s 是速度噪声量级。
  EXPECT_EQ(SelectModel(Vehicle(0.3, 0.3), {}), ModelKind::kStatic)
    << "把速度噪声当运动方向去外推了";
}

TEST(ModelSelector, MovingVehicleGetsLaneFollow)
{
  EXPECT_EQ(SelectModel(Vehicle(4.0, 3.8), {}), ModelKind::kLaneFollow);
}

TEST(ModelSelector, MovingPedestrianGetsConstantVelocity)
{
  TargetSnapshot ped;
  ped.velocity = {1.2, 0.0};
  ped.length_m = 0.4;  // 行人尺度 —— 用**尺寸**分档，不用 classification
  ped.width_m = 0.4;
  ped.net_displacement_1s_m = 1.1;
  EXPECT_EQ(SelectModel(ped, {}), ModelKind::kConstantVelocity);
}

TEST(ModelSelector, RefusesImplausiblySpeedyTrack)
{
  // S0 实测：墙沿簇形交替焊出 11.9 / 15.5 m/s 的假航迹。ODD 上限 8.33 ——
  // 园区里没有东西跑这么快，坏状态不许外推。
  EXPECT_EQ(SelectModel(Vehicle(15.5, 15.0), {}), ModelKind::kStatic)
    << "物理上不可能的速度被拿去编了轨迹";
}

TEST(ModelSelector, RefusesVelocityWithoutDisplacementBacking)
{
  // S1 体检：结构物航迹 |v|>0.5 占 24.5% —— 簇形交替的假速度，位置原地摆。
  // 声称 5.5 m/s、1 s 净位移只有 0.4 m（要求 ≥ 5.5×0.5 = 2.75）→ 拦下。
  EXPECT_EQ(SelectModel(Vehicle(5.5, 0.4), {}), ModelKind::kStatic)
    << "没有净位移背书的速度被信了 —— S1 量出的 24.5% 假速度全会变成飞行幽灵";
}

TEST(ModelSelector, RefusesVelocityWithoutHistory)
{
  // 没有历史 = 不知道 = 不编（新目标头一秒按静态处理，代价写在
  // prediction.md §5：真实新目标的预测延迟 ~1 s，而它的速度本来就没收敛）。
  TargetSnapshot target = Vehicle(4.0, 0.0);
  target.net_displacement_1s_m.reset();
  EXPECT_EQ(SelectModel(target, {}), ModelKind::kStatic);
}

TEST(ModelSelector, DisplacementGateScalesWithClaimedSpeed)
{
  // 门限是**比例**不是常数：4 m/s 要 2 m 背书，1 m/s 只要 0.5 m ——
  // 慢目标不因为绝对位移小而被误拦。
  EXPECT_EQ(SelectModel(Vehicle(1.0, 0.6), {}), ModelKind::kLaneFollow);
  EXPECT_EQ(SelectModel(Vehicle(4.0, 0.6), {}), ModelKind::kStatic);
}
