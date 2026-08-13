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
//  微型行为树的 tick 语义（CP-P7-A ② 的短路半边；滞回半边在 test_longitudinal）
//
//  判据全部是**执行次数**：短路语义的唯一可观测后果就是"后面的孩子没被 tick"。
//  只断言返回值的话，一个把全部孩子都 tick 一遍再汇总的实现照样绿 ——
//  而那个实现会让带副作用的 Action 在不该执行时执行（正是安全要防的）。
// =============================================================================

#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

#include "ads_planning/behavior_tree.hpp"

namespace
{

using ads_planning::Action;
using ads_planning::BtNode;
using ads_planning::collect;
using ads_planning::Condition;
using ads_planning::Fallback;
using ads_planning::Sequence;
using ads_planning::TickStatus;

/// 计数用的探针节点：记录自己被 tick 了几次，返回预设状态。
class Probe : public BtNode
{
public:
  explicit Probe(TickStatus status) : status_(status) {}
  TickStatus tick() override
  {
    ++count_;
    return status_;
  }
  int count() const { return count_; }

private:
  TickStatus status_;
  int count_{0};
};

TEST(BehaviorTree, ConditionMapsBoolToStatus)
{
  Condition truthy([] { return true; });
  Condition falsy([] { return false; });
  EXPECT_EQ(truthy.tick(), TickStatus::kSuccess);
  EXPECT_EQ(falsy.tick(), TickStatus::kFailure);
}

TEST(BehaviorTree, SequenceShortCircuitsOnFailure)
{
  auto first = std::make_unique<Probe>(TickStatus::kSuccess);
  auto second = std::make_unique<Probe>(TickStatus::kFailure);
  auto third = std::make_unique<Probe>(TickStatus::kSuccess);
  Probe * p1 = first.get();
  Probe * p2 = second.get();
  Probe * p3 = third.get();

  Sequence sequence(collect(std::move(first), std::move(second), std::move(third)));
  EXPECT_EQ(sequence.tick(), TickStatus::kFailure);
  EXPECT_EQ(p1->count(), 1);
  EXPECT_EQ(p2->count(), 1);
  EXPECT_EQ(p3->count(), 0) << "Failure 之后的孩子不许被 tick —— 短路是语义不是优化";
}

TEST(BehaviorTree, SequencePropagatesRunning)
{
  auto first = std::make_unique<Probe>(TickStatus::kRunning);
  auto second = std::make_unique<Probe>(TickStatus::kSuccess);
  Probe * p2 = second.get();
  Sequence sequence(collect(std::move(first), std::move(second)));
  EXPECT_EQ(sequence.tick(), TickStatus::kRunning);
  EXPECT_EQ(p2->count(), 0);
}

TEST(BehaviorTree, FallbackShortCircuitsOnSuccess)
{
  auto first = std::make_unique<Probe>(TickStatus::kFailure);
  auto second = std::make_unique<Probe>(TickStatus::kSuccess);
  auto third = std::make_unique<Probe>(TickStatus::kFailure);
  Probe * p1 = first.get();
  Probe * p2 = second.get();
  Probe * p3 = third.get();

  Fallback fallback(collect(std::move(first), std::move(second), std::move(third)));
  EXPECT_EQ(fallback.tick(), TickStatus::kSuccess);
  EXPECT_EQ(p1->count(), 1);
  EXPECT_EQ(p2->count(), 1);
  EXPECT_EQ(p3->count(), 0) << "Success 之后的孩子不许被 tick";
}

TEST(BehaviorTree, FallbackAllFailuresFails)
{
  Fallback fallback(collect(
    std::make_unique<Probe>(TickStatus::kFailure), std::make_unique<Probe>(TickStatus::kFailure)));
  EXPECT_EQ(fallback.tick(), TickStatus::kFailure);
}

TEST(BehaviorTree, GuardedActionOnlyRunsWhenConditionHolds)
{
  // 本项目树的最小型：Sequence(Condition, Action)。
  // 条件假时动作**一次都不许执行** —— 树的安全语义整个建立在这上面。
  int executed = 0;
  bool armed = false;
  Sequence guarded(collect(
    std::make_unique<Condition>([&] { return armed; }), Action::always([&] { ++executed; })));

  EXPECT_EQ(guarded.tick(), TickStatus::kFailure);
  EXPECT_EQ(executed, 0);
  armed = true;
  EXPECT_EQ(guarded.tick(), TickStatus::kSuccess);
  EXPECT_EQ(executed, 1);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
