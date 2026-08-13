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

#ifndef ADS_PLANNING__BEHAVIOR_TREE_HPP_
#define ADS_PLANNING__BEHAVIOR_TREE_HPP_

// =============================================================================
//  behavior_tree —— 微型行为树（P7-S2，自研，决策四）
//
//  为什么自研而不是 BehaviorTree.CPP：重型依赖（先问后做），且第一版必须
//  手推 tick 语义。为什么不做 XML 加载：树是**安全逻辑的一部分**，
//  运行时可变结构与「安全逻辑不可关闭」（SPEC §11）相性差 ——
//  树在代码里显式写出，评审看得见每一个分支。
//
//  ⚠️ **树只做行为选择（选状态标签），不裁决约束**（behavior.md §0 红线 1/2）。
//     约束合成在 longitudinal.hpp 的 merge() —— 树外、取最保守、不可关。
//     所以这里的节点没有「跳过约束」的表达能力，结构上就没有。
//
//  tick 语义（经典三态）：
//    Condition  fn() ? Success : Failure
//    Action     执行 fn 并返回它给出的状态（默认包装返回 Success）
//    Sequence   逐个 tick，遇到非 Success 立刻短路返回它（全 Success 才 Success）
//    Fallback   逐个 tick，遇到非 Failure 立刻短路返回它（全 Failure 才 Failure）
//  Running 保留在枚举里以维持完整语义 —— 本项目的条件与动作都是瞬时的
//  （一个规划周期内出结果），当前没有节点返回它，但 Sequence/Fallback 对它的
//  短路行为是对的、且有用例钉着，将来引入耗时动作时不需要改机器。
// =============================================================================

#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace ads_planning
{

enum class TickStatus
{
  kSuccess,
  kFailure,
  kRunning,
};

/// @brief 行为树节点基类。tick() 每个规划周期被调一次。
class BtNode
{
public:
  virtual ~BtNode() = default;
  virtual TickStatus tick() = 0;
};

/// @brief 条件节点：包装一个布尔谓词。
class Condition : public BtNode
{
public:
  explicit Condition(std::function<bool()> fn) : fn_(std::move(fn)) {}
  TickStatus tick() override { return fn_() ? TickStatus::kSuccess : TickStatus::kFailure; }

private:
  std::function<bool()> fn_;
};

/// @brief 动作节点：执行副作用（本项目里 = 记状态标签），返回执行状态。
class Action : public BtNode
{
public:
  /// 完整形式：动作自己报告状态。
  explicit Action(std::function<TickStatus()> fn) : fn_(std::move(fn)) {}
  /// 便捷形式：无返回值的动作视为总是成功。
  static std::unique_ptr<Action> always(std::function<void()> fn)
  {
    return std::make_unique<Action>([fn = std::move(fn)] {
      fn();
      return TickStatus::kSuccess;
    });
  }
  TickStatus tick() override { return fn_(); }

private:
  std::function<TickStatus()> fn_;
};

/// @brief 顺序节点：全部成功才成功，遇到非 Success 短路。
class Sequence : public BtNode
{
public:
  explicit Sequence(std::vector<std::unique_ptr<BtNode>> children) : children_(std::move(children))
  {
  }
  TickStatus tick() override
  {
    for (auto & child : children_) {
      const TickStatus status = child->tick();
      if (status != TickStatus::kSuccess) {
        return status;
      }
    }
    return TickStatus::kSuccess;
  }

private:
  std::vector<std::unique_ptr<BtNode>> children_;
};

/// @brief 备选节点：第一个非 Failure 的孩子决定结果。
class Fallback : public BtNode
{
public:
  explicit Fallback(std::vector<std::unique_ptr<BtNode>> children) : children_(std::move(children))
  {
  }
  TickStatus tick() override
  {
    for (auto & child : children_) {
      const TickStatus status = child->tick();
      if (status != TickStatus::kFailure) {
        return status;
      }
    }
    return TickStatus::kFailure;
  }

private:
  std::vector<std::unique_ptr<BtNode>> children_;
};

/// @brief 便捷构造：把若干节点收进一个 vector（unique_ptr 不能用初始化列表）。
template <typename... Nodes>
std::vector<std::unique_ptr<BtNode>> collect(Nodes &&... nodes)
{
  std::vector<std::unique_ptr<BtNode>> children;
  children.reserve(sizeof...(nodes));
  (children.push_back(std::forward<Nodes>(nodes)), ...);
  return children;
}

}  // namespace ads_planning

#endif  // ADS_PLANNING__BEHAVIOR_TREE_HPP_
