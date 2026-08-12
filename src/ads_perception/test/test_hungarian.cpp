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
//  匈牙利算法的 L1 判据
//
//  ⚠️ **核心判据是与穷举对账**，不是几个手算例子。
//
//  这个算法写错的表现是「大部分时候对、偶尔配错一对」—— 比如漏掉势更新里
//  那个 else 分支，算法仍然终止、仍然给出一个匹配，只是**不是最优的**。
//  手算例子过不了几个这种 bug；而随机矩阵 + 全排列穷举能。
//  （与 ads_map 的 Dijkstra 用穷举脚本对账是同一条规矩。）
//
//  ## 故障注入实测（2026-08-11）
//
//  | 注入 | 结果 |
//  |---|---|
//  | 漏掉势更新里的 `else` 分支（未访问列不减 delta） | **红 2 条**（两条穷举对账） |
//  | 不拆被禁的配对 | **红 1 条** |
//
//  第一条正是"看起来能跑但不是最优"的那种错误：算法仍然终止、仍然给出一个
//  合法匹配，只有总代价不对。**手算例子抓不到它，穷举能。**
// =============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

#include "ads_perception/hungarian.hpp"

namespace
{

using ads_perception::kForbiddenCost;
using ads_perception::SolveAssignment;

/// 穷举所有可能的匹配，返回最小总代价。**只用于小规模对账。**
double BruteForceMinimum(const std::vector<std::vector<double>> & cost)
{
  const int rows = static_cast<int>(cost.size());
  const int cols = static_cast<int>(cost[0].size());
  // 枚举列的全排列，取前 rows 个分给各行（rows ≤ cols 时是完备的）。
  std::vector<int> columns(cols);
  std::iota(columns.begin(), columns.end(), 0);
  double best = std::numeric_limits<double>::max();
  do {
    double total = 0.0;
    for (int i = 0; i < rows; ++i) {
      total += cost[i][columns[i]];
    }
    best = std::min(best, total);
  } while (std::next_permutation(columns.begin(), columns.end()));
  return best;
}

double TotalCost(const std::vector<std::vector<double>> & cost, const std::vector<int> & assignment)
{
  double total = 0.0;
  for (std::size_t i = 0; i < assignment.size(); ++i) {
    if (assignment[i] >= 0) {
      total += cost[i][assignment[i]];
    }
  }
  return total;
}

}  // namespace

// ---------------------------------------------------------------------------
//  核心：与穷举对账
// ---------------------------------------------------------------------------
TEST(Hungarian, MatchesBruteForceOnRandomSquareMatrices)
{
  std::mt19937 rng(20260811);
  std::uniform_real_distribution<double> value(0.0, 10.0);
  int checked = 0;
  for (int size = 1; size <= 6; ++size) {
    for (int trial = 0; trial < 40; ++trial) {
      std::vector<std::vector<double>> cost(size, std::vector<double>(size));
      for (auto & row : cost) {
        for (double & v : row) {
          v = value(rng);
        }
      }
      const std::vector<int> assignment = SolveAssignment(cost);
      ASSERT_EQ(assignment.size(), static_cast<std::size_t>(size));

      // 每一行都要配上，且列不重复。
      std::vector<int> used;
      for (int i = 0; i < size; ++i) {
        ASSERT_GE(assignment[i], 0) << size << "×" << size << " 方阵里第 " << i << " 行没配上";
        used.push_back(assignment[i]);
      }
      std::sort(used.begin(), used.end());
      ASSERT_TRUE(std::unique(used.begin(), used.end()) == used.end()) << "同一列被配给了两行";

      EXPECT_NEAR(TotalCost(cost, assignment), BruteForceMinimum(cost), 1e-9)
        << size << "×" << size << " 第 " << trial << " 次：不是最优匹配";
      ++checked;
    }
  }
  printf("[          ] 与穷举对账 %d 个随机方阵（1×1 到 6×6），全部一致\n", checked);
}

TEST(Hungarian, MatchesBruteForceOnRectangularMatrices)
{
  // 矩形是常态：航迹数与检测数几乎不会相等。
  // ⚠️ **两个方向都要测** —— 实现里 rows > cols 时走的是"转置后求解再转回来"
  //    那条分支，转回来时把下标搞反是一个非常容易犯且只在这一侧暴露的错误。
  std::mt19937 rng(777);
  std::uniform_real_distribution<double> value(0.0, 10.0);
  for (const auto & shape : std::vector<std::pair<int, int>>{{2, 5}, {5, 2}, {3, 4}, {4, 3}}) {
    for (int trial = 0; trial < 30; ++trial) {
      std::vector<std::vector<double>> cost(shape.first, std::vector<double>(shape.second));
      for (auto & row : cost) {
        for (double & v : row) {
          v = value(rng);
        }
      }
      const std::vector<int> assignment = SolveAssignment(cost);
      ASSERT_EQ(assignment.size(), static_cast<std::size_t>(shape.first));

      const int expected_pairs = std::min(shape.first, shape.second);
      int actual_pairs = 0;
      for (const int column : assignment) {
        if (column >= 0) {
          ++actual_pairs;
          EXPECT_LT(column, shape.second);
        }
      }
      EXPECT_EQ(actual_pairs, expected_pairs)
        << shape.first << "×" << shape.second << " 应当配上 " << expected_pairs << " 对";

      // 穷举：行多于列时转置再穷举（BruteForceMinimum 要求 rows ≤ cols）。
      double expected_cost = 0.0;
      if (shape.first <= shape.second) {
        expected_cost = BruteForceMinimum(cost);
      } else {
        std::vector<std::vector<double>> transposed(shape.second, std::vector<double>(shape.first));
        for (int i = 0; i < shape.first; ++i) {
          for (int j = 0; j < shape.second; ++j) {
            transposed[j][i] = cost[i][j];
          }
        }
        expected_cost = BruteForceMinimum(transposed);
      }
      EXPECT_NEAR(TotalCost(cost, assignment), expected_cost, 1e-9)
        << shape.first << "×" << shape.second << " 第 " << trial << " 次不是最优";
    }
  }
  printf("[          ] 矩形矩阵（2×5 / 5×2 / 3×4 / 4×3）各 30 次，全部与穷举一致\n");
}

// ---------------------------------------------------------------------------
//  ⚠️ 这是匈牙利存在的理由：贪心会配错
// ---------------------------------------------------------------------------
TEST(Hungarian, BeatsGreedyOnTheCaseThatMotivatesIt)
{
  // 两条航迹 A、B 与两个检测 1、2：
  //   贪心按顺序处理 A → 拿走最近的 1，B 只好要 2，总代价 1.0 + 5.0 = 6.0
  //   最优是 A→2、B→1，总代价 1.1 + 1.05 = 2.15
  //
  // ⚠️ 配错的后果不是"差一点"，是**两个目标的 ID 互换** ——
  //    它们的速度估计各自跳到对方的历史上，P6 会预测出两条交叉的荒谬轨迹。
  const std::vector<std::vector<double>> cost = {{1.00, 1.10}, {1.05, 5.00}};
  const std::vector<int> assignment = SolveAssignment(cost);
  printf(
    "[          ] 贪心会给出 6.00，匈牙利给出 %.2f（最优 2.15）\n", TotalCost(cost, assignment));
  EXPECT_EQ(assignment[0], 1);
  EXPECT_EQ(assignment[1], 0);
  EXPECT_NEAR(TotalCost(cost, assignment), 2.15, 1e-9);
}

// ---------------------------------------------------------------------------
//  被禁的配对
// ---------------------------------------------------------------------------
TEST(Hungarian, LeavesForbiddenPairsUnassigned)
{
  // 第 0 行只有第 1 列可选，第 1 行只有第 0 列可选。
  std::vector<std::vector<double>> cost = {{kForbiddenCost, 2.0}, {3.0, kForbiddenCost}};
  std::vector<int> assignment = SolveAssignment(cost);
  EXPECT_EQ(assignment[0], 1);
  EXPECT_EQ(assignment[1], 0);

  // 第 0 行**没有**可选列 —— 必须留 −1，而不是硬塞一个被禁的。
  //
  // ⚠️ 算法本身求的是**完美**匹配，它会为了凑满而选中被禁的边。
  //    不在返回前拆掉的话，一条航迹会被"配"给一个远在天边的检测 ——
  //    而那看起来只是"关联质量不好"，不会有人怀疑到这里。
  cost = {{kForbiddenCost, kForbiddenCost}, {3.0, 4.0}};
  assignment = SolveAssignment(cost);
  printf("[          ] 无可选列的行 → %d（应为 −1）\n", assignment[0]);
  EXPECT_EQ(assignment[0], -1) << "被禁的配对没被拆掉";
  EXPECT_GE(assignment[1], 0);
}

// ---------------------------------------------------------------------------
//  边界与防御
// ---------------------------------------------------------------------------
TEST(Hungarian, HandlesEmptyAndDegenerateShapes)
{
  EXPECT_TRUE(SolveAssignment({}).empty());
  // 有行无列：全部未配。
  const std::vector<int> no_columns = SolveAssignment({{}, {}});
  ASSERT_EQ(no_columns.size(), 2U);
  EXPECT_EQ(no_columns[0], -1);
  EXPECT_EQ(no_columns[1], -1);
}

TEST(Hungarian, ThrowsOnInfinityInsteadOfHanging)
{
  // ⚠️ 这条守着一个**死循环**：算法内部做 cost − u − v，inf − inf = NaN，
  //    而 NaN 参与比较恒为 false ⟹ 最小值搜索静默跳过所有列，
  //    j1 保持 0，外层 do-while 的条件恒真。
  //    「不允许配对」必须用 kForbiddenCost（一个大的**有限**数）。
  EXPECT_THROW(
    SolveAssignment({{std::numeric_limits<double>::infinity(), 1.0}, {2.0, 3.0}}),
    std::invalid_argument);
  EXPECT_THROW(SolveAssignment({{std::nan(""), 1.0}, {2.0, 3.0}}), std::invalid_argument);
}

TEST(Hungarian, RejectsRaggedMatrices)
{
  EXPECT_THROW(SolveAssignment({{1.0, 2.0}, {3.0}}), std::invalid_argument);
}
