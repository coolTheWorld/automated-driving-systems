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

#include "ads_perception/hungarian.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace ads_perception
{

namespace
{

/// 核心求解，要求 `rows ≤ cols`。返回 `column_of_row[i]`。
///
/// e-maxx 的经典 O(n³) 势函数实现，内部用 1-indexed（0 号是虚拟的"自由行"）。
/// 保持 1-indexed 是有意的：这个算法的所有公开推导都按 1-indexed 写，
/// 改成 0-indexed 之后每一处下标都要心算平移一位，而那正是它最容易写错的地方。
std::vector<int> SolveTall(const std::vector<std::vector<double>> & cost, int rows, int cols)
{
  const double kInfinity = std::numeric_limits<double>::max();
  std::vector<double> u(rows + 1, 0.0);         // 行的势
  std::vector<double> v(cols + 1, 0.0);         // 列的势
  std::vector<int> row_of_column(cols + 1, 0);  // 列 j 当前配给了哪一行
  std::vector<int> way(cols + 1, 0);            // 增广路径上的前驱列

  for (int i = 1; i <= rows; ++i) {
    row_of_column[0] = i;
    int j0 = 0;
    std::vector<double> min_value(cols + 1, kInfinity);
    std::vector<char> used(cols + 1, 0);

    // 沿最小 slack 一路走，直到碰到一个自由列。
    do {
      used[j0] = 1;
      const int i0 = row_of_column[j0];
      double delta = kInfinity;
      int j1 = 0;
      for (int j = 1; j <= cols; ++j) {
        if (used[j] != 0) {
          continue;
        }
        const double current = cost[i0 - 1][j - 1] - u[i0] - v[j];
        if (current < min_value[j]) {
          min_value[j] = current;
          way[j] = j0;
        }
        if (min_value[j] < delta) {
          delta = min_value[j];
          j1 = j;
        }
      }
      // ⚠️ 这一步是算法的心脏：把势整体平移 delta，使得刚选中的那条边
      //    的 slack 归零，同时**不破坏**已有匹配的最优性。
      //    漏掉 else 分支（未访问列的 min_value 也要减 delta）的话，
      //    算法仍然会终止、也仍然给出一个匹配 —— 只是**不是最优的**。
      //    那正是"大部分时候对、偶尔配错一对"的来源。
      for (int j = 0; j <= cols; ++j) {
        if (used[j] != 0) {
          u[row_of_column[j]] += delta;
          v[j] -= delta;
        } else {
          min_value[j] -= delta;
        }
      }
      j0 = j1;
    } while (row_of_column[j0] != 0);

    // 沿 way 回溯，把交替路径上的匹配翻转过来。
    do {
      const int j1 = way[j0];
      row_of_column[j0] = row_of_column[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> column_of_row(rows, -1);
  for (int j = 1; j <= cols; ++j) {
    if (row_of_column[j] > 0) {
      column_of_row[row_of_column[j] - 1] = j - 1;
    }
  }
  return column_of_row;
}

}  // namespace

std::vector<int> SolveAssignment(const std::vector<std::vector<double>> & cost)
{
  if (cost.empty()) {
    return {};
  }
  const int rows = static_cast<int>(cost.size());
  const int cols = static_cast<int>(cost[0].size());
  for (const auto & row : cost) {
    if (static_cast<int>(row.size()) != cols) {
      throw std::invalid_argument("SolveAssignment: 代价矩阵各行长度不一致");
    }
    for (const double value : row) {
      // ⚠️ 非有限值必须拦。算法内部做 `cost - u - v`，inf−inf = NaN，
      //    而 NaN 参与比较恒为 false ⟹ 最小值搜索静默跳过所有列，
      //    j1 保持 0，循环条件 `row_of_column[0] != 0` 恒真 ⟹ **死循环**。
      //    「不允许配对」要用 kForbiddenCost，不要用 inf。
      if (!std::isfinite(value)) {
        throw std::invalid_argument(
          "SolveAssignment: 代价矩阵含非有限值 —— 「不允许配对」请用 kForbiddenCost，"
          "用 inf 会让内部的 cost−u−v 变成 NaN，最小值搜索静默失效后死循环");
      }
    }
  }
  if (cols == 0) {
    return std::vector<int>(rows, -1);
  }

  std::vector<int> result;
  if (rows <= cols) {
    result = SolveTall(cost, rows, cols);
  } else {
    // 转置后求解再转回来 —— 算法要求 rows ≤ cols。
    std::vector<std::vector<double>> transposed(cols, std::vector<double>(rows));
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        transposed[j][i] = cost[i][j];
      }
    }
    const std::vector<int> row_of_column = SolveTall(transposed, cols, rows);
    result.assign(rows, -1);
    for (int j = 0; j < cols; ++j) {
      if (row_of_column[j] >= 0) {
        result[row_of_column[j]] = j;
      }
    }
  }

  // ⚠️ 拆掉被禁的配对。算法求的是**完美**匹配，它会为了凑满而选中
  //    kForbiddenCost 的边 —— 不拆的话一条航迹会被"配"给一个远在天边的
  //    检测，而那看起来只是"关联质量不好"。
  for (int i = 0; i < rows; ++i) {
    if (result[i] >= 0 && cost[i][result[i]] >= kForbiddenCost) {
      result[i] = -1;
    }
  }
  return result;
}

}  // namespace ads_perception
