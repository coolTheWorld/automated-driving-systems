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

#ifndef ADS_PERCEPTION__HUNGARIAN_HPP_
#define ADS_PERCEPTION__HUNGARIAN_HPP_

// =============================================================================
//  匈牙利算法（Kuhn-Munkres）：求代价最小的二分图完美匹配
//
//  用途：把上一帧的**航迹**与这一帧的**检测**配对，使总代价最小。
//
//  ## ⚠️ 为什么不用贪心最近邻
//
//  贪心（每条航迹各自认领最近的检测）在**目标靠近时会配错**，而配错的
//  后果不是"差一点"，是**两个目标的 ID 互换** —— 于是它们的速度估计
//  各自跳到对方的历史上，P6 会预测出两条交叉的荒谬轨迹。
//
//  一个具体的例子（两条航迹 A、B 与两个检测 1、2）：
//      d(A,1)=1.0  d(A,2)=1.1
//      d(B,1)=1.05 d(B,2)=5.0
//  贪心按顺序处理 A 时把 1 给了它，B 只好要 2，总代价 1.0+5.0=6.0；
//  最优是 A→2、B→1，总代价 1.1+1.05=2.15。**差了近三倍。**
//  这个用例在测试里有，它是本文件存在的理由。
//
//  ## 实现：O(n³) 的势函数（potential）版本
//
//  目标数在园区场景里只有几个到几十个，n³ 最多几万次操作 —— 所以
//  **实现的可读性优先于常数优化**，不引入 LAPJV 那类复杂加速。
//
//  ⚠️ 这个算法容易写错，而写错的表现是"大部分时候对、偶尔配错一对"。
//     所以判据必须与**穷举**对账（小规模全排列），而不是只测几个手算例子
//     —— 与 ads_map 的 Dijkstra 用穷举脚本对账是同一条规矩。
// =============================================================================

#include <limits>
#include <vector>

namespace ads_perception
{

/// 代价矩阵里表示「这一对不允许匹配」的哨兵值。
///
/// ⚠️ **不要用 `std::numeric_limits<double>::infinity()`**：算法内部要做
/// `cost - u - v` 的减法，`inf - inf = NaN`，而 NaN 参与比较恒为 false，
/// 于是最小值搜索会静默地跳过所有列，最终陷入死循环或给出乱七八糟的匹配。
/// 用一个"大到不可能被选中、但仍是有限数"的值。
constexpr double kForbiddenCost = 1.0e12;

/// 求最小代价匹配。
///
/// @param cost 代价矩阵，`cost[i][j]` = 第 i 行配第 j 列的代价。
///             允许**矩形**（行数 ≠ 列数）。不允许的配对填 `kForbiddenCost`。
/// @return 长度 = 行数；`result[i]` 是配给第 i 行的列号，**未配上时为 −1**。
/// @throws std::invalid_argument 矩阵不规整（各行长度不一），或含非有限值。
///
/// ⚠️ 代价 ≥ `kForbiddenCost` 的配对会在返回前被拆掉（置 −1）——
/// 算法本身求的是**完美**匹配，它会为了凑满而选中被禁的配对。
/// 不拆的话，一条航迹会被"配"给一个远在天边的检测，而那看起来只是
/// "关联质量不好"。
std::vector<int> SolveAssignment(const std::vector<std::vector<double>> & cost);

}  // namespace ads_perception

#endif  // ADS_PERCEPTION__HUNGARIAN_HPP_
