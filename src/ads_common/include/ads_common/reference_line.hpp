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

#ifndef ADS_COMMON__REFERENCE_LINE_HPP_
#define ADS_COMMON__REFERENCE_LINE_HPP_

// =============================================================================
//  参考线 —— 折线的弧长参数化、逐点曲率、最近点投影。纯 C++17，**不依赖 ROS**
//
//  一串位姿（/route/path）只告诉你「路在哪」。上层真正要的是
//  「我离它多远、车头偏了多少、这里弯多急、还剩多长」。这一层负责把后者算出来。
//
//  **两个消费者，所以它在 ads_common 而不在某一个模块里**（P3-S1 下沉，
//  在此之前它叫 ads_control::TrackedPath）：
//
//    - ads_control：投影结果就是 Stanley 的横向/航向误差
//    - ads_planning：同一个投影结果就是 Frenet 的 (s, d)，
//                    lateral_error_m 即 d，heading_error_rad 用来推 d′
//
//  规划包**不能**反过来依赖控制包（方向反了），而这一层有实打实的逻辑
//  （弧长累加、曲率差分、局部搜索），复制两份必然漂移 ——
//  这与 Pose2D 那种「没有逻辑的 POD 复制两份无所谓」是两回事，见下面的 note。
//
//  完整推导见 docs/modules/control.md §2。这里只重复三条**做错了不会报错**的：
//
//  1. 弧长必须按**实际点距**累加，不能用「序号 × 采样步长」。
//     map_node 的 0.5 m 走的是**道路参考线**弧长，而路径是**车道中心线**，
//     两者差一个因子 (1 − t·κ)。本项目地图上弯道外侧的实际点距是 0.5729 m，
//     大 14.58%。用 0.5 的后果：曲率偏大 14.58% → 弯道限速偏小 6.58%，
//     车明显开得慢，而没有任何一层报错。它在直路上完全正确 ——
//     这正是它危险的地方。
//
//  2. 角度差必须过 ads_common::angle_diff。路径穿过朝向 ±π 的路段
//     （本地图上就是所有朝西行驶的段）时，裸差会得到 ≈ ∓2π，
//     除以 0.57 m 得到曲率 ≈ ∓11 1/m（半径 9 cm），
//     速度剖面立刻把限速压到 0.37 m/s —— **车在直路上爬行**，
//     而单元测试大概率不跨 ±π，全绿。
//
//  3. 最近点用**局部搜索**，不是为了快，是为了对。环线上自车前后必然存在
//     几何距离相近的两段路径（刚驶出路口时尤其明显），全局最近点会在两者
//     之间跳，症状是**转角瞬间打死**。
// =============================================================================

#include <cstddef>
#include <optional>
#include <vector>

namespace ads_common
{

/// @brief 平面位姿。坐标系为地图系（ENU：x 东、y 北、z 上），与 ROS 的 `map` 一致。
///
/// @note 与 `ads_map::Pose2D` **有意保持为两份**。地图和控制/规划是 SPEC §3.3
///       意义上的不同模块，只通过 ROS 话题通信 —— 让下游编译期依赖地图包，
///       等于把「换一个路径来源」的成本提高到改依赖树。
///       代价是这个三字段 POD 有两份，这是有意接受的重复：**它没有逻辑，
///       不存在"改出分歧"的可能**。
///
///       分界线就在这里：**没有逻辑的类型可以复制，有逻辑的必须共用。**
///       所以 Pose2D 复制两份、而 `ReferenceLine` 和 `angle_diff` 下沉到本包。
struct Pose2D
{
  double x_m{0.0};
  double y_m{0.0};
  /// 航向角，单位**弧度**。x 轴正方向为 0，逆时针为正（REP-103）。
  double heading_rad{0.0};
};

/// @brief 路径上的一个采样点，含预处理算出来的弧长与曲率。
struct PathPoint
{
  double x_m{0.0};
  double y_m{0.0};
  /// 路径在该点的切线方向，单位弧度。
  double heading_rad{0.0};
  /// 从路径起点算起的**累计弧长**，单位 m。按实际点距累加（见文件头第 1 条）。
  double s_m{0.0};
  /// 曲率 1/R，单位 1/m。**正 = 左转（逆时针）**，直线为 0。
  ///
  /// @note 符号约定与 `docs/modules/map_and_routing.md` 一致。命名带 `curvature`
  ///       而不是光秃秃的 `k_`，是因为 Stanley 的横向增益也叫 k ——
  ///       两个含义不同的 k 出现在同一个表达式里迟早出事。
  double curvature_inv_m{0.0};
};

/// @brief 把一个查询位姿投影到路径上的结果。
///
/// 所有量都在**投影点**处求值，而不是在最近的采样点处 —— 采样点间距 0.57 m，
/// 直接取最近采样点会带来最多半个点距（0.29 m）的量化误差，
/// 而那正是横向误差判据（0.30 m）的量级。
struct PathProjection
{
  /// 投影落在线段 `[index, index + 1]` 上。下一拍把它当 `hint` 传回来。
  std::size_t index{0};
  /// 投影点在该线段上的归一化位置，落在 [0, 1]。
  double ratio{0.0};

  double x_m{0.0};
  double y_m{0.0};
  /// 路径在投影点处的切线方向，单位弧度。
  double heading_rad{0.0};
  /// 投影点的累计弧长，单位 m。
  double s_m{0.0};
  /// 投影点的曲率，单位 1/m，正 = 左转。
  double curvature_inv_m{0.0};

  /// **横向误差**，单位 m。查询点在路径**左侧为正**（docs/modules/control.md §0）。
  ///
  /// @note 教科书的 Stanley 用右正，所以本项目的控制律里会出现一个减号。
  ///       选左正是为了和仓库其余部分一致：地图的横向偏移 t 是左正、
  ///       ROS 的 y 轴是左、转角 δ 是左正。
  double lateral_error_m{0.0};
  /// **航向误差** `ψ = angle_diff(查询朝向, 路径切向)`，单位 rad，落在 [−π, π]。
  double heading_error_rad{0.0};
};

/// @brief 一条经过预处理的参考线。
///
/// 构造时一次性算出全部点的累计弧长与曲率，之后只做查询。
/// **不可变**：路径变了就换一个对象，不提供原地更新 —— 原地更新会让
/// 「上一拍的索引」这类缓存状态跨越两条不同的路径，那是一个很难查的错。
class ReferenceLine
{
public:
  /// @brief 从位姿序列构建。
  ///
  /// @param poses 路径点，至少 2 个，全部字段必须有限，相邻点距必须大于 `kMinSpacingM`。
  /// @throw std::invalid_argument 点数不足、含非有限值，或存在重合/近重合的相邻点。
  ///
  /// @note **重合点是抛异常而不是跳过**。跳过的话，一条含重合点的路径会被
  ///       静默接受，而重合点通常意味着上游采样有 bug（P1 就出过一次：
  ///       `ceil(span/step)` 让路径最后两个点重合，RViz 里完全看不出来）。
  ///       静默修复会让那个 bug 永远留在上游。
  ///
  /// @note **非有限值单独判**，因为 NaN 参与任何比较都返回 false，
  ///       点距检查对它恒为假、会原样放行。而且要拦 **±inf 不只是 NaN**。
  ///       本仓库已吃过两次同源的亏，见 CLAUDE.md 陷阱表「用比较去拦非有限值」。
  explicit ReferenceLine(std::vector<Pose2D> poses);

  /// @brief 全部路径点（含预处理结果）。
  const std::vector<PathPoint> & points() const noexcept { return points_; }

  /// @brief 路径总长度，单位 m（= 最后一点的累计弧长）。
  double length_m() const noexcept { return points_.back().s_m; }

  /// @brief 把查询位姿投影到路径上。
  ///
  /// @param query 查询位姿。做 Stanley 时这里传的是**前轴**中心的位置
  ///              加**车身**航向（前轴换算见 docs/modules/control.md §3.2）。
  /// @param hint  上一拍返回的 `index`。传 `std::nullopt` 表示全局搜索。
  /// @param window 局部搜索的半窗口，单位**点数**。仅在 `hint` 有值时生效。
  /// @return 投影结果。
  ///
  /// @note **`hint` 不是性能优化，是正确性要求。** 环线上自车前后必然存在
  ///       几何距离相近的两段路径，全局最近点会在两者之间跳变，
  ///       症状是转角瞬间打死。只有首次跟踪一条新路径时才该传 `nullopt`。
  ///
  /// @note ⚠️ **查询点越过路径终点之后，`lateral_error_m` 会趋于 0，
  ///       而车其实已经开出去很远了。**
  ///
  ///       原因：投影被夹到端点，而 `lateral_error_m` 按定义只取偏移量的
  ///       **横向分量**，纵向那一份被丢掉了。车沿着路径末端切线一直开，
  ///       横向分量始终是 0 —— 一个只看横向误差的上层会认为「跟得很好」。
  ///
  ///       **所以「到达终点 / 冲过终点」必须用 `s_m` 和 `ratio` 判，
  ///       不能用 `lateral_error_m`。** 判据是 `ratio == 1 且 index 是最后一段`。
  ///       本层不做特殊处理是有意的：怎么算「到达」是策略（S4 的
  ///       `goal.stop_distance_m`），不是几何。
  ///       行为由 `ClampsBeyondBothEnds` 用例钉住。
  PathProjection project(
    const Pose2D & query, std::optional<std::size_t> hint = std::nullopt,
    std::size_t window = kDefaultSearchWindow) const;

  /// 相邻点距的下限，单位 m。小于它视为重合点。
  ///
  /// 取 1 mm：远小于任何有意义的路径采样步长（本项目 0.5 m），
  /// 又远大于浮点噪声，不会把正常路径误判成退化路径。
  static constexpr double kMinSpacingM = 1e-3;

  /// 局部搜索的默认半窗口，单位点数。
  ///
  /// 0.5729 m/点 × 30 ≈ 17 m。依据：控制一拍（20 ms）最多走 8.333 × 0.02 = 0.17 m，
  /// 也就是不到半个点 —— 30 个点的窗口留了两个数量级的余量，
  /// 足够吸收调度抖动和短暂的 TF 延迟。
  /// 规划一拍（100 ms）走 0.83 m ≈ 1.5 个点，同样远在窗口内。
  /// 调小 → 车速高或某一拍卡顿时可能"追不上"路径；
  /// 调大 → 退化成全局搜索，重新引入跨段跳变。
  static constexpr std::size_t kDefaultSearchWindow = 30;

private:
  std::vector<PathPoint> points_;
};

}  // namespace ads_common

#endif  // ADS_COMMON__REFERENCE_LINE_HPP_
