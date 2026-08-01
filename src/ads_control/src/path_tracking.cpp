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

#include "ads_control/path_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "ads_common/angles.hpp"

namespace ads_control
{

namespace
{

/// 线段退化判据，单位 m²（比较的是长度平方，省一次开方）。
/// 与 TrackedPath::kMinSpacingM 同源：构造时已经拒掉了更短的段，
/// 这里只是投影计算里的除零保护，属于「不该发生但发生了也不能崩」。
constexpr double kMinSegmentLengthSquaredM2 = TrackedPath::kMinSpacingM * TrackedPath::kMinSpacingM;

}  // namespace

TrackedPath::TrackedPath(std::vector<Pose2D> poses)
{
  // ---------------------------------------------------------------------------
  //  校验
  // ---------------------------------------------------------------------------
  // 少于 2 个点连一段切线都定义不出来，更谈不上曲率。
  // 返回一个"空路径"对象让调用方去判空，等于把这个错误往下游推一层 ——
  // 而下游最可能的表现是「车不动，没有任何日志」。
  if (poses.size() < 2) {
    throw std::invalid_argument(
      "TrackedPath 至少需要 2 个点，收到 " + std::to_string(poses.size()) +
      " 个。上游路径是空的还是只发了起点？");
  }

  // ⚠️ 非有限值必须**单独判**，不能指望后面的比较拦住它。
  //
  //    NaN 参与任何比较都返回 false，所以下面那句 `spacing_m < kMinSpacingM`
  //    对 NaN **恒为假** —— 校验会原样放行，`s_m` 全变 NaN，曲率全变 NaN，
  //    一路传到转角。本仓库已经吃过两次同源的亏：vehicle_cmd_bridge 的指令
  //    限幅，和这里。已提升进 CLAUDE.md 陷阱表「用比较去拦非有限值」。
  //
  //    放行的后果不是崩溃，而是**误诊**：NaN 转角会被 vehicle_cmd_bridge 的
  //    isfinite 检查挡下并触发看门狗刹停，于是现场是「车自己停了，
  //    日志说收到非有限指令」，所有人都会去查控制器 —— 而错在路径里。
  //    在这里判掉，报的就是「第 N 个路径点是非有限值」。
  for (std::size_t i = 0; i < poses.size(); ++i) {
    if (
      !std::isfinite(poses[i].x_m) || !std::isfinite(poses[i].y_m) ||
      !std::isfinite(poses[i].heading_rad)) {
      throw std::invalid_argument(
        "路径第 " + std::to_string(i) + " 个点含非有限值（x=" + std::to_string(poses[i].x_m) +
        ", y=" + std::to_string(poses[i].y_m) + ", yaw=" + std::to_string(poses[i].heading_rad) +
        "）。上游发的路径本身就是坏的，不要往下传。");
    }
  }

  points_.resize(poses.size());
  for (std::size_t i = 0; i < poses.size(); ++i) {
    points_[i].x_m = poses[i].x_m;
    points_[i].y_m = poses[i].y_m;
    points_[i].heading_rad = poses[i].heading_rad;
  }

  // ---------------------------------------------------------------------------
  //  累计弧长：按**实际点距**，不是「序号 × 采样步长」
  // ---------------------------------------------------------------------------
  // 见头文件第 1 条。这里多写一行 hypot 换掉一次乘法，买的是
  // 「弯道上曲率不偏 14.58%」—— 而那个偏差不会有任何一层报错。
  points_[0].s_m = 0.0;
  for (std::size_t i = 1; i < points_.size(); ++i) {
    const double dx_m = points_[i].x_m - points_[i - 1].x_m;
    const double dy_m = points_[i].y_m - points_[i - 1].y_m;
    const double spacing_m = std::hypot(dx_m, dy_m);

    // 重合点显式报错。理由见头文件：静默跳过会把上游的采样 bug 永久掩盖。
    if (spacing_m < kMinSpacingM) {
      throw std::invalid_argument(
        "路径第 " + std::to_string(i - 1) + " 与第 " + std::to_string(i) + " 点几乎重合（相距 " +
        std::to_string(spacing_m) + " m < " + std::to_string(kMinSpacingM) +
        " m）。上游采样多算了一步？");
    }
    points_[i].s_m = points_[i - 1].s_m + spacing_m;
  }

  // ---------------------------------------------------------------------------
  //  曲率：中心差分 κ = Δθ / Δs
  // ---------------------------------------------------------------------------
  // 内点用中心差分（对圆弧二阶精确），两端只能用单侧差分（一阶）。
  //
  // ⚠️ 角度差**必须**过 angle_diff。裸写 θ[i+1] − θ[i−1] 在路径穿过
  //    朝向 ±π 的地方会得到 ≈ ∓2π，算出半径 9 cm 的假弯，
  //    速度剖面立刻把车压到爬行速度。见头文件第 2 条。
  const std::size_t last = points_.size() - 1;
  for (std::size_t i = 0; i < points_.size(); ++i) {
    const std::size_t lo = (i == 0) ? 0 : i - 1;
    const std::size_t hi = (i == last) ? last : i + 1;
    const double d_heading_rad =
      ads_common::angle_diff(points_[lo].heading_rad, points_[hi].heading_rad);
    const double d_s_m = points_[hi].s_m - points_[lo].s_m;
    // d_s_m 恒 > 0：上面已经拒掉了所有小于 kMinSpacingM 的段。
    points_[i].curvature_inv_m = d_heading_rad / d_s_m;
  }
}

PathProjection TrackedPath::project(
  const Pose2D & query, std::optional<std::size_t> hint, std::size_t window) const
{
  const std::size_t segment_count = points_.size() - 1;

  // ---------------------------------------------------------------------------
  //  搜索范围
  // ---------------------------------------------------------------------------
  // 有 hint 就只在它附近找。这**不是性能优化** —— 环线上自车前后必然存在
  // 几何距离相近的两段路径，全局最近点会在两者之间跳，转角瞬间打死。
  std::size_t first_segment = 0;
  std::size_t last_segment = segment_count - 1;
  if (hint.has_value()) {
    const std::size_t centre = std::min(*hint, segment_count - 1);
    first_segment = (centre > window) ? centre - window : 0;
    last_segment = std::min(centre + window, segment_count - 1);
  }

  // ---------------------------------------------------------------------------
  //  逐段投影，取最近
  // ---------------------------------------------------------------------------
  // 对**线段**投影而不是取最近的采样点：采样点间距 0.57 m，取最近点会引入
  // 最多 0.29 m 的量化误差 —— 那正是横向误差判据（0.30 m）的量级。
  std::size_t best_index = first_segment;
  double best_ratio = 0.0;
  double best_distance_squared_m2 = std::numeric_limits<double>::infinity();

  for (std::size_t i = first_segment; i <= last_segment; ++i) {
    const double seg_dx_m = points_[i + 1].x_m - points_[i].x_m;
    const double seg_dy_m = points_[i + 1].y_m - points_[i].y_m;
    const double seg_length_squared_m2 = seg_dx_m * seg_dx_m + seg_dy_m * seg_dy_m;

    // 构造函数已保证不会走到这里，但除零保护不该依赖"别人已经检查过了"。
    double ratio = 0.0;
    if (seg_length_squared_m2 > kMinSegmentLengthSquaredM2) {
      const double to_query_dx_m = query.x_m - points_[i].x_m;
      const double to_query_dy_m = query.y_m - points_[i].y_m;
      ratio = (to_query_dx_m * seg_dx_m + to_query_dy_m * seg_dy_m) / seg_length_squared_m2;
      // 夹到 [0, 1]：投影落在线段外时，最近点就是端点。
      ratio = std::clamp(ratio, 0.0, 1.0);
    }

    const double foot_x_m = points_[i].x_m + ratio * seg_dx_m;
    const double foot_y_m = points_[i].y_m + ratio * seg_dy_m;
    const double dx_m = query.x_m - foot_x_m;
    const double dy_m = query.y_m - foot_y_m;
    const double distance_squared_m2 = dx_m * dx_m + dy_m * dy_m;

    if (distance_squared_m2 < best_distance_squared_m2) {
      best_distance_squared_m2 = distance_squared_m2;
      best_index = i;
      best_ratio = ratio;
    }
  }

  // ---------------------------------------------------------------------------
  //  在投影点处插值
  // ---------------------------------------------------------------------------
  const PathPoint & p0 = points_[best_index];
  const PathPoint & p1 = points_[best_index + 1];

  PathProjection result;
  result.index = best_index;
  result.ratio = best_ratio;
  result.x_m = p0.x_m + best_ratio * (p1.x_m - p0.x_m);
  result.y_m = p0.y_m + best_ratio * (p1.y_m - p0.y_m);
  result.s_m = p0.s_m + best_ratio * (p1.s_m - p0.s_m);
  // 曲率是普通标量，直接线性插值。
  result.curvature_inv_m =
    p0.curvature_inv_m + best_ratio * (p1.curvature_inv_m - p0.curvature_inv_m);
  // 朝向**不是**普通标量：跨 ±π 时线性插值会插到反方向去。
  // 先取最短角度差再按比例加，最后归一化。
  result.heading_rad = ads_common::normalize_angle(
    p0.heading_rad + best_ratio * ads_common::angle_diff(p0.heading_rad, p1.heading_rad));

  // ---------------------------------------------------------------------------
  //  横向误差与航向误差
  // ---------------------------------------------------------------------------
  // e = 把 (查询点 − 投影点) 投到路径切向的**左**法向 (−sinθ, cosθ) 上。
  //
  // 用这个写法而不是叉积的 z 分量，是因为叉积还要记「要不要取负」，
  // 而这一式一眼能验：路径朝 +x（θ=0）时法向是 (0, 1)，
  // 查询点在 y = +1 处得 e = +1 —— 左侧为正，对上了。
  const double sin_heading = std::sin(result.heading_rad);
  const double cos_heading = std::cos(result.heading_rad);
  result.lateral_error_m =
    -sin_heading * (query.x_m - result.x_m) + cos_heading * (query.y_m - result.y_m);

  // ψ = θ_path − θ_vehicle，取最短。裸减会在跨 ±π 时给出 ≈ ±2π，
  // Stanley 据此把方向盘打死 —— 这正是 ads_common 存在的理由。
  result.heading_error_rad = ads_common::angle_diff(query.heading_rad, result.heading_rad);

  return result;
}

}  // namespace ads_control
