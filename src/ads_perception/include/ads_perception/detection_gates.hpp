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

#ifndef ADS_PERCEPTION__DETECTION_GATES_HPP_
#define ADS_PERCEPTION__DETECTION_GATES_HPP_

// =============================================================================
//  检测准入门（物理先验，纯函数，无 ROS）
//
//  L-Shape 拟合出的框在进入跟踪之前要过两道**按物理先验**收的门（都不是按场景调的
//  补丁，理由与实测在各自参数的注释里）：
//    ① 剃刀条门 —— 既薄**又矮**的簇（CARLA 生成路面接缝的单环弧段残留）不是目标；
//    ② 浮空碎片门 —— 底离地**高**且竖向延展**扁**的簇（车顶远端环碎片）不是目标。
//  两道门都是「两个条件缺一不可」：只按薄收门会吞掉正对的盒状目标（车尾面 1.3 m 高、
//  行人 1.5 m —— L-Shape 量的是**可见剖面**，正对时深度只剩雷达噪声，实测 min(l,w)
//  p50 0.047）；只按高收门会吞掉墙后只露上半身的行人（底 1.0、延展 0.7）。
//
//  ⚠️ 2026-08-16 复审前它们内联在 perception_node 里，只有 L3-G 守着；抽成纯函数是为了
//     让每个阈值有毫秒级的 L1（test_detection_gates.cpp），门的边界（≥ 还是 >）写死在这里。
// =============================================================================

namespace ads_perception
{

/// 准入门参数（perception_params.yaml `cluster.*` 那几项，推导见 yaml）。
struct AdmissionParams
{
  /// 剃刀条门：min(长, 宽) 低于此值 **且** 竖向延展低于 razor_max_height_m 才剃，m。
  double razor_min_extent_m{0.1};
  /// 剃刀条门的竖向延展上限，m（接缝残留两环竖向散布之上、ODD 最矮目标锥桶 0.8 之下）。
  double razor_max_height_m{0.3};
  /// 浮空碎片门：簇底离地**高于**此值 **且** 竖向延展低于 floating_max_height_m 才剃，m。
  double floating_min_bottom_m{1.0};
  /// 浮空碎片门的竖向延展上限，m。
  double floating_max_height_m{0.3};
};

/// 一个框过门的结果。
enum class Admission
{
  kAccepted,          ///< 进跟踪
  kRazorStrip,        ///< 剃刀条（薄且矮）
  kFloatingFragment,  ///< 浮空碎片（高且扁）
};

/// 对一个 L-Shape 框做准入判定。
///
/// @param length_m 框长（≥ 宽，L-Shape 的约定），m
/// @param width_m 框宽，m
/// @param height_m 簇的竖向延展（z_max − z_min），m
/// @param bottom_above_ground_m 簇最低点离拟合地面的高度（n·p + d，n 朝上），m
/// @param params 门参数
/// @return 准入结果；两道门按顺序判（剃刀条优先 —— 与 node 里原来的顺序一致，诊断计数不变）
/// @throws std::invalid_argument 任一输入非有限（NaN 参与比较恒假 —— 门会**静默放行**，
///         所以必须显式拦，见 CLAUDE.md 陷阱表「用比较去拦非有限值」）
Admission AdmitDetection(
  double length_m, double width_m, double height_m, double bottom_above_ground_m,
  const AdmissionParams & params);

}  // namespace ads_perception

#endif  // ADS_PERCEPTION__DETECTION_GATES_HPP_
