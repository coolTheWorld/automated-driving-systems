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

#ifndef ADS_COMMON__NUMERIC_CHECKS_HPP_
#define ADS_COMMON__NUMERIC_CHECKS_HPP_

// =============================================================================
//  入参校验的三个小工具
//
//  为什么值得单独抽一个文件、又为什么在 ads_common：
//  **有逻辑、且有多个消费者。** 起初只有 ads_control 里三处（Stanley、速度剖面、
//  速度环），P3-S4 之后变成跨两个包五处（再加 quintic、lattice）——
//  而这件事本身是本仓库**已经吃过两次亏**的那一类：
//
//      NaN 参与**任何**比较都返回 false。
//      所以 `if (x <= 0.0) throw;` 对 NaN **恒为假**，一条都拦不住。
//
//  各写各的必然有人只写半句。写在一处、名字里带上 "Finite"，
//  就没法在不看见 isfinite 的情况下把它用错。（见 CLAUDE.md 陷阱表
//  「用比较去拦非有限值」—— 那条已经咬过 vehicle_cmd_bridge 和 ads_control 两次。）
//
//  ⚠️ ±inf 也要拦，不只是 NaN：本项目的 gpu_lidar 无回波射线返回的就是 ±inf，
//     而 inf 能通过 `> 0` 这类检查。
// =============================================================================

#include <cmath>
#include <stdexcept>
#include <string>

namespace ads_common
{

/// @brief 要求"有限"（正负都合法）。用于运行期入参，如误差、车速、dt。
inline void RequireFinite(double value, const char * what, const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(
      std::string(what) + ": " + name + " 非有限（" + std::to_string(value) +
      "）。坏数据在上游，不要让它污染下游的持久状态。");
  }
}

/// @brief 要求"有限且严格为正"。用于增益、限值这类**零值即奇点**的参数。
inline void RequireFinitePositive(double value, const char * what, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(
      std::string(what) + "::" + name + " 必须是有限正数，收到 " + std::to_string(value) +
      "。是不是配置里漏了这一项（聚合初始化会把漏掉的项填 0）？");
  }
}

/// @brief 要求"有限且非负"。用于 `K_i` 这类**零值是合法默认**的参数。
///
/// @note 与上一个分开是有意的：把 `K_i = 0` 也拦掉的话，
///       control.md §4.4「初值 K_i = 0」就成了一个跑不起来的建议。
inline void RequireFiniteNonNegative(double value, const char * what, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(
      std::string(what) + "::" + name + " 必须是有限非负数，收到 " + std::to_string(value) + "。");
  }
}

}  // namespace ads_common

#endif  // ADS_COMMON__NUMERIC_CHECKS_HPP_
