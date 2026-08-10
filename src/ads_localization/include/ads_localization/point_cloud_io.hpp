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

#ifndef ADS_LOCALIZATION__POINT_CLOUD_IO_HPP_
#define ADS_LOCALIZATION__POINT_CLOUD_IO_HPP_

// =============================================================================
//  极简 ASCII PCD 读取器
//
//  为什么自己写而不是拉 PCL：本包的 lib/ 是**纯 C++17，零 ROS 依赖**
//  （SPEC §3.3），而 PCL 是个几十 MB 的重型库，为了读一个 x/y/z 的文本文件
//  引入它不划算 —— 而且 `maps/campus_cloud.pcd` 是**我们自己生成的**，
//  格式完全受控（见 scripts/gen_map.py 的 render_cloud）。
//
//  ⚠️ 它**只支持本项目生成的那种子集**：ASCII、FIELDS 恰好是 x y z。
//     遇到二进制、别的字段组合、或缺字段一律**抛异常**，不做静默降级 ——
//     静默降级的症状是"点云读进来了但少了一半"，而 NDT 会照常收敛到一个错位姿。
// =============================================================================

#include <Eigen/Core>

#include <string>
#include <vector>

namespace ads_localization
{

/// 从 ASCII PCD 文件读点云。
///
/// @param path PCD 文件路径。
/// @return map 系下的点，单位 m。
/// @throws std::runtime_error 文件打不开、格式不支持、或声明点数与实际行数不符。
std::vector<Eigen::Vector3d> LoadPcdAscii(const std::string & path);

}  // namespace ads_localization

#endif  // ADS_LOCALIZATION__POINT_CLOUD_IO_HPP_
