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

#ifndef ADS_MAP__OPENDRIVE_PARSER_HPP_
#define ADS_MAP__OPENDRIVE_PARSER_HPP_

// =============================================================================
//  OpenDRIVE 解析 —— 纯 C++17，**不依赖 ROS**
//
//  实现的是规范的一个**子集**：参考线只支持 line 和 arc，见 geometry.hpp。
//
//  设计上只有一条铁律：**读不懂的东西一律抛异常，绝不静默跳过。**
//
//  为什么这条值得写成铁律
//  ----------------------
//  静默跳过一段读不懂的几何，结果是路网里少了一段路。而少一段路的症状是：
//  地图加载成功、RViz 画得出来、路由也返回一条路径 —— 只是那条路径绕了远路，
//  或者更糟，在缺口处「跳」过去了。没有任何一环会报错，
//  而你要花很久才会想到去数一数路网里到底有几条路。
//
//  把 tinyxml2 藏在实现文件里也是有意的：下游包只需要链接 ads_map，
//  不必知道我们用什么解析 XML，也不会因此在自己的编译期引入 tinyxml2 的头。
// =============================================================================

#include <string>

#include "ads_map/road_map.hpp"

namespace ads_map
{

/// @brief 从 OpenDRIVE 文本解析出路网。
///
/// @param xml_text  完整的 .xodr 文件内容。
/// @param source_name 出错信息里用来标识来源的名字（通常是文件路径）。
///                    传空字符串也可以，只是报错会难定位一点。
/// @return 解析出的路网。
///
/// @throw std::runtime_error XML 格式错误、必需字段缺失、
///        或遇到**不支持的参考线几何类型**（spiral / poly3 / paramPoly3）。
///        异常信息里会带上出问题的 road / geometry 的 id 与 s 值。
RoadMap parse_opendrive(const std::string & xml_text, const std::string & source_name);

/// @brief 从 .xodr 文件读取并解析路网。
///
/// @param path 文件路径。
/// @return 解析出的路网。
/// @throw std::runtime_error 文件打不开，或解析失败（见 parse_opendrive）。
RoadMap load_opendrive(const std::string & path);

}  // namespace ads_map

#endif  // ADS_MAP__OPENDRIVE_PARSER_HPP_
