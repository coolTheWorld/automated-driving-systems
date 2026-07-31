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

#ifndef ADS_MAP__ROAD_MAP_HPP_
#define ADS_MAP__ROAD_MAP_HPP_

// =============================================================================
//  路网数据模型 —— 纯 C++17，**不依赖 ROS**
//
//  字段命名刻意贴近 OpenDRIVE 的元素名（road / laneSection / lane / junction），
//  这样对着规范或对着 .xodr 文件读代码时不需要做心智翻译。
//  多一层「更漂亮的抽象」意味着多一处可能理解错的地方。
//
//  车道编号约定（这是 OpenDRIVE 规范定的，不是我们的选择）
//  ------------------------------------------------------
//      lane  0  参考线本身，宽度恒为 0，不可行驶
//      lane -1  参考线**右侧**第一条，沿 s 增大方向行驶
//      lane +1  参考线**左侧**第一条，逆 s 方向行驶
//
//  右侧通行下 −1 是顺行道、+1 是对向道。这个符号约定是 P1-S3 建**有向**
//  车道图的依据 —— 建成无向图的话，Dijkstra 会算出「掉头逆行更近」的路径，
//  而它在 RViz 里看起来是一条完全正常的平滑曲线。
// =============================================================================

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "ads_map/geometry.hpp"

namespace ads_map
{

/// @brief 车道宽度的三次多项式：w(ds) = a + b·ds + c·ds² + d·ds³。
///
/// 等宽车道只用到 a。高次项用于展宽段/渐变段 —— 本项目的地图目前不产生它们，
/// 但解析和求值都支持，因为 CARLA 生成的地图里很常见。
struct LaneWidth
{
  /// 该多项式生效的起点，相对**所属 laneSection 起点**的弧长，单位 m。
  double s_offset_m{0.0};
  double a{0.0};
  double b{0.0};
  double c{0.0};
  double d{0.0};

  /// @brief 求宽度。
  /// @param ds_m 相对**本多项式起点**（即 s_offset_m）的弧长，单位 m。
  /// @return 车道宽度，单位 m。
  double width_at(double ds_m) const;
};

/// @brief 一条车道。
struct Lane
{
  /// 车道编号，见文件头的约定。
  int id{0};
  /// OpenDRIVE 的车道类型字符串，如 "driving" / "none" / "sidewalk"。
  /// 保留原始字符串而不转成枚举：规范里的取值有二十多种且还在增加，
  /// 转枚举意味着每次遇到新值都要改代码，而我们只关心是不是 "driving"。
  std::string type;
  /// 宽度多项式，按 s_offset_m 递增排列。至少一个。
  std::vector<LaneWidth> widths;
  /// 车道级链接：本车道在**前驱/后继道路**上接的是哪条车道。
  /// 常规道路两端接的是路口时为空 —— 那种情况由 Junction 的 lane_links 描述。
  std::optional<int> predecessor_id;
  std::optional<int> successor_id;

  /// @brief 求本车道在给定处的宽度。
  /// @param ds_m 相对**所属 laneSection 起点**的弧长，单位 m。
  /// @return 车道宽度，单位 m。
  double width_at(double ds_m) const;
};

/// @brief 一个车道段。道路沿 s 方向可以分成若干段，段内车道数与编号不变。
struct LaneSection
{
  /// 本段起点在道路参考线上的弧长，单位 m。
  double s0_m{0.0};
  /// 左侧车道（id > 0），按 id 递增排列。
  std::vector<Lane> left;
  /// 右侧车道（id < 0），按 id 递减排列（−1, −2, …）。
  std::vector<Lane> right;

  /// @brief 按编号查车道。
  /// @return 找到返回指针，找不到返回 nullptr。id 为 0（中心车道）也返回 nullptr ——
  ///         中心车道宽度恒为 0，不参与任何几何计算。
  const Lane * find_lane(int lane_id) const;
};

/// @brief 链接目标的类型。
enum class ElementType
{
  kRoad,
  kJunction,
};

/// @brief 接触点：链接到目标道路的哪一端。
enum class ContactPoint
{
  kStart,
  kEnd,
};

/// @brief 道路的前驱/后继链接。
struct RoadLink
{
  ElementType element_type{ElementType::kRoad};
  int element_id{-1};
  /// 只有 element_type == kRoad 时有意义。
  std::optional<ContactPoint> contact_point;
};

/// @brief 一条道路。常规道路和路口内的连接道路都用它表示。
struct Road
{
  int id{-1};
  std::string name;
  /// 参考线总长，单位 m。
  double length_m{0.0};
  /// 所属路口 id。**−1 表示这不是连接道路**（OpenDRIVE 规定用 −1 而非省略）。
  int junction_id{-1};
  /// 参考线的几何段，按 s0_m 递增排列。
  std::vector<Geometry> geometries;
  /// 车道段，按 s0_m 递增排列。至少一个。
  std::vector<LaneSection> lane_sections;
  std::optional<RoadLink> predecessor;
  std::optional<RoadLink> successor;

  /// @brief 求参考线在弧长 s 处的位姿。
  /// @param s_m 沿参考线的弧长，单位 m。有效范围 [0, length_m]。
  /// @return 位姿，坐标为地图系（ENU）。
  /// @throw std::out_of_range 当 s_m 越界时。**故意不做静默截断** ——
  ///        截断会让「路由算出了一个超出道路长度的点」这种上游 bug
  ///        表现成「路径末端有一小段贴着路口不动」，极难定位。
  Pose2D reference_pose_at(double s_m) const;

  /// @brief 求某条车道的中心线在弧长 s 处的位姿。
  /// @param lane_id 车道编号，见文件头约定。0 直接返回参考线位姿。
  /// @param s_m 沿**参考线**的弧长，单位 m。有效范围 [0, length_m]。
  /// @return 位姿。航向与参考线相同 —— 等距偏移曲线处处平行于原曲线。
  /// @throw std::out_of_range s_m 越界；std::invalid_argument 车道不存在。
  ///
  /// @note 横向偏移 t 以参考线**左侧**为正，逐条累加内侧车道的宽度：
  ///           t = ±( Σ 内侧车道宽 + 本车道宽/2 )，右侧车道取负。
  ///       直接写 `sign·(|id|−0.5)·width` 只在**所有车道等宽**时成立，
  ///       这里按规范逐条累加，变宽车道也对。
  Pose2D lane_center_pose_at(int lane_id, double s_m) const;

  /// @brief 找到覆盖弧长 s 的车道段。
  /// @throw std::out_of_range 当 s_m 越界或没有任何车道段时。
  const LaneSection & lane_section_at(double s_m) const;
};

/// @brief 路口连接里的车道对应关系。
struct LaneLink
{
  /// 来路（incoming_road）上的车道编号。
  int from_lane_id{0};
  /// 连接道路上的车道编号。
  int to_lane_id{0};
};

/// @brief 路口内的一条连接：从某条来路经某条连接道路驶出。
struct JunctionConnection
{
  int id{-1};
  int incoming_road_id{-1};
  int connecting_road_id{-1};
  /// 连接道路的哪一端贴着来路。
  ContactPoint contact_point{ContactPoint::kStart};
  std::vector<LaneLink> lane_links;
};

/// @brief 一个路口。
struct Junction
{
  int id{-1};
  std::string name;
  std::vector<JunctionConnection> connections;
};

/// @brief 地图的地理参照。
///
/// 把地图局部坐标 (0,0) 钉在一个真实经纬度上。
/// ⚠️ 必须与 Gazebo 世界的 `<spherical_coordinates>` 一致，
///    否则两个仿真环境里同一个 x/y 会换算出不同经纬度（SPEC §4.1）。
struct GeoReference
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  /// 原始 PROJ 字符串，保留以备将来接入真正的投影库。
  std::string proj_string;
};

/// @brief 一张完整的路网。
struct RoadMap
{
  GeoReference geo_reference;
  /// 按 id 索引。用 std::map 而不是 vector：OpenDRIVE 的 id 不保证连续，
  /// 也不保证从 0 开始（本项目的连接道路就是从 10 起编号的）。
  std::map<int, Road> roads;
  std::map<int, Junction> junctions;

  /// @brief 按 id 取道路。
  /// @throw std::out_of_range 找不到时。**不返回 nullptr** ——
  ///        地图里查不到一个被引用的 id 说明地图本身坏了，
  ///        这种情况应当立刻停，而不是让空指针传播到下游。
  const Road & road(int road_id) const;

  /// @brief 按 id 取路口。
  /// @throw std::out_of_range 找不到时。
  const Junction & junction(int junction_id) const;
};

}  // namespace ads_map

#endif  // ADS_MAP__ROAD_MAP_HPP_
