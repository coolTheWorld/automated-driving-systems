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

#include "ads_map/opendrive_parser.hpp"

#include <tinyxml2.h>

#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ads_map
{

namespace
{

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

/// 统一的报错出口。把上下文（哪条路、哪一段）拼进异常信息 ——
/// 一张地图有上千个元素，只说「缺少属性」等于没说。
[[noreturn]] void fail(const std::string & where, const std::string & what)
{
  throw std::runtime_error("解析 OpenDRIVE 失败：" + where + " —— " + what);
}

const char * required_attr(const XMLElement * elem, const char * name, const std::string & where)
{
  const char * value = elem->Attribute(name);
  if (value == nullptr) {
    fail(where, std::string("缺少必需属性 ") + name);
  }
  return value;
}

double required_double(const XMLElement * elem, const char * name, const std::string & where)
{
  double value = 0.0;
  if (elem->QueryDoubleAttribute(name, &value) != tinyxml2::XML_SUCCESS) {
    fail(where, std::string("属性 ") + name + " 缺失或不是数字");
  }
  return value;
}

int required_int(const XMLElement * elem, const char * name, const std::string & where)
{
  int value = 0;
  if (elem->QueryIntAttribute(name, &value) != tinyxml2::XML_SUCCESS) {
    fail(where, std::string("属性 ") + name + " 缺失或不是整数");
  }
  return value;
}

double optional_double(const XMLElement * elem, const char * name, double fallback)
{
  double value = fallback;
  elem->QueryDoubleAttribute(name, &value);  // 失败时保持 fallback，符合语义
  return value;
}

ContactPoint parse_contact_point(const char * text, const std::string & where)
{
  const std::string value = (text == nullptr) ? "" : text;
  if (value == "start") {
    return ContactPoint::kStart;
  }
  if (value == "end") {
    return ContactPoint::kEnd;
  }
  fail(where, "contactPoint 只能是 start 或 end，实际是 \"" + value + "\"");
}

/// 从 PROJ 字符串里抠出一个 `+key=value` 的数值。
///
/// 只做这一件事，不引入完整的 PROJ 库：P1 需要的只是「地图原点的经纬度是多少」，
/// 用来跟 Gazebo 世界的 <spherical_coordinates> 对账。真正的投影换算要到
/// P4 接 GNSS 时才需要，那时再决定是引 PROJ 还是自己写横轴墨卡托。
double parse_proj_value(const std::string & proj, const std::string & key, double fallback)
{
  const std::string needle = "+" + key + "=";
  const std::size_t pos = proj.find(needle);
  if (pos == std::string::npos) {
    return fallback;
  }
  try {
    return std::stod(proj.substr(pos + needle.size()));
  } catch (const std::exception &) {
    return fallback;
  }
}

// ---------------------------------------------------------------------------
//  参考线
// ---------------------------------------------------------------------------

Geometry parse_geometry(const XMLElement * elem, const std::string & road_where)
{
  Geometry geom;
  geom.s0_m = required_double(elem, "s", road_where);

  const std::string where = road_where + " 的 <geometry s=" + std::to_string(geom.s0_m) + ">";

  geom.x_m = required_double(elem, "x", where);
  geom.y_m = required_double(elem, "y", where);
  geom.heading_rad = required_double(elem, "hdg", where);
  geom.length_m = required_double(elem, "length", where);

  if (geom.length_m <= 0.0) {
    fail(where, "length 必须为正，实际是 " + std::to_string(geom.length_m));
  }

  // 几何类型由**子元素**决定。这里是整个解析器最关键的一段：
  // 遇到不支持的类型必须炸，不能当没看见（见头文件里的说明）。
  const XMLElement * line = elem->FirstChildElement("line");
  const XMLElement * arc = elem->FirstChildElement("arc");

  if (line != nullptr && arc != nullptr) {
    fail(where, "同时含 <line> 和 <arc>，一段几何只能有一种类型");
  }
  if (line != nullptr) {
    geom.curvature_inv_m = 0.0;
    return geom;
  }
  if (arc != nullptr) {
    geom.curvature_inv_m = required_double(arc, "curvature", where + " 的 <arc>");
    if (geom.curvature_inv_m == 0.0) {
      // 曲率为 0 的圆弧就是直线，但写成 <arc> 说明生成方对自己的意图不清楚。
      // 放行的话，后面所有「按曲率判断直线/弯道」的代码都要多考虑一种情形。
      fail(where, "<arc> 的 curvature 为 0；直线请用 <line>");
    }
    return geom;
  }

  // 走到这里说明是 spiral / poly3 / paramPoly3，或者干脆没有类型子元素。
  std::string found;
  for (const XMLElement * child = elem->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement()) {
    if (!found.empty()) {
      found += ", ";
    }
    found += child->Name();
  }
  fail(
    where, "不支持的参考线几何类型 [" + (found.empty() ? std::string("无") : found) +
             "]。"
             "本项目只实现 line 与 arc（见 ads_map/geometry.hpp 的说明）。"
             "**这是有意的拒绝，不是缺陷** —— 静默跳过会让路网少一段而无人知晓。");
}

// ---------------------------------------------------------------------------
//  车道
// ---------------------------------------------------------------------------

Lane parse_lane(const XMLElement * elem, const std::string & section_where)
{
  Lane lane;
  lane.id = required_int(elem, "id", section_where);

  const std::string where = section_where + " 的 <lane id=" + std::to_string(lane.id) + ">";
  const char * type = elem->Attribute("type");
  lane.type = (type == nullptr) ? "" : type;

  const XMLElement * link = elem->FirstChildElement("link");
  if (link != nullptr) {
    const XMLElement * pred = link->FirstChildElement("predecessor");
    if (pred != nullptr) {
      lane.predecessor_id = required_int(pred, "id", where + " 的 <link><predecessor>");
    }
    const XMLElement * succ = link->FirstChildElement("successor");
    if (succ != nullptr) {
      lane.successor_id = required_int(succ, "id", where + " 的 <link><successor>");
    }
  }

  for (const XMLElement * w = elem->FirstChildElement("width"); w != nullptr;
       w = w->NextSiblingElement("width")) {
    LaneWidth width;
    width.s_offset_m = optional_double(w, "sOffset", 0.0);
    width.a = required_double(w, "a", where + " 的 <width>");
    width.b = optional_double(w, "b", 0.0);
    width.c = optional_double(w, "c", 0.0);
    width.d = optional_double(w, "d", 0.0);
    lane.widths.push_back(width);
  }

  // 中心车道（id 0）宽度恒为 0，规范里它就不带 <width>。其余车道必须有。
  if (lane.id != 0 && lane.widths.empty()) {
    fail(where, "可行驶车道必须至少有一条 <width>");
  }
  return lane;
}

LaneSection parse_lane_section(const XMLElement * elem, const std::string & road_where)
{
  LaneSection section;
  section.s0_m = optional_double(elem, "s", 0.0);

  const std::string where = road_where + " 的 <laneSection s=" + std::to_string(section.s0_m) + ">";

  const XMLElement * left = elem->FirstChildElement("left");
  if (left != nullptr) {
    for (const XMLElement * l = left->FirstChildElement("lane"); l != nullptr;
         l = l->NextSiblingElement("lane")) {
      section.left.push_back(parse_lane(l, where));
    }
  }
  const XMLElement * right = elem->FirstChildElement("right");
  if (right != nullptr) {
    for (const XMLElement * l = right->FirstChildElement("lane"); l != nullptr;
         l = l->NextSiblingElement("lane")) {
      section.right.push_back(parse_lane(l, where));
    }
  }

  if (section.left.empty() && section.right.empty()) {
    fail(where, "没有任何可行驶车道（left 和 right 都为空）");
  }
  return section;
}

// ---------------------------------------------------------------------------
//  道路
// ---------------------------------------------------------------------------

std::optional<RoadLink> parse_road_link(
  const XMLElement * link, const char * tag, const std::string & road_where)
{
  if (link == nullptr) {
    return std::nullopt;
  }
  const XMLElement * elem = link->FirstChildElement(tag);
  if (elem == nullptr) {
    return std::nullopt;
  }

  const std::string where = road_where + " 的 <link><" + tag + ">";
  RoadLink road_link;

  const std::string element_type = required_attr(elem, "elementType", where);
  if (element_type == "road") {
    road_link.element_type = ElementType::kRoad;
  } else if (element_type == "junction") {
    road_link.element_type = ElementType::kJunction;
  } else {
    fail(where, "elementType 只能是 road 或 junction，实际是 \"" + element_type + "\"");
  }

  road_link.element_id = required_int(elem, "elementId", where);

  const char * contact = elem->Attribute("contactPoint");
  if (contact != nullptr) {
    road_link.contact_point = parse_contact_point(contact, where);
  } else if (road_link.element_type == ElementType::kRoad) {
    // 连到一条具体道路却不说接哪一端，无法建图 —— 这种地图是坏的。
    fail(where, "elementType=road 时必须给出 contactPoint");
  }
  return road_link;
}

Road parse_road(const XMLElement * elem)
{
  Road road;
  road.id = required_int(elem, "id", "<road>");

  const std::string where = "<road id=" + std::to_string(road.id) + ">";
  const char * name = elem->Attribute("name");
  road.name = (name == nullptr) ? "" : name;
  road.length_m = required_double(elem, "length", where);
  road.junction_id = required_int(elem, "junction", where);

  const XMLElement * link = elem->FirstChildElement("link");
  road.predecessor = parse_road_link(link, "predecessor", where);
  road.successor = parse_road_link(link, "successor", where);

  const XMLElement * plan_view = elem->FirstChildElement("planView");
  if (plan_view == nullptr) {
    fail(where, "缺少 <planView>");
  }
  for (const XMLElement * g = plan_view->FirstChildElement("geometry"); g != nullptr;
       g = g->NextSiblingElement("geometry")) {
    road.geometries.push_back(parse_geometry(g, where));
  }
  if (road.geometries.empty()) {
    fail(where, "<planView> 里没有任何 <geometry>");
  }

  // 几何段必须首尾相接地覆盖 [0, length]。这里只查**起止与总长**，
  // 逐段的接续连续性由 P1-S1 的生成器测试保证 —— 那边是构造时保证的，
  // 这边只需要确认读进来的东西没被截断。
  const Geometry & last = road.geometries.back();
  const double covered = last.s0_m + last.length_m;
  if (std::abs(covered - road.length_m) > 1e-6) {
    fail(
      where, "几何段覆盖的总长 " + std::to_string(covered) + " m 与 <road length> 声明的 " +
               std::to_string(road.length_m) + " m 不一致");
  }

  const XMLElement * lanes = elem->FirstChildElement("lanes");
  if (lanes == nullptr) {
    fail(where, "缺少 <lanes>");
  }
  for (const XMLElement * s = lanes->FirstChildElement("laneSection"); s != nullptr;
       s = s->NextSiblingElement("laneSection")) {
    road.lane_sections.push_back(parse_lane_section(s, where));
  }
  if (road.lane_sections.empty()) {
    fail(where, "<lanes> 里没有任何 <laneSection>");
  }
  return road;
}

// ---------------------------------------------------------------------------
//  路口
// ---------------------------------------------------------------------------

Junction parse_junction(const XMLElement * elem)
{
  Junction junction;
  junction.id = required_int(elem, "id", "<junction>");

  const std::string where = "<junction id=" + std::to_string(junction.id) + ">";
  const char * name = elem->Attribute("name");
  junction.name = (name == nullptr) ? "" : name;

  for (const XMLElement * c = elem->FirstChildElement("connection"); c != nullptr;
       c = c->NextSiblingElement("connection")) {
    JunctionConnection conn;
    conn.id = required_int(c, "id", where + " 的 <connection>");

    const std::string conn_where = where + " 的 <connection id=" + std::to_string(conn.id) + ">";
    conn.incoming_road_id = required_int(c, "incomingRoad", conn_where);
    conn.connecting_road_id = required_int(c, "connectingRoad", conn_where);
    conn.contact_point = parse_contact_point(c->Attribute("contactPoint"), conn_where);

    for (const XMLElement * l = c->FirstChildElement("laneLink"); l != nullptr;
         l = l->NextSiblingElement("laneLink")) {
      LaneLink lane_link;
      lane_link.from_lane_id = required_int(l, "from", conn_where + " 的 <laneLink>");
      lane_link.to_lane_id = required_int(l, "to", conn_where + " 的 <laneLink>");
      conn.lane_links.push_back(lane_link);
    }
    if (conn.lane_links.empty()) {
      // 没有 laneLink 的连接在图上是一条**断掉的**边：路由会以为能走，
      // 但没有任何车道对应关系，下游取车道时会失败。
      fail(conn_where, "没有任何 <laneLink>，这条连接在车道图上无法落地");
    }
    junction.connections.push_back(conn);
  }
  return junction;
}

}  // namespace

RoadMap parse_opendrive(const std::string & xml_text, const std::string & source_name)
{
  const std::string src = source_name.empty() ? std::string("<字符串>") : source_name;

  XMLDocument doc;
  if (doc.Parse(xml_text.c_str(), xml_text.size()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error(
      "解析 OpenDRIVE 失败：" + src + " 不是合法的 XML —— " +
      std::string(doc.ErrorStr() == nullptr ? "（无详情）" : doc.ErrorStr()));
  }

  const XMLElement * root = doc.FirstChildElement("OpenDRIVE");
  if (root == nullptr) {
    throw std::runtime_error("解析 OpenDRIVE 失败：" + src + " 的根元素不是 <OpenDRIVE>");
  }

  RoadMap map;

  const XMLElement * header = root->FirstChildElement("header");
  if (header != nullptr) {
    const XMLElement * geo = header->FirstChildElement("geoReference");
    if (geo != nullptr && geo->GetText() != nullptr) {
      map.geo_reference.proj_string = geo->GetText();
      map.geo_reference.latitude_deg =
        parse_proj_value(map.geo_reference.proj_string, "lat_0", 0.0);
      map.geo_reference.longitude_deg =
        parse_proj_value(map.geo_reference.proj_string, "lon_0", 0.0);
    }
  }

  for (const XMLElement * r = root->FirstChildElement("road"); r != nullptr;
       r = r->NextSiblingElement("road")) {
    Road road = parse_road(r);
    const int id = road.id;
    if (!map.roads.emplace(id, std::move(road)).second) {
      throw std::runtime_error(
        "解析 OpenDRIVE 失败：" + src + " 里道路 id=" + std::to_string(id) + " 重复");
    }
  }
  if (map.roads.empty()) {
    throw std::runtime_error("解析 OpenDRIVE 失败：" + src + " 里没有任何 <road>");
  }

  for (const XMLElement * j = root->FirstChildElement("junction"); j != nullptr;
       j = j->NextSiblingElement("junction")) {
    Junction junction = parse_junction(j);
    const int id = junction.id;
    if (!map.junctions.emplace(id, std::move(junction)).second) {
      throw std::runtime_error(
        "解析 OpenDRIVE 失败：" + src + " 里路口 id=" + std::to_string(id) + " 重复");
    }
  }

  return map;
}

RoadMap load_opendrive(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("打不开 OpenDRIVE 文件：" + path);
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return parse_opendrive(buffer.str(), path);
}

}  // namespace ads_map
