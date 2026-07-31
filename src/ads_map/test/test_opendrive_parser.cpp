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

// =============================================================================
//  OpenDRIVE 解析的 L1 测试
//
//  最后一个用例 LaneCentresMatchThePythonGenerator 是 **P1 的 CP-P1-A 检查点**。
//
//  为什么要拿两套实现对账，各自的单元测试还不够
//  --------------------------------------------
//  scripts/gen_map.py（Python）和本包（C++）是同一份几何的两套独立实现。
//  如果两边对 OpenDRIVE 的约定理解不一致 —— 曲率的符号、车道横向偏移的正方向、
//  圆弧起始朝向的定义 —— **两边各自的单元测试都会通过**，因为各自都自洽。
//
//  不做这个对账的话，症状要等到 P2 才出现：车沿路径开偏了，
//  而你第一反应一定是去调 Stanley 的增益。那时候你调的是一个正确的控制器。
//
//  ⚠️ 这个检查的价值完全建立在「两套实现相互独立」上。
//     哪天要是有人让 C++ 侧改成读 CSV 来省事，对账就变成自己跟自己比，
//     而它看起来仍然是绿的 —— 那比没有这个检查更糟。
// =============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ads_map/opendrive_parser.hpp"

namespace
{

/// 逐点对账的判据：**1 mm**（P1 计划里 CP-P1-A 定的数）。
///
/// 判据要够严到任何**约定层面**的理解分歧都会立刻炸：
/// 横向偏移方向搞反 = 差一个车道宽 3.5 m，曲率符号搞反 = 差数米，
/// 都比 1 mm 大四个数量级以上。
///
/// 实测残差 **0.00073 mm**，余量约 1400 倍。
///
/// ⚠️ 这个残差已经不是「两套实现的分歧」了，而是**采样基准 CSV 自己的文本精度**：
///    reference_samples.csv 的 x/y/heading 都写 6 位小数，舍入上限 5e-7，
///    位置上合成起来正好 ≈ 7.3e-7 m。也就是说两边的一致性已经到了
///    这个比对能分辨的极限，再往下就是在量 CSV 的格式而不是量代码。
///
///    **所以不要试图靠调紧判据来「提高严格度」** —— 那只会撞上格式下限而误报。
///    真要提高分辨率，得先把 CSV 的精度提上去；但目前没有值得分辨的东西。
///
/// 曾经踩过的坑：.xodr 里的 curvature 和 hdg 一度也只写 6 位小数，
/// 那时朝向余量只剩 1.5 倍 —— 一个随时会因为改地图而误报的脆弱判据。
/// 根因是「同一个格式串套不同量纲」，详见 scripts/gen_map.py 的 precise()。
constexpr double kCrossCheckTolM = 1e-3;

/// 朝向的对账判据。实测残差 5.0e-7 rad（同样是 CSV 的格式下限），余量约 20 倍。
constexpr double kCrossCheckTolRad = 1e-5;

const ads_map::RoadMap & campus_map()
{
  // 静态局部变量：整个测试进程只解析一次。地图不大（24 KB），
  // 但每个用例都重新解析会让「L1 测试保持毫秒级」这条越来越难守。
  static const ads_map::RoadMap map = ads_map::load_opendrive(ADS_MAP_XODR_PATH);
  return map;
}

/// 拼一个最小可用的 .xodr，planView 的内容由调用方给。
/// 用于测试各种畸形输入 —— 它们不该出现在真实地图里，所以只能手工构造。
std::string minimal_xodr(const std::string & plan_view_body, double road_length = 10.0)
{
  std::ostringstream out;
  out << "<?xml version=\"1.0\"?>\n"
      << "<OpenDRIVE>\n"
      << "  <header revMajor=\"1\" revMinor=\"6\"/>\n"
      << "  <road name=\"r\" length=\"" << road_length << "\" id=\"1\" junction=\"-1\">\n"
      << "    <planView>\n"
      << plan_view_body << "    </planView>\n"
      << "    <lanes>\n"
      << "      <laneSection s=\"0.0\">\n"
      << "        <center><lane id=\"0\" type=\"none\"/></center>\n"
      << "        <right><lane id=\"-1\" type=\"driving\">"
      << "<width sOffset=\"0.0\" a=\"3.5\"/></lane></right>\n"
      << "      </laneSection>\n"
      << "    </lanes>\n"
      << "  </road>\n"
      << "</OpenDRIVE>\n";
  return out.str();
}

struct SampleRow
{
  int road_id{0};
  int lane_id{0};
  double s_m{0.0};
  double x_m{0.0};
  double y_m{0.0};
  double heading_rad{0.0};
};

/// 读取 Python 生成器导出的车道中心线采样基准。
std::vector<SampleRow> load_samples(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    return {};  // 交给调用方断言，这里不抛 —— 测试里报错信息更有用
  }

  std::vector<SampleRow> rows;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("road_id", 0) == 0) {
      continue;  // 注释行与表头
    }
    std::istringstream ss(line);
    std::string cell;
    SampleRow row;
    // 顺序与 CSV 表头一致：road_id,lane_id,s_m,x_m,y_m,heading_rad
    std::getline(ss, cell, ',');
    row.road_id = std::stoi(cell);
    std::getline(ss, cell, ',');
    row.lane_id = std::stoi(cell);
    std::getline(ss, cell, ',');
    row.s_m = std::stod(cell);
    std::getline(ss, cell, ',');
    row.x_m = std::stod(cell);
    std::getline(ss, cell, ',');
    row.y_m = std::stod(cell);
    std::getline(ss, cell, ',');
    row.heading_rad = std::stod(cell);
    rows.push_back(row);
  }
  return rows;
}

}  // namespace

// ---------------------------------------------------------------------------
//  结构
// ---------------------------------------------------------------------------

TEST(OpenDriveParser, LoadsTheCampusMap)
{
  const ads_map::RoadMap & map = campus_map();
  // 3 条常规路 + 12 条连接路。数目由 P1-S1 的生成器测试独立保证，
  // 这里比对是为了确认**解析过程没有漏掉任何一条** ——
  // 漏了的症状是路网少一段而无人报错。
  EXPECT_EQ(map.roads.size(), 15u);
  EXPECT_EQ(map.junctions.size(), 2u);
}

TEST(OpenDriveParser, GeoReferenceCarriesTheMapOrigin)
{
  // 与 config/campus_map.yaml 的 geo_origin 以及 Gazebo 世界的
  // <spherical_coordinates> 是同一个点。三者一致由 Python 侧的
  // test_world_geo_origin_matches_the_map 守着；这里只确认 C++ 读得出来。
  const ads_map::GeoReference & geo = campus_map().geo_reference;
  EXPECT_NEAR(geo.latitude_deg, 31.230416, 1e-9);
  EXPECT_NEAR(geo.longitude_deg, 121.473701, 1e-9);
  EXPECT_NE(geo.proj_string.find("+proj=tmerc"), std::string::npos);
}

TEST(OpenDriveParser, NormalRoadsCarryBothDirections)
{
  const ads_map::RoadMap & map = campus_map();
  for (const auto & [id, road] : map.roads) {
    ASSERT_FALSE(road.lane_sections.empty()) << "道路 " << id << " 没有车道段";
    const ads_map::LaneSection & section = road.lane_sections.front();
    if (road.junction_id < 0) {
      // 常规道路是双向的：左右各一条。
      EXPECT_EQ(section.left.size(), 1u) << "道路 " << id;
      EXPECT_EQ(section.right.size(), 1u) << "道路 " << id;
    } else {
      // 连接道路对应一个转向动作，必须单向。多一条反向车道，
      // 车道图就会多出一条谁都没打算提供的通路，车会在路口里逆行。
      EXPECT_TRUE(section.left.empty()) << "连接道路 " << id << " 不该有反向车道";
      EXPECT_EQ(section.right.size(), 1u) << "连接道路 " << id;
    }
  }
}

TEST(OpenDriveParser, JunctionConnectionsAllHaveLaneLinks)
{
  const ads_map::RoadMap & map = campus_map();
  for (const auto & [id, junction] : map.junctions) {
    EXPECT_EQ(junction.connections.size(), 6u) << "路口 " << id << " 的连接数不是 6";
    for (const ads_map::JunctionConnection & conn : junction.connections) {
      EXPECT_FALSE(conn.lane_links.empty())
        << "路口 " << id << " 的连接 " << conn.id << " 没有 laneLink";
    }
  }
}

TEST(OpenDriveParser, RoadLengthMatchesTheSumOfItsGeometries)
{
  for (const auto & [id, road] : campus_map().roads) {
    double covered = 0.0;
    for (const ads_map::Geometry & g : road.geometries) {
      covered += g.length_m;
    }
    EXPECT_NEAR(covered, road.length_m, 1e-6) << "道路 " << id;
  }
}

// ---------------------------------------------------------------------------
//  拒绝畸形输入 —— 读不懂的东西一律抛异常，绝不静默跳过
// ---------------------------------------------------------------------------

TEST(OpenDriveParser, AcceptsAMinimalWellFormedMap)
{
  // 先证明这个最小样板本身是合法的，否则下面几条「应当抛异常」的用例
  // 可能是因为别的原因抛的 —— 那就成了假通过。
  const std::string xml = minimal_xodr(
    "      <geometry s=\"0.0\" x=\"0.0\" y=\"0.0\" hdg=\"0.0\" length=\"10.0\">"
    "<line/></geometry>\n");
  EXPECT_NO_THROW(ads_map::parse_opendrive(xml, "minimal"));
}

TEST(OpenDriveParser, RejectsSpiralGeometryExplicitly)
{
  // spiral（回旋线）是 CARLA 官方 Town 地图里最常见的几何类型。
  // 我们不支持它 —— 但必须**明确拒绝**而不是当没看见。
  const std::string xml = minimal_xodr(
    "      <geometry s=\"0.0\" x=\"0.0\" y=\"0.0\" hdg=\"0.0\" length=\"10.0\">"
    "<spiral curvStart=\"0.0\" curvEnd=\"0.1\"/></geometry>\n");
  try {
    ads_map::parse_opendrive(xml, "spiral");
    FAIL() << "含 spiral 的地图被静默接受了 —— 那会让路网少一段而无人知晓";
  } catch (const std::runtime_error & e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("spiral"), std::string::npos) << "报错没说清是哪种几何：" << msg;
    EXPECT_NE(msg.find("不支持"), std::string::npos) << msg;
  }
}

TEST(OpenDriveParser, RejectsGeometryWithNoTypeElement)
{
  const std::string xml =
    minimal_xodr("      <geometry s=\"0.0\" x=\"0.0\" y=\"0.0\" hdg=\"0.0\" length=\"10.0\"/>\n");
  EXPECT_THROW(ads_map::parse_opendrive(xml, "notype"), std::runtime_error);
}

TEST(OpenDriveParser, RejectsZeroCurvatureArc)
{
  // 曲率为 0 的圆弧其实就是直线，但写成 <arc> 说明生成方对自己的意图不清楚。
  // 放行的话，后面所有「按曲率判断直线/弯道」的代码都要多考虑一种情形。
  const std::string xml = minimal_xodr(
    "      <geometry s=\"0.0\" x=\"0.0\" y=\"0.0\" hdg=\"0.0\" length=\"10.0\">"
    "<arc curvature=\"0.0\"/></geometry>\n");
  EXPECT_THROW(ads_map::parse_opendrive(xml, "zeroarc"), std::runtime_error);
}

TEST(OpenDriveParser, RejectsGeometryThatDoesNotCoverTheDeclaredLength)
{
  // 几何只铺了 5 m，road 却声称 10 m。这种地图上，s ∈ (5, 10] 的求值
  // 会落到最后一段的延长线上 —— 静默地给出一个「看起来合理」的点。
  const std::string xml = minimal_xodr(
    "      <geometry s=\"0.0\" x=\"0.0\" y=\"0.0\" hdg=\"0.0\" length=\"5.0\">"
    "<line/></geometry>\n",
    10.0);
  EXPECT_THROW(ads_map::parse_opendrive(xml, "short"), std::runtime_error);
}

TEST(OpenDriveParser, RejectsMissingRequiredAttribute)
{
  const std::string xml = minimal_xodr(
    "      <geometry s=\"0.0\" y=\"0.0\" hdg=\"0.0\" length=\"10.0\">"
    "<line/></geometry>\n");  // 少了 x
  EXPECT_THROW(ads_map::parse_opendrive(xml, "noattr"), std::runtime_error);
}

TEST(OpenDriveParser, RejectsOutOfRangeStation)
{
  const ads_map::Road & road = campus_map().road(1);
  EXPECT_THROW(road.reference_pose_at(-1.0), std::out_of_range);
  EXPECT_THROW(road.reference_pose_at(road.length_m + 1.0), std::out_of_range);
  // 边界本身必须是合法的 —— 车开到路的尽头是常态，不是错误。
  EXPECT_NO_THROW(road.reference_pose_at(0.0));
  EXPECT_NO_THROW(road.reference_pose_at(road.length_m));
}

// ---------------------------------------------------------------------------
//  CP-P1-A：与 Python 生成器逐点对账
// ---------------------------------------------------------------------------

TEST(OpenDriveParser, LaneCentresMatchThePythonGenerator)
{
  const std::vector<SampleRow> samples = load_samples(ADS_MAP_SAMPLES_PATH);

  // 空表也会让下面的循环「全部通过」。断言条数是为了不让空循环冒充成功 ——
  // 采样基准要是没读到（路径错了、文件没生成），这个检查点会给出**虚假的通过**，
  // 而它恰恰是本阶段用来兜底的那一道。
  ASSERT_GT(samples.size(), 2000u)
    << "采样基准只读到 " << samples.size() << " 行。文件：" << ADS_MAP_SAMPLES_PATH
    << "\n  没生成的话运行：python3 scripts/gen_map.py";

  const ads_map::RoadMap & map = campus_map();

  double max_pos_err = 0.0;
  double max_ang_err = 0.0;
  SampleRow worst{};

  for (const SampleRow & row : samples) {
    const ads_map::Road & road = map.road(row.road_id);
    const ads_map::Pose2D got = road.lane_center_pose_at(row.lane_id, row.s_m);

    const double pos_err = std::hypot(got.x_m - row.x_m, got.y_m - row.y_m);
    // 朝向直接相减即可：两侧都是沿参考线连续累加的，不存在跨 ±π 的问题。
    const double ang_err = std::abs(got.heading_rad - row.heading_rad);

    if (pos_err > max_pos_err) {
      max_pos_err = pos_err;
      worst = row;
    }
    max_ang_err = std::max(max_ang_err, ang_err);
  }

  std::cout << "  [对账] " << samples.size() << " 个采样点，" << "最大位置偏差 "
            << max_pos_err * 1000.0 << " mm，" << "最大朝向偏差 " << max_ang_err << " rad"
            << std::endl;

  EXPECT_LT(max_pos_err, kCrossCheckTolM)
    << "最大偏差出现在道路 " << worst.road_id << " 车道 " << worst.lane_id << " 的 s=" << worst.s_m
    << " 处。\n"
    << "  偏差接近半个车道宽（1.75 m）说明横向偏移的正方向理解反了；\n"
    << "  偏差随 s 增大说明曲率符号或积分公式有分歧。";
  EXPECT_LT(max_ang_err, kCrossCheckTolRad);
}
