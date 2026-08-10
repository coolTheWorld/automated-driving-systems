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

#include "ads_localization/point_cloud_io.hpp"

#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ads_localization
{

std::vector<Eigen::Vector3d> LoadPcdAscii(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("LoadPcdAscii: 打不开 " + path);
  }

  std::string fields;
  std::string data_format;
  int64_t declared_points = -1;
  std::string line;

  // ---- 头 ---------------------------------------------------------------
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;  // gen_map.py 在头里写了几行中文说明
    }
    std::istringstream stream(line);
    std::string key;
    stream >> key;
    if (key == "FIELDS") {
      std::getline(stream, fields);
    } else if (key == "POINTS") {
      stream >> declared_points;
    } else if (key == "DATA") {
      stream >> data_format;
      break;  // DATA 之后就是数据了
    }
  }

  if (data_format != "ascii") {
    throw std::runtime_error(
      "LoadPcdAscii: 只支持 DATA ascii，收到 \"" + data_format +
      "\"。本读取器只服务于 scripts/gen_map.py 生成的那一种格式。");
  }
  // 去掉 FIELDS 后面的前导空格再比。写死 " x y z" 是有意的：
  // 字段顺序不同（比如 "x y z intensity"）时下面按列取值就会错位，
  // 而错位之后点云看起来仍然"是一片点"，只是全都在错的地方。
  if (fields != " x y z") {
    throw std::runtime_error("LoadPcdAscii: 只支持 FIELDS x y z，收到 \"" + fields + "\"");
  }

  // ---- 数据 -------------------------------------------------------------
  std::vector<Eigen::Vector3d> points;
  if (declared_points > 0) {
    points.reserve(static_cast<size_t>(declared_points));
  }
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  while (file >> x >> y >> z) {
    points.emplace_back(x, y, z);
  }

  // 声明点数与实际行数必须一致。不一致意味着文件被截断或被手改过 ——
  // 而"少了一半点"的点云不会让 NDT 报错，它会照常收敛到一个错位姿。
  if (declared_points >= 0 && static_cast<size_t>(declared_points) != points.size()) {
    throw std::runtime_error(
      "LoadPcdAscii: 头里声明 POINTS " + std::to_string(declared_points) + "，实际读到 " +
      std::to_string(points.size()) + " 个 —— 文件被截断了？");
  }
  if (points.empty()) {
    throw std::runtime_error("LoadPcdAscii: " + path + " 里一个点都没有");
  }
  return points;
}

}  // namespace ads_localization
