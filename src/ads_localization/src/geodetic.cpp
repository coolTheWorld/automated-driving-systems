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

#include "ads_localization/geodetic.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "ads_common/numeric_checks.hpp"

namespace ads_localization
{

namespace
{

// WGS84 椭球参数。**不要换成球面近似** —— 差 0.5%，在 ±100 m 上就是 0.5 m
// 的系统偏差，而 SPEC §1 的定位判据是 0.3 m。见头文件里的实测对账。
constexpr double kSemiMajorAxisM = 6378137.0;
constexpr double kFlattening = 1.0 / 298.257223563;
constexpr double kFirstEccentricitySquared = kFlattening * (2.0 - kFlattening);
constexpr double kDegToRad = M_PI / 180.0;

/// 在给定纬度处的子午圈曲率半径 M 与卯酉圈曲率半径 N。
///
/// 南北向 1 rad 对应 M 米，东西向 1 rad 对应 N·cos(φ) 米 ——
/// **两者不同**，用同一个半径是最常见的错法（那等于把地球当球）。
struct CurvatureRadii
{
  double meridian_m;        ///< M：南北向
  double prime_vertical_m;  ///< N：东西向（还要再乘 cos φ）
};

CurvatureRadii RadiiAt(double latitude_rad)
{
  const double sin_lat = std::sin(latitude_rad);
  const double w = 1.0 - kFirstEccentricitySquared * sin_lat * sin_lat;
  return {
    kSemiMajorAxisM * (1.0 - kFirstEccentricitySquared) / (w * std::sqrt(w)),
    kSemiMajorAxisM / std::sqrt(w)};
}

}  // namespace

void GeoOrigin::Validate() const
{
  ads_common::RequireFinite(latitude_deg, "GeoOrigin", "latitude_deg");
  ads_common::RequireFinite(longitude_deg, "GeoOrigin", "longitude_deg");
  ads_common::RequireFinite(elevation_m, "GeoOrigin", "elevation_m");
  if (std::abs(latitude_deg) > 90.0) {
    throw std::invalid_argument(
      "GeoOrigin::latitude_deg 必须在 ±90 之间，收到 " + std::to_string(latitude_deg));
  }
  if (std::abs(longitude_deg) > 180.0) {
    throw std::invalid_argument(
      "GeoOrigin::longitude_deg 必须在 ±180 之间，收到 " + std::to_string(longitude_deg));
  }
  // 极点附近 cos(φ) → 0，东西向换算会炸。本项目的 ODD 里不会出现，
  // 但静默给出无穷大的东向坐标比报错糟得多。
  if (std::abs(latitude_deg) > 89.0) {
    throw std::invalid_argument("GeoOrigin::latitude_deg 太靠近极点，切平面近似在这里失效");
  }
}

Eigen::Vector3d GeodeticToLocal(
  double latitude_deg, double longitude_deg, double altitude_m, const GeoOrigin & origin)
{
  origin.Validate();
  // ⚠️ 必须显式判有限性。NavSatFix 在无定位时会填 NaN，
  //    而用比较去拦一条都拦不住（NaN 参与任何比较都返回 false）。
  ads_common::RequireFinite(latitude_deg, "GeodeticToLocal", "latitude_deg");
  ads_common::RequireFinite(longitude_deg, "GeodeticToLocal", "longitude_deg");
  ads_common::RequireFinite(altitude_m, "GeodeticToLocal", "altitude_m");

  const double origin_lat_rad = origin.latitude_deg * kDegToRad;
  const CurvatureRadii radii = RadiiAt(origin_lat_rad);

  const double north_m = (latitude_deg - origin.latitude_deg) * kDegToRad * radii.meridian_m;
  const double east_m = (longitude_deg - origin.longitude_deg) * kDegToRad *
                        radii.prime_vertical_m * std::cos(origin_lat_rad);
  return {east_m, north_m, altitude_m - origin.elevation_m};
}

Eigen::Vector3d LocalToGeodetic(const Eigen::Vector3d & local_m, const GeoOrigin & origin)
{
  origin.Validate();
  for (int i = 0; i < 3; ++i) {
    ads_common::RequireFinite(local_m[i], "LocalToGeodetic", "local_m");
  }

  const double origin_lat_rad = origin.latitude_deg * kDegToRad;
  const CurvatureRadii radii = RadiiAt(origin_lat_rad);

  const double latitude_deg = origin.latitude_deg + local_m.y() / (radii.meridian_m * kDegToRad);
  const double longitude_deg =
    origin.longitude_deg +
    local_m.x() / (radii.prime_vertical_m * std::cos(origin_lat_rad) * kDegToRad);
  return {latitude_deg, longitude_deg, local_m.z() + origin.elevation_m};
}

}  // namespace ads_localization
