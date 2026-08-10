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
//  经纬高 ↔ map 系换算的 L1 测试
//
//  核心判据是**与 gz-math 逐点对账**。
//
//  为什么必须对账而不是只做往返一致性：往返一致只能证明「我的正变换和我的
//  逆变换互逆」—— **同步地错着也是互逆的**。而这个换算一旦用错地球模型
//  （比如把椭球当成球），偏差是 0.5%，在 ±100 m 上就是 0.5 m 的**系统性**
//  误差，而 SPEC §1 的定位判据是 0.3 m。
//
//  这与 P1 的 reference_samples.csv 是同一个套路：想验证一个实现，
//  就得有一份**独立于它**的标准答案。这里的标准答案来自 gz-math ——
//  Gazebo 生成 NavSat 读数时用的正是它。
// =============================================================================

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

#include "ads_localization/geodetic.hpp"

namespace
{

using ads_localization::GeodeticToLocal;
using ads_localization::GeoOrigin;
using ads_localization::LocalToGeodetic;

/// 与 config/campus_map.yaml 的 geo_origin、以及 worlds/*.sdf 的
/// <spherical_coordinates> 完全一致。三处不一致的症状是定位稳定地偏一个常量。
const GeoOrigin kCampusOrigin{31.230416, 121.473701, 10.0};

/// gz-math `SphericalCoordinates::PositionTransform(LOCAL2 → SPHERICAL)`
/// 在 kCampusOrigin 上的输出（2026-08-11 实测）。
///
/// 字段：local x(东) / y(北) / z(上)，然后是 gz-math 给出的 纬度 / 经度 / 高程。
struct OracleSample
{
  double east_m;
  double north_m;
  double up_m;
  double latitude_deg;
  double longitude_deg;
  double altitude_m;
};

const std::vector<OracleSample> kOracle{
  {100, 0, 0, 31.230415995716708, 121.474750604266632, 10.0007832209},
  {0, 100, 0, 31.231317927837033, 121.473700999999991, 10.0007870803},
  {-60, 40, 5, 31.230776769324081, 121.473071235540587, 15.0004078904},
  // 自车 spawn 位姿（worlds/campus_loop.sdf）
  {30, -51.75, 0, 31.229949251909090, 121.474015879732320, 10.0002812743},
  {0, 0, 0, 31.230415999999995, 121.473700999999991, 9.9999999991},
  // CP-P2-B 的目标点
  {91.75, 20, 0, 31.230596381971818, 121.474664013744018, 10.0006908029},
};

}  // namespace

TEST(Geodetic, MatchesGzMathPointByPoint)
{
  for (const OracleSample & sample : kOracle) {
    const Eigen::Vector3d local =
      GeodeticToLocal(sample.latitude_deg, sample.longitude_deg, sample.altitude_m, kCampusOrigin);

    // 水平判据 1 mm。真做错了（球面近似、或 M/N 用混）偏差是**米**级，
    // 不会卡在这个阈值附近。
    EXPECT_NEAR(local.x(), sample.east_m, 1e-3)
      << "东向：(" << sample.east_m << ", " << sample.north_m << ")";
    EXPECT_NEAR(local.y(), sample.north_m, 1e-3)
      << "北向：(" << sample.east_m << ", " << sample.north_m << ")";

    // ⚠️ 垂直方向判据放宽到 1 mm：切平面模型忽略了地球曲率造成的高度落差
    //    d²/(2R)，100 m 处正好 0.78 mm。这是**已知的模型近似**，不是 bug，
    //    相对 GNSS 4 m 的垂直噪声完全可忽略。
    EXPECT_NEAR(local.z(), sample.up_m, 1e-3) << "垂直";
  }
}

TEST(Geodetic, RoundTripIsConsistent)
{
  // 往返一致**不能**代替上面那条对账（同步地错着也是互逆的），
  // 但它能抓住正逆变换之间的不对称笔误。
  for (const OracleSample & sample : kOracle) {
    const Eigen::Vector3d local(sample.east_m, sample.north_m, sample.up_m);
    const Eigen::Vector3d geo = LocalToGeodetic(local, kCampusOrigin);
    const Eigen::Vector3d back = GeodeticToLocal(geo.x(), geo.y(), geo.z(), kCampusOrigin);
    EXPECT_LT((back - local).norm(), 1e-9);
  }
}

TEST(Geodetic, MeridianAndPrimeVerticalRadiiAreNotTheSame)
{
  // 用同一个半径处理南北和东西是最常见的错法（那等于把地球当成球）。
  // 在纬度 31° 上，1 度纬度 ≈ 110 873 m，1 度经度 ≈ 95 275 m —— 差 14%。
  // 这条用例把「两者必须不同」钉住。
  const Eigen::Vector3d one_deg_north = GeodeticToLocal(
    kCampusOrigin.latitude_deg + 1.0, kCampusOrigin.longitude_deg, kCampusOrigin.elevation_m,
    kCampusOrigin);
  const Eigen::Vector3d one_deg_east = GeodeticToLocal(
    kCampusOrigin.latitude_deg, kCampusOrigin.longitude_deg + 1.0, kCampusOrigin.elevation_m,
    kCampusOrigin);

  EXPECT_NEAR(one_deg_north.y(), 110873.0, 50.0) << "1 度纬度对应的米数";
  EXPECT_NEAR(one_deg_east.x(), 95275.0, 50.0) << "1 度经度对应的米数";
  EXPECT_GT(one_deg_north.y() - one_deg_east.x(), 10000.0)
    << "南北与东西用了同一个半径 —— 那等于把地球当成了球";
}

TEST(Geodetic, RejectsNonFiniteAndIllegalInputs)
{
  // NavSatFix 在无定位时会填 NaN。用比较去拦一条都拦不住 ——
  // NaN 参与任何比较都返回 false（CLAUDE.md 记着两次教训）。
  for (const double bad :
       {std::nan(""), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(GeodeticToLocal(bad, 121.0, 10.0, kCampusOrigin), std::invalid_argument);
    EXPECT_THROW(GeodeticToLocal(31.0, bad, 10.0, kCampusOrigin), std::invalid_argument);
    EXPECT_THROW(GeodeticToLocal(31.0, 121.0, bad, kCampusOrigin), std::invalid_argument);
  }

  GeoOrigin bad_origin{95.0, 0.0, 0.0};
  EXPECT_THROW(GeodeticToLocal(31.0, 121.0, 10.0, bad_origin), std::invalid_argument);

  // 极点附近 cos(φ) → 0，东西向换算会炸。静默给出无穷大比报错糟得多。
  GeoOrigin polar{89.5, 0.0, 0.0};
  EXPECT_THROW(GeodeticToLocal(89.5, 0.0, 0.0, polar), std::invalid_argument);
}
