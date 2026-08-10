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

#ifndef ADS_LOCALIZATION__GEODETIC_HPP_
#define ADS_LOCALIZATION__GEODETIC_HPP_

// =============================================================================
//  经纬高 → map 系（ENU）的换算
//
//  ## 为什么这件事不能凭感觉写
//
//  `/gnss` 是 `sensor_msgs/NavSatFix`（经纬高），而 ESKF 要的是 map 系的米。
//  中间这一步换算里藏着一个**会直接吃掉判据余量**的陷阱：
//
//      用错地球半径模型（球面 vs 椭球）差 0.5%，在 ±100 m 的园区上
//      就是 0.5 m 的**系统性**偏差 —— 而 SPEC §1 的判据是 0.3 m。
//
//  而且这个偏差是系统性的：它不会让滤波器发散，只会让定位**稳定地偏一点**，
//  于是所有人去查 ESKF 的参数，而错在一次单位换算里。
//
//  ## 所以这里的公式是**问 Gazebo 本人问出来的**
//
//  2026-08-11 用 gz-math 的 `SphericalCoordinates::PositionTransform`
//  （Gazebo 生成 NavSat 读数时用的正是它）当预言机实测：
//
//      北向 1000 m → 6 352 596 m/rad      WGS84 子午圈曲率半径 M(31.23°) = 6 352 583
//      东向 1000 m → 5 458 822 m/rad      N(31.23°)·cos φ           = 5 458 769
//
//  吻合到 1e-5，所以 Gazebo 用的就是**标准 WGS84 曲率半径的切平面近似**，
//  不是球面近似。本文件照抄这个模型，并把 gz-math 的输出固化成测试基准
//  —— 与 P1 的 `reference_samples.csv` 是同一个套路：
//  想验证一个实现，就得有一份**独立于它**的标准答案。
//
//  ## 已知的近似误差（可忽略，但要知道它在那里）
//
//  切平面模型忽略了地球曲率造成的高度落差 `d²/(2R)`：
//  100 m 处 0.78 mm。相对 GNSS 4 m 的垂直噪声完全可忽略。
//
//  园区尺度（±100 m）下不需要迭代式的精确解，也不需要按当前纬度重算曲率半径
//  —— M 在 100 m 内只变 0.5 m/deg。
// =============================================================================

#include <Eigen/Core>

namespace ads_localization
{

/// 地图的大地原点。**必须与世界文件的 `<spherical_coordinates>` 一致** ——
/// 两者不一致的症状是定位稳定地偏一个常量，而没有任何模块报错。
///
/// 本项目里它来自 `config/campus_map.yaml` 的 `geo_origin`，
/// 由 launch 作为 ROS 参数传进来（`ads_localization` 不依赖 `ads_map`，
/// 两者是 SPEC §3.3 意义上的两个模块）。
struct GeoOrigin
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double elevation_m{0.0};

  /// 检查取值在合法范围内，否则抛 std::invalid_argument。
  void Validate() const;
};

/// 经纬高 → map 系（ENU：x 东、y 北、z 上），单位 m。
///
/// @throws std::invalid_argument 输入非有限，或原点非法。
Eigen::Vector3d GeodeticToLocal(
  double latitude_deg, double longitude_deg, double altitude_m, const GeoOrigin & origin);

/// map 系（ENU）→ 经纬高。上面那个的逆，供测试做往返一致性检查。
///
/// @return (纬度 deg, 经度 deg, 高程 m)
Eigen::Vector3d LocalToGeodetic(const Eigen::Vector3d & local_m, const GeoOrigin & origin);

}  // namespace ads_localization

#endif  // ADS_LOCALIZATION__GEODETIC_HPP_
