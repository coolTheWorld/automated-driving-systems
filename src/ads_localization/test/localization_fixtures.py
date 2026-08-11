# Copyright 2026 孙帅
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Shared parameter plumbing for the L3-G localization launch test.

⚠️ **这份搬运逻辑与 `stack.launch.py` 的 `_localization_nodes()` 是两份。**
   本包不 depend `ads_bringup`（反过来依赖会成环），与 ads_control 的
   `closed_loop_common.py` 是同一个处境、同一个理由。

   代价是漂移风险。缓解手段有两条，缺一不可：
     ① 两边读的是**同一批 YAML**（campus_map.yaml / vehicle_params.yaml），
        真正的单一来源在那里，这里只是键名映射；
     ② `localization_node` 的参数默认值全是 0 / 空串，搬漏了会在构造函数里
        指名报错（大地原点 Validate、map_pcd_path 空），而不是静默跑偏。

⚠️ **两个源码树里的文件用环境变量传进来，不走 package share。**
   `maps/campus_cloud.pcd` 和 `config/campus_map.yaml` 都不属于本包的
   安装物 —— 前者是地图生成物（几 MB，装进每个包是浪费），后者归 ads_map。
   而本包**有意不依赖 ads_map**（SPEC §3.3：两者是两个模块）。
   CMake 用 `ENV` 把源码树的绝对路径传给 add_launch_test，与
   `test_ndt.cpp` 用 `ADS_CAMPUS_CLOUD_PCD` 编译宏是同一套做法。
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


# 自车起点：与 worlds/campus_loop.sdf 的 spawn 一致（南侧道路东行车道）。
# 与 ads_control 的 L3-G 用同一个起点，两者的数出现分歧时能直接对比。
START_X_M = 30.0
START_Y_M = -51.75
START_YAW_RAD = 0.0

# 路线：沿南侧道路向东约 55 m。**刻意留在这一段**——
# 再往东（x > 95）杆件稀疏，30 m 内的结构点从 5000 掉到 1900，
# NDT 的约束跟着变弱，判据会开始抖。这不是算法问题，是场景问题。
CRUISE_SPEED_MPS = 4.0
START_DELAY_S = 4.0


def _campus_map() -> dict:
    """
    Load campus_map.yaml from the source tree path CMake handed us.

    :return: parsed campus map config
    """
    return yaml.safe_load(Path(os.environ['ADS_CAMPUS_MAP_YAML']).read_text(encoding='utf-8'))


def _vehicle() -> dict:
    """
    Load vehicle_params.yaml from this package's share directory.

    :return: parsed vehicle params
    """
    share = Path(get_package_share_directory('ads_localization')) / 'config'
    return yaml.safe_load((share / 'vehicle_params.yaml').read_text(encoding='utf-8'))


def fake_sensors_params() -> dict:
    """
    Collect fake_sensors parameters.

    ⚠️ 噪声与杆臂**必须与 localization_node 读同一份 YAML**。
    两边各填一遍的话，滤波器会以为传感器比实际更好（或更差），
    而症状是"估计过度自信"——协方差收得很紧、误差却在涨，没有任何报错。

    :return: 传给 fake_sensors 的参数字典
    """
    vehicle = _vehicle()
    geo = _campus_map()['geo_origin']
    lidar = vehicle['sensors']['lidar']
    imu_noise = vehicle['sensors']['imu']['noise']
    gnss = vehicle['sensors']['gnss']
    return {
        'geo_origin.latitude_deg': geo['latitude_deg'],
        'geo_origin.longitude_deg': geo['longitude_deg'],
        'geo_origin.elevation_m': geo['elevation_m'],
        'truth.start_x_m': START_X_M,
        'truth.start_y_m': START_Y_M,
        'truth.start_yaw_rad': START_YAW_RAD,
        'truth.cruise_speed_mps': CRUISE_SPEED_MPS,
        'truth.start_delay_s': START_DELAY_S,
        'noise.gyro_stddev_rad_s': imu_noise['gyro_stddev_rad_s'],
        'noise.accel_stddev_mps2': imu_noise['accel_stddev_mps2'],
        'noise.gnss_horizontal_stddev_m': gnss['noise']['position_horizontal_stddev_m'],
        'noise.gnss_vertical_stddev_m': gnss['noise']['position_vertical_stddev_m'],
        'noise.lidar_stddev_m': lidar['noise_stddev_m'],
        'gnss.lever_arm_x_m': gnss['mount_x_m'],
        'gnss.lever_arm_y_m': gnss['mount_y_m'],
        'gnss.lever_arm_z_m': gnss['mount_z_m'],
        'lidar.mount_z_m': lidar['mount_z_m'],
        'lidar.vertical_fov_min_rad': lidar['vertical_fov_min_rad'],
        'lidar.vertical_fov_max_rad': lidar['vertical_fov_max_rad'],
        # ⚠️ 量程刻意**短于**真雷达的 50 m。抠一帧扫描要遍历全部 8.2 万个
        #    先验点，50 m 下单帧点数近万，NDT 在 CI 机器上就追不上 10 Hz 了
        #    ——而追不上的表现是"陈旧点云被丢"，看起来像链路坏了。
        #    30 m 下仍有约 5000 结构点，足够把六个自由度都约束住。
        'lidar.range_max_m': 30.0,
        'map_pcd_path': os.environ['ADS_CAMPUS_CLOUD_PCD'],
    }


def localization_params() -> dict:
    """
    Collect localization_node parameters — mirrors stack.launch.py's _localization_nodes().

    :return: 传给 localization_node 的参数字典
    """
    vehicle = _vehicle()
    geo = _campus_map()['geo_origin']
    imu_noise = vehicle['sensors']['imu']['noise']
    gnss = vehicle['sensors']['gnss']
    gnss_noise = gnss['noise']
    return {
        'geo_origin.latitude_deg': geo['latitude_deg'],
        'geo_origin.longitude_deg': geo['longitude_deg'],
        'geo_origin.elevation_m': geo['elevation_m'],
        'eskf.gyro_noise_rad_s': imu_noise['gyro_stddev_rad_s'],
        'eskf.accel_noise_mps2': imu_noise['accel_stddev_mps2'],
        'eskf.gyro_bias_rw_rad_s': imu_noise['gyro_dynamic_bias_stddev_rad_s'],
        'eskf.accel_bias_rw_mps2': imu_noise['accel_dynamic_bias_stddev_mps2'],
        'eskf.init_gyro_bias_std_rad_s': imu_noise['gyro_bias_stddev_rad_s'],
        'eskf.init_accel_bias_std_mps2': imu_noise['accel_bias_stddev_mps2'],
        'gnss.horizontal_std_m': gnss_noise['position_horizontal_stddev_m'],
        'gnss.vertical_std_m': gnss_noise['position_vertical_stddev_m'],
        'gnss.lever_arm_x_m': gnss['mount_x_m'],
        'gnss.lever_arm_y_m': gnss['mount_y_m'],
        'gnss.lever_arm_z_m': gnss['mount_z_m'],
        # 冷启动的航向先验。真栈里从世界文件读 spawn 朝向，这里就是假车的起点朝向。
        # ⚠️ 不给它的症状是 NDT 初值差 90°+ 直接落在收敛域外 ——
        #    这正是 CP-P4-B 五个根因之一。
        'initial_yaw_rad': START_YAW_RAD,
        'map_pcd_path': os.environ['ADS_CAMPUS_CLOUD_PCD'],
        'use_sim_time': True,
    }
