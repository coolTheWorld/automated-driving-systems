cmake_minimum_required(VERSION 3.16)
project(ads_localization)

# SPEC §7：C++17，全量警告。理由见 ads_common/CMakeLists.txt。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Eigen。头文件库，见 package.xml 里关于「不是新依赖」的说明。
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# RequireFinite 三兄弟。纯 C++、无 ROS。
find_package(ads_common REQUIRED)

# -----------------------------------------------------------------------------
# 库：纯 C++17，**不链接任何 ROS 库**
# -----------------------------------------------------------------------------
# SPEC §3.3「算法与 ROS 强制解耦」。P4-S4 会加 node/localization_node.cpp，
# 届时本文件里会出现 rclcpp —— 但那是那个 target 的依赖，
# 本 target（libads_localization.so）依然一个 ROS 库都不许链接。
#
# 机械校验在下面的 BUILD_TESTING 段（no_ros_dependencies），跟着 colcon test 走。
# **从本包第一天就在** —— ads_control 的教训是「CMakeLists 里声称有脚本守着，
# 而那个脚本从没被写出来」，整个 P2 只靠纪律维持。
add_library(${PROJECT_NAME} SHARED
  src/eskf.cpp
  src/geodetic.cpp
  src/ndt.cpp
  src/ndt_align.cpp
  src/point_cloud_io.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# PUBLIC：公开头文件 eskf.hpp 的签名里出现 Eigen 类型（状态向量、协方差矩阵），
# 类型进了对外契约，include 路径就必须传递给下游。
# ads_common 只在 .cpp 里用（RequireFinite），所以是 PRIVATE。
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PRIVATE ads_common::ads_common)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module Eigen3 ads_common)

# -----------------------------------------------------------------------------
# 节点：ROS 包装层（P4-S4）
# -----------------------------------------------------------------------------
# 这一层**只做翻译**：ROS 消息 ↔ lib 的调用。定位算法一行都不在这里。
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(localization_node node/localization_node.cpp)
target_link_libraries(localization_node ${PROJECT_NAME})
ament_target_dependencies(localization_node
  rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs tf2 tf2_ros)

install(TARGETS localization_node
  DESTINATION lib/${PROJECT_NAME})

# 车辆参数装一份到本包 share（理由同 ads_control：ROS 包安装后必须自包含，
# 不能依赖"工作区恰好挂在 /workspace"这种只在本容器成立的假设）。
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/vehicle_params.yaml
  DESTINATION share/${PROJECT_NAME}/config)

# -----------------------------------------------------------------------------
# L1 单元测试
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 用 clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效
  # —— 那个函数在调用时就把 linter 列表定死了，设晚了不生效**且不报错**。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # L1：ESKF。判据全部建立在**合成 IMU 数据**上 ——
  # 真值轨迹已知 → 反算理想 IMU → 注入已知噪声与零偏，
  # 于是「零偏估得准不准」这种在真仿真里根本看不见的量可以直接对账。
  # CP-P4-A 的四条判据就在这里。
  ament_add_gtest(test_eskf test/test_eskf.cpp)
  target_link_libraries(test_eskf ${PROJECT_NAME} ads_common::ads_common)

  # L1：NDT 的体素化。判据分两类 ——
  #   ① 高斯拟合对不对：与解析解比对（已知分布采样出来的点）
  #   ② **退化情形有没有被处理**：共面点云（协方差 rank 2）、稀疏体素
  # 第 ② 类是本文件的核心，它守的是 P4 全阶段那条主线在体素尺度上的重现。
  # 用真实的 maps/campus_cloud.pcd 做端到端体检，路径由 CMake 传进去 ——
  # 不写死相对路径，那个在 CI 里的工作目录不一定成立。
  ament_add_gtest(test_ndt test/test_ndt.cpp)
  target_link_libraries(test_ndt ${PROJECT_NAME} ads_common::ads_common)
  target_compile_definitions(test_ndt PRIVATE
    ADS_CAMPUS_CLOUD_PCD="${CMAKE_CURRENT_SOURCE_DIR}/../../maps/campus_cloud.pcd")

  # L1：NDT 配准。三类判据 ——
  #   ① 解析梯度 vs **数值微分**（消除符号错误，那类错误的现象会伪装成"收敛域小"）
  #   ② 在真实 campus_cloud.pcd 上恢复已知变换
  #   ③ **纯地面点云必须报退化** ← P4-1 决策一那条主线在配准这一步的落点
  ament_add_gtest(test_ndt_align test/test_ndt_align.cpp)
  target_link_libraries(test_ndt_align ${PROJECT_NAME} ads_common::ads_common)
  target_compile_definitions(test_ndt_align PRIVATE
    ADS_CAMPUS_CLOUD_PCD="${CMAKE_CURRENT_SOURCE_DIR}/../../maps/campus_cloud.pcd")

  # L1：经纬高 ↔ map 系换算。核心判据是**与 gz-math 逐点对账** ——
  # Gazebo 生成 NavSat 读数用的正是它，所以那是一份独立的标准答案。
  # 只做往返一致性是不够的：同步地错着也是互逆的，而用错地球模型
  # 在 ±100 m 上就是 0.5 m 的系统偏差，判据只有 0.3 m。
  ament_add_gtest(test_geodetic test/test_geodetic.cpp)
  target_link_libraries(test_geodetic ${PROJECT_NAME} ads_common::ads_common)

  # ---------------------------------------------------------------------------
  # L3-G：不需要 GPU 的定位闭环（假传感器 + localization_node）
  # ---------------------------------------------------------------------------
  # 假传感器**不 install**：它是测试夹具，不是仿真数据源，不该出现在
  # SIM_SOURCES 里，也不该被人误当仿真器用（理由见文件头）。
  # launch_testing 通过 $<TARGET_FILE:> 直接拿绝对路径，不需要 install。
  find_package(rosgraph_msgs REQUIRED)
  add_executable(fake_sensors test/fake_sensors_node.cpp)
  target_link_libraries(fake_sensors ${PROJECT_NAME})
  ament_target_dependencies(fake_sensors
    rclcpp sensor_msgs nav_msgs geometry_msgs rosgraph_msgs tf2 tf2_ros)

  # 两个源码树里的文件走 ENV 传进去，不走 package share：
  #   - campus_cloud.pcd 是地图生成物，几 MB，装进每个包是浪费；
  #   - campus_map.yaml 归 ads_map，而本包**有意不依赖它**（SPEC §3.3）。
  # 与 test_ndt 用 ADS_CAMPUS_CLOUD_PCD 编译宏是同一套做法。
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_closed_loop_localization.py
    TIMEOUT 180
    ENV
      FAKE_SENSORS_EXECUTABLE=$<TARGET_FILE:fake_sensors>
      ADS_CAMPUS_CLOUD_PCD=${CMAKE_CURRENT_SOURCE_DIR}/../../maps/campus_cloud.pcd
      ADS_CAMPUS_MAP_YAML=${CMAKE_CURRENT_SOURCE_DIR}/../../config/campus_map.yaml
      # ⚠️ 独占一个 ROS_DOMAIN_ID。colcon 按**包**并行，本测试会与 ads_control
      #    的两个闭环同时在跑，默认 domain 0 下话题和 TF 全是共享的。
      #    实测：不隔离时本测试的「/tf_static 上不许有 map→odom」会读到
      #    ads_control 那个测试起的 static_transform_publisher，两边一起红。
      #    分配表：ads_control 41 / 42，ads_localization 43。
      ROS_DOMAIN_ID=43)

  # 机械校验：libads_localization.so 的 DT_NEEDED 里不许有 ROS 库。
  # 做成 CTest 而不是 shell 脚本，是因为要人记得去跑的检查等于没有检查。
  add_test(NAME no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DTARGET_FILE=$<TARGET_FILE:${PROJECT_NAME}>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_no_ros_deps.cmake)
endif()

ament_package()
