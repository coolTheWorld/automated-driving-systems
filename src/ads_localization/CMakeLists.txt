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
  src/eskf.cpp)

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

  # 机械校验：libads_localization.so 的 DT_NEEDED 里不许有 ROS 库。
  # 做成 CTest 而不是 shell 脚本，是因为要人记得去跑的检查等于没有检查。
  add_test(NAME no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DTARGET_FILE=$<TARGET_FILE:${PROJECT_NAME}>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_no_ros_deps.cmake)
endif()

ament_package()
