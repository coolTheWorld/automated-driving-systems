cmake_minimum_required(VERSION 3.16)
project(ads_perception)

# SPEC §7：C++17，全量警告。理由见 ads_common/CMakeLists.txt。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ads_common REQUIRED)

# -----------------------------------------------------------------------------
# 库：纯 C++17，**不链接任何 ROS 库**
# -----------------------------------------------------------------------------
# SPEC §3.3「算法与 ROS 强制解耦」。机械校验在下面的 BUILD_TESTING 段
# （no_ros_dependencies），跟着 colcon test 走 —— **从本包第一天就在**。
add_library(${PROJECT_NAME} SHARED
  src/ground_segmentation.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# PUBLIC Eigen：公开头文件的签名里出现 Eigen::Vector3d，类型进了对外契约。
# ads_common 只在 .cpp 里用（RequireFinite），所以是 PRIVATE。
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PRIVATE ads_common::ads_common)

install(DIRECTORY include/ DESTINATION include)
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
  # clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效
  # —— 那个函数在调用时就把 linter 列表定死了，设晚了不生效**且不报错**。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # L1：地面分割。
  #
  # ⚠️ **判据必须跑在带坡度的合成地面上。** campus 的地面是理想平面，
  #    在那上面 RANSAC 与「z < 阈值」结果完全相同 —— 判据没有区分力。
  #    见 ground_segmentation.hpp 的文件头与 plan.md P5-1 决策二。
  ament_add_gtest(test_ground_segmentation test/test_ground_segmentation.cpp)
  target_link_libraries(test_ground_segmentation ${PROJECT_NAME} ads_common::ads_common)

  # 机械校验：libads_perception.so 的 DT_NEEDED 里不许有 ROS 库。
  add_test(NAME no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DTARGET_FILE=$<TARGET_FILE:${PROJECT_NAME}>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_no_ros_deps.cmake)
endif()

ament_package()
