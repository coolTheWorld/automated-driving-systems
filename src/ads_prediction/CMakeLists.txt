cmake_minimum_required(VERSION 3.16)
project(ads_prediction)

# 与全仓库一致：C++17 + 全告警。
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
# ⚠️ 模块间 lib 依赖（P6-1 决策二，用户拍板）：地图是静态先验，依据见
#    package.xml 的 description 与 SPEC §3.3 的注解。不要据此推广。
find_package(ads_map REQUIRED)

# ---------------------------------------------------------------------------
#  lib 层：纯算法，零 ROS（no_ros_dependencies 那道 CTest 机械保证）
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/motion_model.cpp
  src/lane_follow.cpp
  src/model_selector.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# PUBLIC/PRIVATE 按「类型是否进对外契约」分（与 ads_perception 同一条判据）：
#   Eigen   —— types.hpp 的字段类型，PUBLIC；
#   ads_map —— lane_follow.hpp 的参数类型（LaneGraph），PUBLIC；
#   ads_common —— 只在 .cpp 里用（ReferenceLine / RequireFinite），PRIVATE。
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen ads_map::ads_map
  PRIVATE ads_common::ads_common)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 ads_common ads_map)

# 配置 YAML 从仓库根 config/ 安装进本包 share（S3 的节点读它）。
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config/prediction_params.yaml
  DESTINATION share/${PROJECT_NAME}/config)

# ---------------------------------------------------------------------------
#  node 层：只做消息翻译 + 逐 id 历史 + 可视化，没有自己的单元测试
#（与 perception_node 同一条理由）。ROS 依赖属于这个 target，
#  libads_prediction.so 一个 ROS 库都不链。
# ---------------------------------------------------------------------------
find_package(rclcpp REQUIRED)
find_package(ads_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(prediction_node node/prediction_node.cpp)
target_link_libraries(prediction_node
  ${PROJECT_NAME}
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  ${ads_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${visualization_msgs_TARGETS})
install(TARGETS prediction_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ⚠️ 必须在 ament_lint_auto_find_test_dependencies() **之前**设排除，
  #    那个函数在调用时就把 linter 列表定死了（CLAUDE.md lint 陷阱表）。
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # 真园区地图：弧线车道 / successor 连续性 / 路口分叉的用例都跑在它上面
  # （与 ads_map 的测试同一份，同一个宏名）。
  set(ADS_MAP_XODR_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../maps/campus.xodr")

  # CP-P6-A ①⑥⑧：恒速/静态/椭圆律的闭式对账。
  ament_add_gtest(test_motion_model test/test_motion_model.cpp)
  target_link_libraries(test_motion_model ${PROJECT_NAME})

  # 决策五的五条规则：速度门限、ODD 上限、位移一致性、尺寸档位。
  ament_add_gtest(test_model_selector test/test_model_selector.cpp)
  target_link_libraries(test_model_selector ${PROJECT_NAME})

  # CP-P6-A ②③④⑤⑦：弧线跟随、跨段连续、截断、分歧对账、逆行拒绝。
  ament_add_gtest(test_lane_follow test/test_lane_follow.cpp)
  target_link_libraries(test_lane_follow ${PROJECT_NAME})
  target_compile_definitions(test_lane_follow PRIVATE
    ADS_MAP_XODR_PATH="${ADS_MAP_XODR_PATH}")

  # 机械保证：libads_prediction.so 零 ROS 依赖（与其余四个包同一份 .cmake，
  # 文件头写明必须「从本包第一天就在」）。
  add_test(NAME no_ros_dependencies
    COMMAND ${CMAKE_COMMAND} -DTARGET_FILE=$<TARGET_FILE:${PROJECT_NAME}>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_no_ros_deps.cmake)

  # L3-G：假障碍物流 + prediction_node 的闭环（不需要 GPU，进 CI）。
  # DOMAIN_ID 分配表：41=ads_control 闭环、42=障碍物闭环、43=定位闭环、
  # 44=感知闭环、45=bridge 看门狗、46=odom 鲁棒性、**47=本测试**、48=轨迹超时、
  # 49=行为闭环。
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_closed_loop_prediction.py
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=47)
endif()

ament_package()
