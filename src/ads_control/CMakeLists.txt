cmake_minimum_required(VERSION 3.16)
project(ads_control)

# SPEC §7：C++17，全量警告。理由见 ads_common/CMakeLists.txt。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 角度归一化与最短角度差。纯 C++、无 ROS，见 package.xml 里的说明。
find_package(ads_common REQUIRED)

# -----------------------------------------------------------------------------
# 库：纯 C++17，**不链接任何 ROS 库**
# -----------------------------------------------------------------------------
# SPEC §3.3「算法与 ROS 强制解耦」。P2-S4 会加 node/control_node.cpp，
# 届时本文件里会出现 rclcpp —— 但那是 control_node 这个 target 的依赖，
# 本 target（libads_control.so）依然一个 ROS 库都不许链接。
#
# 与 ads_map 完全同构，包括那道机械校验：靠纪律维持的约束迟早会破，
# 靠脚本维持的不会。P2-S4 的 verify_control.sh 会 ldd 这个 .so。
add_library(${PROJECT_NAME} SHARED
  src/path_tracking.cpp
  src/stanley.cpp
  src/speed_profile.cpp
  src/speed_controller.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# PRIVATE：ads_common 只出现在 .cpp 里，公开头文件不含它的任何类型。
# 下游链接 ads_control 时不该被迫知道我们用谁算角度差。
target_link_libraries(${PROJECT_NAME} PRIVATE ads_common::ads_common)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# -----------------------------------------------------------------------------
# 测试
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 用 clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效
  # —— 那个函数在调用时就把 linter 列表定死了，设晚了不生效**且不报错**。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # L1：路径预处理（弧长、曲率、最近点投影、横向/航向误差）。
  # 全部与解析解比对，不读任何文件、不起 ROS，毫秒级。
  ament_add_gtest(test_path_tracking test/test_path_tracking.cpp)
  target_link_libraries(test_path_tracking ${PROJECT_NAME} ads_common::ads_common)

  # L1：横向 Stanley + CP-P2-A 的闭环收敛判据。
  # 闭环用的运动学自行车模型在 test/kinematic_bicycle.hpp —— **测试夹具，非产品代码**，
  # 所以它不进 add_library。P2-S5 若把它包成 ROS 假车节点，那时才搬家。
  ament_add_gtest(test_stanley test/test_stanley.cpp)
  # PRIVATE 的 ads_common 不会传递过来，而测试自己要用 normalize_angle，所以显式链接。
  target_link_libraries(test_stanley ${PROJECT_NAME} ads_common::ads_common)
  # 夹具头文件与被测头文件不在同一目录，给测试单独加 test/ 到搜索路径。
  target_include_directories(test_stanley PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)

  # L1：速度剖面（曲率限速 + 前后向扫描）。纯几何，判据全是闭式解。
  ament_add_gtest(test_speed_profile test/test_speed_profile.cpp)
  target_link_libraries(test_speed_profile ${PROJECT_NAME} ads_common::ads_common)

  # L1：速度环 PI + 抗饱和。含 gazebo_bridge 那段"设定值超前实测不得超过 1 m/s"
  # 的抗饱和环节建模 —— 见 docs/modules/control.md §4.5。
  ament_add_gtest(test_speed_controller test/test_speed_controller.cpp)
  target_link_libraries(test_speed_controller ${PROJECT_NAME})
endif()

ament_package()
