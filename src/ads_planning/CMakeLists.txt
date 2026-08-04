cmake_minimum_required(VERSION 3.16)
project(ads_planning)

# SPEC §7：C++17，全量警告。理由见 ads_common/CMakeLists.txt。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 参考线几何 + 角度工具。纯 C++、无 ROS，见 package.xml 里的说明。
find_package(ads_common REQUIRED)

# -----------------------------------------------------------------------------
# 库：纯 C++17，**不链接任何 ROS 库**
# -----------------------------------------------------------------------------
# SPEC §3.3「算法与 ROS 强制解耦」。P3-S5 会加 node/planning_node.cpp，
# 届时本文件里会出现 rclcpp —— 但那是 planning_node 这个 target 的依赖，
# 本 target（libads_planning.so）依然一个 ROS 库都不许链接。
#
# 机械校验在下面的 BUILD_TESTING 段里（no_ros_dependencies），跟着 colcon test 走。
# **从本包第一天就在**，不是等到有人违反了再补 —— ads_control 的教训是
# 「CMakeLists 里声称有脚本守着，而那个脚本从没被写出来」，整个 P2 只靠纪律维持。
add_library(${PROJECT_NAME} SHARED
  src/frenet.cpp
  src/quintic.cpp
  src/collision.cpp
  src/lattice.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# PUBLIC：本包的公开头文件 frenet.hpp 里 #include 了 ads_common/reference_line.hpp，
# 签名里也出现 ads_common::ReferenceLine / Pose2D。类型进了对外契约，
# include 路径与链接就都必须传递给下游 —— 缺了的话下游会在**嵌套的**那一行
# 报 "file not found"，看起来像 ads_common 装坏了，根因却在这里。
target_link_libraries(${PROJECT_NAME} PUBLIC ads_common::ads_common)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ads_common)

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

  # L1：Frenet ↔ 笛卡尔双向变换。判据全部是解析解或往返一致性，
  # 不读任何文件、不起 ROS，毫秒级。CP-P3-A 就是它。
  ament_add_gtest(test_frenet test/test_frenet.cpp)
  target_link_libraries(test_frenet ${PROJECT_NAME} ads_common::ads_common)

  # L1：五次多项式。判据是「六个边界条件逐项复现」+ 已知闭式解，全部解析。
  ament_add_gtest(test_quintic test/test_quintic.cpp)
  target_link_libraries(test_quintic ${PROJECT_NAME})

  # L1：OBB 相交与间距。含 SPEC §8 点名的三种边界情形（相切、包含、退化）。
  ament_add_gtest(test_collision test/test_collision.cpp)
  target_link_libraries(test_collision ${PROJECT_NAME})

  # L1：横向 lattice。**含 planning.md §6 那个可行性不等式的两侧**
  # —— 贴边障碍物可绕、车道中心障碍物几何上无解必须报不可行。
  ament_add_gtest(test_lattice test/test_lattice.cpp)
  target_link_libraries(test_lattice ${PROJECT_NAME} ads_common::ads_common)

  # 机械校验：libads_planning.so 的 DT_NEEDED 里不许有 ROS 库。
  # 做成 CTest 而不是 shell 脚本，是因为要人记得去跑的检查等于没有检查。
  add_test(NAME no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DTARGET_FILE=$<TARGET_FILE:${PROJECT_NAME}>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_no_ros_deps.cmake)
endif()

ament_package()
