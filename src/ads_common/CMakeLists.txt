cmake_minimum_required(VERSION 3.16)
project(ads_common)

# SPEC §7：C++17，全量警告。-Wpedantic 会挑出依赖非标准扩展的写法，
# 这类写法换个编译器就崩，而 CI 只跑一个编译器时发现不了。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# -----------------------------------------------------------------------------
# 库：纯 C++17，**不链接任何 ROS 库**
# -----------------------------------------------------------------------------
# 这是 SPEC §3.3「算法与 ROS 强制解耦」的落地：本包只有 lib/ 没有 node/。
# 一旦这里出现 rclcpp，L1 测试就从毫秒级掉到秒级（每个测试都要初始化
# ROS 上下文），测试一慢就没人跑，等于白写。
#
# find_package 里也刻意没有 rclcpp —— 靠构建系统把这条约束钉死，
# 比写在文档里管用：谁想加 ROS 依赖，必须先改这个文件，改的时候就会看到这段。
add_library(${PROJECT_NAME} SHARED
  src/angles.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# 让下游包 find_package(ads_common) 后能直接链接到它。
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# -----------------------------------------------------------------------------
# L1 单元测试
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 用 clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效，
  # 详细理由见 gazebo_bridge/CMakeLists.txt 与仓库根 .clang-format。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()

  # 用 ament_add_gtest 而不是裸 add_executable + add_test：
  # 它会自动生成 xunit XML 结果文件，colcon test-result 才能汇总；
  # 裸写的话测试跑了但 `colcon test-result --all` 里看不到，
  # 表现为「测试明明在跑，汇总里却是 0 tests」。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_angles test/test_angles.cpp)
  target_link_libraries(test_angles ${PROJECT_NAME})
endif()

ament_package()
