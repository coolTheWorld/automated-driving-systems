cmake_minimum_required(VERSION 3.16)
project(ads_map)

# SPEC §7：C++17，全量警告。理由见 ads_common/CMakeLists.txt。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# -----------------------------------------------------------------------------
# 本包目前只有测试。
#
# lib/（OpenDRIVE 解析、参考线求值、车道图、Dijkstra 路由）在 P1-S2/S3 加，
# node/（ROS 包装层）在 P1-S4 加。
# 现在就把包建起来，是因为 scripts/gen_map.py 的 L1 测试需要一个能被
# colcon test-result 汇总的家 —— 放在 colcon 之外的测试等于没有测试，
# 因为本项目判定成败一律用 `colcon test-result --all`。
# -----------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 用 clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效，
  # 详细理由见 gazebo_bridge/CMakeLists.txt 与仓库根 .clang-format。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()

  # L1 单元测试：地图生成器的几何内核 + 生成物的结构自洽性。
  # 用 ament_add_pytest_test 而不是裸调 pytest，理由见 ads_bringup/CMakeLists.txt。
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gen_map test/test_gen_map.py)
endif()

ament_package()
