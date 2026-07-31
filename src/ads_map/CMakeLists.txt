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

# XML 解析。tinyxml2 已经在镜像里，且是 urdf / rviz_common / sdformat_urdf
# 的既有依赖 —— 用它**不引入新的第三方依赖**。
# 按 ROS 2 惯例先 find vendor 包（它保证 tinyxml2 能被找到并补齐导入目标），
# 再 find 系统配置拿到 tinyxml2::tinyxml2 这个现代导入目标。
find_package(tinyxml2_vendor REQUIRED)
find_package(tinyxml2 REQUIRED)

# -----------------------------------------------------------------------------
# 库：纯 C++17，**不链接任何 ROS 库**
# -----------------------------------------------------------------------------
# SPEC §3.3「算法与 ROS 强制解耦」。node/（ROS 包装层）在 P1-S4 才加，
# 到时候它是**另一个** target，本 target 依然不许出现 rclcpp。
#
# find_package 里刻意没有 rclcpp —— 靠构建系统把这条约束钉死，比写在文档里管用：
# 谁想加 ROS 依赖，必须先改这个文件，改的时候就会看到这段。
add_library(${PROJECT_NAME} SHARED
  src/geometry.cpp
  src/road_map.cpp
  src/opendrive_parser.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# PRIVATE 而不是 PUBLIC：tinyxml2 只出现在 opendrive_parser.cpp 里，
# 公开头文件一个 tinyxml2 的类型都没有。下游包链接 ads_map 时
# 不该被迫知道我们用什么解析 XML，也不该在自己的编译期引入它的头。
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2::tinyxml2)

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
  # 用 clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效，
  # 详细理由见 gazebo_bridge/CMakeLists.txt 与仓库根 .clang-format。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()

  # L1 单元测试：地图生成器的几何内核 + 生成物的结构自洽性（纯 Python）。
  # 用 ament_add_pytest_test 而不是裸调 pytest，理由见 ads_bringup/CMakeLists.txt。
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gen_map test/test_gen_map.py)

  # L1 单元测试：C++ 侧的几何求值与 OpenDRIVE 解析。
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry ${PROJECT_NAME})

  ament_add_gtest(test_opendrive_parser test/test_opendrive_parser.cpp)
  target_link_libraries(test_opendrive_parser ${PROJECT_NAME})
  # 测试要读真实的地图与采样基准。路径经编译期宏传入，而不是靠运行时的
  # 相对路径去猜 —— gtest 的工作目录由 colcon 决定，猜错的表现是
  # 「本地能跑、CI 上文件找不到」，而那时你会先怀疑 CI 的挂载。
  target_compile_definitions(test_opendrive_parser PRIVATE
    ADS_MAP_XODR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../maps/campus.xodr"
    ADS_MAP_SAMPLES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/reference_samples.csv")
endif()

ament_package()
