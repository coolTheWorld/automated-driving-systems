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

# 角度归一化。本仓库自己的纯 C++ 公共库，**不是第三方依赖**，
# 且它同样禁止链接 ROS（见 ads_common/package.xml），不会破坏本包的无 ROS 约束。
# 与其在 ads_map 里再写一份 angle_diff，不如用它 —— 同一段逻辑躺两份，
# 迟早会在其中一份里改出分歧，而分歧不会有任何一层报错。
find_package(ads_common REQUIRED)

# -----------------------------------------------------------------------------
# 库：纯 C++17，**不链接任何 ROS 库**
# -----------------------------------------------------------------------------
# SPEC §3.3「算法与 ROS 强制解耦」。P1-S4 加了 node/map_node.cpp，于是本文件
# 下面**确实**出现了 rclcpp —— 但那是 map_node 这个 target 的依赖，
# 本 target（libads_map.so）依然一个 ROS 库都不许链接。
#
# 约束的落点从「find_package 里没有 rclcpp」变成了「本 target 的
# target_link_libraries 里没有 rclcpp」，强度不如从前，所以补了一道机械校验：
# scripts/verify_map.sh 会 ldd 这个 .so，出现任何 ROS 库就失败。
# 靠纪律维持的约束迟早会破，靠脚本维持的不会。
add_library(${PROJECT_NAME} SHARED
  src/geometry.cpp
  src/road_map.cpp
  src/opendrive_parser.cpp
  src/lane_graph.cpp
  src/routing.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 两个都是 PRIVATE 而不是 PUBLIC：tinyxml2 只出现在 opendrive_parser.cpp 里，
# ads_common 只出现在 lane_graph.cpp 里，公开头文件里一个它们的类型都没有。
# 下游包链接 ads_map 时不该被迫知道我们用什么解析 XML、用谁算角度差，
# 也不该在自己的编译期引入它们的头。
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2::tinyxml2 ads_common::ads_common)

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
# 节点：ROS 包装层（P1-S4）
# -----------------------------------------------------------------------------
# 这一层**只做翻译**：ROS 消息 ↔ lib 的调用。算法一行都不在这里，
# 所以它没有自己的单元测试也不可怕 —— 该测的都在 lib 的 27 个 L1 用例里。
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(map_node node/map_node.cpp)
target_link_libraries(map_node ${PROJECT_NAME})
ament_target_dependencies(map_node
  rclcpp
  ament_index_cpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros)

install(TARGETS map_node
  DESTINATION lib/${PROJECT_NAME})

# 把仓库根的地图装进本包的 share。
#
# 与 gazebo_bridge 装 vehicle_params.yaml 是同一个理由：ROS 包安装后必须自包含，
# 不能依赖「工作区恰好挂在 /workspace」这种只在本容器成立的假设 ——
# 云端 CARLA 那侧路径就不一样。colcon --symlink-install 建的是符号链接，
# 所以并没有产生第二份地图，SPEC §4.1 的单一来源仍然成立。
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../maps/campus.xodr
  DESTINATION share/${PROJECT_NAME}/maps)

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

  # 测试要读真实的地图与采样基准。路径经编译期宏传入，而不是靠运行时的
  # 相对路径去猜 —— gtest 的工作目录由 colcon 决定，猜错的表现是
  # 「本地能跑、CI 上文件找不到」，而那时你会先怀疑 CI 的挂载。
  set(ADS_MAP_XODR_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../maps/campus.xodr")

  ament_add_gtest(test_opendrive_parser test/test_opendrive_parser.cpp)
  target_link_libraries(test_opendrive_parser ${PROJECT_NAME})
  target_compile_definitions(test_opendrive_parser PRIVATE
    ADS_MAP_XODR_PATH="${ADS_MAP_XODR_PATH}"
    ADS_MAP_SAMPLES_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/reference_samples.csv")

  # P1-S3：车道图的结构与几何自洽性。
  ament_add_gtest(test_lane_graph test/test_lane_graph.cpp)
  target_link_libraries(test_lane_graph ${PROJECT_NAME})
  target_compile_definitions(test_lane_graph PRIVATE ADS_MAP_XODR_PATH="${ADS_MAP_XODR_PATH}")

  # P1-S3：Dijkstra 路由。
  ament_add_gtest(test_routing test/test_routing.cpp)
  target_link_libraries(test_routing ${PROJECT_NAME})
  target_compile_definitions(test_routing PRIVATE ADS_MAP_XODR_PATH="${ADS_MAP_XODR_PATH}")
endif()

ament_package()
