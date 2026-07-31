cmake_minimum_required(VERSION 3.16)
project(gazebo_bridge)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(ads_msgs REQUIRED)

# 传感器方向：Gazebo → ROS
add_executable(lidar_preprocessor
  src/lidar_preprocessor_node.cpp)
ament_target_dependencies(lidar_preprocessor
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs)

# 控制方向：ROS → Gazebo
add_executable(vehicle_cmd_bridge
  src/vehicle_cmd_bridge_node.cpp)
ament_target_dependencies(vehicle_cmd_bridge
  rclcpp
  geometry_msgs
  nav_msgs
  ads_msgs)

install(TARGETS lidar_preprocessor vehicle_cmd_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

# 把仓库根目录的车辆参数装进本包的 share。
#
# 为什么要这么绕：launch 需要车身尺寸来算自车包围盒，而 config/vehicle_params.yaml
# 在仓库根目录、不在本包里。ROS 包安装后应当自包含，不能依赖「工作区恰好挂在
# /workspace」这种只在本容器成立的假设 —— 云端 CARLA 那侧路径就不一样。
#
# 装进来的是**同一个文件**（colcon --symlink-install 会建符号链接），
# 所以并没有产生第二份参数，SPEC §4.1 的单一来源仍然成立。
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/vehicle_params.yaml
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # clang-format 取代 ROS 2 默认的 uncrustify —— 理由见仓库根 .clang-format 顶部。
  # 两者规则不同，同时开会互相要求改回去，所以必须显式排掉一个。
  #
  # ⚠️ 这行必须在 ament_lint_auto_find_test_dependencies() **之前**：
  #    那个函数在被调用的一刻就把 linter 列表定死了，之后再设变量没有任何效果，
  #    而且不会报错 —— 表现为「明明排除了 uncrustify，它还在跑」。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)

  # 让 clang-format 用仓库根那一份配置，而不是它自带的内置默认值。
  # 目的是**单一来源**：编辑器（从当前文件向上查找 .clang-format）和 CI
  # 用的是同一个文件。否则就成了"仓库一份、工具内置一份"，靠纪律维持一致 ——
  # 这个项目在车辆参数上已经吃过同类的亏（SPEC §4.1）。
  # 而且本项目的配置刻意偏离了官方（IncludeBlocks: Preserve），不指过来必然不一致。
  #
  # ⚠️ 只能用这个变量，**不要**写 ament_clang_format(CONFIG_FILE ...)：
  #    package.xml 里声明 test_depend 后，lint hook 会自动注册一个名为
  #    clang_format 的测试，手动再调一次就撞名，报
  #    `add_test given test NAME "clang_format" which already exists`。
  # 路径里的 ../../.. 是从 src/ads_simulation/gazebo_bridge/ 回到仓库根。
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../../.clang-format)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
