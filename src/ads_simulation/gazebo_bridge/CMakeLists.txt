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
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(lidar_preprocessor
  src/lidar_preprocessor_node.cpp)
ament_target_dependencies(lidar_preprocessor
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs)

install(TARGETS lidar_preprocessor
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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
