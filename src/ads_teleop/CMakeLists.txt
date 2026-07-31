cmake_minimum_required(VERSION 3.16)
project(ads_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ads_msgs REQUIRED)

add_executable(keyboard_teleop src/keyboard_teleop_node.cpp)
ament_target_dependencies(keyboard_teleop rclcpp ads_msgs)

install(TARGETS keyboard_teleop
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# 车辆限值来自仓库根目录的单一来源，装进本包 share 供 launch 读取。
# 理由与 gazebo_bridge 里同样的一段相同：ROS 包安装后应当自包含，
# 不能假设工作区恰好挂在 /workspace。symlink-install 下这是同一个文件，
# 不会产生第二份参数。
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config/vehicle_params.yaml
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
