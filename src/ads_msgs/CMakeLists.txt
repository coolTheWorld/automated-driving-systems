cmake_minimum_required(VERSION 3.16)
project(ads_msgs)

# 消息包不编译 C++ 代码，但 rosidl 生成的代码会用到编译选项，
# 所以这里仍然要设。-Wall -Wextra 是 SPEC §7 对全部包的要求。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleCmd.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 消息包没有版权头可查，但保留 lint 是为了统一：
  # 所有包都跑同一套检查，例外越少越不容易漏。
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
