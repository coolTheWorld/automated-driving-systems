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
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 消息包没有版权头可查，但保留 lint 是为了统一：
  # 所有包都跑同一套检查，例外越少越不容易漏。
  #
  # 用 clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效，
  # 详细理由见 gazebo_bridge/CMakeLists.txt 与仓库根 .clang-format。
  # 本包目前没有 .cpp，clang-format 会检查 0 个文件 —— 保留是为了将来加代码时
  # 不必再想起这件事（少一个"忘了配"的机会）。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  # 指向仓库根那份 .clang-format。只能用这个变量，不要手动调 ament_clang_format()
  # ——lint hook 已经自动注册了同名测试，再调一次会撞名。
  # 详细理由见 gazebo_bridge/CMakeLists.txt。
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
