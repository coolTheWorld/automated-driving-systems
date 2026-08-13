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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

# 感知方向（**真值**）：config/obstacles.yaml → /perception/obstacles。
# P5 接上真感知后换掉它即可，planning_node 一行不用改 ——
# 话题名与消息类型在 P0a 就定死了，这正是提前定接口的意义。
add_executable(obstacle_truth
  src/obstacle_truth_node.cpp)
ament_target_dependencies(obstacle_truth
  rclcpp
  ads_msgs
  nav_msgs        # P5-S1：动态目标的真值来自 /model/<name>/pose_gt
  tf2
  tf2_geometry_msgs)

# P5-S1：让动态目标沿航点走。**仿真道具，不是算法** —— 它订阅真值是合法的，
# 判断标准是「输出会不会进到被评测的链路里」，见节点文件头。
find_package(ads_common REQUIRED)
add_executable(npc_controller
  src/npc_controller_node.cpp)
ament_target_dependencies(npc_controller
  rclcpp
  geometry_msgs
  nav_msgs)
target_link_libraries(npc_controller ads_common::ads_common)

install(TARGETS lidar_preprocessor vehicle_cmd_bridge obstacle_truth npc_controller
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
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/vehicle_params.yaml
  # 障碍物场景定义。**真值发布器读的就是它**（由 launch 搬运成参数），
  # 而 Gazebo 侧的模型由 scripts/gen_obstacles.py 从同一个文件生成 ——
  # 于是「车看到的」与「车会撞上的」根本不存在第二份数据。
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/obstacles.yaml
  # P5-S1：动态目标场景。同一条规矩 —— 真值发布器与 gen_dynamic_actors.py
  # 读的是同一个文件，所以不存在第二份数据。
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/dynamic_actors.yaml
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

  # 看门狗行为测试：**不需要 Gazebo**（节点用墙钟，测试直接喂 /vehicle_cmd）。
  # 守着「被丢弃的 NaN 指令不许喂狗」—— check_vehicle_cmd.py 抓不到它，
  # 理由见测试文件头。这是本包第一个行为测试（此前只有 lint）。
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_vehicle_cmd_bridge_watchdog.py
    TIMEOUT 60
    ENV
      # ⚠️ 独占一个 ROS_DOMAIN_ID。colcon 按**包**并行跑测试，
      #    分配表：ads_control 41/42/46/48/49、ads_localization 43、
      #    ads_perception 44、gazebo_bridge 45、ads_prediction 47。
      ROS_DOMAIN_ID=45)
endif()

ament_package()
