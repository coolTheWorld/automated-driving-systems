cmake_minimum_required(VERSION 3.16)
project(ads_control)

# SPEC §7：C++17，全量警告。理由见 ads_common/CMakeLists.txt。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 角度归一化与最短角度差。纯 C++、无 ROS，见 package.xml 里的说明。
find_package(ads_common REQUIRED)

# -----------------------------------------------------------------------------
# 库：纯 C++17，**不链接任何 ROS 库**
# -----------------------------------------------------------------------------
# SPEC §3.3「算法与 ROS 强制解耦」。P2-S4 会加 node/control_node.cpp，
# 届时本文件里会出现 rclcpp —— 但那是 control_node 这个 target 的依赖，
# 本 target（libads_control.so）依然一个 ROS 库都不许链接。
#
# 与 ads_map 同构，包括那道机械校验：靠纪律维持的约束迟早会破，靠脚本维持的不会。
# 校验在下面的 BUILD_TESTING 段里（no_ros_dependencies），跟着 colcon test 走，
# 所以 CI 每次推送都会跑。
#
# ⚠️ 这行注释从 P2-S1 起写的是「P2-S4 的 verify_control.sh 会 ldd 这个 .so」，
#    **而那个脚本从来没被写出来过** —— P2-S5 逐个 grep 才发现。
#    也就是说整个 P2 期间这条约束只靠纪律维持，而这段注释自己论证的
#    恰恰是纪律靠不住。教训不是"要仔细"，是**凡是声称"有脚本守着"的地方，
#    都得能被 grep 证伪**。
add_library(${PROJECT_NAME} SHARED
  src/stanley.cpp
  src/speed_profile.cpp
  src/speed_controller.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# PUBLIC 而不是 PRIVATE —— **P3-S1 之后这条变了，别照抄旧注释**。
# 参考线几何下沉到 ads_common 后，本包的公开头文件里出现了
# `#include "ads_common/reference_line.hpp"`，签名里也出现了
# ads_common::ReferenceLine / PathProjection / Pose2D（speed_profile.hpp、stanley.hpp）。
#
# PUBLIC 传递的是**两样东西**，缺任何一样下游都编不过或链不上：
#   ① ads_common 的 include 路径 —— 没有它，下游 #include ads_control/stanley.hpp
#      会在**嵌套的**那一行报 "file not found"，报错位置指向 ads_common 的头，
#      看起来像 ads_common 装坏了，而根因在这一行；
#   ② libads_common.so —— 下游一旦调用 ReferenceLine 的成员就要它。
#
# 顺带一提：本包**自己**的 .cpp 现在不引用 ads_common 的任何**符号**
# （用 angle_diff 的是已搬走的 path_tracking.cpp），所以 libads_control.so 的
# DT_NEEDED 里看不到 libads_common.so。**那是正常的，不是没链上** ——
# 依赖此刻纯粹是头文件层面的，PUBLIC 仍然必需。
target_link_libraries(${PROJECT_NAME} PUBLIC ads_common::ads_common)

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
# 节点：ROS 包装层（P2-S4）
# -----------------------------------------------------------------------------
# 这一层**只做翻译**：ROS 消息 ↔ lib 的调用。控制算法一行都不在这里，
# 所以它没有自己的单元测试也不可怕 —— 该测的都在 lib 的 71 个 L1 用例里。
find_package(rclcpp REQUIRED)
find_package(ads_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(control_node node/control_node.cpp)
target_link_libraries(control_node ${PROJECT_NAME})
ament_target_dependencies(control_node
  rclcpp
  ads_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  diagnostic_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros)

install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME})

# 控制器自己的调参装进本包 share。
#
# ⚠️ **车辆能力参数（轴距/转角/加减速）不在这里**，它们在
#    config/vehicle_params.yaml，由 launch 搬运。两份参数文件的分工是
#    「车能做什么」vs「我们想怎么开」—— 混在一起的话，换一辆车要去
#    控制参数里翻，而那正是 SPEC §4.1「行为漂移」的温床。
#
# 车辆参数也装一份进来（**符号链接，不是拷贝**，SPEC §4.1 的单一来源仍成立）。
# 与 gazebo_bridge / ads_map 装它们各自需要的文件是同一个理由：
# ROS 包安装后必须**自包含**，不能依赖"工作区恰好挂在 /workspace"这种
# 只在本容器成立的假设 —— 云端那侧路径就不一样。
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/control_params.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/vehicle_params.yaml
  DESTINATION share/${PROJECT_NAME}/config)

# -----------------------------------------------------------------------------
# 测试
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 用 clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效
  # —— 那个函数在调用时就把 linter 列表定死了，设晚了不生效**且不报错**。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # L1：横向 Stanley + CP-P2-A 的闭环收敛判据。
  # 闭环用的运动学自行车模型在 test/kinematic_bicycle.hpp —— **测试夹具，非产品代码**，
  # 所以它不进 add_library。P2-S5 若把它包成 ROS 假车节点，那时才搬家。
  ament_add_gtest(test_stanley test/test_stanley.cpp)
  target_link_libraries(test_stanley ${PROJECT_NAME} ads_common::ads_common)
  # kinematic_bicycle.hpp 仍在本包 test/ 下（它是控制专有的被控对象模型）；
  # path_fixtures.hpp 已随参考线搬进 ads_common 的 include/，无需再加路径。
  target_include_directories(test_stanley PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)

  # L1：速度剖面（曲率限速 + 前后向扫描）。纯几何，判据全是闭式解。
  ament_add_gtest(test_speed_profile test/test_speed_profile.cpp)
  target_link_libraries(test_speed_profile ${PROJECT_NAME} ads_common::ads_common)

  # L1：速度环 PI + 抗饱和。含 gazebo_bridge 那段"设定值超前实测不得超过 1 m/s"
  # 的抗饱和环节建模 —— 见 docs/modules/control.md §4.5。
  ament_add_gtest(test_speed_controller test/test_speed_controller.cpp)
  target_link_libraries(test_speed_controller ${PROJECT_NAME})

  # 假车节点：不需要 GPU 的闭环时间源 + 运动学车辆（L3-G，见文件头）。
  # 放在 test/ 且**不 install**：它是测试夹具，不是仿真数据源，
  # 不该出现在 SIM_SOURCES 里，也不该被人误当仿真器用。
  # launch_testing 通过 $<TARGET_FILE:> 直接拿绝对路径，不需要 install。
  find_package(rosgraph_msgs REQUIRED)
  add_executable(fake_vehicle test/fake_vehicle_node.cpp)
  ament_target_dependencies(fake_vehicle
    rclcpp ads_msgs nav_msgs geometry_msgs rosgraph_msgs tf2 tf2_ros)

  # L3-G：不需要 GPU 的端到端闭环（假车 + map_node + control_node）。
  # 验的是**节点接线**（话题/QoS/TF/参数/时序），不是控制律也不是真物理 ——
  # 三层各验各的，见 test_closed_loop.py 的文件头。
  #
  # 假车的可执行文件路径通过环境变量传进去：它**不 install**（是测试夹具，
  # 不是仿真数据源），所以 launch 里没法用 package/executable 的方式找到它。
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_closed_loop.py
    TIMEOUT 120
    ENV FAKE_VEHICLE_EXECUTABLE=$<TARGET_FILE:fake_vehicle>)

  # 机械校验：libads_control.so 不许链接任何 ROS 库（SPEC §3.3）。
  #
  # 做成 CTest 而不是 shell 脚本，因为**一个要人记得去执行的检查等于没有检查** ——
  # colcon test 会跑它，CI 也就跟着跑了。校验逻辑与理由见那个 .cmake 文件。
  add_test(NAME no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DTARGET_FILE=$<TARGET_FILE:${PROJECT_NAME}>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_no_ros_deps.cmake)
endif()

ament_package()
