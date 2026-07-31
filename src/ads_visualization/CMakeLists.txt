cmake_minimum_required(VERSION 3.16)
project(ads_visualization)

find_package(ament_cmake REQUIRED)

# 纯资源包：不编译任何代码，只把 URDF 和 RViz 配置装到 share 下，
# 让别的包能用 get_package_share_directory() 找到它们。
install(DIRECTORY urdf rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
