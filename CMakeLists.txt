cmake_minimum_required(VERSION 3.8)
project(nav2_mps_layer_plugin)

set(lib_name ${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
    rclcpp
    nav2_costmap_2d
    pluginlib)

add_library(${lib_name} SHARED
    src/mps_layer.cpp)

include_directories(include
                    include/nav2_mps_layer_plugin)

install(TARGETS ${lib_name}
        DESTINATION lib)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_costmap_2d nav2_mps_layer_plugin.xml)
ament_target_dependencies(${lib_name} ${dependencies})

ament_package()
