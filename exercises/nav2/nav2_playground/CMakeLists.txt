cmake_minimum_required(VERSION 3.20)
project(nav2_playground)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_common REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_executable(waypoint_client_node src/nav2_playground/waypoint_client_node.cpp)
ament_target_dependencies(waypoint_client_node rclcpp rclcpp_action geometry_msgs nav2_msgs tf2_geometry_msgs)

add_executable(patrolling_node src/nav2_playground/patrolling_node.cpp src/patrolling_node_main.cpp)
ament_target_dependencies(patrolling_node rclcpp rclcpp_action geometry_msgs nav2_msgs tf2_geometry_msgs)

add_executable(patrolling_node_template src/nav2_playground/patrolling_node_template.cpp src/patrolling_node_main.cpp)
ament_target_dependencies(patrolling_node_template rclcpp rclcpp_action geometry_msgs nav2_msgs tf2_geometry_msgs)

add_library(${library_name} SHARED
  src/plugins/print_pick_task_executor.cpp
)
ament_target_dependencies(${library_name}
rclcpp rclcpp_lifecycle geometry_msgs nav2_waypoint_follower nav2_core pluginlib nav2_msgs
)
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_waypoint_follower plugins.xml)

install(TARGETS waypoint_client_node patrolling_node patrolling_node_template DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

install(DIRECTORY include/
DESTINATION include
)
install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/waypoint_client_node.py
  scripts/patrolling_node.py
  scripts/patrolling_node_template.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(ament_cmake
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav2_waypoint_follower
  nav2_core
  pluginlib
  rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_launch_test(
    test/test_slam_launch.py
    TIMEOUT 120
  )

  add_launch_test(
    test/test_patrol_launch.py
    TIMEOUT 180
  )
endif()

ament_package()
