cmake_minimum_required(VERSION 3.8)
project(turtlebot_actionlib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(maze_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(navtopose REQUIRED)

find_library(nav2 nav2pose_lib)
include_directories( include )
add_library( maze SHARED src/burger_action.cpp src/waffle_action.cpp src/maze_action.cpp)
ament_target_dependencies(maze rclcpp rclcpp_action maze_msgs navtopose)

add_executable( server src/maze_server.cpp )
target_link_libraries( server maze ${nav2})
ament_target_dependencies( server rclcpp rclcpp_action maze_msgs navtopose nav2_msgs)

add_executable( client src/maze_client.cpp )
target_link_libraries( client maze ${nav2})
ament_target_dependencies( client rclcpp rclcpp_action maze_msgs navtopose nav2_msgs)

install( DIRECTORY include/ DESTINATION include )
install( TARGETS maze  LIBRARY DESTINATION lib )
install( TARGETS server client  DESTINATION lib/${PROJECT_NAME} )

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

ament_package()
