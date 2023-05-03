cmake_minimum_required(VERSION 3.8)
project(navtopose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories( include)
ament_export_include_directories(include)
add_library( nav2pose_lib SHARED src/NavToPose.cpp )
ament_target_dependencies( nav2pose_lib rclcpp rclcpp_action geometry_msgs nav2_msgs )

# add_executable( nav2pose_server src/NavToPose_server.cpp )
# target_link_libraries( nav2pose_server nav2pose_lib )

# add_executable( nav2pose_client src/NavToPose_client.cpp )
# target_link_libraries( nav2pose_client nav2pose_lib )

install( DIRECTORY include/ DESTINATION include )
install( TARGETS nav2pose_lib LIBRARY DESTINATION lib )
# install( TARGETS nav2pose_server nav2pose_client DESTINATION lib/${PROJECT_NAME} )

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
