cmake_minimum_required(VERSION 3.8)
project(biti)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  std_msgs
  std_srvs
)  

include_directories(include)

add_executable(simple_example src/simpleExample.cpp)
add_executable(crossdoor src/main_crossdoor.cpp src/lib/crossdoor_behavior.cpp)
add_executable(crossdoor_with_ros src/crossdoor_with_ros.cpp src/lib/crossdoor_behavior.cpp)
# add_executable(crossdoor_with_ros_v2 src/main.cpp src/lib/crossdoor_node.cpp)

ament_target_dependencies(simple_example ${dependencies})
ament_target_dependencies(crossdoor ${dependencies})
ament_target_dependencies(crossdoor_with_ros ${dependencies})

install(TARGETS
  simple_example
  crossdoor
  crossdoor_with_ros
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
