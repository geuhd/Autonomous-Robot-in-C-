cmake_minimum_required(VERSION 3.8)
project(practical)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_msgs REQUIRED)        
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# The rgpio (lg) library installs to /usr/local -- tell CMake where to find it
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(go_to_x src/go_to_x.cpp)
ament_target_dependencies(go_to_x rclcpp geometry_msgs turtlesim)

# --- Motor controller: needs GPIO, so it links rgpio ---
add_executable(simple_diff_drive src/simple_diff.cpp)
ament_target_dependencies(simple_diff_drive rclcpp std_msgs geometry_msgs)
target_link_libraries(simple_diff_drive rgpio)

# --- Drive controller: pure ROS 2, no GPIO ---
add_executable(simple_drive_controller src/simple_drive_controller.cpp)
ament_target_dependencies(simple_drive_controller rclcpp geometry_msgs nav_msgs)

# install the executables so `ros2 run practical <node>` can find them
install(TARGETS
  go_to_x
  simple_diff_drive
  simple_drive_controller
  DESTINATION lib/${PROJECT_NAME})

# install launch files so `ros2 launch practical <file>` can find them
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

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
