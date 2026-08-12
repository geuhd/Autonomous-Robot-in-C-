cmake_minimum_required(VERSION 3.8)
project(practical_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(simple_diff_drive src/simple_diff_drive.cpp)
ament_target_dependencies(simple_diff_drive rclcpp std_msgs geometry_msgs)
target_link_libraries(simple_diff_drive rgpio)

add_executable(tick_publisher src/tick_publisher.cpp)
ament_target_dependencies(tick_publisher rclcpp std_msgs)
target_link_libraries(tick_publisher rgpio)

add_executable(tf_luna_publisher src/tf_luna_publisher.cpp)
# Link the ROS 2 dependencies to the executable
ament_target_dependencies(tf_luna_publisher rclcpp sensor_msgs)

add_executable(imu_publisher src/imu_publisher.cpp)
ament_target_dependencies(imu_publisher rclcpp sensor_msgs)
target_link_libraries(imu_publisher rgpio)

# Install all the executables so ROS 2 can find them
install(TARGETS
  simple_diff_drive
  tick_publisher
  imu_publisher
  tf_luna_publisher
  DESTINATION lib/${PROJECT_NAME})
  
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
