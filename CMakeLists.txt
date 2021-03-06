cmake_minimum_required(VERSION 3.5)

project(ros2_test_subscriber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(test_subscriber src/test_subscriber.cpp)
ament_target_dependencies(test_subscriber rclcpp std_msgs std_srvs)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install executable nodes
install(TARGETS test_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
