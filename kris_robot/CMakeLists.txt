cmake_minimum_required(VERSION 3.5)
project(kris_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(kris_robot src/kris_robot.cpp src/main.cpp)
target_link_libraries(kris_robot PRIVATE rclcpp::rclcpp ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${tf2_TARGETS} wiringPi)

install(TARGETS
  kris_robot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()