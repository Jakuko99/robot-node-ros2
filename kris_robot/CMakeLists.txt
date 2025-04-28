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

add_executable(kris_robot src/kris_robot.cpp)
target_link_libraries(kris_robot PRIVATE rclcpp::rclcpp ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${tf2_TARGETS})

install(TARGETS
  kris_robot
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(kris_robot)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# add_executable(kris_robot src/kris_robot.cpp)
# target_include_directories(kris_robot PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# target_compile_features(kris_robot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS kris_robot
# DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
# find_package(ament_lint_auto REQUIRED)
# # the following line skips the linter which checks for copyrights
# # comment the line when a copyright and license is added to all source files
# set(ament_cmake_copyright_FOUND TRUE)
# # the following line skips cpplint (only works in a git repo)
# # comment the line when this package is in a git repo and when
# # a copyright and license is added to all source files
# set(ament_cmake_cpplint_FOUND TRUE)
# ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
