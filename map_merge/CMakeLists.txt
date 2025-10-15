cmake_minimum_required(VERSION 3.5)
project(multirobot_map_merge)

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
find_package(image_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

# we want static linking for now
add_library(combine_grids STATIC
  src/combine_grids/grid_compositor.cpp
  src/combine_grids/grid_warper.cpp
  src/combine_grids/merging_pipeline.cpp
)

# add_dependencies(combine_grids)
target_link_libraries(combine_grids PRIVATE ${OpenCV_LIBS} rclcpp::rclcpp ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${tf2_TARGETS})

add_executable(map_merge src/map_merge.cpp)

# add_dependencies(map_merge ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(map_merge combine_grids ${OpenCV_LIBS} rclcpp::rclcpp ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${tf2_TARGETS} ${map_msgs_TARGETS} ${tf2_geometry_msgs_TARGETS} Boost::thread)
install(TARGETS
  map_merge
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
