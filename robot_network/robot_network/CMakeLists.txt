cmake_minimum_required(VERSION 3.10)
project(MapMerger)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Add executable
add_executable(map_merger map_merger.cpp)

# Link OpenCV libraries
target_link_libraries(map_merger ${OpenCV_LIBS})
