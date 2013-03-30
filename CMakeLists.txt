cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ball_tracking)

set(PCL_DIR "~/pcl/build")
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ball_tracking src/ball_tracking.cpp)
target_link_libraries (ball_tracking ${PCL_LIBRARIES})

