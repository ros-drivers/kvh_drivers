cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_add_boost_directories()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(cereal_port/include)
rosbuild_add_executable(dsp3000 src/dsp3000.cpp)
target_link_libraries(dsp3000 cereal_port)
rosbuild_link_boost(dsp3000 thread)

add_subdirectory(cereal_port)
