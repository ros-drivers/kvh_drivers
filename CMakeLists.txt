cmake_minimum_required(VERSION 2.8.3)
project(dsp3000)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs
)

include_directories(cereal_port/include)
add_definitions("-std=c++11 -Wall -Wextra -Werror")

add_subdirectory(cereal_port)

add_executable(dsp3000 src/dsp3000.cpp)
target_link_libraries(dsp3000 ${catkin_LIBRARIES} cereal_port)
find_package(Boost REQUIRED COMPONENTS thread system)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(dsp3000 ${Boost_LIBRARIES})


