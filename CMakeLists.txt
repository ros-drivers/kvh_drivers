cmake_minimum_required(VERSION 2.8.3)
project(kvh)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp roslint std_msgs)

catkin_package()

set_directory_properties(PROPERTIES INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/cereal_port/include;${catkin_INCLUDE_DIRS}")

if (CMAKE_COMPILER_IS_GNUCXX)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Wextra;-Werror")
endif()

include_directories(${Boost_INCLUDE_DIRS} include)
add_executable(dsp3000 src/dsp3000.cpp src/serial_port.cpp include/kvh/serial_port.h)
target_link_libraries(dsp3000 ${catkin_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS thread system)
target_link_libraries(dsp3000 ${Boost_LIBRARIES})

install(TARGETS dsp3000
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

roslint_cpp()
roslint_add_test()
