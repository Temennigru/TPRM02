cmake_minimum_required(VERSION 2.8.3)
project(tp2)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  genmsg
  message_generation
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(explore Robot/explore.cpp Graph/PGM.cpp)
add_dependencies(explore tp2_explore)
target_link_libraries(explore ${catkin_LIBRARIES})
