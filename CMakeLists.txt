cmake_minimum_required(VERSION 2.8.3)
project(tp1)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  genmsg
  message_generation
)

add_message_files(FILES ccoord.msg)
# add_service_files(FILES AddTwoInts.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(control Robot/control.cpp)
add_executable(path Graph/main.cpp Graph/Graph2D.cpp Graph/VisibilityGraph.cpp Graph/RRT.cpp Graph/PGM.cpp Graph/SegmentIntersect.cpp Graph/MovementRestrictions.cpp Robot/pointPublisher.cpp)
add_dependencies(control tp1_control)
add_dependencies(path tp1_path )
target_link_libraries(control ${catkin_LIBRARIES})
target_link_libraries(path ${catkin_LIBRARIES})


