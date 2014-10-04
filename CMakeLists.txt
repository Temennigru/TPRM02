cmake_minimum_required(VERSION 2.8.3)
project(tp1)

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
# add_executable(diffcontrol src/diff_control.cpp)
add_dependencies(control tp1_control)
# add_dependencies(diffcontrol tp1_diffcontrol)
target_link_libraries(control ${catkin_LIBRARIES})
# target_link_libraries(diffcontrol ${catkin_LIBRARIES})

