cmake_minimum_required(VERSION 2.8.3)
project(proactive_human_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(proactive_detection src/proactive_detection.cpp)
target_link_libraries(proactive_detection ${catkin_LIBRARIES})
add_dependencies(proactive_detection proactive_human_detection_messages_cpp)
