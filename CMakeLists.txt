cmake_minimum_required(VERSION 2.8.3)
project(cear_jenni)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  urdf
  std_msgs
  nav_msgs
  message_generation
  cv_bridge
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  nav_msgs
#)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cear_jenni
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs message_runtime
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}  
)

add_executable(pmapping nodes/pmapping.cpp)
add_executable(tmp_mq nodes/tmp_mq.cpp)

add_dependencies(pmapping cear_jenni_generate_messages_cpp)
add_dependencies(tmp_mq cear_jenni_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(pmapping
   ${catkin_LIBRARIES}
   ${cv_bridge_LIBRARIES}
 )

target_link_libraries(tmp_mq
   ${catkin_LIBRARIES}
   ${cv_bridge_LIBRARIES}
 )

