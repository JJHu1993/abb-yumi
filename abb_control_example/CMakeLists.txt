cmake_minimum_required(VERSION 2.8.3)
project(abb_control_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin
  REQUIRED
  roscpp
  trajectory_msgs)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  trajectory_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(ikSolverVel_control_node src/ikSolverVel_control_node.cpp)
add_executable(jointTrajectory_control_node src/jointTrajectory_control_node.cpp)

target_link_libraries(ikSolverVel_control_node ${catkin_LIBRARIES})
target_link_libraries(jointTrajectory_control_node ${catkin_LIBRARIES})
#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
