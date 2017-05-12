cmake_minimum_required(VERSION 2.8.3)
project(abb_launch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  abb_description
  abb_communication_interface
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yumi_launch
  CATKIN_DEPENDS abb_description abb_communication_interface
#  DEPENDS system_lib
)


#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_yumi_launch.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
