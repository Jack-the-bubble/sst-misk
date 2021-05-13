cmake_minimum_required(VERSION 3.0.2)
project(basic_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  roscpp
  sst_interfaces
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller ${PROJECT_SOURCE_DIR}/src/controller.cc)
target_link_libraries(controller
  ${catkin_LIBRARIES}
)
# add_dependencies(controller sst_interfaces)



