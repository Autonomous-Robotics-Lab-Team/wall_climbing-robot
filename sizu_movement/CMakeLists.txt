cmake_minimum_required(VERSION 3.0.2)
project(sizu_movement)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2_ros
  tf
  tf2_geometry_msgs
  urdf
  geometry_msgs
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(JetsonGPIO)
find_package(X11 REQUIRED)
# find_package(CasADi REQUIRED)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_Overlay_INCLUDE_DIRS}
  ${CasADi_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(gpio_control src/gpio_control.cpp)
## Add dependencies


## Add dependencies

target_link_libraries(gpio_control )
target_link_libraries(gpio_control
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)



add_executable(move src/move.cpp)
target_link_libraries(move JetsonGPIO::JetsonGPIO)
target_link_libraries(move
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  # ${CasADi_LIBRARIES}
)


# add_executable(robot_control src/robot_control.cpp)
# target_link_libraries(robot_control
#   ${catkin_LIBRARIES}
#   ${Eigen3_LIBRARIES}
#   ${CasADi_LIBRARIES}
#   EstimateParameters_lib
# )






add_library(robot_control_lib src/robot_control.cpp)
target_link_libraries(robot_control_lib
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  # ${CasADi_LIBRARIES}
)

## EstimateParameters is a reusable component (no main()), build as a library
add_library(EstimateParameters_lib src/EstimateParameters.cpp)
target_link_libraries(EstimateParameters_lib
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  # ${CasADi_LIBRARIES}
  robot_control_lib
)

add_executable(main src/main.cpp)
target_link_libraries(main
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  # ${CasADi_LIBRARIES}
  robot_control_lib
  EstimateParameters_lib
)

add_executable(keyboard_control src/keyboard_control.cpp)
target_link_libraries(keyboard_control
  ${catkin_LIBRARIES}
)


## EstimateParameters is a reusable component (no main()), build as a library
# add_library(EstimateParameters_lib src/EstimateParameters.cpp)
# target_link_libraries(EstimateParameters_lib
#   ${catkin_LIBRARIES}
#   ${Eigen3_LIBRARIES}
#   ${CasADi_LIBRARIES}
# )
# target_link_libraries(mytarget JetsonGPIO::JetsonGPIO)
# Python version removed, using C++ implementation instead
