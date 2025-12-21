cmake_minimum_required(VERSION 3.0.2)
project(motor)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)


catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES motor_src
  CATKIN_DEPENDS roscpp sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include  # 添加 libcanbus 头文件路径
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  /usr/local/lib  # 添加 libcanbus 库路径
)

# 查找 libcanbus 库
find_library(LIBCANBUS_LIB canbus
  PATHS /usr/local/lib
  NO_DEFAULT_PATH
)

# add_library(motor_src SHARED src/can_kvaser.cpp)

add_executable(motor_control src/motor_control.cpp)
add_executable(can_bus src/can_bus.cpp)
# add_executable(can_kvaser src/can_kvaser.cpp)

# target_link_libraries(motor_control 
#   ${CMAKE_CURRENT_SOURCE_DIR}/lib/libmotor_src.so 
#   ${catkin_LIBRARIES}
# )

target_link_libraries(can_bus 
    ${catkin_LIBRARIES}
    ${LIBCANBUS_LIB}  # 链接 libcanbus
    usb-1.0
)

target_link_libraries(motor_control 
    ${catkin_LIBRARIES}
    ${LIBCANBUS_LIB}  # 链接 libcanbus
    usb-1.0
)

# target_link_libraries(can_kvaser 
#   ${catkin_LIBRARIES} 
#   canlib
# )

# target_link_libraries(motor_src 
#   libcanlib.so 
#   libcanlib.so.1
# )

# set_target_properties(motor_src PROPERTIES
#   LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
# )

#############
## Install ##
#############
# ... [保留原有安装部分] ... 