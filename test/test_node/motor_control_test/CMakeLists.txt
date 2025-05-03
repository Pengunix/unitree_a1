cmake_minimum_required(VERSION 3.0.2)
project(motor_control_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  unitree_a1
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES unitree_a1
  CATKIN_DEPENDS roscpp std_msgs 
  #  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


add_executable(Control_test src/control.cpp)
target_link_libraries(Control_test m pthread ${catkin_LIBRARIES})

# add_executable(test test/motor_test.cpp)
# target_link_libraries(test m pthread)

install(TARGETS Control_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
