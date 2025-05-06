cmake_minimum_required(VERSION 3.10)

project(unitree_a1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(FILES MotorCmd.msg MotorData.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES unitree_a1
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  #  DEPENDS system_lib
)
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


add_executable(Motor_node src/Motor_node.cpp)
target_link_libraries(Motor_node m pthread ${catkin_LIBRARIES})

# add_executable(test test/motor_test.cpp)
# target_link_libraries(test m pthread)

install(TARGETS Motor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
