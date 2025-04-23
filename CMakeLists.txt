cmake_minimum_required(VERSION 3.10)
 
project(serial_port)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
 
 
# add_executable(${PROJECT_NAME} src/main.cpp)
# target_link_libraries(${PROJECT_NAME} m pthread)

add_executable(test test/motor_test.cpp)
target_link_libraries(test m pthread)