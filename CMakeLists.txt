cmake_minimum_required(VERSION 3.2)
 
# 指定项目的名称
project(serial_port)
 
# 头文件目录
include_directories(include)
 
# 源文件目录
aux_source_directory(src SRC_FILES)
 
# 设置包含所有源文件的变量
set(SOURCES ${SRC_FILES})
 
# 添加要编译的可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})
 
# 添加可执行文件所需要的库（例如数学库）
target_link_libraries(${PROJECT_NAME} m)