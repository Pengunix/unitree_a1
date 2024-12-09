#include "unitreeA1_cmd.h"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <chrono>
#include "serial_port.h"

motor_send_t cmd; 
motor_recv_t data; 


int main() {
    int fd = serial_init("/dev/ttyUSB0");  ///dev/ttyCH343USB2
    if(fd<0)
    {
        std::cout <<"serial init error!" <<std::endl;
        return 1;
    }
    else
    {
        std::cout <<"serial init success!" <<fd<<std::endl;
    }

    
    cmd.id=2;
    cmd.mode=10;

    cmd.K_P=0.05;
    cmd.K_W=0.01;
    cmd.Pos=0;
    cmd.T=0.;
    cmd.W=0;
    

    // 获取起始时间点
    auto start = std::chrono::high_resolution_clock::now();
    for(int i = 0; i<1000; i++)
    {
        serial_SendRecv(fd, cmd, data);
        //usleep(200);
    }

    // 获取结束时间点
    auto end = std::chrono::high_resolution_clock::now();

    // 计算耗时（以毫秒为单位）
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // 打印结果
    std::cout << "程序运行时间: " << duration.count() << " us" << std::endl;
    
    while(1) 
    {   
        // cmd.id=2;
        // cmd.T = 0.20;
        serial_SendRecv(fd, cmd, data);
        //usleep(200);
        // //printf("actual w: %f\n", data.W);
    }
    
    close(fd);
    return 0;

    
}


