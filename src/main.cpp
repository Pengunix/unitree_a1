#include "motor_msg.h"
#include "uart.hpp"
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

MotorCmd cmd;
MotorData data;

int main() {
  Uart serial("/dev/ttyUSB0");

  cmd.id = 0;
  cmd.mode = 5;

  // cmd.K_P = 0.010;
  cmd.K_P = 0.01;
  // cmd.K_W = 0.15;
  cmd.K_W = 0.05;
  cmd.Pos = 0;
  cmd.T = 0.0;
  cmd.W = 0;
  uint32_t packs;
  while (1) {
    auto start = std::chrono::high_resolution_clock::now();
    serial.SendRecv(cmd);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    usleep(1000);
    MotorData mdata = serial.GetMotorData();
    printf("Rx: %d\tCosts: %ldus\tTemp: %d\tPos: %f\tT: %f\tW: %f\n", packs,
           duration.count(), mdata.Temp, mdata.Pos, mdata.T, mdata.W);
    packs++;
  }
  return 0;
}
