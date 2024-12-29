#include "motor_msg.h"
#include "uart.hpp"
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

MotorCmd cmd;
MotorData data;

int main() {
  Uart serial("/dev/ttyUSB0");
  // auto uart = std::make_shared<Uart>("/dev/ttyUSB0");
  cmd.id = 0;
  cmd.mode = 10;

  cmd.K_P = 0.002;
  // cmd.K_P = 0.01;
  // cmd.K_W = 0.15;
  cmd.K_W = 0.005;
  cmd.Pos = 3;
  cmd.T = 0.0;
  cmd.W = 0.0;
  uint32_t packs;
  while (1) {
    auto start = std::chrono::high_resolution_clock::now();
    // uart->SendRecv(cmd);
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
