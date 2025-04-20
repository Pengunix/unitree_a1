#include "motor_msg.h"
#include "uart.hpp"
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <thread>
#include <unistd.h>
#include <math.h>

MotorCmd cmd;
MotorData data;

int main() {
  auto uart = std::make_shared<Uart>("/dev/ttyUSB0");
  cmd.id = 1;
  cmd.mode = 10;

  cmd.K_P = 0.005;
  // cmd.K_P = 0.01;
  // cmd.K_W = 0.15;
  cmd.K_W = 0.005;
  cmd.Pos = 0;
  cmd.T = 0.0;
  cmd.W = 0.0;
  MotorData mdata;
  for (int i=0;i<100;i++) {
    cmd.mode = 0;
    uart->SendRecv(cmd);
    MotorData zero_state = uart->GetMotorData();
    printf("zero state pos: %lf \n", zero_state.Pos);
    usleep(1000);
  }
  cmd.mode = 10;
  while (1) {
    for (int i = 0; i < 3; i++) {
      auto start = std::chrono::high_resolution_clock::now();
      do {
        cmd.Pos = i * 2;
        uart->SendRecv(cmd);
        mdata = uart->GetMotorData();
        printf("diff: %lf \t pos: %lf \n", fabs(mdata.Pos - cmd.Pos), mdata.Pos);

      } while (fabs(mdata.Pos - cmd.Pos) < 1);
      auto end = std::chrono::high_resolution_clock::now();
      auto duration =
          std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      printf("costs %d \n", duration.count());
      sleep(1);
    }
    for (int i = 3; i > 0; i--) {
      auto start = std::chrono::high_resolution_clock::now();
      do {
        cmd.Pos = i * 2;
        uart->SendRecv(cmd);
        printf("diff: %lf \t pos: %lf \n", fabs(mdata.Pos - cmd.Pos), mdata.Pos);
        mdata = uart->GetMotorData();

      } while (fabs(mdata.Pos - cmd.Pos) < 1);
      auto end = std::chrono::high_resolution_clock::now();
      auto duration =
          std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      printf("costs %d \n", duration.count());
      sleep(1);
    }
  }
  return 0;
}
