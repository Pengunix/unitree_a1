#include "motor.hpp"
using namespace std::chrono_literals;

int main() {
  std::cout << "0---------------------------------------------------"
            << std::endl;
  Leg legFL("/dev/ttyUSB0", "FL", {0, 1, 2});
  std::cout << "1---------------------------------------------------"
            << std::endl;
  Leg legFR("/dev/ttyUSB1", "FR", {0, 1, 2});
  std::cout << "2---------------------------------------------------"
            << std::endl;
  Leg legBL("/dev/ttyUSB2", "BL", {0, 1, 2});
  std::cout << "3---------------------------------------------------"
            << std::endl;
  Leg legBR("/dev/ttyUSB3", "BR", {0, 1, 2});
  std::cout << "done-------------------------" << std::endl;
  while (1) {
    auto start = std::chrono::high_resolution_clock::now();
    legFL.UpdateMotor(0, 0, 0, 0, 0, 0, 0);
    legFL.UpdateMotor(1, 0, 0, 0, 0, 0, 0);
    legFL.UpdateMotor(2, 0, 0, 0, 0, 0, 0);

    legFR.UpdateMotor(0, 0, 0, 0, 0, 0, 0);
    legFR.UpdateMotor(1, 0, 0, 0, 0, 0, 0);
    legFR.UpdateMotor(2, 0, 0, 0, 0, 0, 0);

    legBL.UpdateMotor(0, 0, 0, 0, 0, 0, 0);
    legBL.UpdateMotor(1, 0, 0, 0, 0, 0, 0);
    legBL.UpdateMotor(2, 0, 0, 0, 0, 0, 0);

    legBR.UpdateMotor(0, 0, 0, 0, 0, 0, 0);
    legBR.UpdateMotor(1, 0, 0, 0, 0, 0, 0);
    legBR.UpdateMotor(2, 0, 0, 0, 0, 0, 0);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    printf("costs %d \n", duration.count());

    std::this_thread::sleep_for(1s);
  }
}