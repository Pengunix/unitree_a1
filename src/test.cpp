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
    MotorData mfl0 = legFL[0]->getMotorData();
    printf("Pos: %lf", mfl0.Pos);
    printf("test");
    MotorData mfl1 = legFL[1]->getMotorData();
    printf("Pos: %lf", mfl1.Pos);
    // MotorData mfl2 = legFL[2]->getMotorData();

    // MotorData mfr0 = legFR[0]->getMotorData();
    // MotorData mfr1 = legFR[1]->getMotorData();
    // MotorData mfr2 = legFR[2]->getMotorData();

    // MotorData mbl0 = legBL[0]->getMotorData();
    // MotorData mbl1 = legBL[1]->getMotorData();
    // MotorData mbl2 = legBL[2]->getMotorData();


    // MotorData mbr0 = legBR[0]->getMotorData();
    // MotorData mbr1 = legBR[1]->getMotorData();
    // MotorData mbr2 = legBR[2]->getMotorData();
    // printf("FL0Pos: %lf \t FL1Pos: %ls \t FL2Pos: %lf \t FR0 Pos: %lf \t FR1Pos: %lf \t FR2Pos: %lf \n", mfl0.Pos, mfl1.Pos, mfl2.Pos, mfr0.Pos, mfr1.Pos, mfr2.Pos);
    // printf("BL0Pos: %lf \t BL1Pos: %ls \t BL2Pos: %lf \t BR0 Pos: %lf \t BR1Pos: %lf \t BR2Pos: %lf \n", mbl0.Pos, mbl1.Pos, mbl2.Pos, mbr0.Pos, mbr1.Pos, mbr2.Pos);

    std::this_thread::sleep_for(10ms);
  }
}