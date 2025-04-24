#include "motor.hpp"
using namespace std::chrono_literals;
constexpr std::array<int, 3> FL_dir = {1, 1, 1};
constexpr std::array<int, 3> FR_dir = {1, -1, -1};
constexpr std::array<int, 3> BL_dir = {-1, 1, 1};
constexpr std::array<int, 3> BR_dir = {-1, -1, -1};

int main() {
  std::cout << "FL---------------------------------------------------"
           << std::endl;
  Leg legFL("/dev/ttyUSB0", "FL", FL_dir);
  std::cout << "FR---------------------------------------------------"
            << std::endl;
  Leg legFR("/dev/ttyUSB1", "FR", FR_dir);
  std::cout << "BL---------------------------------------------------"
            << std::endl;
  Leg legBL("/dev/ttyUSB2", "BL", BL_dir);
  std::cout << "BR---------------------------------------------------"
            << std::endl;
  Leg legBR("/dev/ttyUSB3", "BR", BR_dir);
  std::cout << "Done------------------------------------------------" << std::endl;
  while (1) {
    auto start = std::chrono::high_resolution_clock::now();
    legFL.UpdateMotor(0, 0, 0, 0, 0, 0, 0);
    legFL.UpdateMotor(1, 0, 0, 0, 0, 0, 0);
    legFL.UpdateMotor(2, 0, 0.005, 0.005, 1, 0, 0);

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
    MotorData mfl0 = legFL[0]->getMotorData();
    printf("FL0id: %d, FL0Pos: %lf \n", mfl0.motor_id, mfl0.Pos);

    // MotorData mfl1 = legFL[1]->getMotorData();
    // printf("FL1Pos: %lf \t", mfl1.Pos);
    // MotorData mfl2 = legFL[2]->getMotorData();
    // printf("FL2Pos: %lf \t", mfl2.Pos);

    // MotorData mfr0 = legFR[0]->getMotorData();
    // printf("FR0Pos: %lf \t", mfr0.Pos);
    // MotorData mfr1 = legFR[1]->getMotorData();
    // printf("FR1Pos: %lf \t", mfr1.Pos);
    // MotorData mfr2 = legFR[2]->getMotorData();
    // printf("FR2Pos: %lf \n", mfr2.Pos);

    // MotorData mbl0 = legBL[0]->getMotorData();
    // printf("BL0Pos: %lf \t", mbl0.Pos);
    // MotorData mbl1 = legBL[1]->getMotorData();
    // printf("BL1Pos: %lf \t", mbl1.Pos);
    // MotorData mbl2 = legBL[2]->getMotorData();
    // printf("BL2Pos: %lf \t", mbl2.Pos);

    // MotorData mbr0 = legBR[0]->getMotorData();
    // printf("BR0Pos: %lf \t", mbr0.Pos);
    // MotorData mbr1 = legBR[1]->getMotorData();
    // printf("BR1Pos: %lf \t", mbr1.Pos);
    // MotorData mbr2 = legBR[2]->getMotorData();
    // printf("BR1Pos: %lf \n", mbr2.Pos);

    std::this_thread::sleep_for(10ms);
  }
}