#include "motor.hpp"
#include "unitree_a1/MotorCmd.h"
#include "unitree_a1/MotorData.h"
#include <mutex>
#include <ros/console.h>
#include <ros/ros.h>

constexpr std::array<int, 3> FL_dir = {1, 1, 1};
constexpr std::array<int, 3> FR_dir = {1, -1, -1};
constexpr std::array<int, 3> BL_dir = {-1, 1, 1};
constexpr std::array<int, 3> BR_dir = {-1, -1, -1};
std::mutex cmdMutex;

std::array<MotorCmd, 3> FL_cmd;
std::array<MotorCmd, 3> FR_cmd;
std::array<MotorCmd, 3> BL_cmd;
std::array<MotorCmd, 3> BR_cmd;
MotorCmd tmpcmd;

std::shared_ptr<Leg> legFL;
std::shared_ptr<Leg> legFR;
std::shared_ptr<Leg> legBL;
std::shared_ptr<Leg> legBR;

ros::Publisher MotorStatePub;

void MotorCmdCallback(const unitree_a1::MotorCmd::ConstPtr &msg);
void SigintHandler(int sig);

void pubState() {
  unitree_a1::MotorData PubMsg;
  std::shared_ptr<Leg> legs[4] = {legFL, legFR, legBL, legBR};
  // 仅解算姿态
  for (int legid = 0; legid <= 3; legid++) {
    for (int motorid = 0; motorid <= 2; motorid++) {
      MotorData tmpdata = legs[legid]->getMotorState(motorid);
      PubMsg.legid.emplace_back(legid);
      PubMsg.motorid.emplace_back(motorid);
      PubMsg.error.emplace_back(tmpdata.MError);
      PubMsg.acc.emplace_back(tmpdata.Acc);
      PubMsg.mode.emplace_back(tmpdata.mode);
      PubMsg.pos.emplace_back(tmpdata.Pos);
      PubMsg.tau.emplace_back(tmpdata.T);
      PubMsg.vel.emplace_back(tmpdata.W);
      PubMsg.temp.emplace_back(tmpdata.Temp);
    }
  }
  MotorStatePub.publish(PubMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Motor_node");
  ros::NodeHandle nodeHandle;

  ros::Rate loop_rate(200);
  signal(SIGINT, SigintHandler);

  ROS_INFO("FL---------------------------------------------------");
  legFL = std::make_shared<Leg>("/dev/ttyUSB0", "FL", FL_dir);
  ROS_INFO("FR---------------------------------------------------");
  legFR = std::make_shared<Leg>("/dev/ttyUSB1", "FR", FR_dir);
  ROS_INFO("BL---------------------------------------------------");
  legBL = std::make_shared<Leg>("/dev/ttyUSB2", "BL", BL_dir);
  ROS_INFO("BR---------------------------------------------------");
  legBR = std::make_shared<Leg>("/dev/ttyUSB3", "BR", BR_dir);
  ROS_INFO("Done------------------------------------------------");

  // 这里没有从电机位置中读取上电解算数据，需电机命令发布者先获取零状态数据，再切换电机模式
  // 上电获取数据发送全零数据即可
  tmpcmd.mode = 0;
  tmpcmd.K_P = 0;
  tmpcmd.K_W = 0;
  tmpcmd.Pos = 0;
  tmpcmd.W = 0;
  tmpcmd.T = 0;
  FL_cmd.fill(tmpcmd);
  FR_cmd.fill(tmpcmd);
  BL_cmd.fill(tmpcmd);
  BR_cmd.fill(tmpcmd);

  ros::Subscriber MotorCmdSub =
      nodeHandle.subscribe("/motor_cmd", 100, MotorCmdCallback);
  MotorStatePub =
      nodeHandle.advertise<unitree_a1::MotorData>("/motor_state", 100);
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  while (ros::ok()) {

    for (int i = 0; i < 3; i++) {
      // 需解算位置数据为电机真实数据
      std::lock_guard<std::mutex> lck(cmdMutex);
      legFL->UpdateMotor(i, FL_cmd[i].mode, FL_cmd[i].K_P, FL_cmd[i].K_W,
                         FL_cmd[i].Pos, FL_cmd[i].T, FL_cmd[i].W);
      legFR->UpdateMotor(i, FR_cmd[i].mode, FR_cmd[i].K_P, FR_cmd[i].K_W,
                         FR_cmd[i].Pos, FR_cmd[i].T, FR_cmd[i].W);
      legBL->UpdateMotor(i, BL_cmd[i].mode, BL_cmd[i].K_P, BL_cmd[i].K_W,
                         BL_cmd[i].Pos, BL_cmd[i].T, BL_cmd[i].W);
      legBR->UpdateMotor(i, BR_cmd[i].mode, BR_cmd[i].K_P, BR_cmd[i].K_W,
                         BR_cmd[i].Pos, BR_cmd[i].T, BR_cmd[i].W);
    }
    pubState();
    loop_rate.sleep();
  }
}

inline void copyCmd(int index, std::array<MotorCmd, 3> &cmd,
                    const unitree_a1::MotorCmd::ConstPtr &msg) {
  cmd.at(msg->motorid[index]).mode = msg->mode[index];
  // 传入的Kp kw均为输出轴 故转子kp kw由此计算得来 9.1为减速比
  cmd.at(msg->motorid[index]).K_P = (msg->kp[index] / (9.1 * 9.1)) / 26.07;
  cmd.at(msg->motorid[index]).K_W = (msg->kd[index] / (9.1 * 9.1)) * 100.0;
  cmd.at(msg->motorid[index]).Pos = msg->pos[index];
  cmd.at(msg->motorid[index]).T = msg->tau[index];
  cmd.at(msg->motorid[index]).W = msg->vel[index];
}
void MotorCmdCallback(const unitree_a1::MotorCmd::ConstPtr &msg) {
  std::lock_guard<std::mutex> lck(cmdMutex);
  for (int i = 0; i < msg->legid.size(); i++) {
    if (msg->legid[i] == 0) {
      copyCmd(i, FL_cmd, msg);
    } else if (msg->legid[i] == 1) {
      copyCmd(i, FR_cmd, msg);
    } else if (msg->legid[i] == 2) {
      copyCmd(i, BL_cmd, msg);
    } else if (msg->legid[i] == 3) {
      copyCmd(i, BR_cmd, msg);
    }
  }
}

void SigintHandler(int sig) {
  ROS_INFO("shutting down!");
  ros::shutdown();
  exit(0);
}
