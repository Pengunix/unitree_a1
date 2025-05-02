#include "motor.hpp"
#include "unitree_a1/MotorCmd.h"
#include "unitree_a1/MotorData.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <functional>
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
    loop_rate.sleep();
  }
}
inline void copyCmd(std::array<MotorCmd, 3> &cmd,
                    const unitree_a1::MotorCmd::ConstPtr &msg) {
  cmd.at(msg->motorid).mode = msg->mode;
  cmd.at(msg->motorid).K_P = msg->kp;
  cmd.at(msg->motorid).K_W = msg->kd;
  cmd.at(msg->motorid).Pos = msg->pos;
  cmd.at(msg->motorid).T = msg->tau;
  cmd.at(msg->motorid).W = msg->vel;
}

void pubState(std::shared_ptr<Leg> leg,
              const unitree_a1::MotorCmd::ConstPtr &msg) {
  unitree_a1::MotorData PubMsg;
  MotorData mdata = leg->getMotorState(msg->motorid);
  // 仅解算姿态
  PubMsg.legid = msg->legid;
  PubMsg.motorid = msg->motorid;
  PubMsg.error = mdata.MError;
  PubMsg.mode = mdata.mode;
  PubMsg.pos = mdata.Pos;
  PubMsg.tau = mdata.T;
  PubMsg.vel = mdata.W;
  PubMsg.temp = mdata.Temp;
  MotorStatePub.publish(PubMsg);
}
void MotorCmdCallback(const unitree_a1::MotorCmd::ConstPtr &msg) {
  std::lock_guard<std::mutex> lck(cmdMutex);
  if (msg->legid == 0) {
    copyCmd(FL_cmd, msg);
    pubState(legFL, msg);
  } else if (msg->legid == 1) {
    copyCmd(FR_cmd, msg);
    pubState(legFR, msg);
  } else if (msg->legid == 2) {
    copyCmd(BL_cmd, msg);
    pubState(legBL, msg);
  } else if (msg->legid == 3) {
    copyCmd(BR_cmd, msg);
    pubState(legBR, msg);
  }
}

void SigintHandler(int sig) {
  ROS_INFO("shutting down!");
  ros::shutdown();
  exit(0);
}
