#include "unitree_a1/MotorCmd.h"
#include "unitree_a1/MotorData.h"
#include <csignal>
#include <mutex>
#include <ros/console.h>
#include <ros/ros.h>

void SigintHandler(int sig);
void MotorStateCallback(const unitree_a1::MotorData::ConstPtr &msg);

struct MotorState {
  uint8_t leg_id;
  uint8_t motor_id; // 电机ID
  unsigned char mode;     // 0:空闲, 5:开环转动, 10:闭环FOC控制
  int Temp;               // 温度
  int MError;             // 错误码

  float T;   // 当前实际电机输出力矩
  float W;   // 当前实际电机速度（高速）
  float Pos; // 当前电机位置
  float LW;  // 当前实际电机速度（低速）
  int Acc;   // 电机转子加速度
};

std::array<MotorState, 3> FL_State;
std::array<MotorState, 3> FR_State;
std::array<MotorState, 3> BL_State;
std::array<MotorState, 3> BR_State;

int main(int argc, char **argv) {
  ros::init(argc, argv, "Control_test");
  ros::NodeHandle nodeHandle;
  signal(SIGINT, SigintHandler);

  ros::Subscriber MotorStateSub =
      nodeHandle.subscribe("/motor_state", 100, MotorStateCallback);
  ros::Publisher MotorCmdPub =
      nodeHandle.advertise<unitree_a1::MotorCmd>("/motor_cmd", 100);
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::Rate loop_rate(500);

  unitree_a1::MotorCmd cmd;
  cmd.legid = 0;
  cmd.motorid = 2;
  // 模式设置为空闲，获取初始状态
  cmd.mode = 0;
  cmd.kp = 0;
  cmd.kd = 0;
  cmd.pos = 0;
  cmd.tau = 0;
  cmd.vel = 0;

  while (ros::ok()) {
    // 务必发送空闲状态获取灵位置
    MotorCmdPub.publish(cmd);
    loop_rate.sleep();
  }
}

void SigintHandler(int sig) {
  ROS_INFO("shutting down!");
  ros::shutdown();
  exit(0);
}

void MotorStateCallback(const unitree_a1::MotorData::ConstPtr &msg) {
  // 需要按照返回的leg_id motor_id更新对应数据
  ROS_INFO("Received Pos: %lf", msg->pos);
}