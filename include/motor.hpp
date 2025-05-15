#include "motor_msg.h"
#include "uart.hpp"
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>
constexpr double PI = 3.141592653589793;
constexpr double hip_offset = 0;
constexpr double thigh_offset = (-PI / 2 + 0.3628) * 9.1;
constexpr double calf_offset = (PI - 0.3628) * 9.1;

template <typename T> class Queue {
public:
  Queue(int maxSize) : m_iMaxSize(maxSize), m_iCnt(0) {}

  ~Queue() = default;

  void Put(const T &x) {
    std::unique_lock<std::mutex> lock(m_mutex);
    //有一个判断谓词,等价于while(!Pred){m_notFull.wait()}
    m_notFull.wait(lock, std::bind(&Queue::CanPut, this));
    m_deqDatas.push_back(x);
    ++m_iCnt;
    if (m_iCnt > m_iMaxSize) {
      m_iCnt = m_iMaxSize;
    }

    //手动释放锁，减小锁的范围
    lock.unlock();

    // m_notEmpty.notify_one();
    m_notEmpty.notify_all();
  }

  void Clear() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_deqDatas.clear();
    lock.unlock();
  }

  T Get() {
    std::unique_lock<std::mutex> lock(m_mutex);
    //有一个判断谓词,等价于while(!Pred){m_notEmpty.wait()}
    m_notEmpty.wait(lock, std::bind(&Queue::CanGet, this));
    T front(m_deqDatas.front());
    m_deqDatas.pop_front();
    --m_iCnt;
    //手动释放锁，减小锁的范围
    lock.unlock();

    // m_notFull.notify_one();
    m_notFull.notify_all();
    return front;
  }

  bool Empty() {
    std::lock_guard<std::mutex> lock(this->m_mutex);
    return m_deqDatas.empty();
  }

  bool Full() {
    std::lock_guard<std::mutex> lock(this->m_mutex);
    return m_iCnt == m_iMaxSize;
  }

private:
  //判断谓词
  bool CanPut() { return m_iCnt < m_iMaxSize; }
  bool CanGet() { return m_iCnt > 0; }

private:
  //计数
  int m_iCnt;
  int m_iMaxSize;
  //容器类型为deque
  std::deque<T> m_deqDatas;
  //互斥量，对队列进行同步保护
  std::mutex m_mutex;
  //用于限制生产者线程
  std::condition_variable m_notFull;
  //用于限制消费者线程
  std::condition_variable m_notEmpty;
};

class Motor {
public:
  Motor(std::shared_ptr<Uart> MotorUart, int MotorID, int8_t dir,
        double offset) {
    uart = std::move(MotorUart);
    _cmd.id = MotorID;
    this->id = MotorID;
    // 约束dir变量，有时间改为枚举
    if (dir == -1 || dir == 1) {
      this->dir = dir;
    } else {
      dir = 0;
      std::cout << "Motor dir Error" << std::endl;
    }
    this->offset = offset;
    using namespace std::chrono_literals;
    for (int i = 0; i < 50; i++) {
      _motorStatus = setMotorProp(0, 0, 0, 0, 0, 0);
      start_pose = _motorStatus.Pos;
      std::this_thread::sleep_for(1ms);
    }
    cal_start_pose = calGetPose(start_pose);

    if (dir == 1) {
      zero_pose = start_pose + offset;
      if (this->id == 0) {
        if (this->id == 0) {
          max_pose = std::max(calGetPose(zero_pose - 1 * 9.1),
                              calGetPose(start_pose + 1 * 9.1));
          min_pose = std::min(calGetPose(zero_pose - 1 * 9.1),
                              calGetPose(start_pose + 1 * 9.1));
        }
      } else if (this->id == 1) {
        max_pose = std::max(calGetPose(zero_pose - 9.1 * PI / 5),
                            calGetPose(start_pose + PI / 2 * 9.1));
        min_pose = std::min(calGetPose(zero_pose - 9.1 * PI / 5),
                            calGetPose(start_pose + PI / 2 * 9.1));
      } else if (this->id == 2) {
        max_pose =
            std::max(calGetPose(zero_pose - 3), calGetPose(start_pose - 5));
        min_pose =
            std::min(calGetPose(zero_pose - 3), calGetPose(start_pose - 5));
      }
    } else if (dir == -1) {
      zero_pose = start_pose - offset;
      if (this->id == 0) {
        max_pose = std::max(calGetPose(zero_pose + 1 * 9.1),
                            calGetPose(start_pose - 1 * 9.1));
        min_pose = std::min(calGetPose(zero_pose + 1 * 9.1),
                            calGetPose(start_pose - 1 * 9.1));
      }
      if (this->id == 1) {
        max_pose = std::max(calGetPose(zero_pose + 9.1 * PI / 5),
                            calGetPose(start_pose - PI / 2 * 9.1));
        min_pose = std::min(calGetPose(zero_pose + 9.1 * PI / 5),
                            calGetPose(start_pose - PI / 2 * 9.1));
      }
      if (this->id == 2) {
        max_pose =
            std::max(calGetPose(zero_pose - 3), calGetPose(start_pose + 5));
        min_pose =
            std::min(calGetPose(zero_pose - 3), calGetPose(start_pose + 5));
      }
    }
    std::cout << "电机ID：" << (int)this->id << "\t初始位置："
              << this->cal_start_pose << "\t零位置：" << zero_pose << "\t下限位："
              << min_pose << "\t上限位：" << max_pose << "\n";
  }
  double getCalStartPose() const { return cal_start_pose; }

  inline double calGetPose(double p) const {
    if (dir == 1) {
      return (-zero_pose + p) / 9.1;
    } else if (dir == -1) {
      return (zero_pose - p) / 9.1;
    }
    // 此处仅为避免编译器警告，正常情况下dir只有以上两种情况
    return 0;
  }
  inline double calSetpose(double pose) {
    if (dir == 1) {
      return pose * 9.1 + zero_pose;
    } else if (dir == -1) {
      return -pose * 9.1 + zero_pose;
    }
    // 此处仅为避免编译器警告，正常情况下dir只有以上两种情况
    return 0;
  }
  MotorData setMotorProp(int mode, float kp, float kd, float pos, float t,
                         float w) {
    _cmd.mode = mode;
    _cmd.K_P = kp;
    _cmd.K_W = kd;
    _cmd.Pos = pos;
    _cmd.T = t;
    _cmd.W = w;
    uart->SendRecv(_cmd);
    {
      std::lock_guard<std::mutex> lck(this->mReadLock);
      MotorData data = uart->GetMotorData();
      if (data.motor_id == _cmd.id) {
        // 更改速度方向
        data.W *= dir;
        _motorStatus = data;
      }
    }
    return _motorStatus;
  }
  MotorData setMotorProp(const MotorCmd &cmd) {
    uart->SendRecv(cmd);
    {
      std::lock_guard<std::mutex> lck(this->mReadLock);
      MotorData data = uart->GetMotorData();
      if (data.motor_id == _cmd.id) {
        data.W *= dir;
        _motorStatus = data;
      }
    }
    return _motorStatus;
  }
  // 力位混合控制
  void setMotor(float kp, float kd, float q, float dq, float tau) {
    _cmd.mode = 10;
    _cmd.T = std::max(-0.5f, std::min(tau, 0.5f));
    _cmd.W = dq;
    _cmd.T = _cmd.T;
    _cmd.K_P = std::max(-0.03f, std::min(kp, 0.03f));
    _cmd.K_W = std::max(-8.5f, std::min(kd, 8.5f));
    _cmd.Pos = calSetpose(q);
    uart->SendRecv(_cmd);
  }
  void setMotorPose(float pose) // 位置模式
  {
    _cmd.mode = 10;
    _cmd.K_P = rotor_kp;
    _cmd.K_W = rotor_kd;
    _cmd.Pos = pose;
    _cmd.W = 0.0;
    _cmd.T = 0;
    uart->SendRecv(_cmd);
  }
  void setMotorDamp(float kd) // 阻尼模式
  {
    _cmd.K_P = 0.;
    _cmd.K_W = kd;
    _cmd.Pos = 0;
    _cmd.W = 0.0;
    _cmd.T = 0.0;
    uart->SendRecv(_cmd);
  }
  void setMotorTorque(float t) // 力矩模式
  {
    _cmd.K_P = 0.;
    _cmd.K_W = 0;
    _cmd.Pos = 0;
    _cmd.W = 0.0;
    _cmd.T = t;
    _cmd.T = std::max(-0.5f, std::min(_cmd.T, 0.5f));
    uart->SendRecv(_cmd);
    
  }
  MotorData getMotorData() {
    mReadLock.lock();
    MotorData snap = _motorStatus;
    mReadLock.unlock();
    return snap;
  }

  double getMotorPos() const { return calGetPose(_motorStatus.Pos); }

private:
  MotorCmd _cmd;
  MotorData _motorStatus;
  uint8_t id;
  // 电机上电起始位置
  double start_pose;
  double cal_start_pose;
  // 计算得到电机机械限位
  double min_pose;
  double max_pose;
  // 计算得到的零点位置
  double zero_pose;
  // 零点位置相铰于上电位置的偏移
  double offset;
  // 电机正方向
  int8_t dir;
  float output_kp = 25;
  float output_kd = 0.6;
  float gear_ratio = 9.1;
  float rotor_kp = (output_kp / (gear_ratio * gear_ratio)) / 26.07;
  float rotor_kd = (output_kd / (gear_ratio * gear_ratio)) * 100.0;
  std::shared_ptr<Uart> uart;
  std::mutex mReadLock;
};

class Leg {
public:
  Leg(std::string UartPath, std::string LegName, std::array<int, 3> dirs)
      : _legName(LegName) {
    uart = std::make_shared<Uart>(UartPath);
    qMotorCmd = std::make_unique<Queue<MotorCmd>>(100);
    // TODO(me) 动态调整大小，方便使用id找到Motor对象
    _motors[0] = std::make_shared<Motor>(uart, 0, dirs[0], hip_offset);
    _motors[1] = std::make_shared<Motor>(uart, 1, dirs[1], thigh_offset);
    _motors[2] = std::make_shared<Motor>(uart, 2, dirs[2], calf_offset);

    startControl = true;
    _thread = std::make_unique<std::thread>(&Leg::task, this);
  }
  ~Leg() {
    startControl = false;
    _thread->join();
  }
  // 这里也只解算了Pos，kp和kd可能需要乘除减速比
  void UpdateMotor(int motorID, int mode, float kp, float kd, float pos,
                   float t, float w) {
    MotorCmd cmd;
    cmd.id = motorID;
    cmd.mode = mode;
    cmd.K_P = kp;
    cmd.K_W = kd;
    cmd.Pos =  _motors.at(motorID)->calSetpose(pos);
    cmd.T = t;
    cmd.W = w;
    if (qMotorCmd->Full()) {
      std::cout << "Queue FULL!! Motor May Be Disconnected" << std::endl;
    } else {
      qMotorCmd->Put(cmd);
    }
  }

  // 获取电机原始数据
  MotorData getMotorData(int MotorID) {
    return _motors.at(MotorID)->getMotorData();
  }
  // 获取电机状态，提供解算数据
  MotorData getMotorState(int MotorID) {
    MotorData orin = _motors.at(MotorID)->getMotorData();
    orin.Pos = _motors.at(MotorID)->getMotorPos();
    return orin;
  }

  std::shared_ptr<Motor> operator[](int Motorid) { return _motors[Motorid]; }

private:
  std::string _legName;
  std::array<std::shared_ptr<Motor>, 3> _motors;
  std::unique_ptr<std::thread> _thread;
  std::shared_ptr<Uart> uart;
  std::unique_ptr<Queue<MotorCmd>> qMotorCmd;
  bool startControl = false;

  void task() {
    while (startControl) {
      MotorCmd cmd = qMotorCmd->Get();
      _motors[cmd.id]->setMotorProp(cmd);
    }
  }
};
