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
  Motor(std::shared_ptr<Uart> MotorUart, int MotorID) {
    _cmd.id = MotorID;
    uart = std::move(MotorUart);

    using namespace std::chrono_literals;
    for (int i = 0; i < 50; i++) {
      _motorStatus = setMotorProp(0, 0, 0, 0, 0, 0);
      start_pose = _motorStatus.Pos;
      std::this_thread::sleep_for(1ms);
    }
  }
  MotorData setMotorProp(int mode, float kp, float kw, float pos, float t,
                         float w) {
    _cmd.mode = mode;
    _cmd.K_P = kp;
    _cmd.K_W = kw;
    _cmd.Pos = pos;
    _cmd.T = t;
    _cmd.W = w;
    uart->SendRecv(_cmd);
    {
      std::lock_guard<std::mutex> lck(this->mReadLock);
      MotorData data = uart->GetMotorData();
      if (data.motor_id == _cmd.id) {
        _motorStatus = data;
      }
    }
    return _motorStatus;
  }
  MotorData setMotorProp(const MotorCmd &cmd) {
    _cmd = cmd;
    uart->SendRecv(_cmd);
    {
      std::lock_guard<std::mutex> lck(this->mReadLock);
      MotorData data = uart->GetMotorData();
      if (data.motor_id == _cmd.id) {
        _motorStatus = data;
      }
    }
    return _motorStatus;
  }
  MotorData getMotorData() {
    mReadLock.lock();
    MotorData snap = _motorStatus;
    mReadLock.unlock();
    return snap;
  }

private:
  MotorCmd _cmd;
  MotorData _motorStatus;
  float start_pose;
  float min_pose;
  float max_pose;
  float zero_pose;
  int8_t dir;
  std::shared_ptr<Uart> uart;
  std::mutex mReadLock;
};

class Leg {
public:
  Leg(std::string UartPath, std::string LegName,
      std::initializer_list<int> motors)
      : _legName(LegName) {
    uart = std::make_shared<Uart>(UartPath);
    qMotorCmd = std::make_unique<Queue<MotorCmd>>(100);
    // TODO(me) 动态调整大小，方便使用id找到Motor对象
    _motors.reserve(4);
    for (const int &i : motors) {
      _motors.emplace(_motors.begin() + i, std::make_shared<Motor>(uart, i));
    }
    startControl = true;
    _thread = std::make_unique<std::thread>(&Leg::task, this);
  }
  ~Leg() {
    startControl = false;
    _thread->join();
  }

  void UpdateMotor(int motorID, int mode, float kp, float kw, float pos,
                   float t, float w) {
    MotorCmd cmd;
    cmd.id = motorID;
    cmd.mode = mode;
    cmd.K_P = kp;
    cmd.K_W = kw;
    cmd.Pos = pos;
    cmd.T = t;
    cmd.W = w;
    if (qMotorCmd->Full()) {
      std::cout << "Queue FULL!! Motor May Be Disconnected" << std::endl;
    } else {
      qMotorCmd->Put(cmd);
    }
  }

  MotorData getMotorData(int MotorID) {
    return _motors[MotorID]->getMotorData();
  }

  std::shared_ptr<Motor> operator[](int Motorid) { return _motors[Motorid]; }

private:
  std::string _legName;
  std::vector<std::shared_ptr<Motor>> _motors;
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
