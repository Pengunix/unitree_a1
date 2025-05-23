#pragma once
#include "asm-generic/ioctls.h"
#include "asm-generic/termbits.h"
#include "motor_msg.h"
#include <cassert>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/serial.h>
#include <linux/tty_flags.h>
#include <memory>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>

class Uart {
public:
  Uart(std::string dev, int baudrate = 4800000) {
    _fd = open(dev.c_str(), O_RDWR);
    usleep(1000);
    // 清空缓冲区
    ioctl(_fd, TCFLSH, 0);
    ioctl(_fd, TCFLSH, 1);
    ioctl(_fd, TCFLSH, 2);

    // 串口基础属性，波特率，校验位，停止位 8N1
    _tio.c_cflag &= ~CBAUD;
    _tio.c_cflag &= ~PARENB;
    _tio.c_cflag &= ~CSTOPB;
    _tio.c_cflag &= ~CRTSCTS;
    _tio.c_cflag &= ~(CSIZE | PARENB);
    _tio.c_cflag |= CS8;
    // 设置非标波特率
    _tio.c_cflag |= BOTHER;
    // 无延迟
    _tio.c_cflag |= O_NDELAY;
    // 关闭流控
    _tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    // 忽略终端特性
    _tio.c_iflag &=
        ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP);
    // 输入波特率
    _tio.c_ispeed = baudrate;
    // 输出波特率
    _tio.c_ospeed = baudrate;
    // 忽略终端特性
    _tio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    // 打开原始模式，每次读出1字节
    _tio.c_cc[VMIN] = 1;
    _tio.c_cc[VTIME] = 0;

    int res = ioctl(_fd, TCSETS2, &_tio);
    if (res != 0) {
      std::cerr << "Set serial ERROR!!!" << std::endl;
    }
  }
  ~Uart() { close(_fd); }
  int SendRecv(const MotorCmd &cmd) {
    tcflush(_fd, TCIOFLUSH);
    _cmd = cmd;
    // 计算发送数据
    _calComData();
    int wsize = write(_fd, &_cmd.motorRawData, 34);
    if (wsize <= 0) {
      std::cerr << "Error in Writing Data" << std::endl;
      return -1;
    }

#if 0
    printf("Transmitted ");
    for (int i = 0; i < wsize; i++) {
      uint8_t *byte = (uint8_t *)&_cmd.motorRawData;
      printf("%02X ", *(byte + i));
    }
    printf("\n");
#endif

    usleep(50);
    int rsize = Read();
    if (rsize <= 0) {
      std::cerr << "Read Error!" << std::endl;
      return -1;
    }

#if 0
    printf("Received ");
    for (int i = 0; i < rsize; i++) {
      uint8_t *byte = (uint8_t *)&_buffer;
      printf("%02X ", *(byte + i));
    }
    printf("\n");
#endif
    // 判断包头及CRC校验 四字节x18 数据长度写死
    if (_buffer[0] == 0xFE && _buffer[1] == 0xEE &&
        crc32_core((uint32_t *)_buffer, 18) ==
            *(uint32_t *)(_buffer + 78 - 4)) {
      memset(&(_rdata.motor_recv_data), 0, sizeof(_rdata.motor_recv_data));
      memcpy(&(_rdata.motor_recv_data), _buffer, 78);
    } else {
      std::cerr << "CRC Error" << std::endl;
      return -1;
    }

    return 0;
  }

  int Read(int tout_us = 900) {
    fd_set inputs;
    struct timeval tout;
    tout.tv_sec = 0;
    tout.tv_usec = tout_us;

    int num = 0, ret = 0;

    FD_ZERO(&inputs);
    FD_SET(_fd, &inputs);

    // 使用select设置读取超时时长
    ret = select(_fd + 1, &inputs, (fd_set *)NULL, (fd_set *)NULL, &tout);
    if (ret < 0) {
      std::cerr << "Select Error" << std::endl;
      return ret;
    }
    if (ret > 0) {
      if (FD_ISSET(_fd, &inputs)) {
        num = read(_fd, _buffer, 78);
      }
    }
    if (num < 78) {
      num += read(_fd, _buffer + num, 78 - num);
    }
    return num;
  }
  // 接收数据转换
  MotorData GetMotorData() {
    _rdata.motor_id = _rdata.motor_recv_data.head.motorID;
    _rdata.mode = _rdata.motor_recv_data.Mdata.mode;
    _rdata.Temp = _rdata.motor_recv_data.Mdata.Temp;
    _rdata.MError = _rdata.motor_recv_data.Mdata.MError;
    _rdata.T = _rdata.motor_recv_data.Mdata.T / 256.0f;
    _rdata.W = _rdata.motor_recv_data.Mdata.W / 128.0f;
    _rdata.Pos = _rdata.motor_recv_data.Mdata.Pos * 6.2832 / 16384;

    return _rdata;
  }

  static inline uint32_t crc32_core(uint32_t *ptr, uint32_t len) {
    uint32_t bits;
    uint32_t i;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (i = 0; i < len; i++) {
      xbit = 1 << 31;
      data = ptr[i];
      for (bits = 0; bits < 32; bits++) {
        if (CRC32 & 0x80000000) {
          CRC32 <<= 1;
          CRC32 ^= dwPolynomial;
        } else
          CRC32 <<= 1;
        if (data & xbit)
          CRC32 ^= dwPolynomial;

        xbit >>= 1;
      }
    }
    return CRC32;
  }

private:
  termios2 _tio;
  int _fd;
  MotorCmd _cmd;
  MotorData _rdata;
  uint8_t _buffer[256];

  // 发送数据转换
  void _calComData() {
    _cmd.motorRawData.head.start[0] = 0xFE;
    _cmd.motorRawData.head.start[1] = 0xEE;
    _cmd.motorRawData.head.motorID = _cmd.id;
    _cmd.motorRawData.head.reserved = 0x00;

    _cmd.motorRawData.Mdata.mode = _cmd.mode;
    _cmd.motorRawData.Mdata.ModifyBit = 0xFF;
    _cmd.motorRawData.Mdata.ReadBit = 0x00;
    _cmd.motorRawData.Mdata.reserved = 0x00;

    _cmd.motorRawData.Mdata.Modify.F = 0;

    _cmd.motorRawData.Mdata.T = _cmd.T * 256;
    _cmd.motorRawData.Mdata.W = _cmd.W * 128;
    _cmd.motorRawData.Mdata.Pos = _cmd.Pos / 6.2832f * 16384.0f;
    _cmd.motorRawData.Mdata.K_P = _cmd.K_P * 2048;
    _cmd.motorRawData.Mdata.K_W = _cmd.K_W * 1024;
    _cmd.motorRawData.Mdata.LowHzMotorCmdIndex = 0;
    _cmd.motorRawData.Mdata.LowHzMotorCmdByte = 0;

    _cmd.motorRawData.Mdata.Res[0] = _cmd.Res;

    _cmd.motorRawData.CRCdata.u32 =
        crc32_core((uint32_t *)(&_cmd.motorRawData), 7);
  }
};