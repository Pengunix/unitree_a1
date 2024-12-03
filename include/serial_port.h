#ifndef __SERIAL_PORT__
#define __SERIAL_PORT__

#include "motor_msg.h"

int set_serial_port(int fd, int baud_rate);
int serial_init(const char* portname);
int serial_SendRecv(int fd, motor_send_t& cmd, motor_recv_t& data);

#endif