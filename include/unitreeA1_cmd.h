
#ifndef __UNITREEA1_CMD__
#define __UNITREEA1_CMD__

#include "motor_msg.h"


extern motor_send_t cmd; 

extern motor_recv_t data; 

extern uint8_t A1cmd[34];
extern uint8_t Date[78];

extern void send_cmd(motor_send_t& cmd);
extern int receive(motor_recv_t& data);
uint32_t crc32_core(uint32_t *ptr, uint32_t len);

#endif
