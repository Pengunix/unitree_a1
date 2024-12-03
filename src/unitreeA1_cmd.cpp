#include "motor_msg.h"
#include <string.h>
#include <stdio.h>
#include "unitreeA1_cmd.h"
#include <iostream>

// uint8_t A1cmd[34];
// uint8_t Date[78];

// CRC校验位的代码
uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t bits;
    uint32_t i;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}


void send_cmd(motor_send_t& cmd)
{
    cmd.motor_send_data.head.start[0] = 0xFE;
    cmd.motor_send_data.head.start[1] = 0xEE;
    cmd.motor_send_data.head.motorID = cmd.id;
    cmd.motor_send_data.head.reserved = 0x00;

    cmd.motor_send_data.Mdata.mode = cmd.mode;
    cmd.motor_send_data.Mdata.ModifyBit = 0x00;
    cmd.motor_send_data.Mdata.ReadBit = 0x00;
    cmd.motor_send_data.Mdata.reserved = 0x00;
    cmd.motor_send_data.Mdata.Modify.F = 0;
    cmd.motor_send_data.Mdata.T = cmd.T * 256;
    cmd.motor_send_data.Mdata.W = cmd.W * 128;
    cmd.motor_send_data.Mdata.Pos = (int)((cmd.Pos / 6.2832f) * 16384.0f);
    cmd.motor_send_data.Mdata.K_P = cmd.K_P * 2048;
    cmd.motor_send_data.Mdata.K_W = cmd.K_W * 1024;
    cmd.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    cmd.motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    cmd.motor_send_data.Mdata.Res[0] = cmd.Res;

    cmd.motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)(&cmd.motor_send_data), 7);

    // //将结构体中的数据转移到字节数组A1cmd里，便于串口发送
    // memcpy(A1cmd, &cmd.motor_send_data, 34);

}


 void receive(motor_recv_t& data)
 {
    if(data.motor_recv_data.CRCdata.u32 == crc32_core((uint32_t *)(&data.motor_recv_data), 18))
    {
        //printf("recv_crc: %d, crc: %d", data.motor_recv_data.CRCdata.u32, crc32_core((uint32_t *)(&data.motor_recv_data), 18));
        //读取数据（物理数据）
        data.Temp =     data.motor_recv_data.Mdata.Temp;
        data.MError =   data.motor_recv_data.Mdata.MError;
        data.T =        data.motor_recv_data.Mdata.T/256.0f;
        data.W =        data.motor_recv_data.Mdata.W/128.0f;
        data.Pos =      data.motor_recv_data.Mdata.Pos*6.2832/16384;

        // //读取数据（原始数据, 还未除之前乘的值）
        // data.motor_id = data.motor_recv_data.head.motorID;
        // data.mode =     data.motor_recv_data.Mdata.mode;
        // data.Temp =     data.motor_recv_data.Mdata.Temp;
        // data.MError =   data.motor_recv_data.Mdata.MError;
        // data.T =        data.motor_recv_data.Mdata.T;
        // data.W =        data.motor_recv_data.Mdata.W;
        // data.Pos =      data.motor_recv_data.Mdata.Pos2;

    }
    else
    {
        std::cout<<"recevice data incorrectly!"<<std::endl;
    }

 }
