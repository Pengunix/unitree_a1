## 宇树A1电机控制
使用FT4232H方案，高速USB转四路485芯片。

## 代码使用
**low_latency_timer.sh** 脚本，需要在每次开关机，每次插拔模块都要执行。为了避免麻烦,在ubuntu 20.04下，创建udev规则
```shell
ACTION=="add",SUBSYSTEM=="usb-serial",DRIVER=="ftdi_sio",ATTR{latency_timer}="1"
```

## 项目使用
**main.cpp**仅测试使用，在项目中使用时只需要将**uart.hpp**，**motor_msg.h**头文件加入包含路径即可。

## 测试节点
复制**test_node/motor_control_test**到工作空间一起编译即可.