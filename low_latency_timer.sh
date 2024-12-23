sudo bash -c "echo 1 >/sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# 使用udev
# ACTION=="add",SUBSYSTEM=="usb-serial",DRIVER=="ftdi_sio",ATTR{latency_timer}="1"