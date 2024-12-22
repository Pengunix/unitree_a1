sudo bash -c "echo 1 >/sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
