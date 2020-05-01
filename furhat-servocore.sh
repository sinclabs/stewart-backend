#!/bin/bash
sudo chmod a+rw /dev/ttyUSB0 &&\
  sudo chmod 777 /sys/bus/usb-serial/devices/ttyUSB0/latency_timer &&\
  sudo echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer


