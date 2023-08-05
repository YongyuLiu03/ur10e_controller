#!/usr/bin/env python3 
# manual control printhead motor, lower input -> higher rotate speed
import serial
import time

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
time.sleep(2)

while(1):
    x = input()
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.5)
