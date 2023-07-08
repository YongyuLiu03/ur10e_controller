#!/usr/bin/env python3 
import serial
import time

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
time.sleep(2)

while(1):
    x = input()
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.5)
# arduino.write(bytes(str(3000), 'utf-8')) 
