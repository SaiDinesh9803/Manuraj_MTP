#!/usr/bin/python3


import os
os.system("sudo pigpiod")
import serial
import socket
import time
import pigpio
from esc import *
import matplotlib.pyplot as plt
import numpy as np


ESC1 = 17
ESC2 = 22
ESC3 = 27

print("Wait 3 sec")
time.sleep(2)
esc1 = ESC(ESC1)
esc2 = ESC(ESC2)
esc3 = ESC(ESC3)
time.sleep(4)

print("ESC calibrated !")

gyro = [0,0,0]

ser=serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
ser.write(b"1 gyrop.p\r")

gyro=[0,0,0]

T = 0.01                                	#sensor time differene







try:
	while True:
		a = ser.readline().decode().strip().split(",")
		
		if a[0].split(":")[0] == "GP":
			gyro=[a[4],a[5],a[6]]
			print(gyro)
			cur_pitch_rate = float(gyro[1])
			cur_yaw_rate = float(gyro[2])
			
			
				
		#______________________________thresholding__________________________
		
		
		cur_pitch_rate = 0 if abs(cur_pitch_rate) < 9e-2 else cur_pitch_rate
		cur_yaw_rate = 0 if abs(cur_yaw_rate) < 9e-2 else cur_yaw_rate
		
		



