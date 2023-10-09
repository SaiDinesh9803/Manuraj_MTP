#!/usr/bin/python3



import os
os.system("sudo pigpiod")

import time
time.sleep(1)
import numpy as np
import matplotlib.pyplot as plt
import math

import serial
import socket 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "0.0.0.0"
UDP_PORT = 5050
sock.setblocking(False)
sock.bind((UDP_IP,UDP_PORT))
## Enter above UDP_PORT in mobile app
time.sleep(1)


#import serial

## Tell here the pins where ESC are connected (These are gpio numbers and not board numbers)
ESC1 = 17
ESC2 = 22
ESC3 = 27

### Initialize the ESC
from esc import *

print("Wait 3 sec")
esc1 = ESC(ESC1)
esc2 = ESC(ESC2)
esc3 = ESC(ESC3)
time.sleep(3)

print("ESC Initialised")

#### IMU STUFF Adafruit BNO055
import board
import adafruit_bno055

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("ESC Calibrated, Now working on IMU, Make some movement for IMU!!!")





for i in range(2000):
    print(sensor.calibration_status)
print("calibration done!!!")




A = np.array([[0.0440 ,1.0000 ,0 ,0 ,0,0 ,0 ,0],[-0.1628,0,1.0000 ,0,0,0 ,0 ,0],[ -0.3629,0,0,1.0000, 0 ,0 ,0,0],[ -0.2224,0,0 ,0,0,0,0,0],[0.0630,1.0000,0 ,0 ,0.8564,1.0000,0,0],[0.0610,0,1.0000,0 ,-0.0057 ,0,1.0000 ,0],[-0.1786,0,0,1.0000,0.0056,0,0,1.0000],[ 0.0286,0,0,0,0.0115,0,0,0]])
B = np.array([[0.0011,0,0],[0.0015,0,0],[0.0016,0,0],[0.0004,0,0],[-0.0001,0,0.0018],[-1.86201554817768e-05,0.00102480088850876,0],[5.37984130535581e-07,0.0154157522579145,0.0152521045050444],[-2.61937614032718e-05,-0.00218422543057594,0.00558644901604837]])
C = np.array([[1,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0]])
L_mat = np.array([[0.0439773436578530,0],[-0.162817941079882,0],[-0.362908377673675,0],[-0.222443653494071,0],[0,0.856414148461907],[0,-0.00570589563991637],[0,0.00563672038836704],[0,0.0114827184276413]])


L_inf = np.array([[27.2009670233405,782.639105983975,74.4468969019551,69.6438388575778,0.00621968504715075,0.00733723739723062,0.00975450631977159,-0.000702852393968871],[-39.9425588046378,-86.8986385930758,11.3007447903314,128.692219853852,3.06686979439385,5.19318717071491,36.5469671848425,44.8561675150583],[-7.46119535954868,9.93203315896896,14.3555364645276,46.0570670938473,38.9290556779785,43.8416571471618,18.0862841957191,16.0154786026889]])

#L_inf = np.array([[27.1874299018737,782.523904795480,74.8435981263452,69.5910393069883,0.107027548408093,0.126339149237488,0.204463631626791,-0.0138521145876795],[-82.8819399161916,-659.574647062590,5.92196564076836,173.592459826108,-441.962842592006,-516.094198097744,46.0418998133117,61.1168215674635],[36.9129555321742,598.926148779509,2.73580393398741,2.54357348976144,488.437017907600,570.330158490365,0.00764625637795720,-0.000483201827264151]])

#Equilibruim values

m1 = 0.3
m2 = 0
Uh = 1550
# Ul = 1199
Uh1 = 1700
Ul = 1270
um = (Uh1+Ul)/2
Inn_filter = 0.9
um1 = 1300
um2 = 1300

prev_pitch = 0
prev_yaw = 0

Ys = np.array([[m1],[m2]])
#Ys = Ys.transpose()
Us = np.array([[um],[um1],[um2]])
#Us = Us.transpose()
Uh_mat = np.array([[Uh],[Uh],[Uh]])
Ul_mat = np.array([[Ul],[Ul],[Ul]])
uh = Uh_mat - Us
ul = Ul_mat - Us



def bound(x,lb,ub):
	return np.clip(x,lb,ub)	
	



goal_pitch = 0
goal_yaw = 0
data = ''
mass = 1100
hover = 0
Base_pwm = 1300

xk = np.zeros((8,1))
Yk = np.array([[0],[0]])    #output
Rk = np.array([[0],[0]])   #setpoint
Uk = np.array([[0],[0],[0]])   #input
Ek = np.array([[0],[0],[0]])   #error
Yks = np.array([[0],[0]])    #simulated output

k = 1
T = 0.01
T1 = 0.01
offset = 49
e_I = np.array([[0],[0]])
K_I = np.array([[0,0],[0.00000,0],[0.0000,0.00]])
kTime = np.array([0])
K1 = 15
K2 = 15
bias = 0
bias1 = 20   #380
bias2 = 50   #550
goal_pitch_pre = 0
goal_yaw_pre = 0
yawe_I = 0

prev_goal_y = 0
prev_goal_p = 0

wind_up = 100

try:

	while True:
		
		
		try:
			data = sock.recv(1024, socket.MSG_DONTWAIT)
			#print(data)
		except BlockingIOError as e:
			#print("passing")
			pass
		if not isinstance(data, str):
			data = data.decode("utf-8")
		if not isinstance(data, str):
			data = data.decode("utf-8")
		if data[:3] == "pit":
			goal_pitch = int(data[-3:])

		elif data[:3] == "yaw":
			goal_yaw = int(data[-3:])
		elif data[:3] == "ywn":
			goal_yaw = int(data[-3:])
			goal_yaw = -goal_yaw
			
		if data == "alloff":
			break
			
		if abs(goal_yaw) <=2 :
			goal_yaw = prev_goal_y
		if goal_pitch <=2:
			goal_pitch = prev_goal_p
		prev_goal_y = goal_yaw
		prev_goal_p = goal_pitch
			
		
		try:
			tmeas_yaw, tmeas_pitch, tmeas_roll = sensor.euler
			meas_yaw= tmeas_yaw
			meas_roll = tmeas_roll
			meas_pitch = -tmeas_pitch
			print(tmeas_pitch,tmeas_yaw)
		except:
			print("continuing...!!")
			esc2.set(min_value)
			esc3.set(min_value)
			continue
		if k == 1:
			print("wait 3 sec")
			offset = tmeas_pitch
			prev_pitch = np.radians(-offset)
			time.sleep(3)
		g1 = goal_pitch
		goal_pitch -= offset
		
	
		
		
		if meas_yaw is not None and meas_roll is not None and meas_pitch is not None:
			## Pitch
			pitch_angle = meas_pitch
			yaw_angle = meas_yaw


			pitch = math.radians(pitch_angle)
			yaw = math.radians(yaw_angle)
			if yaw > math.pi:
				yaw -= 2*math.pi
			if  abs(pitch-prev_pitch)  > np.radians(15) :
				pitch = prev_pitch
			if abs(yaw-prev_yaw) > np.radians(15) :
				yaw = prev_yaw
			
			Yk = np.hstack((Yk,np.array([[pitch],[yaw]])))
			print(f" Pitch: {np.degrees(pitch)} yaw : {np.degrees(yaw)}")
			
			goal_pitch = math.radians(goal_pitch)
			goal_yaw = math.radians(goal_yaw)
			if goal_yaw > math.pi:
				goal_yaw -= 2*math.pi
			g1 = math.radians(g1)
			
			print(f"Goal_pitch : {np.degrees(goal_pitch)} goal_yaw : {np.degrees(goal_yaw)}")
			
			error = goal_yaw - yaw
			error1 = goal_pitch - pitch
			
			Rk = np.append(Rk,np.array([[goal_pitch],[goal_yaw]]),axis=1)
			r_temp = Rk[:,k].reshape(2,1)
			rk = r_temp - Ys
			
			y_temp = Yk[:,k].reshape(2,1)
			yk = y_temp -Ys
			e_o = rk - y_temp
			if k==1:
				xk = np.array([yk[0],[0],[0],[0],yk[1],[0],[0],[0]])
			
			
			ek = yk - np.dot(C,xk)
			e_state = xk - np.array([rk[0],[0],[0],[0],rk[1],[0],[0],[0]])

			
			#feed-forward
			
			FF = np.array([[hover+(mass*math.sin(g1))],[0],[0]])
			
			yaw_e = float(e_o[1])
			
			yawe_I = yawe_I + yaw_e*T1
			
			#Integral
			# e_I = e_I + e_o*T1
			
			# e_I[1] = min(wind_up,e_I[1]) if e_I[1]>=0 else max(-wind_up,e_I[1])
			
			yawe_I = min(wind_up,yawe_I) if yawe_I>=0 else max(-wind_up,yawe_I)
			
			uk = FF- np.dot(L_inf,e_state)+ np.dot(K_I,e_I)
			#uk = FF- np.dot(L_inf,xk)+ np.dot(K_I,e_I)
			# print(f'error :{e_o[1]} , addition :{e_I[1]}')
			# print(f'before: {uk}')
			yaw_in = float(uk[1])
			yaw_in1 = float(uk[2])
			# if yaw_e>0:
				# yaw_in +=  K1*yawe_I + bias*yaw_e
			# elif yaw_e<=0:
				# yaw_in1 += K2*abs(yawe_I) + bias*yaw_e
			#uk[1] += uk[1] + K1*e_I[1]
			if yaw_e > 0.02:
				yaw_in += bias1*yaw_e
			if yaw_e < -0.02:
				yaw_in1 += bias2*abs(yaw_e)
			uk[1] = yaw_in
			uk[2] = yaw_in1
			
			uk = bound(uk,ul,uh)
			#uk = uk -FF
			
			xk = np.dot(A,xk) + np.dot(B,uk) + np.dot(L_mat,ek)
			yks = np.dot(C,xk)+np.array([[-0.7],[-1.0]])
			
			Yks = np.append(Yks,yks,axis=1)
			
			
			U_apply = uk+Us
			
			print("pitch_in::",U_apply[0])
			
			#print(f"Input {U_apply}")
			ma = U_apply[0]
			nu = U_apply[1]
			ra = U_apply[2]
			
			if yaw_e > 0.02:
				nu = Base_pwm + nu/12
				ra = Base_pwm
			elif yaw_e < -0.02:
				ra = Base_pwm + ra/12
				nu = Base_pwm
			else:
				ra = Base_pwm
				nu = Base_pwm
			U_apply[0] = ma 
			U_apply[1] = nu
			U_apply[2] = ra
			
			Uk = np.append(Uk,U_apply,axis = 1)
			
			esc1.set(ma)
			esc2.set(nu)
			esc3.set(ra)
			
			
			
			prev_pitch = pitch
			prev_yaw = yaw
			
			kTime = np.hstack((kTime,np.array([(k-1)*T])))
			
			# plt.figure(1)
			# plt.subplot(211)
			# plt.scatter(kTime[k], Rk[0][k], s= 8,c ='b', marker='*')
			# plt.scatter(kTime[k],Yk[0][k],s = 10, c='r', marker='o')
			# plt.ylabel('Y_1(k) (rad)')
			# plt.xlabel('Time(second)')
			# plt.subplot(212)
			# plt.scatter(kTime[k], Rk[1][k], s= 8,c ='b', marker='*')
			# plt.scatter(kTime[k],Yk[1][k],s = 10, c='r', marker='o')
			# plt.ylabel('Y_2(k) (rad)')
			# plt.xlabel('Time(second)')
			# plt.legend(['Setpoint', 'Output'])
			# plt.pause(0.0001)
			
			
			# plt.figure(2)
			# plt.subplot(211)
			# plt.scatter(kTime[k], Yks[0][k], s= 8,c ='b', marker='*')
			# plt.scatter(kTime[k],Yk[0][k],s = 10, c='r', marker='o')
			# plt.ylabel('Y_1(k) (rad)')
			# plt.xlabel('Time(second)')
			# plt.subplot(212)
			# plt.scatter(kTime[k], Yks[1][k], s= 8,c ='b', marker='*')
			# plt.scatter(kTime[k],Yk[1][k],s = 10, c='r', marker='o')
			# plt.ylabel('Y_2(k) (rad)')
			# plt.xlabel('Time(second)')
			# plt.legend(['Simulated', 'Actual'])
			# plt.pause(0.0001)
			
			k+=1
			#time.sleep(1)
except KeyboardInterrupt:
	pass
		

esc1.kill_esc()
esc2.kill_esc()
esc3.kill_esc()
pi.stop()


#kTime = kTime.transpose()

print("RMSE of yaw:", np.degrees(np.sqrt(np.mean((Yk[1, :] - Rk[1, :])**2))))
print("RMSE of pitch:",np.degrees(np.sqrt(np.mean((Yk[0, :] - Rk[0, :])**2))))

print("RMSE of yaw sim:",np.degrees(np.sqrt(np.mean((Yk[1, :] - Yks[1, :])**2))))
print("RMSE of pitch sim:",np.degrees(np.sqrt(np.mean((Yk[0, :] - Yks[0, :])**2))))


plt.figure(1)
plt.subplot(311)
plt.title('Manipulated inputs')
plt.grid()
plt.ylabel('U_1(k)')
plt.plot(kTime, Uk[0, :-1], 'b-', linewidth=2)

plt.subplot(312)
plt.grid()
plt.ylabel('U_2(k)')
plt.plot(kTime, Uk[1, :-1], 'b-', linewidth=2)

plt.subplot(313)
plt.grid()
plt.ylabel('U_2(k)')
plt.plot(kTime, Uk[2, :-1], 'b-', linewidth=2)
plt.ylim([min_value,max_value])
plt.xlabel('Time(second)')

plt.figure(2)
plt.subplot(211)
plt.title('Output')
plt.grid()
plt.plot(kTime, np.degrees(Rk[0, :-1]), 'b--', linewidth=3)
plt.plot(kTime, np.degrees(Yk[0, :-1]), 'r-', linewidth=2)
plt.plot(kTime, np.degrees(Yks[0, :-1]), 'k-', linewidth=2)
plt.ylabel('Pitch (deg)')
plt.xlabel('Time(second)')

plt.subplot(212)
plt.grid()
plt.plot(kTime, np.degrees(Rk[1, :-1]), 'b--', linewidth=3)
plt.plot(kTime, np.degrees(Yk[1, :-1]), 'r-', linewidth=2)
plt.plot(kTime, np.degrees(Yks[1,:-1]), 'k-', linewidth=2)
plt.ylabel('Yaw(deg)')
plt.xlabel('Time(second)')
plt.legend(['Setpoint','Actual Output', 'Sim.Output'])

plt.show()
