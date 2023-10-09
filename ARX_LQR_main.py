
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


#State space model

A = np.array([[0.0440 ,1.0000 ,0 ,0 ,0,0 ,0 ,0],[-0.1628,0,1.0000 ,0,0,0 ,0 ,0],[ -0.3629,0,0,1.0000, 0 ,0 ,0,0],[ -0.2224,0,0 ,0,0,0,0,0],[0.0630,1.0000,0 ,0 ,0.8564,1.0000,0,0],[0.0610,0,1.0000,0 ,-0.0057 ,0,1.0000 ,0],[-0.1786,0,0,1.0000,0.0056,0,0,1.0000],[ 0.0286,0,0,0,0.0115,0,0,0]])
B = np.array([[0.0011,0,0],[0.0015,0,0],[0.0016,0,0],[0.0004,0,0],[-0.0001,0,0.0018],[-1.86201554817768e-05,0.00102480088850876,0],[5.37984130535581e-07,0.0154157522579145,0.0152521045050444],[-2.61937614032718e-05,-0.00218422543057594,0.00558644901604837]])
C = np.array([[1,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0],[0,0,0,0,0,0,0,1]])
L_mat = np.array([[0.0439773436578530,0,0.100000000000000],[-0.162817941079882,0,0.100000000000000],[-0.362908377673675,0,0.100000000000000],[-0.222443653494071,0,0.100000000000000],[0,0.856414148461907,0.100000000000000],[0,-0.00570589563991637,0.100000000000000],[0,0.00563672038836704,0.100000000000000],[0,0.0114827184276413,0.100000000000000]])


#Equilibruim values

m1 = 0.3
m2 = 0
m3 = 0
# Uh = 1700
# Ul = 1199
Uh = 1700
Ul = 1200
um = (Uh+Ul)/2
Inn_filter = 0.8
um1 = 1450      #1580,30 Um1,mass
um2 = 1420
um3 = 1360

#Sate input and output...

stat_n = 8
outpt_r = 3
inpt_m = 3


#mass = 1300


# gyro = [0,0,0]

# ser=serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
# ser.write(b"1 gyrop.p 1 compass.p\r")

# gyro = [0,0,0]
# angle = [0,0,0]


T = 0.1

kTime = np.array([0])

bias = np.array([[-1.0],[-1.13],[0]])

xk = np.zeros((8,1))
Yk = np.array([[0],[0]])    #output
Rk = np.array([[0],[0]])   #setpoint
Uk = np.array([[0],[0],[0]])   #input
Ek = np.array([[0],[0],[0]])   #error
Yks = np.array([[0],[0]])    #simulated output

Xk = np.array([[0],[0],[0],[0],[0],[0],[0],[0]])
Yk = np.zeros((outpt_r,1))
Uk = np.zeros((inpt_m,1))

Ys = np.array([[m1],[m2],[m3]])
#Ys = Ys.transpose()
#Us = np.array([um,um,um])
Us = np.array([[um1],[um],[um]])
#Us = Us.transpose()
Uh_mat = np.array([[Uh],[Uh],[Uh]])
Ul_mat = np.array([[Ul],[Ul],[Ul]])
uh = Uh_mat - Us
ul = Ul_mat - Us

#print("ul,uh",ul,uh)
#print(ul.shape,uh.shape)


L_inf = np.array([[27.2009670233405,782.639105983975,74.4468969019551,69.6438388575778,0.00621968504715075,0.00733723739723062,0.00975450631977159,-0.000702852393968871],[-39.9425588046378,-86.8986385930758,11.3007447903314,128.692219853852,3.06686979439385,5.19318717071491,36.5469671848425,44.8561675150583],[-7.46119535954868,9.93203315896896,14.3555364645276,46.0570670938473,38.9290556779785,43.8416571471618,18.0862841957191,16.0154786026889]])


#Innovation filter

phi_e = np.dot(Inn_filter, np.eye(outpt_r))
Ku = np.dot(C, np.linalg.inv(np.eye(stat_n) - A))
Ku = np.dot(Ku, B)
#Ke = np.dot(C, np.linalg.inv(np.eye(stat_n) - A))
#Ke = np.dot(Ke, L_mat)

eye = np.eye(stat_n) # Create an identity matrix of size stat_n
inv = np.linalg.inv(eye - A) # Invert (eye(stat_n) - A)
Ke =  np.dot(C, np.dot(inv, L_mat))
z_hat = np.zeros((stat_n,1))
z_s = np.zeros((stat_n,1))
err = np.zeros((outpt_r,1))
err_f = np.zeros((outpt_r,1))
u_s = np.zeros((inpt_m,1))
uk = np.zeros((inpt_m,1))
uks = np.zeros((inpt_m,1))

Rk = np.zeros((outpt_r,1))
Yks = np.zeros((outpt_r,1))

k=0
Ku_inv = np.linalg.inv(Ku)
#print("Ku",Ku_inv.shape)

prev_pitch = 0
prev_yaw = 0
prev_goal_y = 0
prev_goal_p = 0
prev_pitch_in = min_value
inc = 0.2
Base_pwm = 1300

def bound(x,lb,ub):
	return np.clip(x,lb,ub)



goal_pitch = 0
goal_yaw = 0
data = ''
mass = 20


try:
	while True:
		#print(f'k={k}')

		try:
			data = sock.recv(1024, socket.MSG_DONTWAIT)
			#print(data)
		except BlockingIOError as e:
			#print("passing")
			pass
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
			#print(tmeas_pitch,tmeas_yaw)
		except:
			esc2.set(min_value)
			esc3.set(min_value)
			print("continuing...!!")
			continue
		
		if k == 0:
			print("wait 3 sec")
			offset = tmeas_pitch
			prev_pitch = np.radians(-offset)
			time.sleep(3)
		g1 = goal_pitch
		goal_pitch -= offset
		
		print(f"Goal_pitch : {goal_pitch} goal_yaw : {goal_yaw}")
		
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
			
			Yk = np.hstack((Yk,np.array([[pitch],[yaw],[0]])))
			print(f" Pitch: {np.degrees(pitch)} yaw : {np.degrees(yaw)}")
			
			goal_pitch = math.radians(goal_pitch)
			goal_yaw = math.radians(goal_yaw)
			if goal_yaw > math.pi:
				goal_yaw -= 2*math.pi
			g1 = math.radians(g1)
			
			Rk = np.append(Rk,np.array([[goal_pitch],[goal_yaw],[0]]),axis=1)
			r_temp = Rk[:,k].reshape(3,1)
			rks = r_temp - Ys
			rks = rks.reshape(3,1)
			
			
			#y_temp = Yk[:,k].reshape(3,1)
			y_temp = Yks[:,k].reshape(3,1)
			yks = y_temp -Ys
			error = goal_yaw - yaw
			error1 = goal_pitch - pitch
			int_err = yks - np.dot(C, z_hat)
			int_err = int_err.reshape(3,1)
			err = np.hstack((err,int_err))
			e_inter = np.dot(phi_e, err_f[:, k-1])+np.dot((np.eye(outpt_r) - phi_e), err[:, k])
			e_inter = e_inter.reshape(3,1)
			
			err_f = np.hstack((err_f,e_inter))
			chee = np.dot(Ke + np.eye(inpt_m), err_f[:,k])
			chee = chee.reshape(3,1)
			chee = rks-chee
			u_int = np.dot(Ku_inv, chee)
			u_int = u_int.reshape(3,1)

			u_s = np.hstack((u_s,u_int))
			z_int = np.linalg.inv(np.eye(stat_n) - A).dot(B.dot(u_int) + np.dot(L_mat, e_inter))
			z_int = z_int.reshape(stat_n,1)
			
			z_s = np.hstack((z_s,z_int))
			uks = chee - np.dot(L_inf, (z_hat - z_int))
				
			uks = bound(uks,ul,uh)
			# for i in range(uks.shape[0]):
				# curr = int(uks[i])
				# curr1 = int(uh[i])
				# curr2 = int(ul[i])
				# if curr >= curr1:
					# uks[i] = uh[i]
				# elif curr <= curr2:
					# uks[i] = ul[i]
			

			
			uk = np.hstack((uk,uks))
			U_apply = Us + uks
			z_hat = np.dot(A,z_hat) + np.dot(B,uks) + np.dot(L_mat,int_err)			
			
			yks = np.dot(C,z_hat)+bias
			Yks = np.hstack((Yks,yks))
			U_apply[0] += mass*math.sin(abs(g1))          #feed forward
			#print(U_apply.shape)
			#print(U_apply)
			ma = U_apply[0]
			nu = U_apply[1]
			ra = U_apply[2]
			#print(error)
			# if error1 > 0.02:
				# ma  += 8
			# elif error1< 0.02:
				# ma -= 8
				
			if ma-prev_pitch_in > inc:
				ma = prev_pitch_in+inc
			elif ma- prev_pitch_in < inc:
				ma = prev_pitch_in - inc
			prev_pitch_in = ma
			
			U_apply[0] = ma
			
			
			
			
			esc1.set(ma)
			if error>0.025:
				esc2.set(Base_pwm +nu/15)
				esc3.set(Base_pwm)
				U_apply[1] = Base_pwm +nu/15
				U_apply[2] = Base_pwm 
			elif error<-0.025:
				esc3.set(Base_pwm+ra/13)
				esc2.set(Base_pwm)
				U_apply[2] = Base_pwm +nu/15
				U_apply[1] = Base_pwm
			else:
				esc2.set(Base_pwm)
				esc3.set(Base_pwm)
				U_apply[1] = Base_pwm 
				U_apply[2] = Base_pwm
				
			Uk = np.hstack((Uk,U_apply))
			
			# plt.figure(5)
			# plt.subplot(211)
			# plt.scatter(kTime[0][k], Rk[0][k], s= 8,c ='b', marker='*')
			# plt.scatter(kTime[0][k],Yk[0][k],s = 10, c='r', marker='o')
			# plt.ylabel('Y_1(k) (rad)')
			# plt.xlabel('Time(second)')
			# plt.subplot(212)
			# plt.scatter(kTime[0][k], Rk[1][k], s= 8,c ='b', marker='*')
			# plt.scatter(kTime[0][k],Yk[1][k],s = 10, c='r', marker='o')
			# plt.ylabel('Y_2(k) (rad)')
			# plt.xlabel('Time(second)')
			# plt.legend(['Setpoint', 'Sim.Output'])
			# plt.pause(0.05)
				
			k += 1
			prev_pitch = pitch
			prev_yaw = yaw
			kTime = np.hstack((kTime,np.array([(k-1)*T])))
except KeyboardInterrupt:
	pass
esc1.kill_esc()
esc2.kill_esc()
esc3.kill_esc()
pi.stop()


# Yk[:,Ns-1] = Yk[:,k]

# Uk[:, Ns-1] = Us + uks
# Rk[:, Ns-1] = rks + Ys
# err[:, Ns-1] = err[:, k]
# z_s[:, Ns-1] = z_s[:, k]
# u_s[:, Ns-1] = u_s[:, k]

#kTime[:,Ns-1] = Ns*T

#kTime = kTime.transpose()

print("RMSE of yaw:", np.degrees(np.sqrt(np.mean((Yk[1, :] - Rk[1, :])**2))))
print("RMSE of pitch:", np.degrees(np.sqrt(np.mean((Yk[0, :] - Rk[0, :])**2))))

plt.figure(1)
plt.subplot(311)
plt.title('Manipulated inputs')
plt.grid()
plt.ylabel('U_1(k)')
plt.plot(kTime, Uk[0, :], 'b-', linewidth=2)
plt.ylim([min_value,max_value])


plt.subplot(312)
plt.grid()
plt.ylabel('U_2(k)')
plt.plot(kTime, Uk[1, :], 'b-', linewidth=2)

plt.subplot(313)
plt.grid()
plt.ylabel('U_2(k)')
plt.plot(kTime, Uk[2, :], 'b-', linewidth=2)
plt.xlabel('Time(second)')

plt.figure(2)
plt.subplot(211)
plt.title('Outputs')
plt.grid()
plt.plot(kTime, np.degrees(Rk[0, :]), 'b--', linewidth=3)
#plt.plot(kTime, np.degrees(Yks[0, :]), 'r-', linewidth=2)
plt.plot(kTime, np.degrees(Yk[0, :]), 'k-', linewidth=2)
plt.ylabel('Pitch (dig)')
plt.xlabel('Time(second)')

plt.subplot(212)
plt.grid()
plt.plot(kTime, np.degrees(Rk[1, :]), 'b--', linewidth=3)
#plt.plot(kTime, np.degrees(Yks[1, :]), 'r-', linewidth=2)
plt.plot(kTime, np.degrees(Yk[1, :]), 'k-', linewidth=2)
plt.ylabel('Yaw(dig)')
plt.xlabel('Time(second)')
plt.legend(['Setpoint','Output'])

# plt.figure()
# plt.subplot(211)
# plt.title('Target State')
# plt.grid()
# plt.plot(kTime, z_s[0, :], 'r-', linewidth=2)
# plt.ylabel('Y_1(k) (rad)')
# plt.xlabel('Time(second)')

# plt.subplot(212)
# plt.grid()
# plt.plot(kTime, z_s[4, :], 'r-', linewidth=2)
# plt.ylabel('Y_2(k)(rad)')
# plt.xlabel('Time(second)')

# plt.figure()
# plt.subplot(311)
# plt.title('Innovation')
# plt.grid()
# plt.ylabel('u_{s1}(k)')
# plt.plot(kTime, err[0, :], 'b-', linewidth=2)
# plt.ylabel('Y_2(k)(rad)')
# plt.xlabel('Time(second)')
# plt.subplot(312)
# plt.title('Target inputs')
# plt.grid()
# plt.ylabel('u_{s1}(k)')
# plt.plot(kTime, err[1, :], 'b-', linewidth=2)
# plt.ylabel('Y_2(k)(rad)')
# plt.xlabel('Time(second)')


# plt.figure()
# plt.subplot(311)
# plt.title('Fil.Innovation')
# plt.grid()
# plt.ylabel('u_{s1}(k)')
# plt.plot(kTime, err_f[0, :], 'b-', linewidth=2)
# plt.ylabel('Y_2(k)(rad)')
# plt.xlabel('Time(second)')
# plt.subplot(312)
# plt.title('Target inputs')
# plt.grid()
# plt.ylabel('u_{s1}(k)')
# plt.plot(kTime, err_f[1, :], 'b-', linewidth=2)
# plt.ylabel('Y_2(k)(rad)')
# plt.xlabel('Time(second)')



# plt.figure()
# plt.subplot(311)
# plt.title('Manipulated inputs')
# plt.grid()
# plt.ylabel('U_1(k)')
# plt.plot(kTime, uk[0, :], 'b-', linewidth=2)

# plt.subplot(312)
# plt.grid()
# plt.ylabel('U_2(k)')
# plt.plot(kTime, uk[1, :], 'b-', linewidth=2)

# plt.subplot(313)
# plt.grid()
# plt.ylabel('U_2(k)')
# plt.plot(kTime, uk[2, :], 'b-', linewidth=2)
# plt.xlabel('Time(second)')



plt.show()

	



