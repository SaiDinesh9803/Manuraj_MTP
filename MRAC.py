#!/usr/bin/env python3
import os
os.system("sudo pigpiod")


import time
import pigpio
time.sleep(1)

time.sleep(1)
## Tell here the pins where ESC are connected (These are gpio numbers and not board numbers)
ESC1 = 17
ESC2 = 22
ESC3 = 27
max_value = 1700
min_value = 1199

pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC1, 0)

class ESC:
    def __init__(self, pin):#, location)#, rotation):
        self.gpio_pin = pin
        
        #-------------------------------------------------------------------------------------------
        # Initialize the RPIO DMA PWM for this ESC.
        # In other words, ARM
        #-------------------------------------------------------------------------------------------
        self.pulse_width = 0
        pi.set_servo_pulsewidth(self.gpio_pin, 0)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(self.gpio_pin, max_value)
        time.sleep(0.5)        
        pi.set_servo_pulsewidth(self.gpio_pin, min_value)
        time.sleep(0.5)

    def set(self, pulse_width):
        pulse_width = pulse_width if pulse_width >= min_value else min_value
        pulse_width = pulse_width if pulse_width <= max_value else max_value

        self.pulse_width = pulse_width

        pi.set_servo_pulsewidth(self.gpio_pin, self.pulse_width)
        
        ### This is to debug        
        #print("t {}\tpulse width :{}".format(cnt,self.pulse_width))

    def kill_esc(self):
        pi.set_servo_pulsewidth(self.gpio_pin, 0)


print("Wait 3 sec")
esc1 = ESC(ESC1)
esc2 = ESC(ESC2)
esc3 = ESC(ESC3)
time.sleep(3)
print("ESC Calibrated, Now start commands")

#### IMU STUFF Adafruit BNO055
import board
import busio
import adafruit_bno055

#i2c = busio.I2C(board.SCL, board.SDA)
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("ESC Calibrated, Now working on IMU, Make some movement for IMU!!!")

for i in range(2000):
    print(sensor.calibration_status)
print("calibration done!!!")


import numpy as np
import matplotlib.pyplot as plt
import math
from scipy import signal
from scipy import linalg

n_st = 4  # states
n_ip = 2  # input
n_op = 2  # output
Ns= 10000 # No.of samples
T = 0.01  # sampling time
tme = np.linspace(0, Ns * T, Ns)
fre: float = 0.08                   #0.08

poles = (-0.4, -0.1, -0.3, -0.7)   # [-5, -6, -3, -4]
chara = np.poly(poles)
k = 0
eta = 15
Q = eta * np.identity(n_st)

K_cap = np.zeros((n_ip, n_st))
L_cap = np.identity(n_ip)

x_k = np.zeros((n_st, 1))
#xm_k = np.array([[0.0], [np.radians(-38.0)], [0.0], [0.0]])
xm_k = np.array([[0.0], [0.0], [0.0], [0.0]])

# xm_k = np.zeros((n_st, 1))
r_k = np.zeros((n_ip, Ns))
u_k = np.zeros((n_ip, 1))
e_k = np.zeros((n_st, 1))
PWM = np.zeros((n_ip, 1))
omega = []
const = 1e1                        # magnification in B_m
# generate reference signal
Y_max1 = 2500
Y_min1 = -2500
Value1 = (Y_max1+Y_min1)/2


diff1 = Y_max1 - Y_min1

Y_max2 = 2500
Y_min2 = -2500
Value2 = (Y_max2+Y_min2)/2
diff2 = Y_max2 - Y_min2

r_k[0, :] = diff1 / 6 * (np.sin(2 * np.pi * fre * tme) + np.sin(2 * np.pi * 0.5 * fre * tme) + np.sin(2 * np.pi * 2 * fre * tme)) + Value1
r_k[1, :] = diff2 / 6 * (np.sin(2 * np.pi * fre * tme) + np.sin(2 * np.pi * 0.8 * fre * tme)+ np.sin(2 * np.pi * 5 * fre * tme)) + Value2

#r_k[0, :] = mag*(np.sin(2*np.pi*fre * tme) +  np.cos(2*np.pi*0.5 * fre * tme))
#r_k[1, :] = mag*(np.sin(2*np.pi*fre * tme) +  np.cos(2*np.pi*0.8 * fre * tme))

# Reference model linear SS With known parameters ...

# A_m = np.diag(poles)
A_m = np.array([[0,1,0,0],[0,0,1,0],[0,0,0,1],[-chara[-1],-chara[-2],-chara[-3],-chara[-4]]])
B_m = np.array([[0, 0], [0, 0], [4.5753e-7, -1.527e-8], [-1.6733e-8, 4.7892e-7]])
B_m = const*B_m
C_m = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
D_m = np.zeros((n_op, n_ip))

sys_m = signal.StateSpace(A_m, B_m, C_m, D_m)
sys_md = sys_m.to_discrete(T)
P = linalg.solve_discrete_lyapunov(sys_md.A, Q)  # solve discrete lyapunov
Uh = np.array([[9832100], [9134140]])
Ul = np.array([[1832100], [3167900]])
u_ph = np.array([[1550],[1550]])
u_pl = np.array([[1199],[1300]])
xm_temp = xm_k
offset = 0
ylim = 50

gamma_k = 10/1                              # adaptation rate in K_cap 2010
gamma_l = 1/1                                 #adaptaion rate in L_cap 990
prev_pitch = np.radians(-40)
prev_yaw = 0
prev_pitch_r = 0
prev_yaw_r = 0
sign_change = 0
prev_sign = 1
base_pwm = 1350
gain = np.diag([48**2,5.4**2])             #  np.diag([48**2,12.4**2])  
mass = 400
#gain = np.identity(n_ip)
#print(gain)

#GRAVITY COMPENSTION COMPONENTS....

gug = []
A_mod = 0.07443187
B_mod = 0.08483
Mvg = A_mod-B_mod
Jv = 0.03409233
a1 = B_m[2][0]
#a1 = 1
u_temp_prev = 0

print(P)
def bound(x, lb, ub):
    return np.clip(x, lb, ub)

try:

	while k<Ns:
		# print(k)
		# try:
		#     B = np.dot(sys_md.B, np.linalg.inv(L_cap))
		# except:
		#     B = sys_md.B
		
		
		
		try:
			tmeas_yaw, tmeas_pitch, tmeas_roll = sensor.euler
			yaw_r,pitch_r,v__ = sensor.gyro
			meas_yaw= tmeas_yaw
			meas_roll = tmeas_roll
			meas_pitch = -tmeas_pitch
			
			
			#print(meas_pitch,meas_yaw)
		except:
			print("please check sensor connection !!..")
			#esc1.set(min_value)
			esc2.set(min_value)
			esc3.set(min_value)
			continue
		if meas_yaw is not None and meas_roll is not None and meas_pitch is not None and yaw_r is not None and pitch_r is not None:
			pitch = math.radians(meas_pitch)
			yaw = math.radians(tmeas_yaw)
			if yaw > math.pi:
				yaw -= 2*math.pi
			if  abs(pitch-prev_pitch)  > np.radians(15) :
				pitch = prev_pitch
			if abs(yaw-prev_yaw) > np.radians(45) :
				yaw = prev_yaw
			if  abs(pitch_r-prev_pitch_r)  > 5 :
				pitch_r = prev_pitch_r
			if abs(yaw_r-prev_yaw_r) > 5 :
				yaw_r = prev_yaw_r
				
			#restrict yaw motion
			# if yaw == 0:
				# sign = 1
			# else:
				# sign = yaw/abs(yaw)
			# if sign == -prev_sign:
				# sign_change += 1
				# prev_sign = sign
			# if sign_change == 6:
				# print('yaw is out of bound !!')
				# esc1.set(min_value)
				# esc2.set(min_value)
				# esc3.set(min_value)
				# sign_change = 0
				# print('repair in 5 sec !!!')
				# time.sleep(5)
				
				# continue
			
			xk_temp = np.array([[yaw],[pitch],[pitch_r],[yaw_r]])
			A = A_m + np.dot(sys_md.B, K_cap)
			uk_temp = np.dot(-K_cap, xk_temp.reshape(4, 1)) + np.dot(L_cap, r_k[:, k].reshape(2, 1))
			#print(f'kcapx :{np.dot(-K_cap, xk_temp.reshape(4, 1))}, lcapr{np.dot(L_cap, r_k[:, k].reshape(2, 1))}')
			print(uk_temp[1])
			
			# gravity compensation
			uk_temp[1] = uk_temp[1] - Mvg*math.cos(xm_temp[1])/(Jv*a1)
			
			
			
			xm_temp = np.dot(sys_md.A, xm_temp.reshape(4, 1)) + np.dot(sys_md.B, r_k[:, k].reshape(2, 1))
			ek_temp = xk_temp.reshape(4, 1) - xm_temp.reshape(4, 1)
			K_cap = gamma_k*np.dot(np.dot((sys_md.B).T, P), np.dot(ek_temp, xk_temp.T))
			L_cap = -gamma_l*np.dot(np.dot((sys_md.B).T, P), np.dot(ek_temp, r_k[:, k].reshape(1,2)))
			##print(f"Lcap: {L_cap}, Kcap :{K_cap}")
			uk_temp = np.dot(gain,uk_temp)
			if uk_temp[1] - u_temp_prev > 1000:
				uk_temp[1] = u_temp_prev+1000 
			elif uk_temp[1] - u_temp_prev < 1000:
				uk_temp[1] = u_temp_prev-1000 
			print(uk_temp[1])
			Omega_temp = abs(uk_temp) ** 0.5
			#print(Omega_temp**2, Omega_temp)
			pwm_temp = 1266.408 - 0.0230569 * Omega_temp + 3.301e-5 * abs(uk_temp)
			#pwm_temp = 1400 - 0.0230569 * Omega_temp + 3.301e-5 * abs(uk_temp)
			pwm_temp = bound(pwm_temp.reshape(2,1),u_pl,u_ph)
			
			gra_an = abs(xm_temp[1])+math.radians(38)
			gravity = mass*math.sin(gra_an)
			main_input = gravity+ pwm_temp[1]
			gug.append(gravity)
			if main_input>1650:
				main_input = 1650
			esc1.set(main_input)
			pwh = base_pwm + pwm_temp[0]/10
			#pwl = base_pwm - pwm_temp[0]/10
			if pwh > 1600:
				pwh = 1600
			if ek_temp[0] < -0.03:
				esc2.set(pwh) 
				esc3.set(base_pwm)
				#print(pwh,pwl) 
			elif ek_temp[0]> 0.03: 
				esc2.set(base_pwm)
				esc3.set(pwh)
				#print(pwh,pwl)
			else:
				esc2.set(base_pwm)
				esc3.set(base_pwm)
			
			x_k = np.hstack((x_k, xk_temp.reshape(4, 1)))
			xm_k = np.hstack((xm_k, xm_temp.reshape(4, 1)))
			e_k = np.hstack((e_k, ek_temp.reshape(4, 1)))
			u_k = np.hstack((u_k, uk_temp.reshape(2, 1)))
			PWM = np.hstack((PWM, pwm_temp.reshape(2, 1)))
			k += 1
			
			# print(f"Yaw :{np.degrees(xk_temp[0])},Yaw goal:{np.degrees(xm_temp[0])}")
			# print(f"Pitch :{np.degrees(xk_temp[1])},Pitch goal:{np.degrees(xm_temp[0])}")
			
			omega.append([float(Omega_temp[0]),float(Omega_temp[1])])
			prev_pitch = pitch
			prev_yaw = yaw
			prev_pitch_r = pitch_r
			prev_yaw_r = yaw_r
			u_temp_prev = uk_temp[1]
			#time.sleep(1)
except KeyboardInterrupt:
	esc1.set(min_value)
	esc2.set(min_value)
	esc3.set(min_value)
	
	pass

#print("Eigen values: ", linalg.eig(A_m))
print("RMSE X1 in dig:", np.degrees(np.sqrt(np.mean(e_k[0,:]**2))))
print("RMSE X2 in dig :", np.degrees(np.sqrt(np.mean(e_k[1,:]**2))))
print("RMSE X3 in rad/s :", np.sqrt(np.mean(e_k[2,:]**2)))
print("RMSE X4 in rad/s:", np.sqrt(np.mean(e_k[3,:]**2)))

omega = np.array(omega)
print(omega)
esc1.kill_esc()
esc2.kill_esc()
esc3.kill_esc()
pi.stop()

#print(max(u_k[0,:]),min(u_k[0,:]))
#print(max(u_k[1,:]),min(u_k[1,:]))
#print(np.degrees(max(abs(x_k[1,:]))))

ylim = np.degrees(max(abs(x_k[0,:])))

plt.figure()
plt.subplot(411)
plt.title('states')
plt.grid()
plt.ylabel('Yaw(deg)')
plt.plot(np.degrees(x_k[0, 1:]), 'b-', linewidth=2)
plt.plot(np.degrees(xm_k[0, 1:]), 'k--', linewidth=2)
#plt.ylim([-ylim,ylim])

plt.subplot(412)
plt.grid()
plt.ylabel('Pitch(deg)')
plt.plot(np.degrees(x_k[1,]), 'k-', linewidth=2)
plt.plot(np.degrees(xm_k[1,]), 'r--', linewidth=2)
#plt.ylim([-ylim,ylim])

plt.subplot(413)
plt.grid()
plt.ylabel('Pitch_rate(rad/se)')
plt.plot(x_k[2, :], 'r-', linewidth=2)
plt.plot(xm_k[2, :], 'b--', linewidth=2)
#plt.ylim([-ylim,ylim])

plt.subplot(414)
plt.grid()
plt.ylabel('Yaw_rate(rad/s)')
plt.plot(x_k[3, :], 'r-', linewidth=2)
plt.plot(xm_k[3, :], 'k--', linewidth=2)
plt.xlabel("Time(*0.01s)")
#plt.ylim([-ylim,ylim])

plt.figure(2)
plt.subplot(411)
plt.title('errors')
plt.grid()
plt.ylabel('Yaw_error')
plt.plot(np.degrees(e_k[0, :]), 'b-', linewidth=2)
#plt.ylim([-ylim,ylim])

plt.subplot(412)
plt.grid()
plt.ylabel('Pitch_error')
plt.plot(np.degrees(e_k[1, :]), 'k-', linewidth=2)
#plt.ylim([-ylim,ylim])

plt.subplot(413)
plt.grid()
plt.ylabel('e3')
plt.plot(e_k[2, :], 'r-', linewidth=2)
#plt.ylim([-ylim,ylim])

plt.subplot(414)
plt.grid()
plt.ylabel('e3')
plt.plot(e_k[3, :], 'r-', linewidth=2)
plt.xlabel("Time(*0.01s)")
#plt.ylim([-ylim,ylim])

plt.figure(3)
plt.subplot(211)
plt.title('reference')
plt.grid()
plt.ylabel('r1')
plt.plot(r_k[0, :], 'b-', linewidth=2)

plt.subplot(212)
plt.grid()
plt.ylabel('r2')
plt.plot(r_k[1, :], 'k-', linewidth=2)
plt.xlabel("Time(s)")

plt.figure(4)
plt.subplot(211)
plt.title('manipulated outputs')
plt.grid()
plt.ylabel('Tail_input')
plt.plot(u_k[0, :], 'b-', linewidth=2)

plt.subplot(212)
plt.grid()
plt.ylabel('Main_input')
plt.plot(u_k[1, :], 'k-', linewidth=2)
plt.xlabel("Time(*0.01s)")


plt.figure(5)
plt.subplot(211)
plt.title('PWM')
plt.grid()
plt.ylabel('PWM1')
plt.plot(PWM[0, :], 'b-', linewidth=2)
plt.ylim([1200,1800])

plt.subplot(212)
plt.grid()
plt.ylabel('PWM2')
plt.plot(PWM[1, :], 'k-', linewidth=2)
plt.xlabel("Time(s)")
plt.ylim([1200,1800])

plt.figure()
plt.subplot(211)
plt.grid()
plt.ylabel('X1')
plt.scatter(PWM[0, :],np.degrees(x_k[0,:]))
plt.xlabel("PWM1")

plt.subplot(212)
plt.grid()
plt.ylabel('x2')
plt.scatter(PWM[1, :],np.degrees(x_k[1,:]))
plt.xlabel("PWM2")

plt.figure()
plt.subplot(211)
plt.grid()
plt.ylabel('w1')
plt.plot(omega[:,0],'k-',linewidth = 2)
plt.xlabel("Time")

plt.subplot(212)
plt.grid()
plt.ylabel('w2')
plt.plot(omega[:,1],'k-',linewidth = 2)
plt.xlabel("time")

plt.figure()
plt.plot(gug)




plt.show()
