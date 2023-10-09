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
import adafruit_bno055

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
n_ip = 3  # input
n_op = 2  # output
Ns = 10000 # No.of samples
T = 0.1  # sampling time
tme = np.linspace(0, Ns * T, Ns)
fre: float = 0.01
poles = (-5, -6, -3, -4)   # [-5, -6, -3, -4]
chara = np.poly(poles)
k = 0
eta = 15
Q = eta * np.identity(n_st)

K_cap = np.zeros((n_ip, n_st))
L_cap = np.identity(n_ip)

x_k = np.zeros((n_st, 1))
xm_k = np.array([[0.2], [-0.1], [0.1], [0.1]])
# xm_k = np.zeros((n_st, 1))
r_k = np.zeros((n_ip, Ns))
u_k = np.zeros((n_ip, 1))
e_k = np.zeros((n_st, 1))
PWM = np.zeros((n_ip, 1))
const = 0.1                         # magnification in B_m
# generate reference signal

Y_max1 = 5
Y_min1 = -5
Value1 = 5

pd = np.radians(120)

diff1 = Y_max1 - Y_min1

Y_max2 = 8
Y_min2 = -8
Value2 = 8.0
diff2 = Y_max2 - Y_min2

Y_max3 = 8
Y_min3 = -8
Value3 = 8.0
diff3 = Y_max3 - Y_min3

r_k[0, :] = diff1 / 2 * (np.sin(2 * np.pi * fre * tme) + np.cos(2 * np.pi * 0.5 * fre * tme) + np.cos(2 * np.pi * 2 * fre * tme)) + Value1
r_k[1, :] = diff2 / 2 * (np.sin(2 * np.pi * fre * tme+ pd) + np.cos(2 * np.pi * 0.08 * fre * tme + pd)+ np.sin(2 * np.pi * 0.5 * fre * tme + pd)) + Value2
r_k[2, :] = diff3 / 2 * (np.sin(2 * np.pi * fre * tme -pd) + np.sin(2 * np.pi * 0.7 * fre * tme - pd) + np.cos(2 * np.pi * 0.4 * fre * tme - pd)) + Value3


# Reference model linear SS With known parameters ...

# A_m = np.diag(poles)
A_m = np.array([[0,1,0,0],[0,0,1,0],[0,0,0,1],[-chara[-1],-chara[-2],-chara[-3],-chara[-4]]])
B_m = np.array([[0, 0, 0], [0, 0, 0], [4.57, 1.5, 1.57], [1.67, 4.78, 4.78]])
B_m = const*B_m
C_m = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
D_m = np.zeros((n_op, n_ip))

sys_m = signal.StateSpace(A_m, B_m, C_m, D_m)
sys_md = sys_m.to_discrete(T)
P = linalg.solve_discrete_lyapunov(sys_md.A, Q)  # solve discrete lyapunov

u_ph = np.array([[1550],[1500],[1500]])
u_pl = np.array([[1200],[1199],[1199]])

mass = 30

xm_temp = xm_k
offset = 0
ylim = 30
uliml = 1199
ulimh = 1580

gamma_k = 800                                # adaptation rate in K_cap
gamma_l = 100                                 #adaptaion rate in L_cap
flag = 0

# print(P)
def bound(x, lb, ub):
    return np.clip(x, lb, ub)

try:

	while k<Ns:
		print(k)
		
		try:
			tmeas_yaw, tmeas_pitch, tmeas_roll = sensor.euler
			yaw_r,pitch_r,_ = sensor.gyro
			meas_yaw= tmeas_yaw
			meas_roll = tmeas_roll
			meas_pitch = -tmeas_pitch
			print(tmeas_pitch,tmeas_yaw)
		except:
			print("please check sensor connection !!..")
			esc1.set(min_value)
			esc2.set(min_value)
			esc3.set(min_value)
			continue
		if meas_yaw is not None and meas_roll is not None and meas_pitch is not None and yaw_r is not None and pitch_r is not None:
			pitch = math.radians(meas_pitch)
			yaw = math.radians(tmeas_yaw)
			if yaw > math.pi:
				yaw -= 2*math.pi
				
			xk_temp = np.array([[pitch],[yaw],[pitch_r],[yaw_r]])
			uk_temp = np.dot(-K_cap, x_k[:, k].reshape(n_st, 1)) + np.dot(L_cap, r_k[:, k].reshape(n_ip, 1))
			uk_temp = bound(uk_temp.reshape(n_ip,1),u_pl,u_ph)
			# print(uk_temp)
			#uk_temp = bound(uk_temp.reshape(2, 1), Ul, Uh)
			# print(uk_temp)
			xm_temp = np.dot(sys_md.A, xm_temp.reshape(n_st, 1)) + np.dot(sys_md.B, r_k[:, k].reshape(n_ip, 1))
			# xk_temp = np.dot(A, x_k[:, k].reshape(4, 1)) + np.dot(sys_md.B, uk_temp.reshape(2, 1))
			print("xk :", xk_temp)
			print("x_m:", xm_temp)
			print("rk:", r_k[:,k])
			ek_temp = xk_temp.reshape(4, 1) - xm_temp.reshape(4, 1)
			#print(ek_temp)
			K_cap = gamma_k*np.dot(np.dot((sys_md.B).T, P), np.dot(ek_temp.reshape(n_st, 1), xk_temp.reshape(1, n_st)))
			L_cap = -gamma_l*np.dot(np.dot((sys_md.B).T, P), np.dot(ek_temp.reshape(n_st, 1), r_k[:, k].reshape(1, n_ip))) 
			
			esc1.set(uk_temp[0])
			esc2.set(uk_temp[1])
			esc3.set(uk_temp[2])
			x_k = np.hstack((x_k, xk_temp.reshape(n_st, 1)))
			xm_k = np.hstack((xm_k, xm_temp.reshape(n_st, 1)))
			e_k = np.hstack((e_k, ek_temp.reshape(n_st, 1)))
			u_k = np.hstack((u_k, uk_temp.reshape(n_ip, 1)))
			#PWM = np.hstack((PWM, pwm_temp.reshape(n_ip, 1)))
			k += 1
			#time.sleep(0.1)
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

esc1.kill_esc()
esc2.kill_esc()
esc3.kill_esc()
pi.stop()



plt.figure()
plt.subplot(411)
plt.title('states')
plt.grid()
plt.ylabel('X1(dig)')
plt.plot(np.degrees(x_k[0, :]), 'b-', linewidth=2)
plt.plot(np.degrees(xm_k[0, :]), 'k--', linewidth=2)
plt.ylim([-ylim,ylim])

plt.subplot(412)
plt.grid()
plt.ylabel('X2(dig)')
plt.plot(np.degrees(x_k[1, :]), 'k-', linewidth=2)
plt.plot(np.degrees(xm_k[1, :]), 'r--', linewidth=2)
plt.ylim([-ylim,ylim])

plt.subplot(413)
plt.grid()
plt.ylabel('X3(rad/se)')
plt.plot(x_k[2, :], 'r-', linewidth=2)
plt.plot(xm_k[2, :], 'b--', linewidth=2)
plt.ylim([-ylim,ylim])

plt.subplot(414)
plt.grid()
plt.ylabel('X4 (rad/s)')
plt.plot(x_k[3, :], 'r-', linewidth=2)
plt.plot(xm_k[3, :], 'k--', linewidth=2)
plt.xlabel("Time(s)")
plt.ylim([-ylim,ylim])

plt.figure(2)
plt.subplot(411)
plt.title('errors')
plt.grid()
plt.ylabel('e1')
plt.plot(np.degrees(e_k[0, :]), 'b-', linewidth=2)
plt.ylim([-ylim,ylim])

plt.subplot(412)
plt.grid()
plt.ylabel('e2')
plt.plot(np.degrees(e_k[1, :]), 'k-', linewidth=2)
plt.ylim([-ylim,ylim])

plt.subplot(413)
plt.grid()
plt.ylabel('e3')
plt.plot(e_k[2, :], 'r-', linewidth=2)
plt.ylim([-ylim,ylim])

plt.subplot(414)
plt.grid()
plt.ylabel('e3')
plt.plot(e_k[3, :], 'r-', linewidth=2)
plt.xlabel("Time(s)")
plt.ylim([-ylim,ylim])

plt.figure(3)
plt.subplot(311)
plt.title('reference')
plt.grid()
plt.ylabel('r1')
plt.plot(r_k[0, :], 'b-', linewidth=2)

plt.subplot(312)
plt.grid()
plt.ylabel('r2')
plt.plot(r_k[1, :], 'k-', linewidth=2)
plt.xlabel("Time(s)")

plt.subplot(313)
plt.grid()
plt.ylabel('r2')
plt.plot(r_k[2, :], 'k-', linewidth=2)
plt.xlabel("Time(s)")

plt.figure(4)
plt.subplot(311)
plt.title('manipulated outputs')
plt.grid()
plt.ylabel('U1')
plt.ylim([uliml,ulimh])
plt.plot(u_k[0, :], 'b-', linewidth=2)

plt.subplot(312)
plt.grid()
plt.ylabel('U2')
plt.plot(u_k[1, :], 'k-', linewidth=2)
plt.ylim([uliml,ulimh])
plt.xlabel("Time(s)")

plt.subplot(313)
plt.grid()
plt.ylabel('U2')
plt.plot(u_k[2, :], 'k-', linewidth=2)
plt.ylim([uliml,ulimh])
plt.xlabel("Time(s)")


plt.show()
