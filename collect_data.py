
import os
os.system("sudo pigpiod")


import time
import pigpio
time.sleep(1)


import socket 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "0.0.0.0"
UDP_PORT = 5050
sock.bind((UDP_IP,UDP_PORT))
## Enter above UDP_PORT in mobile app
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

#___________________________________________________________________________________
#Actual code starts from here..........


import numpy as np
from numpy import asarray
from numpy import savetxt
import math
import random

import matplotlib.pyplot as plt








Ns = 10000
frequency = 100
T = 1/frequency
t = np.linspace(0, Ns*T,Ns)
fre = 6

# Set up the frequency vector
# f = np.linspace(1, 6, Ns)
# f1 = np.linspace(1, 10, Ns)
# f2 = np.linspace(1, 20, Ns)
f = 0.08
Y_max1 = 1560
Y_min1 = 1390
Value1 = (Y_max1+Y_min1)/2
diff1 = Y_max1 - Y_min1

Y_max2 = 1400
Y_min2 = 1200
Value2 = (Y_max2+Y_min2)/2
diff2 = Y_max2 - Y_min2

Y_max3 = 1450
Y_min3 = 1200
Value3 = (Y_max3+Y_min3)/2
diff3 = Y_max3 - Y_min3

phase_1 = np.radians(120)
phase_2 = np.radians(240)

Y = []
U = []

# Generate the signal
signal = diff1/2 * np.sin(2*np.pi*f*t)+Value1
signal1 = diff2/2 * np.sin(2*np.pi*f*t+phase_1)+Value2
signal2 = diff3/2 * np.sin(2*np.pi*f*t+phase_2)+Value3
i = 0
prev_pitch = 0
prev_yaw = 0
ta = time.time()
try:
	while True:
		u1 = signal[i]
		u2 = signal1[i]
		u3 = signal2[i]
		esc1.set(u1)
		esc2.set(u2)
		esc3.set(u3)
		try:
			tmeas_yaw, tmeas_pitch, tmeas_roll = sensor.euler
			meas_yaw= tmeas_yaw
			meas_roll = tmeas_roll
			meas_pitch = -tmeas_pitch
			print(tmeas_pitch,tmeas_yaw)
		except:
			print("continuing...!!")
			continue
		if meas_yaw is not None and meas_roll is not None and meas_pitch is not None:
			## Pitch
			print(i)
			if meas_pitch >=50 or meas_pitch <= -50:
				meas_pitch = prev_pitch
			if meas_yaw > 365 or meas_yaw < -2:
				meas_yaw = prev_yaw
			Y.append([meas_pitch,meas_yaw,u1,u2,u3])
			U.append([u1,u2,u3])
			i+=1
			if i>=Ns:
				break
			#time.sleep(0.3)
except KeyboardInterrupt:
	pass
tb = time.time()
esc1.kill_esc()
esc2.kill_esc()
esc3.kill_esc()
pi.stop()
U = np.array(U)
Y = np.array(Y)
print(Y)
print(U)
print("diff : ", tb-ta)	
np.savetxt("input_"+str(f)+".csv",U, delimiter = ",")
np.savetxt("output"+str(f)+".csv",Y, delimiter = ",")


# Plot the signal
plt.figure(1)
plt.subplot(311)
plt.title('Inputs')
plt.grid()
plt.ylabel('U_1(k)')
plt.scatter(t, signal)
plt.subplot(312)
plt.grid()
plt.plot(t, signal1)
plt.ylabel('U_2(k)')
plt.subplot(313)
plt.grid()
plt.plot(t, signal2)
plt.xlabel('Time (s)')
plt.ylabel('U_3(k)')


plt.figure(2)
plt.subplot(211)
plt.title('Outputs')
plt.grid()
plt.ylabel('Y_1(k)')
plt.plot(t, Y[:,0])

plt.subplot(212)
plt.grid()
plt.ylabel('Y_2(k)')
plt.plot(t, Y[:,1])
plt.xlabel('Time (s)')
plt.show()









#Model evaluation



# from sklearn.preprocessing import normalize
num_input = 3
num_output = 2

norm = 0     # 1 for sklearn else for dynamic range
case = 1      # 1 for max value and 2 for variable range


# Input and output

#Y1 = np.loadtxt("output0.05.csv", delimiter=',')
# Y2 = np.loadtxt("output0.06.csv", delimiter=',')
# Y3 = np.loadtxt("output0.08.csv", delimiter=',')
Y3 = Y
row = Y3.shape[0]
column = Y3.shape[1]
# Y_stack = np.zeros((row*3, column))
# Y_stack = np.block(([[Y1], [Y2], [Y3]]))
Y_stack = Y3
np.random.shuffle(Y_stack)
Y = Y_stack[:, 0:2]
U = Y_stack[:, 2:5]
print(U)
# U = np.loadtxt("ut_new.csv", delimiter=',')

U = U.T
# Y = np.loadtxt("yt_new.csv", delimiter=',')
# Normalizing variable by its dynamic range...
U_max = 1700        # Max PWM
Y_max = 360         # maximum angle
Y_min = 0
U_min = 1199
diff_Y = Y_max - Y_min
diff_U = U_max - U_min
if norm == 1:
	Stamp = True
    # U = normalize(U)
    # Y = normalize(Y)
elif norm == 0:
    if case == 1:
        Y = Y/Y_max
        U = U/U_max
    elif case == 2:
        Y = Y - Y_min
        Y = Y/diff_Y
        U = U - U_min
        U = U / diff_U


Yk1 = Y[:, 0]
Yk2 = Y[:, 1]


# Ys1 = np.mean(Yk1)
# Ys2 = np.mean(Yk2)
# std1 = np.std(Yk1)
# std2 = np.std(Yk2)
# Yk1 = (Yk1 - Ys1)/std1
# Yk2 = (Yk2-Ys2)/std2
# Us1 = np.mean(U[:, 0])
# Us2 = np.mean(U[:, 1])
# Us3 = np.mean(U[:, 2])
# stdu1 = np.std(U[:, 0])
# stdu2 = np.std(U[:, 1])
# stdu3 = np.std(U[:, 2])
# Us = np.array([Us1, Us2, Us3]).T
# stdu = np.array([stdu1, stdu2, stdu3]).T

# For calculating Omega,Theta and defined variables.

samples = len(Y)  # No.of samples

print("Number of samples = ", samples)
N = int(0.8 * samples)  # training samples 80%
oder = 6  # oder of identified state space
n = oder // num_output

# estimation parameters


Y1 = np.zeros((n, N - n + 1))  # From 3 to N implies N-2 array...
Y2 = np.zeros((n, N - n + 1))
U1 = np.zeros((n, N - n + 1))
U2 = np.zeros((n, N - n + 1))
U3 = np.zeros((n, N - n + 1))

omega1 = np.zeros((N - 2, 4 * n))  # At a time 1 output and 3 input and 3 time series data (1+3)*3
omega2 = np.zeros((N - 2, 4 * n))

# Output arrays for pseudo inverse

yes1 = np.zeros((N - n + 1, 1))
yes2 = np.zeros((N - n + 1, 1))

# Training procedure starts here

for i in range(3, N + 1):
    j = i - 3
    Y1[:, j] = np.array([Yk1[i - n], Yk1[i - n + 1], Yk1[i - n + 2]]).T
    Y2[:, j] = np.array([Yk1[i - n], Yk1[i - n + 1], Yk1[i - n + 2]]).T
    U1[:, j] = np.array([U[0, i - n], U[0, i - n + 1], U[0, i - n + 2]]).T
    U2[:, j] = np.array([U[1, i - n], U[1, i - n + 1], U[1, i - n + 2]]).T
    U3[:, j] = np.array([U[2, i - n], U[2, i - n + 1], U[2, i - n + 2]]).T
    omega1[j, :] = np.array([(-Y1[:, j]).T, (U1[:, j]).T, (U2[:, j]).T, (U3[:, j]).T]).reshape((1, 12))
    omega2[j, :] = np.array([(-Y2[:, j]).T, (U1[:, j]).T, (U2[:, j]).T, (U3[:, j]).T]).reshape((1, 12))
    yes1[j, :] = Yk1[i]
    yes2[j, :] = Yk2[i]

inv_omega1 = np.linalg.inv(np.dot(omega1.T, omega1))
theta_1 = np.dot(np.dot(inv_omega1, omega1.T), yes1)

inv_omega2 = np.linalg.inv(np.dot(omega2.T, omega2))
theta_2 = np.dot(np.dot(inv_omega2, omega2.T), yes2)

phi1 = np.array([[float(-theta_1[n - 1]), 1, 0], [float(-theta_1[n - 2]), 0, 1], [float(-theta_1[n - 3]), 0, 0]])
phi2 = np.array([[float(-theta_2[n - 1]), 1, 0], [float(-theta_2[n - 2]), 0, 1], [float(-theta_2[n - 3]), 0, 0]])
gamma1 = np.block([np.flip(theta_1[n:n + 3]), np.flip(theta_1[n + 3:n + 6]), np.flip(theta_1[n + 6:n + 9])])
gamma2 = np.block([np.flip(theta_2[n:n + 3]), np.flip(theta_2[n + 3:n + 6]), np.flip(theta_2[n + 6:n + 9])])
zeros_n = np.zeros((n, n))

A_mat = np.block([[phi1, zeros_n], [zeros_n, phi2]])
B_mat = np.block([[gamma1], [gamma2]])
L1 = A_mat[0:3, 0]
L2 = A_mat[3:6, 3]
LA = np.block([L1.reshape(-1, 1), np.zeros((n, 1))])
LB = np.block([np.zeros((n, 1)), L2.reshape(-1, 1)])
L_mat = np.block([[LA], [LB]])
c_esti = np.array([1, 0, 0])
C_mat = np.block([[c_esti, np.zeros((1, n))], [np.zeros((1, n)), c_esti]])

print("State space model given by: ")
print("A_mat = ", A_mat.shape)
print("B_mat = ", B_mat.shape)
print("C_mat = ", C_mat.shape)
print("L_mat = ", L_mat.shape)

# Validation variables
X_hat = np.zeros((oder, samples-N))
Y_hat = np.zeros((num_output, samples-N))

# validation for noise-less variables
X_curl = np.zeros((oder, samples-N))
Y_curl = np.zeros((num_output, samples-N))
Y = Y.T
for i in range(N, samples-1):
    j = i-N
    Y_hat[:, j] = np.dot(C_mat, X_hat[:, j])
    ek = Y[:, i]-Y_hat[:, j]
    X_hat[:, j+1] = np.dot(A_mat, X_hat[:, j])+np.dot(B_mat, U[:, i])+np.dot(L_mat, ek)
    X_curl[:, j+1] = np.dot(A_mat, X_hat[:, j]) + np.dot(B_mat, U[:, i])
    Y_curl[:, j] = np.dot(C_mat, X_hat[:, j])

kTime = np.arange(N, samples*T,samples-N)
if norm == 0:
    if case == 1:
        Y_hat = Y_hat*Y_max
        Yk1 = Yk1*Y_max
        Yk2 = Yk2*Y_max
    elif case == 2:
        Yk1 = Yk1*diff_Y
        Yk1 = Yk1+Y_min
        Yk2 = Yk2 * diff_Y
        Y_hat = Y_hat + Y_min
        Y_hat = Y_hat * diff_Y
        Y_hat = Y_hat + Y_min


rmse_pitch = np.sqrt(np.mean((Y_hat[0, :] - Yk1[N:samples])**2))
print(f"Validation RMSE of pitch: {rmse_pitch} degrees")

rmse_yaw = np.sqrt(np.mean((Y_hat[1, :] - Yk2[N:samples])**2))
print(f"Validation RMSE of yaw: {rmse_yaw} degrees")

# plot for Y1

plt.figure(1)
plt.title('Output Trajectories')
plt.grid(True)
plt.plot(kTime, Yk1[N:samples], 'k-', linewidth=2, label='y')
plt.plot(kTime, Y_hat[0, :], 'r-', linewidth=2, label='y_hat')
# plt.plot(kTime, Y_curl[0, :], 'b-', linewidth=2, label='y_curl')
plt.ylabel('Y_1')
plt.legend()

# plot for Y2

plt.figure(2)
plt.title('Output Trajectories')
plt.grid(True)
plt.plot(kTime, Yk2[N:samples], 'k-', linewidth=2, label='y')
plt.plot(kTime, Y_hat[1, :], 'r-', linewidth=2, label='y_hat')
# plt.plot(kTime, Y_curl[1, :], 'b-', linewidth=2, label='y_curl')
plt.ylabel('Y_2')
plt.legend()

plt.figure(3)
plt.title('Output Trajectories')
plt.grid(True)
plt.plot(Yk1, 'k-', linewidth=2, label='y1')
plt.ylabel('Y_1')
plt.legend()


plt.figure(4)
plt.title('Output Trajectories')
plt.grid(True)
plt.plot(Yk2, 'k-', linewidth=2, label='y1')
plt.ylabel('Y_2')
plt.legend()

plt.show()

