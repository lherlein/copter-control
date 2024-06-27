import numpy as np
import matplotlib.pyplot as plt
from simple_pid import PID
from lib.droneState import DroneStateLinear, DroneState
import json
import time

# Grab constants from JSON file

droneConsts = json.load(open("src/lib/constants.json"))
Ixx = 4 * droneConsts['m_motor'] * droneConsts['r_x'] ** 2
Iyy = 4 * droneConsts['m_motor'] * droneConsts['r_y'] ** 2
Izz = 4 * droneConsts['m_motor'] * droneConsts['r_z'] ** 2

m_drone = 4 * droneConsts['m_motor'] + droneConsts['m_body'] + droneConsts['m_sensor'] + droneConsts['m_battery']

r = droneConsts['r']
g = droneConsts['g']
c2m = np.array([[-r, -r, r, r], [r, -r, -r, r], [1, -1, 1, -1]])

# This program will show simple PID control of roll and pitch of a drone

## Simulation Parameters

simSpeed = 10

#s = control.TransferFunction.s

Kp = 9000 # Proportional Gain
Ki = 100 # Integral Time Constant
Kd = 600 # Derivative Time Constant

desired_roll = 0 # Desired Roll Angle
desired_pitch = 0 # Desired Pitch Angle

roll_pid = PID(Kp, Ki, Kd, setpoint=desired_roll)
pitch_pid = PID(Kp, Ki, Kd, setpoint=desired_pitch)

#G = Kp * (1 + (1/(Ti*s)) + Td*s) # PID Transfer Function

# Generate the time vector

Tdelt = 0.01 # Sampling Time
roll_pid.sample_time = Tdelt/simSpeed
pitch_pid.sample_time = Tdelt/simSpeed
Tstart = 0 # Start Time
Tstop = 10 # End Time
N = int((Tstop - Tstart)/Tdelt) # Number of Samples
t = np.linspace(Tstart, Tstop, N) # Time Vector

print("Simulation will take: ", N*Tdelt/simSpeed, " seconds")

# Initialize vectors/matrices
state = np.zeros([12, N+1]) # State Vector
stateLin = np.zeros([12, N+1])

# State: [x, y, z, phi, theta, psi, u, v, w, p, q, r]

motorsc = np.zeros([4,N+1]) # Motor Speeds
motorscLin = np.zeros([4,N+1]) # Motor Speeds

# Initial Conditions

roll = 0.1 # Initial Roll Angle
pitch = 0 # Initial Pitch Angle

IC = np.array([0, 0, 0, roll, pitch, 0, 0, 0, 0, 0, 0, 0])
state[:,0] = IC
stateLin[:,0] = IC

ICmotors = np.array([0, 0, 0, 0])
motorsc[:,0] = ICmotors
motorscLin[:,0] = ICmotors

# Convenience Variables
X = 0
Y = 1
Z = 2
PHI = 3
THETA = 4
PSI = 5
U = 6
V = 7
W = 8
P = 9
Q = 10
R = 11

index = 1
for i in t:
  del_phi = roll_pid(state[PHI][index-1])
  del_theta = pitch_pid(state[THETA][index-1])

  del_phi_lin = roll_pid(stateLin[PHI][index-1])
  del_theta_lin = pitch_pid(stateLin[THETA][index-1])

  #print("Roll: ", stateLin[PHI][index-1], "Pitch: ", stateLin[THETA][index-1], "del_phi: ", del_phi, "del_theta: ", del_theta)

  del_Lc = Ixx * del_phi * Tdelt
  del_Mc = Iyy * del_theta * Tdelt

  del_Lc_lin = Ixx * del_phi_lin * Tdelt
  del_Mc_lin = Iyy * del_theta_lin * Tdelt

  momentsc = np.array([del_Lc, del_Mc, 0])
  momentscLin = np.array([del_Lc_lin, del_Mc_lin, 0])
  motorscI = np.dot(c2m.T, momentsc) # motor thrust
  motorscILin = np.dot(c2m.T, momentscLin) # motor thrust

  motorsc[:,index] = motorscI
  motorscLin[:,index] = motorscILin

  # Update drone state
  state[:,index] = DroneState(state[:,index-1], motorscI).update(Tdelt)
  stateLin[:,index] = DroneStateLinear(stateLin[:,index-1], motorscI).update(Tdelt)

  index += 1
  time.sleep(Tdelt/simSpeed)


## Plotting

# plot phi, theta, psi

plt.figure()
plt.plot(t, stateLin[PHI][:-1], label='Roll')
plt.plot(t, stateLin[THETA][:-1], label='Pitch')
plt.plot(t, stateLin[PSI][:-1], label='Yaw')
plt.title('Drone Attitude -linear')
plt.xlabel('Time [s]')
plt.ylabel('Angle [rad]')
plt.legend()
plt.grid()
plt.savefig("plots/pitchroll/simple-pid-attitude-linear.png")
plt.close()

# plot x,y,z

plt.figure()
plt.plot(t, stateLin[X][:-1], label='X')
plt.plot(t, stateLin[Y][:-1], label='Y')
plt.plot(t, stateLin[Z][:-1], label='Z')
plt.title('Drone Position -linear')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid()
plt.savefig("plots/pitchroll/simple-pid-position-linear.png")
plt.close()

# plot u,v,w

plt.figure()
plt.plot(t, stateLin[U][:-1], label='u')
plt.plot(t, stateLin[V][:-1], label='v')
plt.plot(t, stateLin[W][:-1], label='w')
plt.title('Drone Velocity -linear')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid()
plt.savefig("plots/pitchroll/simple-pid-velocity-linear.png")
plt.close()

# plot p,q,r

plt.figure()
plt.plot(t, stateLin[P][:-1], label='p')
plt.plot(t, stateLin[Q][:-1], label='q')
plt.plot(t, stateLin[R][:-1], label='r')
plt.title('Drone Angular Velocity -linear')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.legend()
plt.grid()
plt.savefig("plots/pitchroll/simple-pid-angular-velocity-linear.png")
plt.close()

# plot motor speeds

plt.figure()
plt.plot(t, motorscLin[0][:-1], label='Motor 1')
plt.plot(t, motorscLin[1][:-1], label='Motor 2')
plt.plot(t, motorscLin[2][:-1], label='Motor 3')
plt.plot(t, motorscLin[3][:-1], label='Motor 4')
plt.title('Motor Force -linear')
plt.xlabel('Time [s]')
plt.ylabel('Force [N]')
plt.legend()
plt.grid()
plt.savefig("plots/pitchroll/simple-pid-motor-speeds-linear.png")
plt.close()

## Nonlinear
##############################################33

plt.figure()
plt.plot(t, state[PHI][:-1], label='Roll')
plt.plot(t, state[THETA][:-1], label='Pitch')
plt.plot(t, state[PSI][:-1], label='Yaw')
plt.title('Drone Attitude')
plt.xlabel('Time [s]')
plt.ylabel('Angle [rad]')
plt.legend()
plt.grid()
plt.savefig("plots/pitchroll/simple-pid-attitude.png")
plt.close()

# plot x,y,z

plt.figure()
plt.plot(t, state[X][:-1], label='X')
plt.plot(t, state[Y][:-1], label='Y')
plt.plot(t, state[Z][:-1], label='Z')
plt.title('Drone Position')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid()
plt.savefig("plots/pitchroll/simple-pid-position.png")
plt.close()

# plot u,v,w

plt.figure()
plt.plot(t, state[U][:-1], label='u')
plt.plot(t, state[V][:-1], label='v')
plt.plot(t, state[W][:-1], label='w')
plt.title('Drone Velocity')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid()
plt.savefig("plots/pitchroll/simple-pid-velocity.png")
plt.close()

# plot p,q,r

plt.figure()
plt.plot(t, state[P][:-1], label='p')
plt.plot(t, state[Q][:-1], label='q')
plt.plot(t, state[R][:-1], label='r')
plt.title('Drone Angular Velocity')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.legend()
plt.grid()
plt.savefig("plots/pitchroll/simple-pid-angular-velocity.png")
plt.close()

# plot motor speeds

plt.figure()
plt.plot(t, motorsc[0][:-1], label='Motor 1')
plt.plot(t, motorsc[1][:-1], label='Motor 2')
plt.plot(t, motorsc[2][:-1], label='Motor 3')
plt.plot(t, motorsc[3][:-1], label='Motor 4')
plt.title('Motor Force')
plt.xlabel('Time [s]')
plt.ylabel('Force [N]')
plt.legend()
plt.grid()
plt.savefig("plots/pitchroll/simple-pid-motor-speeds.png")
plt.close()
