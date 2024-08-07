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

# Pitch/Roll PID Constants

Kp_pr = 4500 # Proportional Gain
Ki_pr = 100 # Integral Gain
Kd_pr = 300 # Derivative Gain

desired_roll = 0 # Desired Roll Angle
desired_pitch = 0 # Desired Pitch Angle

roll_pid = PID(Kp_pr, Ki_pr, Kd_pr, setpoint=desired_roll)
pitch_pid = PID(Kp_pr, Ki_pr, Kd_pr, setpoint=desired_pitch)

# Thrust PID Constants

Kp_t = .2 # Proportional Gain
Ki_t = 1 # Integral Gain
Kd_t = 0.01 # Derivative Gain

desired_z_position = 0 # Desired Z Position

thrust_pid = PID(Kp_t, Ki_t, Kd_t, setpoint=desired_z_position)

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

roll = 0 # Initial Roll Angle
pitch = 0.1 # Initial Pitch Angle

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
  # Update PID controllers
  del_phi = roll_pid(state[PHI][index-1])
  del_theta = pitch_pid(state[THETA][index-1])
  del_thrust = thrust_pid(state[Z][index-1])

  # Calculate Control Moments
  del_Lc = Ixx * del_phi * Tdelt
  del_Mc = Iyy * del_theta * Tdelt
  del_Nc = 0

  # Calculate Control Force(s)

  Zc = del_thrust * (m_drone/Tdelt)
  m_force = Zc/4

  # Find control force from PID

  momentsc = np.array([del_Lc, del_Mc, 0])
  motorscI_rp = np.dot(c2m.T, momentsc) # motor thrust for roll and pitch

  motorscI_t = np.array([m_force, m_force, m_force, m_force]) # motor thrust for thrust

  motorscI = motorscI_rp + motorscI_t

  motorsc[:,index] = motorscI

  # Update drone state
  state[:,index] = DroneState(state[:,index-1], motorscI).update(Tdelt)

  index += 1
  time.sleep(Tdelt/simSpeed)

## Plots

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
plt.savefig("plots/simple-pid-attitude.png")
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
plt.savefig("plots/simple-pid-position.png")
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
plt.savefig("plots/simple-pid-velocity.png")
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
plt.savefig("plots/simple-pid-angular-velocity.png")
plt.close()

# plot motor speeds

plt.figure()
plt.plot(t, motorsc[0][:-1], label='Motor 1')
plt.plot(t, motorsc[1][:-1], label='Motor 2')
plt.plot(t, motorsc[2][:-1], label='Motor 3')
plt.plot(t, motorsc[3][:-1], label='Motor 4')
plt.title('Motor Thrust')
plt.xlabel('Time [s]')
plt.ylabel('Thrust [N]')
plt.legend()
plt.grid()
plt.savefig("plots/simple-pid-motor-speeds.png")
plt.close()
