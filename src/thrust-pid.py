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

Kp = 1 # Proportional Gain
Ki = 0.001 # Integral Time Constant
Kd = 0.01 # Derivative Time Constant

desired_z_position = -1 # Desired Z Position

thrust_pid = PID(Kp, Ki, Kd, setpoint=desired_z_position)

#G = Kp * (1 + (1/(Ti*s)) + Td*s) # PID Transfer Function

# Generate the time vector

Tdelt = 0.01 # Sampling Time
thrust_pid.sample_time = Tdelt/simSpeed
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

IC = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
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
  del_thrust = thrust_pid(state[Z][index-1])
  del_thrust_lin = thrust_pid(stateLin[Z][index-1])

  # Find control force from PID
  Zc = del_thrust * (m_drone/Tdelt)
  Zc_lin = del_thrust_lin * (m_drone/Tdelt)

  m_force = Zc/4
  m_force_lin = Zc_lin/4

  motorscI = np.array([m_force, m_force, m_force, m_force])
  motorscILin = np.array([m_force_lin, m_force_lin, m_force_lin, m_force_lin])

  motorsc[:,index] = motorscI
  motorscLin[:,index] = motorscILin

  # Update drone state
  state[:,index] = DroneState(state[:,index-1], motorscI).update(Tdelt)
  stateLin[:,index] = DroneStateLinear(stateLin[:,index-1], motorscI).update(Tdelt)

  #print("del_thrust: ", del_thrust, "Zc: ", Zc, "Z: ", state[Z][index-1])

  index += 1
  time.sleep(Tdelt/simSpeed)


## Plotting

# plot phi, theta, psi

plt.figure()
plt.plot(t, stateLin[PHI][:-1], label='Roll')
plt.plot(t, stateLin[THETA][:-1], label='Pitch')
plt.plot(t, stateLin[PSI][:-1], label='Yaw')
plt.title('Drone Attitude -linear - Thrust PID')
plt.xlabel('Time [s]')
plt.ylabel('Angle [rad]')
plt.legend()
plt.grid()
plt.savefig("plots/thrust/simple-pid-attitude-linear-thrust.png")
plt.close()

# plot x,y,z

plt.figure()
plt.plot(t, stateLin[X][:-1], label='X')
plt.plot(t, stateLin[Y][:-1], label='Y')
plt.plot(t, stateLin[Z][:-1], label='Z')
plt.title('Drone Position -linear - Thrust PID')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid()
plt.savefig("plots/thrust/simple-pid-position-linear-thrust.png")
plt.close()

# plot u,v,w

plt.figure()
plt.plot(t, stateLin[U][:-1], label='u')
plt.plot(t, stateLin[V][:-1], label='v')
plt.plot(t, stateLin[W][:-1], label='w')
plt.title('Drone Velocity -linear - Thrust PID')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid()
plt.savefig("plots/thrust/simple-pid-velocity-linear-thrust.png")
plt.close()

# plot p,q,r

plt.figure()
plt.plot(t, stateLin[P][:-1], label='p')
plt.plot(t, stateLin[Q][:-1], label='q')
plt.plot(t, stateLin[R][:-1], label='r')
plt.title('Drone Angular Velocity -linear - Thrust PID')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.legend()
plt.grid()
plt.savefig("plots/thrust/simple-pid-angular-velocity-linear-thrust.png")
plt.close()

# plot motor speeds

plt.figure()
plt.plot(t, motorscLin[0][:-1], label='Motor 1')
plt.plot(t, motorscLin[1][:-1], label='Motor 2')
plt.plot(t, motorscLin[2][:-1], label='Motor 3')
plt.plot(t, motorscLin[3][:-1], label='Motor 4')
plt.title('Motor Thrust -linear - Thrust PID')
plt.xlabel('Time [s]')
plt.ylabel('Thrust [N]')
plt.legend()
plt.grid()
plt.savefig("plots/thrust/simple-pid-motor-speeds-linear-thrust.png")
plt.close()

## Nonlinear
##############################################33

plt.figure()
plt.plot(t, state[PHI][:-1], label='Roll')
plt.plot(t, state[THETA][:-1], label='Pitch')
plt.plot(t, state[PSI][:-1], label='Yaw')
plt.title('Drone Attitude - Thrust PID')
plt.xlabel('Time [s]')
plt.ylabel('Angle [rad]')
plt.legend()
plt.grid()
plt.savefig("plots/thrust/simple-pid-attitude-thrust.png")
plt.close()

# plot x,y,z

plt.figure()
plt.plot(t, state[X][:-1], label='X')
plt.plot(t, state[Y][:-1], label='Y')
plt.plot(t, state[Z][:-1], label='Z')
plt.title('Drone Position - Thrust PID')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()
plt.grid()
plt.savefig("plots/thrust/simple-pid-position-thrust.png")
plt.close()

# plot u,v,w

plt.figure()
plt.plot(t, state[U][:-1], label='u')
plt.plot(t, state[V][:-1], label='v')
plt.plot(t, state[W][:-1], label='w')
plt.title('Drone Velocity - Thrust PID')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid()
plt.savefig("plots/thrust/simple-pid-velocity-thrust.png")
plt.close()

# plot p,q,r

plt.figure()
plt.plot(t, state[P][:-1], label='p')
plt.plot(t, state[Q][:-1], label='q')
plt.plot(t, state[R][:-1], label='r')
plt.title('Drone Angular Velocity - Thrust PID')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.legend()
plt.grid()
plt.savefig("plots/thrust/simple-pid-angular-velocity-thrust.png")
plt.close()

# plot motor speeds

plt.figure()
plt.plot(t, motorsc[0][:-1], label='Motor 1')
plt.plot(t, motorsc[1][:-1], label='Motor 2')
plt.plot(t, motorsc[2][:-1], label='Motor 3')
plt.plot(t, motorsc[3][:-1], label='Motor 4')
plt.title('Motor Thrust - Thrust PID')
plt.xlabel('Time [s]')
plt.ylabel('Thrust [N]')
plt.legend()
plt.grid()
plt.savefig("plots/thrust/simple-pid-motor-speeds-thrust.png")
plt.close()
