import numpy as np
import json
import os

# Grab constants from JSON file

droneConsts = json.load(open("src/lib/constants.json"))
Ixx = 4 * droneConsts['m_motor'] * droneConsts['r_x'] ** 2
Iyy = 4 * droneConsts['m_motor'] * droneConsts['r_y'] ** 2
Izz = 4 * droneConsts['m_motor'] * droneConsts['r_z'] ** 2

m_drone = 4 * droneConsts['m_motor'] + droneConsts['m_body'] + droneConsts['m_sensor'] + droneConsts['m_battery']

r = droneConsts['r']
g = droneConsts['g']
c2m = np.array([[-r, -r, r, r], [r, -r, -r, r], [1, -1, 1, -1]])

## TODO: find thrust and drag constants for yaw control

# Drone State Class

# Take in drone state and motor inputs, output new state
# USE LINEARIZED EQUATIONS OF MOTION 

# State describes ideal physical state of drone

class DroneState:
  def __init__(self, state, motors):
    self.state = state
    # State: [x, y, z, phi, theta, psi, u, v, w, p, q, r]

    self.motors = motors
    # Motors: [f1, f2, f3, f4] - thrust of each motor

  def update(self):
    # Unpack state and motors
    x, y, z, phi, theta, psi, u, v, w, p, q, r = self.state
    f1, f2, f3, f4 = self.motors

    # Find control moments and forces from motor inputs
    cmoments = np.dot(c2m, [f1, f2, f3, f4])
    cforces = np.array([0, 0, -f1 - f2 - f3 - f4])

    # Find angular accelerations
    p_dot = (1/Ixx) * cmoments[0]
    q_dot = (1/Iyy) * cmoments[1]
    r_dot = (1/Izz) * cmoments[2]

    # Find linear accelerations
    u_dot = g*(-1*theta) + (1/m_drone)*cforces[0]
    v_dot = g*(phi) + (1/m_drone)*cforces[1]
    w_dot = g*(0) + (1/m_drone)*cforces[2]

    # Find angular velocities
    phi_dot = p
    theta_dot = q
    psi_dot = r

    # Find linear velocities
    x_dot = u
    y_dot = v
    z_dot = w

    # Create state_dot vector
    state_dot = np.array([x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, u_dot, v_dot, w_dot, p_dot, q_dot, r_dot])

    # Update state
    self.state = self.state + state_dot

    return self.state