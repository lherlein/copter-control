import numpy as np
import json
import math

# Grab constants from JSON file

droneConsts = json.load(open("src/lib/constants.json"))
Ixx = 4 * droneConsts['m_motor'] * droneConsts['r_x'] ** 2
Iyy = 4 * droneConsts['m_motor'] * droneConsts['r_y'] ** 2
Izz = 4 * droneConsts['m_motor'] * droneConsts['r_z'] ** 2

m_drone = 4 * droneConsts['m_motor'] + droneConsts['m_body'] + droneConsts['m_sensor'] + droneConsts['m_battery']

radius = droneConsts['r']
g = droneConsts['g']
c2m = np.array([[-radius, -radius, radius, radius], [radius, -radius, -radius, radius], [1, -1, 1, -1]])

## TODO: find thrust and drag constants for yaw control

# Estimate drag constant from surface area
S = droneConsts['surfaceA']
Cdmu = 0.15 # guess - small front side
Cdu = .75 # guess - moment constant - large side 
rho = 1.225
mu = 0.5 * Cdmu * rho * S
u = 0.5 * Cdu * rho * S


## rotation functions

def r1(phi):
  return np.array([[1, 0, 0], [0, math.cos(phi), -math.sin(phi)], [0, math.sin(phi), math.cos(phi)]])

def r2(theta):
  return np.array([[math.cos(theta), 0, math.sin(theta)], [0, 1, 0], [-math.sin(theta), 0, math.cos(theta)]])

def r3(psi):
  return np.array([[math.cos(psi), -math.sin(psi), 0], [math.sin(psi), math.cos(psi), 0], [0, 0, 1]])

def r321(phi, theta, psi):
  R1 = r1(phi)
  R2 = r2(theta)
  R3 = r3(psi)
  # R321 = R1 * R2 * R3
  # go right to left
  r23 = np.dot(R2, R3)
  r321 = np.dot(R1, r23)
  return r321

# Drone State Class

# Take in drone state and motor inputs, output new state
# USE LINEARIZED EQUATIONS OF MOTION 

# State describes ideal physical state of drone

class DroneStateLinear:
  def __init__(self, state, motors):
    self.state = state
    # State: [x, y, z, phi, theta, psi, u, v, w, p, q, r]

    self.motors = motors
    # Motors: [f1, f2, f3, f4] - thrust of each motor

  def update(self, td):
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
    self.state = self.state + state_dot*td

    return self.state


class DroneState:
  def __init__(self, state, motors):
    self.state = state
    # State: [x, y, z, phi, theta, psi, u, v, w, p, q, r]

    self.motors = motors
    # Motors: [f1, f2, f3, f4] - thrust of each motor

  def update(self, td):
    # Unpack state and motors
    x, y, z, phi, theta, psi, u, v, w, p, q, r = self.state
    f1, f2, f3, f4 = self.motors

    # Find control moments and forces from motor inputs
    cmoments = np.dot(c2m, [f1, f2, f3, f4])
    cforces = np.array([0, 0, -f1 - f2 - f3 - f4])

    # Isolate state components for matrices

    ## position
    pos = np.array([x, y, z])

    ## attitude
    att = np.array([phi, theta, psi])

    ## linear velocity
    vel = np.array([u, v, w])

    ## angular velocity
    ang_vel = np.array([p, q, r])

    # Create rotation matrix
    R = r321(phi, theta, psi)

    ## Position derivatives:

    pos_dot = np.dot(R, vel)

    ## Attitude derivatives:
    # create transfer matrix
    A = np.array([[1,0,0],[math.sin(phi)*math.tan(theta), math.cos(phi), math.sin(phi)*(1/math.cos(theta))], [math.cos(phi)*math.tan(theta), -math.sin(phi), math.cos(phi)*(1/math.cos(theta))]])
    att_dot = np.dot(A, ang_vel)

    ## Linear velocity derivatives:
    # create transfer matrices
    A = np.array([r*v-q*w, p*w-r*u, q*u-p*v])
    B = np.array([-math.sin(theta), math.cos(theta)*math.sin(phi), math.cos(theta)*math.cos(phi)])

    # Calculate Aero Forces
    vmag = (u**2 + v**2 + w**2)**0.5
    F_aero = -mu*vmag*vel

    # Calculate Motor Forces
    F_motor = np.array([0, 0, f1 + f2 + f3 + f4])

    # Calc vel_dot

    vel_dot = A + g*B + (1/m_drone)*F_aero + (1/m_drone)*F_motor

    ## Angular velocity derivatives:
    # create transfer matrices
    A = np.array([((Iyy-Izz)/Ixx)*q*r, ((Izz-Ixx)/Iyy)*p*r, ((Ixx-Iyy)/Izz)*p*q])
    B = np.array([1/Ixx, 1/Iyy, 1/Izz])

    # Calculate aero moments
    ang_vel_mag = (p**2 + q**2 + r**2)**0.5
    M_aero = -u*ang_vel_mag*ang_vel

    # Calc ang_vel_dot
    ang_vel_dot = A + B*M_aero + B*cmoments
    

    ## Create state_dot vector
    state_dot = np.array([pos_dot[0], pos_dot[1], pos_dot[2], att_dot[0], att_dot[1], att_dot[2], vel_dot[0], vel_dot[1], vel_dot[2], ang_vel_dot[0], ang_vel_dot[1], ang_vel_dot[2]])

    ## Update state
    self.state = self.state + state_dot*td

    return self.state
