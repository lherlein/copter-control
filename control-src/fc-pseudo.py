## This is a micropython script


# Import necessary libraries
import numpy as np
from machine import Pin
from sPID import PID
import time

## Pseudo code for PID controller

## Define pins
### Input pins
pitch_in = Pin(26, Pin.IN)
roll_in = Pin(27, Pin.IN)
thrust_in = Pin(28, Pin.IN)
yaw_in = Pin(29, Pin.IN)
### Output pins
motor1 = Pin(3, Pin.OUT)
motor2 = Pin(4, Pin.OUT)
motor3 = Pin(2, Pin.OUT)
motor4 = Pin(1, Pin.OUT)

# Pitch/Roll Gains

Kp_pr = 4500 # Proportional Gain
Ki_pr = 100 # Integral Gain
Kd_pr = 300 # Derivative Gain

# Thrust Gains

Kp_t = .2 # Proportional Gain
Ki_t = 1 # Integral Gain
Kd_t = 0.01 # Derivative Gain

# Inertia Constants
r_x .7
r_y .8
r_z .05
Ixx = 4 * m_motor * r_x**2
Iyy = 4 * m_motor * r_y**2
Izz = 4 * m_motor * r_z**2

# Define angle limits

max_pitch = 45
max_roll = 45

# Define input pulse width limits - from experimentation
pitch_min_pulse = 1000
pitch_max_pulse = 2000
pitch_pulse_arr = np.linspace(pitch_min_pulse, pitch_max_pulse, pitch_max_pulse - pitch_min_pulse + 1)

roll_min_pulse = 1000
roll_max_pulse = 2000
roll_pulse_arr = np.linspace(roll_min_pulse, roll_max_pulse, roll_max_pulse - roll_min_pulse + 1)

thrust_min_pulse = 1000
thrust_max_pulse = 2000
thrust_pulse_arr = np.linspace(thrust_min_pulse, thrust_max_pulse, thrust_max_pulse - thrust_min_pulse + 1)

yaw_min_pulse = 1000
yaw_max_pulse = 2000
yaw_pulse_arr = np.linspace(yaw_min_pulse, yaw_max_pulse, yaw_max_pulse - yaw_min_pulse + 1)

# Transitioni Matrix

## Use ulab (https://github.com/v923z/micropython-ulab) for micropython matrix operations, but use numpy here
c2m = np.array([[-r, -r, r, r], [r, -r, -r, r], [1, -1, 1, -1]])

# Startup Sequence

## Initialize motors

# Define Functions

def readMPU():
  # Read MPU data
  return mpuData

def calcAngles(mpuData):
  # Calculate Roll, Pitch
  return [roll, pitch]

def measure_pwm(pin):
  while pin.value() == 0:
    pass
  start_high = time.ticks_us()
  while pin.value() == 1:
    pass
  end_high = time.ticks_us()
  pulse_length = end_high - start_high
  return pulse_length

def interpretInput():
  # Read input pins
  desired_roll = measure_pwm(pitch_in) # Read desired roll
  desired_pitch = measure_pwm(pitch_in) # Read desired pitch
  desired_thrust = measure_pwm(pitch_in)
  desired_yaw = measure_pwm(pitch_in)
  
  # Convert pulse width to angle
  desired_roll = np.interp(desired_roll, roll_pulse_arr, np.linspace(-max_roll, max_roll, roll_pulse_arr.length))
  desired_pitch = np.interp(desired_pitch, pitch_pulse_arr, np.linspace(-max_pitch, max_pitch, roll_pulse_arr.length))
  desired_yaw = np.interp(desired_yaw, yaw_pulse_arr, np.linspace(-180, 180, roll_pulse_arr.length))
  
  return [desired_roll, desired_pitch, desired_thrust, desired_yaw]

def thrust2u16(thrust):
  # Convert thrust to u16
  return thrust

# Main Loop

while true:

  # Read sensor data
  mpuData = readMPU()
  # Calculate Roll, Pitch
  [roll, pitch] = calcAngles(mpuData)

  # Read desired angles
  [desired_roll, desired_pitch, thrust, desired_yaw] = interpretInput()

  # Define PID controllers

  roll_pid = PID(Kp_pr, Ki_pr, Kd_pr, setpoint=desired_roll)
  pitch_pid = PID(Kp_pr, Ki_pr, Kd_pr, setpoint=desired_pitch)

  #thrust_pid = PID(Kp_t, Ki_t, Kd_t, setpoint=desired_z_position)

  # Calculate del roll/pitch Values

  del_phi = roll_pid(roll)
  del_theta = pitch_pid(pitch)

  # Directly set thrust input value as output value

  motorscI_t = [thrust, thrust, thrust, thrust]

  # Calculate Control Moments
  del_Lc = Ixx * del_phi * Tdelt
  del_Mc = Iyy * del_theta * Tdelt
  del_Nc = 0

  # Calculate Control Force(s)

  # Zc = del_thrust * (m_drone/Tdelt)
  # m_force = Zc/4

  # Find control force from PID

  momentsc = np.array([del_Lc, del_Mc, 0])
  motorscI_rp = np.dot(c2m.T, momentsc) # motor thrust for roll and pitch

  #motorscI_t = np.array([m_force, m_force, m_force, m_force]) # motor thrust for thrust

  motorscI = motorscI_rp + motorscI_t

  ## Send motor thrust to motors
  motor1.value(thrust2u16(motorscI[0]))
  motor2.value(thrust2u16(motorscI[1]))
  motor3.value(thrust2u16(motorscI[2]))
  motor4.value(thrust2u16(motorscI[3]))
