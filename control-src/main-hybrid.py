"""
MicroPython PI/PID flight controller for custom drones.

Currently: use known Ixx, Iyy, Izz values for the drone
Goal: Have the ability to calculate Ixx, Iyy, Izz using bang-bang method
  - bang-bang to 15 degree roll about each axis, measure angular rate, time taken, and torque
  - calculate Ixx, Iyy, Izz

Following pseudo code defined in ./fc-pseudo.py
"""

from machine import Pin, I2C, PWM, UART
from lib.controller import PI, PID
from lib.mpu6050 import MPU6050
from lib.bmp180 import BMP180
import time
import math

def calibrateSensors():
  sealevel = 0
  phiNormal = 0
  thetaNormal = 0
  # Average N readings
  N = 100
  for i in range(N):
    sealevel += bmp.pressure
    imuData = readIMU()
    orientation = calcAngles(imuData)
    phiNormal += orientation[0]
    thetaNormal += orientation[1]
    time.sleep_ms(10)

  bmp.sealevel = sealevel/N
  return [phiNormal/N, thetaNormal/N, sealevel/N]

def readIMU():
  # Get IMU Data
  accelData = mpu.read_accel_data()
  gyroData = mpu.read_gyro_data()

  return [accelData, gyroData]

def readBMP():
  # Get BMP Data
  pressure = bmp.pressure
  altitude = bmp.altitude
  temperature = bmp.temperature

  return [pressure, altitude, temperature]

def calcAngles(imuData): # Not entirely sure this will work when in flight

  # Get Accelerometer Data
  accelData = imuData[0]

  phi = math.atan2(accelData[1], accelData[2])
  theta = math.atan2(accelData[0], accelData[2])

  return [phi, theta]

def rad2deg(rad):
  return rad * 180 / math.pi

def deg2rad(deg):
  return deg * math.pi / 180

def ms2ns(ms):
  return ms * 1000000

def ns2ms(ns):
  return ns / 1000000

def pos2u16(pos):
  # get value between 0 and 100 representing servo position
  # return value between 0 and 65535 representing duty cycle between 0 and 100%
  # MUST KEEP RETURN BETWEEN 5-10% DUTY CYCLE

  # 0 input -> 65535*0.05
  # 100 input -> 65535*0.1

  posPercent = pos / 100

  # convert posPercent to duty cycle within bounds
  duty = 0.05 + posPercent * 0.05

  return int(duty * 65535)

def readPulseWidthUs(pin):
  while pin.value() == 0:
    pass
  start_high = time.ticks_us()
  while pin.value() == 1:
    pass
  end_high = time.ticks_us()
  pulse_length = end_high - start_high
  return pulse_length
  
def pwm2controlAngle(pulse_width):
  # pulse witdth between 1000 and 2000 (x)
  # control angle between -60 and 60 (y)
  # y = c+ ((x-a)*(d-c))/(b-a) 

  a = 1000
  b = 2000
  c = -60
  d = 60

  return c + ((pulse_width - a) * (d - c)) / (b - a)

# Define pins
MOTOR_FRONT_LEFT_PIN = 2
MOTOR_FRONT_RIGHT_PIN = 3
MOTOR_BACK_LEFT_PIN = 4
MOTOR_BACK_RIGHT_PIN = 5

ELEVATOR_SERVO_PIN = 7
AILERON_SERVO_PIN = 8
THROTTLE_SERVO_PIN = 9

PITCH_RC_INPUT_PIN = 18
ROLL_RC_INPUT_PIN = 19
THROTTLE_RC_INPUT_PIN = 20
YAW_RC_INPUT_PIN = 21

MPU6050_SCL_PIN = 17
MPU6050_SDA_PIN = 16

BMP180_SCL_PIN = 15
BMP180_SDA_PIN = 14

# Define some Bounds

max_control_angle = 60
thrust_min_pulse = 1000
servo_middle_pulse = 1500
pwm_freq = 50

"""
BEGIN STARTUP SEQUENCE
"""

print("Starting up...")

### Input pins
pitch_in_pin = Pin(PITCH_RC_INPUT_PIN, Pin.IN)
roll_in_pin = Pin(ROLL_RC_INPUT_PIN, Pin.IN)
thrust_in_pin = Pin(THROTTLE_RC_INPUT_PIN, Pin.IN)
yaw_in_pin = Pin(YAW_RC_INPUT_PIN, Pin.IN)

mpu_sda = Pin(MPU6050_SDA_PIN)
mpu_scl = Pin(MPU6050_SCL_PIN)
bmp_sda = Pin(BMP180_SDA_PIN)
bmp_scl = Pin(BMP180_SCL_PIN)

### Output pins
motorFL_pin = Pin(MOTOR_FRONT_LEFT_PIN, Pin.OUT)
motorFR_pin = Pin(MOTOR_FRONT_RIGHT_PIN, Pin.OUT)
motorBL_pin = Pin(MOTOR_BACK_LEFT_PIN, Pin.OUT)
motorBR_pin = Pin(MOTOR_BACK_RIGHT_PIN, Pin.OUT)

elevator_pin = Pin(ELEVATOR_SERVO_PIN, Pin.OUT)
aileron_pin = Pin(AILERON_SERVO_PIN, Pin.OUT)

# I2C
i2c_mpu = I2C(0, scl=mpu_scl, sda=mpu_sda)
i2c_bmp = I2C(1, scl=bmp_scl, sda=bmp_sda)

# Define Sensors
print("Waking up sensors...")

bmp = BMP180(i2c_bmp)
mpu = MPU6050(i2c_mpu)

mpu.wake()

print("Sensors ready.")

# Define PWM objects
motorFL_pwm = PWM(motorFL_pin)
motorFR_pwm = PWM(motorFR_pin)
motorBL_pwm = PWM(motorBL_pin)
motorBR_pwm = PWM(motorBR_pin)

elevator_pwm = PWM(elevator_pin)
aileron_pwm = PWM(aileron_pin)

# Startup motor PWM signals at minimum value

print("Starting up PWM, setting motors to minimum thrust...")


elevator_pwm.freq(pwm_freq)
elevator_pwm.duty_ns(servo_middle_pulse*1000)
aileron_pwm.freq(pwm_freq)
aileron_pwm.duty_ns(servo_middle_pulse*1000)
motorFL_pwm.freq(pwm_freq)
motorFL_pwm.duty_ns(thrust_min_pulse*1000)
motorFR_pwm.freq(pwm_freq)
motorFL_pwm.duty_ns(thrust_min_pulse*1000)
motorBL_pwm.freq(pwm_freq)
motorBL_pwm.duty_ns(thrust_min_pulse*1000)
motorBR_pwm.freq(pwm_freq)
motorBR_pwm.duty_ns(thrust_min_pulse*1000)

# Calibrate sensors

[phiNormal, thetaNormal, sealevel] = calibrateSensors()

print("Sensors calibrated for: phiNormal: ", phiNormal, " thetaNormal: ", thetaNormal, " sealevel: ", sealevel)

# Initialize PID controllers

kpPr = 1
kiPr = 0.1
kdPr = 0.01

print("Initializing PI controllers...")

pitchPID = PI(kpPr, kiPr) # only control pitch and roll for now
rollPID = PI(kpPr, kiPr)

print("PI controllers initialized.")

# NEED: Ixx, Iyy, r

Ixx = .5
Iyy = .5

r = .3

"""
Main Loop
"""

while True:
  # Read Sensor Data
  imuData = readIMU()
  bmpData = readBMP()

  # Calculate Roll, Pitch angles
  orientation = calcAngles(imuData)

  # Read incoming rc control data
  desired_roll_pwm = readPulseWidthUs(roll_in_pin)
  desired_pitch_pwm = readPulseWidthUs(pitch_in_pin)
  desired_thrust_pwm = readPulseWidthUs(thrust_in_pin)
  desired_yaw_pwm = readPulseWidthUs(yaw_in_pin)

  print("Desired Roll: ", desired_roll_pwm, " Desired Pitch: ", desired_pitch_pwm, " Desired Throttle: ", desired_thrust_pwm, " Desired Yaw: ", desired_yaw_pwm)

  # Calculate desired angles
  desired_roll = pwm2controlAngle(desired_roll_pwm)
  desired_pitch = pwm2controlAngle(desired_pitch_pwm)
  desired_yaw = pwm2controlAngle(desired_yaw_pwm)

  # Give controllers the desired angles
  pitchPID.setpoint = desired_pitch
  rollPID.setpoint = desired_roll

  # Update PID controllers
  del_phi = pitchPID.update(orientation[0])
  del_theta = rollPID.update(orientation[1])
  
  # Calculate control moments
  Lc = Ixx * del_phi
  Mc = Iyy * del_theta
  Nc = 0
  
  # Calculate motor thrusts
  controlThrustFr = (-r*Lc)+ (r*Mc) + Nc
  controlThrustFl = (r*Lc) + (r*Mc) - Nc
  controlThrustBr = (-r*Lc) - (r*Mc) - Nc
  controlThrustBl = (r*Lc) - (r*Mc) + Nc

  # Set motor thrusts
  frThrust = desired_thrust_pwm #+ newton2pwm(controlThrustFr) 
  flThrust = desired_thrust_pwm #+ newton2pwm(controlThrustFl)
  brThrust = desired_thrust_pwm #+ newton2pwm(controlThrustBr)
  blThrust = desired_thrust_pwm #+ newton2pwm(controlThrustBl)

  # Set servo positions - passthrough for now
  elevator_pwm.duty_ns(int(desired_pitch_pwm*1000))
  aileron_pwm.duty_ns(int(desired_roll_pwm*1000))

  #print("elevator: ", desired_pitch_pwm*1000, " aileron: ", desired_roll_pwm*1000)

  # Set motor thrusts
  motorFL_pwm.duty_ns(int(flThrust*1000))
  motorFR_pwm.duty_ns(int(frThrust*1000))
  motorBL_pwm.duty_ns(int(blThrust*1000))
  motorBR_pwm.duty_ns(int(brThrust*1000))