from machine import Pin, I2C, PWM, UART
from lib.controller import PI, PID
from lib.altimu import AltIMU10
from lib.fusion import Fusion
import time
import math
import uasyncio

def calibrateSensors():
  phiNormal = 0
  thetaNormal = 0
  gyroX = 0
  gyroY = 0
  gyroZ = 0
  # Average N readings
  N = 100
  for i in range(N):
    imuData = readIMU()
    gyroX += imuData[1][0]
    gyroY += imuData[1][1]
    gyroZ += imuData[1][2]
    orientation = calcAngles(imuData)
    phiNormal += orientation[0]
    thetaNormal += orientation[1]
    time.sleep_ms(10)

  return [phiNormal/N, thetaNormal/N, gyroX/N, gyroY/N, gyroZ/N]

def getMagData():
  magData = imu.read_magnetometer_raw()
  return tuple(magData)

def CalibrateStopFunction():
  time.sleep_ms(100)
  return True

def calibrateBarometer():
  # Average N readings
  print("Calibrating Barometer")
  N = 100
  barData = 0
  for i in range(N):
    millibars = imu.read_barometer_millibars()
    barData += millibars
    time.sleep_ms(10)
  imu.set_barometer_sea_level_pressure(barData/N)
  print("Barometer Calibrated")

def calibrateFusion():
  # Calibrate fusion
  print("Calibrating Fuse")
  fuse.calibrate(getMagData, CalibrateStopFunction, 2000)
  print("Fuse Calibrated")


def readIMU():
  # Get IMU Data
  accelData = imu.read_accelerometer_g_forces()
  gyroData = imu.read_gyroscope_angular_velocity()
  barData = imu.read_barometer_altitude()
  magData = imu.read_magnetometer_raw()
  #temp = imu.read_barometer_temperature_celcius() # Temp doesn't work

  return [tuple(accelData), tuple(gyroData), barData, tuple(magData)]

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

def thrust2us(thrust):
  # get value between 0 and 100 representing motor thrust
  # thrust = 0 -> 1000 us
  # thrust = 100 -> 2000 us
  if thrust < 0:
    thrust = 0
  if thrust > 100:
    thrust = 100

  return thrust * 10 + 1000

def thrust2ns(thrust):
  return thrust2us(thrust) * 1000

def us2ns(us):
  return us * 1000

def ns2us(ns):
  return ns / 1000

def s2us(s):
  return s * 1000000

def us2s(us):
  return us / 1000000

def angle2thrust(angle):
  # angle in rads!!

  angleRatio = angle / deg2rad(max_control_angle)
  return angleRatio * 100

def gyro2thrust(rate):
  # rate in rads/s
  resultingAngle = rate * CONTROL_PERIOD

  angleRatio = resultingAngle / deg2rad(max_control_angle)
  return rateRatio * 100

led = Pin(25, Pin.OUT)
def blinkOnboardLed():
  led.on()
  time.sleep(.02)
  led.off()

# Define some constants

THROTTLE_TEST_ENABLED = False
CONTROL_FREQUENCY = 50 # Hz
CONTROL_PERIOD = 1/CONTROL_FREQUENCY # seconds
CONTROL_PERIOD_US = s2us(CONTROL_PERIOD) # microseconds
THRUST_GOVERNANCE = 80 # percent, so that control thrust has room to maneuver 
THRUST_FLOOR = 5 # percent, minimum thrust value
CONTROL_THRUST_OFFSET = 5 # Arbitrary value, divide control thrust values by this to dilute control thrust
max_control_angle = 60
thrust_min_pulse = 1000
pwm_freq = 50

# Define pins
MOTOR_FRONT_LEFT_PIN = 18
MOTOR_FRONT_RIGHT_PIN = 19
MOTOR_BACK_LEFT_PIN = 20
MOTOR_BACK_RIGHT_PIN = 21

IMU_SCL_PIN = 17
IMU_SDA_PIN = 16

UART_RX_PIN = 1
UART_TX_PIN = 0

"""
BEGIN STARTUP SEQUENCE
"""

print("Starting up...")

### pin Objects
motorFL_pin = Pin(MOTOR_FRONT_LEFT_PIN, Pin.OUT)
motorFR_pin = Pin(MOTOR_FRONT_RIGHT_PIN, Pin.OUT)
motorBL_pin = Pin(MOTOR_BACK_LEFT_PIN, Pin.OUT)
motorBR_pin = Pin(MOTOR_BACK_RIGHT_PIN, Pin.OUT)

imu_sda = Pin(IMU_SDA_PIN)
imu_scl = Pin(IMU_SCL_PIN)

uart_rx = Pin(UART_RX_PIN)
uart_tx = Pin(UART_TX_PIN)

# Define PWM objects
motorFL_pwm = PWM(motorFL_pin)
motorFR_pwm = PWM(motorFR_pin)
motorBL_pwm = PWM(motorBL_pin)
motorBR_pwm = PWM(motorBR_pin)

# I2C
i2c_imu = I2C(0, scl=imu_scl, sda=imu_sda)

# UART
uart = UART(0, baudrate=115200, rx=uart_rx, tx=uart_tx)

# Define Sensors
print("Waking up sensors...")
imu = AltIMU10(i2c_imu)
imu._initialize_sensors()

# Set up Fusion
fuse = Fusion()

# Calibrate sensors+fusion
print("Calibrating sensors, hold them still...")
calibrateFusion()
calibrateBarometer()
calibrationData = calibrateSensors()

print("Sensors ready.")

# Initialize PID controllers

kpPr = 0.1
kiPr = 0.01
kdPr = 0.001

kpTh = 1
kiTh = 0.1
kdTh = 0.01

print("Initializing PI/PID controllers...")

#pitchPID = PI(kpPr, kiPr) # only control pitch and roll for now
pitchPID = PID(kpPr, kiPr, kdPr)
pitchPID.set_output_limits((-deg2rad(max_control_angle), deg2rad(max_control_angle)))

#rollPID = PI(kpPr, kiPr)
rollPID = PID(kpPr, kiPr, kdPr)
pitchPID.set_output_limits((-deg2rad(max_control_angle), deg2rad(max_control_angle)))

thrustPID = PID(kpTh, kiTh, kdTh)
thrustPID.set_output_limits((THRUST_FLOOR, THRUST_GOVERNANCE))

print("PID controllers initialized.")

# Startup motor PWM signals at minimum value

print("Starting up PWM, setting motors to minimum thrust...")

motorFL_pwm.freq(pwm_freq)
motorFL_pwm.duty_ns(thrust_min_pulse*1000)
motorFR_pwm.freq(pwm_freq)
motorFL_pwm.duty_ns(thrust_min_pulse*1000)
motorBL_pwm.freq(pwm_freq)
motorBL_pwm.duty_ns(thrust_min_pulse*1000)
motorBR_pwm.freq(pwm_freq)
motorBR_pwm.duty_ns(thrust_min_pulse*1000)

time.sleep(3)

if THROTTLE_TEST_ENABLED:

  # Test 20,40,60,80,100 percent motor thrusts

  thrusts = [20, 40, 60, 80, 99]

  for thrust in thrusts:
    print("Setting thrust to ", thrust, " percent.")
    motorFL_pwm.duty_ns(thrust2us(thrust)*1000)
    motorFR_pwm.duty_ns(thrust2us(thrust)*1000)
    motorBL_pwm.duty_ns(thrust2us(thrust)*1000)
    motorBR_pwm.duty_ns(thrust2us(thrust)*1000)
    time.sleep(1)

  motorFL_pwm.duty_ns(thrust_min_pulse*1000)
  motorFR_pwm.duty_ns(thrust_min_pulse*1000)
  motorBL_pwm.duty_ns(thrust_min_pulse*1000)
  motorBR_pwm.duty_ns(thrust_min_pulse*1000)

print("Startup sequence complete.")

# Set initial control values
gyroXcontrol = 0
gyroYcontrol = 0
gyroZcontrol = 0
Z_value = 0

pitchPID.set_setpoint(gyroXcontrol)
rollPID.set_setpoint(gyroYcontrol)

thrustPID.set_setpoint(Z_value)

# Separate gyro data from calibration data
gyroXnorm = calibrationData[2]
gyroYnorm = calibrationData[3]
gyroZnorm = calibrationData[4]

state = "FLY"

def ControlLoop():
  global state
  global gyroXcontrol
  global gyroYcontrol
  global gyroZcontrol
  global Z_value

  if state == "FLY":
    # Read sensors
    imuData = readIMU()
    height = imuData[2]

    time.sleep_us(150)
    #print("IMU Data: ", imuData)

    # correct gyro data
    if (gyroXnorm > 0):
      gyroX = imuData[1][0] - gyroXnorm
    else:
      gyroX = imuData[1][0] + gyroXnorm
    if (gyroYnorm > 0):
      gyroY = imuData[1][1] - gyroYnorm
    else:
      gyroY = imuData[1][1] + gyroYnorm
    if (gyroZnorm > 0):
      gyroZ = imuData[1][2] - gyroZnorm
    else:
      gyroZ = imuData[1][2] + gyroZnorm

    # Read UART for incoming control values
    # Takes ~ 3 ms
    if uart.any():
      incoming = uart.read()
      try:
        incoming = incoming.decode('utf-8')
      except:
        pass
      
      if incoming == "KILL":
        # Kill motors
        motorFL_pwm.duty_ns(thrust_min_pulse*1000)
        motorFR_pwm.duty_ns(thrust_min_pulse*1000)
        motorBL_pwm.duty_ns(thrust_min_pulse*1000)
        motorBR_pwm.duty_ns(thrust_min_pulse*1000)
        state = "DEAD"
      try:
        # split incoming string by commas, transition each string to int
        incoming = incoming.split(',')
        for i in range(len(incoming)):
          try:
            incoming[i] = float(incoming[i])
          except:
            incoming[i] = 0 # will really only trigger during standby msgs
        gyroXcontrol = incoming[0]
        gyroYcontrol = incoming[1]
        gyroZcontrol = incoming[2]
        Z_value = incoming[3]
      except:
        print("Error parsing incoming message.")
    else:
      pass

    # Set PID setpoints
    pitchPID.set_setpoint(gyroXcontrol)
    rollPID.set_setpoint(gyroYcontrol)

    thrustPID.set_setpoint(Z_value)

    # Need to update Z value - how to do this?

    # Control with gyro data
    del_phi = pitchPID.update(gyroX)
    del_theta = rollPID.update(gyroY)

    del_thrust = thrustPID.update(height)

    print("Control Signals: ", del_phi, del_theta, del_thrust)

    #print("Control Signals: ", del_phi, del_theta)

    # # calculate control moments
    # Lc = Ixx * del_phi
    # Mc = Iyy * del_theta
    # Nc = 0
    
    # # calculate motor thrusts
    # controlThrustFr = (-r*Lc)+ (r*Mc) + Nc
    # controlThrustFl = (r*Lc) + (r*Mc) - Nc
    # controlThrustBr = (-r*Lc) - (r*Mc) - Nc
    # controlThrustBl = (r*Lc) - (r*Mc) + Nc

    ### SUPER DUMB CONTROL SCHEME
    # del_phi is an angle value, take it as a percentage of the max control angle and set motor thrusts accordingly
    # if >>= control angle, thrust at 100, if << control angle, thrust at 0
    # Break into 4 quadrants, if del_phi > 0, increase thrust on front motors, decrease thrust on back motors
    # if del_phi < 0, increase thrust on back motors, decrease thrust on front motors
    # same for del_theta but for left and right motors

    try :
      pitchControl = angle2thrust(abs(del_phi))
      rollControl = angle2thrust(abs(del_theta))
    except:
      pitchControl = 0 # ERROR HANDLING, ideally never get here
      rollControl = 0

    time.sleep_us(100)
    try:
      if del_phi > 0:
        pcFL = pitchControl/2
        pcBL = pitchControl/2
        pcFR = 0
        pcBR = 0
      if del_phi < 0:
        pcFL = 0
        pcBL = 0
        pcFR = pitchControl/2
        pcBR = pitchControl/2
      if del_theta > 0:
        rcFL = rollControl/2
        rcBL = 0
        rcFR = rollControl/2
        rcBR = 0
      if del_theta < 0:
        rcFL = 0
        rcBL = rollControl/2
        rcFR = 0
        rcBR = rollControl/2

    except:
      pcFL = 0
      pcBL = 0
      pcFR = 0
      pcBR = 0
      rcFL = 0
      rcBL = 0
      rcFR = 0
      rcBR = 0

    # Realistically these will never equal 0

    controlThrustFr = (pcFR + rcFR)/CONTROL_THRUST_OFFSET
    controlThrustFl = (pcFL + rcFL)/CONTROL_THRUST_OFFSET
    controlThrustBr = (pcBR + rcBR)/CONTROL_THRUST_OFFSET
    controlThrustBl = (pcBL + rcBL)/CONTROL_THRUST_OFFSET

    try:
      thrustFL = controlThrustFl + del_thrust
      thrustFR = controlThrustFr + del_thrust
      thrustBL = controlThrustBl + del_thrust
      thrustBR = controlThrustBr + del_thrust
    except:
      thrustFL = controlThrustFl + THRUST_FLOOR
      thrustFR = controlThrustFr + THRUST_FLOOR
      thrustBL = controlThrustBl + THRUST_FLOOR
      thrustBR = controlThrustBr + THRUST_FLOOR
  
    # Set motor thrusts
    motorFL_pwm.duty_ns(int(thrust2ns(thrustFL)))
    motorFR_pwm.duty_ns(int(thrust2ns(thrustFR)))
    motorBL_pwm.duty_ns(int(thrust2ns(thrustBL)))
    motorBR_pwm.duty_ns(int(thrust2ns(thrustBR)))

  else:
    motorFL_pwm.duty_ns(thrust_min_pulse*1000)
    motorFR_pwm.duty_ns(thrust_min_pulse*1000)
    motorBL_pwm.duty_ns(thrust_min_pulse*1000)
    motorBR_pwm.duty_ns(thrust_min_pulse*1000)
    blinkOnboardLed()

    # Read UART for incoming control values
    if uart.any():
      incoming = uart.read()
      #print("Incoming: ", incoming)
      try:
        incoming = incoming.decode('utf-8')
      except:
        pass

      if incoming == "WAKE":
        print("Waking up...")
        state = "FLY"

# Main Loop
while True:
  start_time = time.ticks_us()
  
  ControlLoop()

  end_time = time.ticks_us()
  loop_time_us = time.ticks_diff(end_time, start_time)

  loopTimeEst = 100

  # Sleep for the remainder of the control period
  if loop_time_us < CONTROL_PERIOD_US:
    time.sleep_us(int(CONTROL_PERIOD_US - loop_time_us - loopTimeEst)) # subtract 50 us to account for loop time

# END OF MAIN LOOP