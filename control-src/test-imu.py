from machine import Pin, I2C, PWM, UART
from lib.altimu import AltIMU10
from lib.fusion import Fusion
import time
import math
import uasyncio

def readIMU():
  # Get IMU Data
  accelData = imu.read_accelerometer_g_forces()
  gyroData = imu.read_gyroscope_angular_velocity()
  barData = imu.read_barometer_altitude()
  magData = imu.read_magnetometer_raw()
  #temp = imu.read_barometer_temperature_celcius() # Temp doesn't work

  return [tuple(accelData), tuple(gyroData), barData, tuple(magData)]

def getMagData():
  magData = imu.read_magnetometer_raw()
  return tuple(magData)

def CalibrateStopFunction():
  uasyncio.sleep(5)
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
  fuse.calibrate(getMagData, CalibrateStopFunction)
  print("Fuse Calibrated")

# Set up Fusion

fuse = Fusion()

# Set up I2C connection
i2c_imu = I2C(0, scl=Pin(17), sda=Pin(16))

# Set up IMU
imu = AltIMU10(i2c_imu)

# Initialize sensors
imu._initialize_sensors()

# Calibrate sensors
calibrateFusion()
calibrateBarometer()

while True:
  imuData = readIMU()
  fuse.update(imuData[0], imuData[1], imuData[3])
  print(imuData)
  print(fuse.pitch, fuse.roll, fuse.heading)