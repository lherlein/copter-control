from machine import Pin, I2C, PWM, UART
from lib.altimu import AltIMU10
import time
import math

def readIMU():
  # Get IMU Data
  accelData = imu.read_accelerometer_raw()
  gyroData = imu.read_gyroscope_raw()
  barData = imu.read_barometer_raw()
  magData = imu.read_magnetometer_raw()

  return [accelData, gyroData, barData, magData]

# Set up I2C connection
i2c_imu = I2C(0, scl=Pin(17), sda=Pin(16))

# Set up IMU

imu = AltIMU10(i2c_imu)

# Initialize sensors
imu._initialize_sensors()


while True:
  imuData = readIMU()
  print(imuData)
  time.sleep_ms(500)