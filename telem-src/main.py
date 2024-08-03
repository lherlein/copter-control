from machine import Pin, I2C, PWM, UART
from lib.mpu6050 import MPU6050
import time
import math
import machine
import socket
import network
import urequests
import ujson # For parsing incoming UDP messages

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

def calcAngles(imuData): # Not entirely sure this will work when in flight

  # Get Accelerometer Data
  accelData = imuData[0]

  phi = math.atan2(accelData[1], accelData[2])
  theta = math.atan2(accelData[0], accelData[2])

  return [phi, theta]

def do_connect():
  wlan = network.WLAN(network.STA_IF)
  wlan.active(True)
  if not wlan.isconnected():
    print('connecting to network...')
    wlan.connect('sofia', '19631964')
    while not wlan.isconnected():
      pass
  # return ip address
  return wlan.ifconfig()[0]

def blinkOnboardLed():
  led.on()
  time.sleep(.2)
  led.off()

def formatNumber(value):
    # Format the number to fit within 6 characters including the decimal point
    formatted_value = "{:6.2f}".format(value)
    # Ensure the string is exactly 6 characters long
    if len(formatted_value) > 6:
        formatted_value = formatted_value[:6]
    return formatted_value

# Define Pins

rx_pin = Pin(1)
tx_pin = Pin(0)
led = Pin("LED", Pin.OUT)

uart = UART(0, baudrate=115200, rx=rx_pin, tx=tx_pin)

# Define I2C Busses
#i2c_mpu = I2C(1, sda=mpuSDA, scl=mpuSCL)

# Define Sensors
#mpu = MPU6050(i2c_mpu)

# Calibrate Everything
#normalValues = calibrateSensors()

# Wake up MPU
#mpu.wake()

# Check if connected to wifi

# If not, connect to wifi

# Establish UDP Client

ip = do_connect()
print(ip)

UDP_IP = ip
UDP_PORT = 5005

# Get addr info
addr_info = socket.getaddrinfo(ip, UDP_PORT)
addr = addr_info[0][-1]

# Establish UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(addr)

# Define STATE
STATE = "STANDBY"

# Main Loop
while True:
  # Read IMU Data
  # Calculate Angles

  imuData = readIMU()
  
  # Craft UDP Data message
  data_package = {
    "type": "IMU_DATA",
    "payload": {
      "accel": imuData[0],
      "gyro": imuData[1]
    }
  }

  # Send UDP Data message
  sock.sendto(ujson.dumps(data_package), addr)

  # Check for incoming message
  data, addr = sock.recvfrom(1024)
  if data:
    blinkOnboardLed()
    try:
      data = data.decode('utf-8')
      print(data)
    except:
      print(data)
      pass
    
    # Parse incoming UDP message
    try:
      udpMessage = ujson.loads(data)
    except:
      print("Error parsing UDP message")
      pass # will handle later

    event_type = udpMessage["type"]
    event_payload = udpMessage["payload"]

    # THREE POSSIBLE INCOMING UDP EVENTS: CHANGE_STATE, UPDATE_CONTROL, KILL
    # Data structured as: {type: ..., payload: ...}

    if event_type == "KILL":
      # Tell FC to kill motors, send kill over uart 5 times
      for i in range(5):
        uart.write("KILL")
        time.sleep_ms(10)
    
    elif event_type == "CHANGE_STATE":
      # Change state to event_payload
      payload_data = ujson.loads(event_payload)

      # payload = {"state": "STATE"}

      state = payload_data["state"]
      STATE = state

    elif event_type == "UPDATE_CONTROL":
      # ONLY PROCESS IF IN FLY STATE
      if STATE == "FLY":
        # Parse control data
        payload_data = ujson.loads(event_payload)

        # payload = {"gyroX": number, "gyroY": number, "gyroZ": number, "Z": number}

        gyroX = payload_data["gyroX"]
        gyroY = payload_data["gyroY"]
        gyroZ = payload_data["gyroZ"]
        Z = payload_data["Z"]
        # Send control data to FC
        uart.write("{},{},{},{}".format(formatNumber(gyroX), formatNumber(gyroY), formatNumber(gyroZ), formatNumber(Z)))
        # the above ensures that the data sent will never exceed 6 characters per value, including the decimal point.
        # this is because the FC reads a maximum of 30 bytes per line, and each value is 6 bytes long -> with commas, that maxes at 28 bytes
  else:
    # Do nothing
    pass
    
  # Force control values based on state - if in standby drive everything to zero
  if STATE == "STANDBY":
    uart.write("0,0,0,0")
  elif STATE == "FLY":
    # Do nothing
    pass
  else:
    # Do nothing
    pass