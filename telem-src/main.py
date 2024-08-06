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
  time.sleep(.02)
  led.off()

def formatNumber(value):
    # Format the number to fit within 6 characters including the decimal point
    formatted_value = "{:6.2f}".format(value)
    # Ensure the string is exactly 6 characters long
    if len(formatted_value) > 6:
        formatted_value = formatted_value[:6]
    return formatted_value

def establishUdpConnection(ip, UDP_PORT):
  try:
    # Get addr info
    addr_info = socket.getaddrinfo(ip, UDP_PORT)
    addr = addr_info[0][-1]

    # Establish UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(0)
    sock.bind(addr)
  except:
    time.sleep(.5)
    establishUdpConnection(ip, UDP_PORT)

  return sock, addr

def s2us(s):
  return s * 1000000

def us2s(us):
  return us / 1000000

# Define Pins

TELEMETRY_FREQUENCY = 100 # Hz, 10x the control frequency
TELEMETRY_PERIOD = 1/TELEMETRY_FREQUENCY
TELEMETRY_PERIOD_US = s2us(TELEMETRY_PERIOD)

IMU_SCL_PIN = 17
IMU_SDA_PIN = 16

# Machine functions
rx_pin = Pin(1)
tx_pin = Pin(0)
led = Pin("LED", Pin.OUT)

# Define I2C Pins
mpuSDA = Pin(IMU_SDA_PIN)
mpuSCL = Pin(IMU_SCL_PIN)

uart = UART(0, baudrate=115200, rx=rx_pin, tx=tx_pin)
i2c_mpu = I2C(0, sda=mpuSDA, scl=mpuSCL)

# Define Sensors
mpu = MPU6050(i2c_mpu)

# Calibrate Everything
normalValues = calibrateSensors()

# Wake up MPU
mpu.wake()

# Connect to WiFi and get IP
ip = do_connect()
print(ip)

UDP_IP = ip
UDP_PORT = 5005

# Establish UDP Connection
print("Establishing UDP Connection")
sock, addr = establishUdpConnection(UDP_IP, UDP_PORT)

print("UDP Connection Established")

# Define STATE
STATE = "STANDBY"

# isolate normal gyro values
gyroXnorm = normalValues[2]
gyroYnorm = normalValues[3]
gyroZnorm = normalValues[4]

print("Entering Main Loop")

def TelemetryLoop():
  global count, STATE
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
  if count % 10 == 0:
    sock.sendto(ujson.dumps(data_package), addr)
    count = 0

  # Check for incoming message
  try:
    data, addrRes = sock.recvfrom(1024)
  except:
    data = None
    pass
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

    # THREE POSSIBLE INCOMING UDP EVENTS: STATE_CHANGE, UPDATE_CONTROL, KILL
    # Data structured as: {type: ..., payload: ...}

    if event_type == "KILL":
      # Tell FC to kill motors, send kill over uart 5 times
      for i in range(5):
        uart.write("KILL")
        time.sleep_ms(10)
        STATE = "DEAD"
    
    elif event_type == "STATE_CHANGE":
      # Change state to event_payload
      #print(event_payload, type(event_payload))
      #payload_data = ujson.loads(event_payload)

      # payload = {"state": "STATE"}

      #state = payload_data["state"]
      state = event_payload["state"]
      STATE = state

      if state == "FLY":
        for i in range(5):
          uart.write("WAKE")
          time.sleep_ms(10)

    elif event_type == "UPDATE_CONTROL":
      # ONLY PROCESS IF IN FLY STATE
      if STATE == "FLY":
        # Parse control data
        #payload_data = ujson.loads(event_payload)
        payload_data = event_payload

        # payload = {"gyroX": number, "gyroY": number, "gyroZ": number, "Z": number}

        gyroX = payload_data["gyroX"]
        gyroY = payload_data["gyroY"]
        gyroZ = payload_data["gyroZ"]
        Z = payload_data["Z"]
        # Send control data to FC
        uart.write("{},{},{},{}".format(gyroX, gyroY, gyroZ, Z))
        # the above ensures that the data sent will never exceed 6 characters per value, including the decimal point.
        # this is because the FC reads a maximum of 30 bytes per line, and each value is 6 bytes long -> with commas, that maxes at 28 bytes
  else:
    # Do nothing
    pass
    
  # Force control values based on state - if in standby drive everything to zero
  print(STATE)
  if STATE == "STANDBY":
    uart.write("0.0000,0.0000,0.0000,0.0000,")
  elif STATE == "FLY":
    # Do nothing
    pass
  elif STATE == "DEAD":
    # Do nothing -> a control message wakes the FC up
    pass
  else:
    # Do nothing
    pass

count = 1
# Main Loop
while True:
  start_time = time.ticks_us()

  TelemetryLoop()
  count += 1

  end_time = time.ticks_us()
  loop_time = time.ticks_diff(end_time, start_time)

  loopTimeEst = 100

  # Sleep for the remainder of the telem period
  if loop_time < TELEMETRY_PERIOD_US:
    time.sleep_us(int(TELEMETRY_PERIOD_US - loop_time - loopTimeEst))