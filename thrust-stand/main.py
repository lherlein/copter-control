"""
MicroPython PI/PID flight controller for custom drones.

Currently: use known Ixx, Iyy, Izz values for the drone
Goal: Have the ability to calculate Ixx, Iyy, Izz using bang-bang method
  - bang-bang to 15 degree roll about each axis, measure angular rate, time taken, and torque
  - calculate Ixx, Iyy, Izz

Following pseudo code defined in ./fc-pseudo.py
"""

from machine import Pin, I2C, PWM, UART
import time
import math

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
THRUST_OUT_PIN = 26

"""
BEGIN STARTUP SEQUENCE
"""

print("Initializing..., plug in battery now.")

### Output pins
motor_pin = Pin(THRUST_OUT_PIN, Pin.OUT)

# Define PWM objects
motor_pwm = PWM(motor_pin)

motor_pwm.freq(50)
motor_pwm.duty_ns(1000*1000)

time.sleep(10)
print("Starting Test")

"""
TEST
"""

# Want to test 20%, 40%, 60%, 80%, 100% power, each for 5 seconds
for i in range(5):
  print("Testing ", (i+1)*20, "% power, Pulse Width: ", 1000 + (i+1)*200)
  duty = 1000 + (i+1)*200
  motor_pwm.duty_ns(duty*1000)
  time.sleep(5)
  print(motor_pwm.duty_ns()/1000)
  
print("Test Complete")
motor_pwm.duty_ns(1000*1000)