# QuadCopter Flight Controller Design

This repository contains all pertinent work for my custom quadcopter Flight Controller (FC). Drone FC's are an amalgamation of multiple components and concepts, and requires multiple systems to be linked together. The design capturing the interlinking of these systems will be captured here as work progresses.

This code is run on-board a drone on your network. To communicate with the drone(s), run the ground station software found here on my github: https://github.com/lherlein/ts-ground-station .

## Table of Contents

- [QuadCopter Flight Controller Design](#quadcopter-flight-controller-design)
  - [Table of Contents](#table-of-contents)
  - [Flight Controller Components](#flight-controller-components)
  - [Hardware/Software List](#hardwaresoftware-list)
  - [Overall Design](#overall-design)
  - [Drone State Machine](#drone-state-machine)
    - [Env Setup](#env-setup)

## Flight Controller Components

- [Telemetry gathering](./telem-src/README.md)
  - IMU
  - GPS
- [Control System](./control-src/README.md)
  - PID Control
    - Pitch/Roll
    - Thrust/Height
    - Yaw
  - IMU
- Control Interpretation
  - Semi-autonomous
    - Give control setpoints over time to control the drone

## Hardware/Software List

__Note__: All choices subject to change

- Telemetry
  - IMU: [MPU 6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
  - GPS: [Goouuu Tech GT-U7](https://www.amazon.com/Navigation-Satellite-Compatible-Microcontroller-Geekstory/dp/B07PRGBLX7)
  - Microprocessor: [Seeed Xiao RP2040](https://www.seeedstudio.com/XIAO-RP2040-v1-0-p-5026.html)
  - Language: [MicroPython](https://micropython.org/)
- Control System
  - Microprocessor: [RPi Pico Board](https://www.raspberrypi.com/products/raspberry-pi-pico/)
  - Language: [MicroPython](https://micropython.org/)

Micropython and Thonny have been chosen in the short term for ease of setup and use. It is incredibly easy to begin testing and understanding equipment with these tools. Over time, I hope to transition to C and cmake.

## Overall Design

<img src="./diagrams/drone-design.svg" alt="Drone Design Block Diagram" width="800">

## Drone State Machine

Each drone, at a high level, is an event-driven State Machine. The two high level states are "flying" and "landed", and the drone transitions between the two based on UDP messaging. Here is the entire state machine:

```mermaid
stateDiagram-v2
  Standby --> LISTENING
  LISTENING --> Standby
  LISTENING --> Fly: CHANGE_STATE - fly
  Fly --> eventHandler: UDP
  eventHandler --> Fly: UPDATE_CONTROL
  eventHandler --> Standby: CHANGE_STATE - standby
  eventHandler --> Dead: KILL
```

### Env Setup

```
source quad/bin/activate
```

```
deactivate
```