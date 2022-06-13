# N2-flight-software
## Introduction
This repository contains code used for flight software for Nakuja Rocket N2. The following description provides a detailed
description of program flow, project structure, library dependencies and/or any other necessary documentation needed to run this 
code successfully.

## Project structure
***

```asm
├── data
│   └── flightdata.txt
├── include
│   ├── checkState.h
│   ├── defs.h
│   ├── functions.h
│   ├── kalmanfilter.h
│   ├── logdata.h
│   ├── readsensors.h
│   └── transmitwifi.h
├── lib
│   └── README
├── LICENSE
├── platformio.ini
├── README.md
├── src
│   └── main.cpp
└── test
    └── README
```

### 1. Folders Description
| Folder  | Description   |
|---|---|
|  include | Contains header files   |
|  lib | Project libraries|
|src| Contains main.cpp source file run by the flight computer   |
|test| Contains unit test files    |

### 2. Files Description
| File | Description |
| ---------| ----------- |
|defs.h  | Contains constants declaration |
|functions.h| Contains general utility functions and prototype function declarations |
|logdata.h | code for SD card mounting and logging |
|checkState.h| Contains state machine logic |
|kalmanfilter.h | Contains kalman filter for accelerometer values |
|readsensors.h | Contains code for sensors on board including the MPU,BMP,GPS |
|transmitwifi.h | Contains WiFi set-up and MQTT code |


## Flight Logic 
The flight program is structured as a state machine with the following states:
| State  | Description  | Waiting for event |
|---|---| -----|
| 0  | Pre-launch Ground    |  20m displacement above ground  |
| 1  | Coasting      |  Zero Velocity |
| 2  | Apogee (Parachute Deployment)    |  20m displacement below apogee  |
| 3  | Descent|  20m displacement above ground |
| 4  | Post-flight Ground    |  No more events  |


