# N2-flight-software
## Introduction
This repository contains code used for flight software for Nakuja Rocket N2. The following description provides a detailed
description of program flow, project structure, library dependencies and/or any other necessary documentation needed to run this 
code successfully.

## Project structure
***

```asm
├───cmake-build-debug
├───include
│   └───defs.h
│   └───functions.h
│   └───global_variables.h
│   └───README.md
├───lib
├───main
│   └───ground-station
          └───ground-station.ino
├───src
   └───main.cpp
└───test
```

### 1. Folders Description
| Folder  | Description   |
|---|---|
|  include | Contains header files   |
|  lib | Project libraries|
|main | Contains the code for ground station micro-controller   |
|src| Contains main.cpp file the is run by the flight computer   |
|test| Contains unit test files    |

### 2. Files Description
|File   | Description   |
|---|---|
| defs.h  | Contains definitions of global constants |
|functions.h   | Functions declaration and definitions |
| global_variables,h   | Declarations of al global variables used in the program   |
| ground-station.ino    | Contains code that runs on the ground station micro-controller   |
|main.cpp   | Main program uploaded to the avionics computer. It cantains the flight logic in terms of state machines. The corresponding functions are called depending on the current state of the rocket   |


## Flight Logic 
The flight program is structured as a state machine with the following states:
| State  | Description  | Waiting for event |
|---|---| -----|
| 0  | Pre-launch Ground    |  20m displacement above ground  |
| 1  | Coasting      |  Zero Velocity |
| 2  | Apogee (Parachute Deployment)    |  20m displacement below apogee  |
| 3  | Descent|  20m displacement above ground |
| 4  | Post-flight Ground    |  No more events  |


