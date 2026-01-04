# 4610EPushBack 2025-26
Team 4610E 2025-26 Robot Revolution — Push Back Robot Codebase

## Overview
This repository contains the source code, header files, and build configuration for Team 4610E’s robot competing in the VEX V5 Robotics Competition: Push Back season.

The project is organized for clean development, maintainability, and fast iteration during the season.


## Repository Structure
├ 4610E Comp Temp Main/ # The Competition Template containing the main Drive Code and the following autonomous routines: Blue Left, Red Left, Blue Right, and Red Right
|
├ 4610E Autons # The minor autonomous routines, such as Skills and Solo AWP

##Subfolder General Structure
├ include/ # Header files (IDs, constants, helper utilities)  
|
├ **src/ # Main C++ source**
|
├ vex/ # VEX project configuration and metadata  
|
├ makefile # Command-line build script  
|
├ .vscode/ # Optional VS Code configuration  
|
└ .gitignore # Ignored files  

## Features

* Drivetrain Control: P-based six-motor drivetrain with proportional forward/reverse, precise turning using the inertial sensor, and internal conversion between units.

* Intake System: Two-stage intake controlled via controller buttons, with forward/reverse and toggle functionality.

* Mechanisms: Wings, adjuster, and tongue actuators toggled independently using controller buttons.

 Autonomous Routines: Modular selectable routines via Brain touchscreen: Blue Left, Red Left, Blue Right, Red Right, Skills Auton, Solo AWP, and debug routines.

 Inertial Sensor Integration: Calibrated pre-match for accurate turning in autonomous.

* Driver Control: Smooth, calibrated drive control with threaded managers for subsystems.

* Screen Feedback: Motor positions, autonomous selection, and debug info displayed on Brain screen.

* Modular Architecture: Threads for drive, intake, mechanisms, and logging for efficient asynchronous control.


## Getting Started

### 1. Clone the Repository
```
bash
git clone https://github.com/Dragonix22/4610EPushBack.git
cd 4610EPushBack
```

### 2. Set Up Environment
Install the VEX V5 toolchain or use VEXcode Pro V5

Recommended editor: VS Code with C++ extensions

### 3. Build the Project
```
bash
make
```
### 4. Deploy to the Robot
Connect a VEX V5 brain



📄 License
Internal project for Team 4610E.
Contact the team for reuse or forking permissions.
