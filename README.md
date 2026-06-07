# 2D Ball Balancer

A nonlinear coupled 2D ball balancing system actuated by two orthogonal planar robotic arms and controlled using a PID-based numerical feedback linearization approach.

## Project Overview

This project presents the mathematical modeling, simulation, control design, and visualization of a nonlinear 2D ball balancing system. The system consists of a ball moving on a tilting plate, where the plate orientation is generated through two planar robotic arm mechanisms.

The PID controller generates desired ball accelerations, which are then converted into physically realizable plate and actuator commands through a numerical inverse mapping layer and robotic arm kinematics.

## Key Features

- Nonlinear coupled ball-on-plate dynamics
- PID-based trajectory tracking control
- Numerical feedback linearization using nonlinear root-finding
- Inverse kinematics of 2R planar robotic arm actuation
- Forward kinematics using homogeneous transformation matrices
- Servo actuator dynamics and velocity rate limiting
- Time-domain tracking analysis
- Empirical frequency-domain characterization
- MATLAB/Simulink implementation
- 3D animation of the system response
- Full technical report


## Repository Structure

```text
matlab/     MATLAB source codes and simulation files
report/     Final project report
videos/     Demonstration and animation videos
images/     Repository visuals and preview images
```

## Status

Project completed. Final source files, report, and demonstration videos are being organized and uploaded.

## Author

Ege Özbülbül  
Mechanical Engineering, Bilkent University