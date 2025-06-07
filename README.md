# 2D Ball Balancer

An Arduino Mega-based two-axis ball balancing system with a PID controller

## 🔧 Components Used
- Arduino Mega 2560
- 16 IR LEDs and TSOP receivers (8 per axis)
- 2 high-speed servo motors
- Rack and pinion mechanical structure
- Voltage regulator module
- External power supplies
- Breadboards, jumper wires
- Capacitors (1000µF, 220µF, 100nF)
- Resistors (1kΩ, 2.2kΩ, etc.)
- Transistor

> ⚠️ For the full circuit and connections, refer to the Fritzing file inside the `Schematics/` folder.

## 📂 Folder Structure
- `/CAD` → 3D mechanical design files
- `/Schematics` → Circuit diagrams
- `/Code` → Arduino code 
- `/Code/Test-Codes` → Experimental test sketches with descriptions
- `/Media` → Photos and videos of the project


## 🧪 Test Codes
All test sketches used during development are located in `Code/Test-Codes/`.  
Each `.ino` file includes a detailed comment block describing:

- The purpose of the test  
- Pin configurations  
- Observations or limitations  

These files document the iterative process that led to the final implementation.

## 🎯 Project Goal
This system uses IR sensor arrays to detect ball position and applies PID control to move a platform via two servos. The aim is to keep the ball centered by adjusting the platform's tilt on both X and Y axes.

## 👤 Author
**Ege Özbülbül**  
Bilkent University – Department of Mechanical Engineering
