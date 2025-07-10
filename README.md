# A Comparative Control Study on a Optical Grid-Based 2D Ball Balancer

An Arduino Mega-based two-axis ball balancing system with a PID controller.

## 🔧 Components Used
- Arduino Mega 2560  
- 16 IR LEDs and TSOP receivers (8 per axis)  
- 2 high-speed servo motors  
- Rack and pinion mechanical structure  
- Voltage regulator module  
- External power supplies  
- Breadboards, jumper wires  
- Capacitors (1000 µF, 220 µF, 100 nF)  
- Resistors (1 kΩ, 2.2 kΩ, etc.)  
- Transistor  

> ⚠️ For the full circuit and connections, refer to the Fritzing file inside the `Schematics/` folder.

---

## 📂 Folder Structure
- `/CAD` → 3D mechanical design files  
- `/Schematics` → Circuit diagrams  
- `/Code`  
  - `Test-Codes/` → Arduino test sketches with descriptions  
  - `Graphic Codes/` → Live plotting for sensor feedback  
  - `Dynamic Analysis Codes/` → Symbolic and numeric MATLAB scripts for modeling and root-finding  
- `/Dynamic Analysis` → Final PDF report of dynamic model  
- `/SIMULINK` → Simulink model, Data Dictionary, and model-specific scripts  
- `/Media`   
  - `feasibilityMappingGraph/` → Final scatter plot of root regions (`finalMap.fig`)

---

## 🧪 Test Codes
All development tests are documented in `Code/Test-Codes/`.  
Each test includes:
- Purpose of the test  
- Wiring/pin setup  
- Notes on behavior or limitations  

These document the iterative prototyping process.

---

## 📊 Dynamic Modeling and Analysis

All scripts are located under `Code/Dynamic Analysis Codes/`.  
Each file performs a specific analysis task. For most files, two versions exist:
- Full version (using complete nonlinear equations of motion)  
- `Simplified` version (uses reduced EOM for faster computations)

### MATLAB Scripts Overview:

- `inhull.m`, `polyfitn.m`, `polyvaln.m`:  
  External helper functions used for polynomial fitting and region testing.

- `eomGraph.m / eomGraphSimplified.m`:  
  For a given set of `x`, `xdot`, `xdotdot`, and `phidot`, computes the roots of the EOM and plots them. Useful for visually inspecting root behavior and servo limitations.

- `eomSimplification.m`:  
  Uses the Simulink model's switching logic to isolate and evaluate the impact of each EOM term on system behavior. Outputs include relative error, standard deviation, and comparison plots.

- `eomSymbolicDerivation.m`:  
  Automatically derives the symbolic left-hand side of the Euler-Lagrange-based EOM.

- `feasibilityMapping.m / feasibilityMappingSimplified.m`:  
  Given ranges of `x`, `xdot`, `xdotdot`, and `phidot`, this script:
  - Checks if roots exist for each combination  
  - Labels each point as:
    - `1` → root exists and is within servo limits  
    - `0` → no root  
    - `-1` → root exists but outside servo limits  
  - Visualizes scatter plot for a selected `x`  
  - Aggregates all valid combinations across `x` values  
  - Intersects all valid sets to form a final safe region  
  - Outputs a 3D scatter plot of feasible regions and saves it as `finalMap.fig` inside `Media/feasibilityMappingGraph/`

- `contourFitting.m / contourFittingSimplified.m`:  
  Takes the final feasible region and fits a polynomial surface (`phidot = f(xdot, xdotdot)`), providing:
  - Fitted equation  
  - R² score to evaluate goodness of fit

- `parameterBalancing.m`:  
  Minimizes stiffness by tuning lumped parameters (mass, inertia) using a variance-minimizing algorithm. Outputs a new balanced parameter set to improve robustness of system dynamics.

---

## 🧩 Simulink Model
The `SIMULINK/` folder contains the nonlinear model used for simulation and controller design.

- `eomIntegratorChain.slx`: Simulink model using switch logic to enable/disable EOM terms  
- `eomParameters.sldd`: External Simulink Data Dictionary containing all system parameters
- `ClosedLoopSystemAcc.slx`: Simulink model using PID control signal as acceleration
- `ClosedLoopSystemAccVar1.slx`: Variation of the `ClosedLoopSystemAcc.slx` for optimized block diagrams and algorithms
 
- All parameter scaling and term analysis are designed for easy integration and switching  

---

## 🎯 Project Goal
This system uses IR sensor arrays to detect ball position and applies PID control to move a platform via two servos.  
The goal is to keep the ball centered by adjusting the tilt of a square platform on both X and Y axes with minimal overshoot and fast response.

---

## 👤 Author
**Ege Özbülbül**  
Bilkent University – Department of Mechanical Engineering
