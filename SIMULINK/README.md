# 📊 Simulink Model

This folder contains the nonlinear Simulink model of the 2D Ball Balancer system **without PID control**. The model simulates the system's natural behavior under small initial displacements.

## Files Included
- `eomSimplificationTest.slx`: Nonlinear Simulink model with lumped system parameters currently set as placeholders
- `eomParameters.sldd`: Simulink Data Dictionary containing all relevant lumped parameters for easy modification

## Notes
- No controllers are included; the system evolves naturally from its initial conditions
- Once the lumped parameters are defined, each term in the equations of motion will be individually disabled to observe its effect on system behavior through the scopes of \( x(t) \) and \( \phi(t) \)
- Terms that have minimal impact will be disregarded in the final model, resulting in a simplified representation of the system

