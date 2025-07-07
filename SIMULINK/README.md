# 📊 Simulink Models – 2D Ball Balancer Project

This folder contains **nonlinear Simulink models** used throughout the development of the 2D Ball Balancer system, including both **open-loop** and **closed-loop** dynamics.

---

## 🌱 1. `eomIntegratorChain.slx`

**Description:**  
This model represents the full nonlinear equations of motion (EOM) of the system with a **chain of integrators**. The `x`, `xdot`, `phi`, and `phidot` values are passed into the EOM blocks to compute accelerations \( \ddot{x} \) and \( \ddot{\phi} \), which are then integrated to regenerate position and velocity.

**Purpose:**
- Designed to observe the **free response** of the system with no external control.
- Every term in the EOM is placed behind a `Switch` block to allow **selective term elimination** for simplification studies.

---

## 🔁 4. `ClosedLoopSystemAcc.slx`

**Description:**  
Implements a **discrete-time PID controller** that outputs a desired acceleration \( \ddot{x}_{PID} \), which is then used to determine the required servo angle \( \phi \) via **inverse dynamics**.
**Structure:**
- PID output is interpreted as a desired \( \ddot{x} \).
- Inverse dynamics block computes corresponding \( \phi \).
- \( \phi \) is sent to the Plant model.
- System is simulated with **discrete steps (50 ms)** matching Arduino control logic.

---

## 🧪 5. `ClosedLoopSystemAccVar1.slx`

**Description:**  
A **variant** of `ClosedLoopSystemAcc` with **modifications in the block structure and clamping logic**.

**Differences:**
- Alternative layout for clarity.
- Implements a different version of the clamping algorithm that constrains \( \phi \) based on rate of change (e.g., phidot limits).
- Useful for comparing controller robustness and clamping strategies.

---

## 🧠 Data Dictionary – `eomParameters.sldd`

A central dictionary file defining all **lumped system parameters**, including:

- Masses (ball, platform, rack, etc.)
- Geometric lengths
- Friction coefficients (viscous, Coulomb, rolling)
- Gravity and conversion constants

This makes it easy to update and synchronize parameters across all models.


