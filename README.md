# Robotic Arm P – Mechatronics Project

This repository documents the design, implementation, and control of **Robot P**, a 2-DoF robotic arm developed for a university mechatronics module. The project integrates **mechanical design**, **kinematic modeling**, and **PID control**, with both simulation (MATLAB/Simulink) and real-time implementation (Arduino Mega).

---

## Project Summary

- **Degrees of Freedom**: 2 (1 Revolute, 1 Prismatic)
- **Control System**: Dual PID controllers with encoder feedback
- **Target Tracking**: Achieved via inverse kinematics for planar X-Z motion
- **Implementation**: Hardware system programmed in Arduino C++, simulated in MATLAB Simulink

---

## ⚙Features

- ✅ **Forward & Inverse Kinematics** (Custom geometry)
- ✅ **PID Control** for revolute and prismatic joints
- ✅ **Quadrature Encoder Feedback** for real-time joint tracking
- ✅ **Keyboard-based Manual Control** (WASD)
- ✅ **Simulink Simulation** for verification and tuning
- ✅ **Custom CAD & 3D Printed Parts**
- ✅ **Fully Documented Report** included below

---

## Repository Structure

| File/Folder | Description |
|-------------|-------------|
| `FullFK&IK_Implementation.ino`  | Arduino sketch implementing encoder feedback, kinematics, and PID |
| `WASD_Control`   | Arduino sketch implementing encoder feedback, WASD-key control and PID |
| `README.md` | This file |
| `RobotDesignMechatronicsFIINAAALLLL.pdf` | Full technical report with detailed analysis, equations, CAD, results |

---

## Kinematics Summary

**Forward Kinematics**:
\[
x = 0.25\cos(\theta) + h\cos(\gamma) \\
z = 0.25\sin(\theta) + h\sin(\gamma) + \Delta d
\]

**Inverse Kinematics**:
\[
\theta = \cos^{-1}\left(\frac{x - h\cos(\gamma)}{0.25}\right) \\
\Delta d = z - 0.25\sin(\theta) - h\sin(\gamma)
\]

---

## Control System

- **PID Gains**: Tuned via Simulink for best performance
    - \(K_p = 2.0\)
    - \(K_i = 0.5\)
    - \(K_d = 0.1\)

- **Implemented with**: `PID_v1.h` on Arduino IDE  
- **Motor Driver**: L298N Dual H-Bridge
- **Microcontroller**: Arduino Mega 2560

---

## Hardware Summary

| Component | Quantity |
|----------|----------|
| Arduino Mega | 1 |
| DC Motors (w/ encoders) | 2 |
| L298N Motor Driver | 1 |
| Custom Revolving Joints (3D printed) | 12 |
| HDPE & Plywood Chassis | ✅ |
| Rack and Pinion System | ✅ |
| End Effector + Gripper | ✅ |

---

## Full Technical Report

For full kinematics derivations, control system modelling, CAD renders, and results:

📄 [RobotDesignMechatronicsFIINAAALLLL.pdf](RobotDesignMechatronicsFIINAAALLLL.pdf)

---

## 🧾 License

This project was created as part of an academic module and is intended for educational and demonstrative purposes. Please credit the authors if you use any portion of the work.

---

## 👥 Authors

- **Harish Raj Venkatnarayanan** – Kinematics, Control Code, Electronics, Final Presentation  
- **James Brownson** – CAD Design Lead, Mechanical Assembly  
- **Victor Osuofia** – Mechanical Support, Fabrication

---

## 💬 Acknowledgements

Thanks to our academic supervisors and Queen Mary University of London for support during this module.
