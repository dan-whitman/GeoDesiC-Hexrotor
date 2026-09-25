# Pydrake Model

**Completed:**
- Basic URDF formulation
- Linearized system about hover equilibrium
- Linear controllability determination about hover equilibirum
- 

**Next Steps:**
- Update URDF to match physical model
- Implement LQR controller
- Implement Pydrake simulation and trajectory tracking
- 

## Overview
This directory includes the current Pydrake model for dynamical modeling and control. The current objective is to linearize the system about the hover equilibrium and establish a simple LQR controller for testing and simulation. More advanced geometric control will be implemented later, as well as a C++ Drake model.

## Contents
- **PodCopter.urdf.xacro:** Xacro URDF model of hexcopter
- **PodCopterView.py:** Current Python script implementing linearized Pydrake model
- **XacroUtils.py:** Python script to parse .xacro file to URDF
