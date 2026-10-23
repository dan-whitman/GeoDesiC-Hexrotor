# Pydrake Model

**Completed:**
- Basic ```.urdf``` and ```.xacro``` hexrotor description
- Linearized system about hover equilibrium
- Linear controllability determination about hover equilibirum
- Functioning simulation and visualization
- Implemented LQR controller around equilibrium
- Analytical solver for arbitrary equilibrium state configuration (simplified and corrected)
- 

**Next Steps:**
- Propeller saturation (and reviersiblity? probably not needed)
- Data output and logging/plotting state and thrusts over time
- Implement numerical solver for equilibrium state and compare to analytical
- Solve for required state configuration to reach desired acceleration at pose
- Implement trajectory tracking following ^^^
- 

**As Needed:**
- Update ```.urdf``` to better match physical hexrotor
- Implement more ```.xacro``` capabilities to automate ```.urdf``` generation
- 

## Overview
This directory includes the current Pydrake model for dynamical modeling and control. The current objective is to linearize the system about the hover equilibrium and establish a simple LQR controller for testing and simulation. More advanced geometric control will be implemented later, as well as a C++ Drake model.

## Contents
- **HexRotorDrake.py:** main script for model
- **/inputs:**
    - ```config.py:``` configuration file for model user inputs
- **/models:**
    - ```HexRotor.urdf.xacro:``` ```.urdf.xacro``` file for the hexrotor
- **/utils:** 
    - ```control.py:``` scripts for the controller design and implementation
    - ```equil.py:``` scripts for equilibrium state solving
    - ```linear.py:``` scripts for model linearization
    - ```print.py:``` scripts for printing model informations in terminal
    - ```simulate.py:``` scripts for model simulation
    - ```system.py:``` scripts for generating plant hexrotor model
    - ```visual.py:``` scripts for visualizing simulation
    - ```xacro.py:``` scripts for .xacro commands
- 

## PYDRAKE CONFIGURATION:
First, you must run the following command in terminal *or* include it in your ```~/.bashrc``` file to establish pydrake environment variable prior to running script (taken from [Drake: Installation via APT](https://drake.mit.edu/apt.html)):
```bash
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
```

You must *also* install ```xacro``` so that the script an parse the ```.xacro.urdf``` file. This is *not* shown in the ```requirements.txt``` file. To do this, in terminal (if using ```pipx```):
```bash
pipx install xacro
pipx ensurepath
```

- Note: this configuration is for standard Ubuntu-26.04 WSL installation/implementation. It may differ for other distros.