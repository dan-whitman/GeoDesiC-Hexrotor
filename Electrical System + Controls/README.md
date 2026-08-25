# Electrical System

**Completed:**
- Joint splitter breakout board
- MA600 breakout board
- MA600 encoder testing and PID calibration

**Next steps:**
- BCD tuning
- Multi joint position based control
- Control loop latency telemetry bringup

## Overview
This directory covers the electrical and embedded systems that support the hexrotor platform. My goal is to make the feedback loop between sensors and the flight controller efficient, simple, and modular, so I'm developing custom PCBs (sensor breakouts, joint signal routing, Pi interfaces) alongside the embedded software that ties sensors, controllers, and actuators together.

## Contents
- **Joint Splitter Altium** — schematic and PCB layout for the joint splitter breakout board, which routes signals from each dual rotor joint pair to the rest of the electrical system.
- **MA600 Breakout Altium** — schematic and PCB layout for the MA600 magnetic encoder breakout board, used for passively sensing joint angle.
