# ATLAS Controls Lab

A robotics controls and estimation sandbox built from first principles in Python.

This repository implements core ideas used in autonomous systems, including PID control, optimal control, state estimation, and attitude stabilization. The goal is to connect mathematical models with clean, reproducible simulations that can be extended toward robotics research, multi-agent systems, and real hardware platforms.

## Modules

### 1. PID Control: Mass-Spring-Damper System

A classical second-order dynamic system controlled using a PID controller.

The simulation tracks a target position and reports final error, overshoot, and control effort.

Generated figures:

- `figures/pid_mass_spring_damper_position.png`
- `figures/pid_mass_spring_damper_control.png`

Run:

```bash
python notebooks/01_pid_mass_spring_damper.py

## Installation
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt