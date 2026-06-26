# PID Control

PID control is one of the most widely used feedback control methods in engineering systems.

The controller computes a control input using proportional, integral, and derivative terms:

```text
u(t) = Kp * e(t) + Ki * integral(e(t)) + Kd * de(t)/dt
```

where:

- `e(t)` is the tracking error
- `Kp` controls proportional response
- `Ki` reduces steady-state error
- `Kd` damps fast changes and overshoot

## Mass-Spring-Damper System

The simulated plant is:

```text
m*x_ddot + c*x_dot + k*x = u
```

State:

```text
x[0] = position
x[1] = velocity
```

The PID controller applies force to move the mass toward a target position.

## Implementation Notes

The controller is implemented from scratch in:

```text
src/controllers/pid.py
```

The plant dynamics are implemented in:

```text
src/dynamics/mass_spring_damper.py
```

The simulation is run from:

```text
notebooks/01_pid_mass_spring_damper.py
```

## What This Demonstrates

This module demonstrates:

- Feedback control
- Error-based correction
- Setpoint tracking
- Overshoot and steady-state error analysis
- Reusable controller design

## Limitations

This is a simple linear second-order system. Future extensions could include actuator saturation, anti-windup logic, nonlinear friction, and automatic gain tuning.
