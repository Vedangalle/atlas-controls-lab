# Pure Pursuit Path Tracking

Pure pursuit is a geometric path tracking controller commonly used for mobile robots and autonomous vehicles.

The controller selects a lookahead point on the reference path and steers the robot toward that point.

## Robot Model

The current simulation uses a unicycle model:

```text
x_dot = v*cos(theta)
y_dot = v*sin(theta)
theta_dot = omega
```

State:

```text
x[0] = x position
x[1] = y position
x[2] = heading angle
```

Inputs:

```text
v = linear velocity
omega = angular velocity
```

## Controller

The controller computes the heading error between the robot heading and the lookahead point.

The angular velocity command is based on:

```text
omega = 2*v*sin(alpha) / Ld
```

where:

- `alpha` is the heading error
- `Ld` is the lookahead distance
- `v` is the commanded linear velocity

## Implementation Notes

Controller:

```text
src/controllers/pure_pursuit.py
```

Dynamics:

```text
src/dynamics/unicycle.py
```

Simulation:

```text
notebooks/05_path_tracking_pure_pursuit.py
```

Animation:

```text
scripts/create_pure_pursuit_gif.py
```

## What This Demonstrates

This module demonstrates:

- Mobile robot path tracking
- Lookahead-point selection
- Heading correction
- Steering command generation
- Tracking error analysis

## Limitations

The current model assumes constant forward velocity and a simple kinematic model. Future extensions could include velocity profiling, obstacle avoidance, and MPC-based path tracking.
