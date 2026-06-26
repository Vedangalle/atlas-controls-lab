# Linear Quadratic Regulator

The Linear Quadratic Regulator is an optimal control method for linear state-space systems.

The system is modeled as:

```text
x_dot = A*x + B*u
```

The controller minimizes the cost function:

```text
J = integral(x.T*Q*x + u.T*R*u) dt
```

where:

- `Q` penalizes state error
- `R` penalizes control effort

The resulting feedback law is:

```text
u = -K*x
```

## Inverted Pendulum

The inverted pendulum is a classic unstable control problem. The goal is to stabilize the pendulum around the upright equilibrium.

State:

```text
x[0] = cart position
x[1] = cart velocity
x[2] = pendulum angle from upright
x[3] = angular velocity
```

## Implementation Notes

The LQR gain is computed using the continuous-time algebraic Riccati equation.

Controller:

```text
src/controllers/lqr.py
```

Dynamics:

```text
src/dynamics/inverted_pendulum.py
```

Simulation:

```text
notebooks/02_inverted_pendulum_lqr.py
```

## What This Demonstrates

This module demonstrates:

- State-space modeling
- Optimal control
- Riccati equation solving
- Feedback gain computation
- Stabilization of an unstable system

## Limitations

The current model is linearized around the upright equilibrium. Future extensions could include nonlinear dynamics, animation, and comparison against PID or MPC.
