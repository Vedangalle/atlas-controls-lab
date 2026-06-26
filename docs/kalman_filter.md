# Kalman Filter

The Kalman filter is a recursive state estimator for linear systems with noisy measurements.

It estimates the hidden state of a system using two steps:

```text
Predict
Update
```

The system model is:

```text
x_k = A*x_{k-1} + B*u_k + w_k
z_k = H*x_k + v_k
```

where:

- `x_k` is the state
- `u_k` is the control input
- `z_k` is the measurement
- `w_k` is process noise
- `v_k` is measurement noise

## 1D Tracking

The current module estimates position and velocity from noisy position measurements.

State:

```text
x[0] = position
x[1] = velocity
```

The simulation compares raw measurement error against Kalman-filtered estimation error.

## Implementation Notes

Estimator:

```text
src/estimation/kalman.py
```

Simulation:

```text
notebooks/03_kalman_filter_1d_tracking.py
```

## What This Demonstrates

This module demonstrates:

- State estimation
- Prediction-update filtering
- Noise handling
- Measurement correction
- RMSE-based estimator evaluation

## Limitations

This implementation assumes linear dynamics and Gaussian noise. Future extensions could include Extended Kalman Filtering for nonlinear robot localization.
