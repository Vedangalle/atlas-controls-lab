import numpy as np


class DroneAttitude1D:
    """
    Simplified 1D drone attitude model for roll stabilization.

    State:
        x[0] = roll angle, radians
        x[1] = roll angular velocity, radians/second

    Input:
        u = control torque, N*m

    Equation:
        I * theta_ddot = u - damping * theta_dot
    """

    def __init__(self, inertia=0.02, damping=0.01):
        self.inertia = inertia
        self.damping = damping

    def dynamics(self, state, control_torque):
        angle, angular_velocity = state

        angular_acceleration = (
            control_torque - self.damping * angular_velocity
        ) / self.inertia

        return np.array([angular_velocity, angular_acceleration])

    def step(self, state, control_torque, dt):
        state_dot = self.dynamics(state, control_torque)
        return state + state_dot * dt
