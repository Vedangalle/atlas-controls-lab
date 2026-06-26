import numpy as np


class UnicycleModel:
    """
    Simple unicycle robot model.

    State:
        x[0] = x position
        x[1] = y position
        x[2] = heading angle, radians

    Inputs:
        v = linear velocity
        omega = angular velocity
    """

    def step(self, state, linear_velocity, angular_velocity, dt):
        x, y, heading = state

        x_dot = linear_velocity * np.cos(heading)
        y_dot = linear_velocity * np.sin(heading)
        heading_dot = angular_velocity

        next_state = np.array([
            x + x_dot * dt,
            y + y_dot * dt,
            heading + heading_dot * dt,
        ])

        next_state[2] = np.arctan2(
            np.sin(next_state[2]),
            np.cos(next_state[2]),
        )

        return next_state
