import numpy as np


class InvertedPendulum:
    """
    Linearized inverted pendulum on a cart.

    State:
        x[0] = cart position
        x[1] = cart velocity
        x[2] = pendulum angle from upright, radians
        x[3] = angular velocity

    Input:
        u = horizontal force applied to the cart
    """

    def __init__(self, cart_mass=1.0, pendulum_mass=0.1, length=0.5, gravity=9.81):
        self.M = cart_mass
        self.m = pendulum_mass
        self.l = length
        self.g = gravity

        self.A, self.B = self.linearized_matrices()

    def linearized_matrices(self):
        M = self.M
        m = self.m
        l = self.l
        g = self.g

        A = np.array([
            [0, 1, 0, 0],
            [0, 0, -(m * g) / M, 0],
            [0, 0, 0, 1],
            [0, 0, ((M + m) * g) / (M * l), 0],
        ])

        B = np.array([
            [0],
            [1 / M],
            [0],
            [-1 / (M * l)],
        ])

        return A, B

    def step(self, state, control_input, dt):
        state_dot = self.A @ state + self.B.flatten() * control_input
        return state + state_dot * dt
