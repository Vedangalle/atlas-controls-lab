import numpy as np
from scipy.linalg import solve_continuous_are


class LQRController:
    """
    Linear Quadratic Regulator controller.

    The controller solves the continuous-time algebraic Riccati equation and computes:

        u = -Kx
    """

    def __init__(self, A, B, Q, R):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.K = self._compute_gain()

    def _compute_gain(self):
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K

    def update(self, state, reference=None):
        if reference is None:
            reference = np.zeros_like(state)

        error = state - reference
        control = -self.K @ error
        return control.item()
