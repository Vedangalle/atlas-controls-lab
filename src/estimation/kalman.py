import numpy as np


class KalmanFilter:
    """
    Discrete-time Kalman filter for linear systems.

    System model:
        x_k = A x_{k-1} + B u_k + w_k
        z_k = H x_k + v_k

    where:
        w_k ~ N(0, Q)
        v_k ~ N(0, R)
    """

    def __init__(self, A, B, H, Q, R, initial_state, initial_covariance):
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R

        self.state = initial_state
        self.covariance = initial_covariance

    def predict(self, control_input):
        self.state = self.A @ self.state + self.B @ control_input
        self.covariance = self.A @ self.covariance @ self.A.T + self.Q
        return self.state

    def update(self, measurement):
        innovation = measurement - self.H @ self.state
        innovation_covariance = self.H @ self.covariance @ self.H.T + self.R

        kalman_gain = (
            self.covariance
            @ self.H.T
            @ np.linalg.inv(innovation_covariance)
        )

        self.state = self.state + kalman_gain @ innovation

        identity = np.eye(self.covariance.shape[0])
        self.covariance = (
            identity - kalman_gain @ self.H
        ) @ self.covariance

        return self.state
