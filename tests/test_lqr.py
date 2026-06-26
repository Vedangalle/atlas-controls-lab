import numpy as np

from src.controllers.lqr import LQRController
from src.dynamics.inverted_pendulum import InvertedPendulum


def test_lqr_gain_has_correct_shape():
    system = InvertedPendulum()

    Q = np.diag([1.0, 1.0, 80.0, 5.0])
    R = np.array([[0.1]])

    controller = LQRController(system.A, system.B, Q, R)

    assert controller.K.shape == (1, 4)


def test_lqr_returns_scalar_control():
    system = InvertedPendulum()

    Q = np.diag([1.0, 1.0, 80.0, 5.0])
    R = np.array([[0.1]])

    controller = LQRController(system.A, system.B, Q, R)

    state = np.array([0.0, 0.0, 0.1, 0.0])
    control = controller.update(state)

    assert isinstance(control, float)


def test_lqr_control_is_finite():
    system = InvertedPendulum()

    Q = np.diag([1.0, 1.0, 80.0, 5.0])
    R = np.array([[0.1]])

    controller = LQRController(system.A, system.B, Q, R)

    state = np.array([0.0, 0.0, 0.1, 0.0])
    control = controller.update(state)

    assert np.isfinite(control)
