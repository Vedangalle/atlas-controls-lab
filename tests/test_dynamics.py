import numpy as np

from src.dynamics.mass_spring_damper import MassSpringDamper
from src.dynamics.inverted_pendulum import InvertedPendulum
from src.dynamics.drone_attitude import DroneAttitude1D
from src.dynamics.unicycle import UnicycleModel


def test_mass_spring_damper_step_returns_finite_state():
    system = MassSpringDamper()
    state = np.array([0.0, 0.0])

    next_state = system.step(state, control_input=1.0, dt=0.01)

    assert next_state.shape == (2,)
    assert np.all(np.isfinite(next_state))


def test_inverted_pendulum_step_returns_finite_state():
    system = InvertedPendulum()
    state = np.array([0.0, 0.0, 0.1, 0.0])

    next_state = system.step(state, control_input=0.0, dt=0.001)

    assert next_state.shape == (4,)
    assert np.all(np.isfinite(next_state))


def test_drone_attitude_step_returns_finite_state():
    system = DroneAttitude1D()
    state = np.array([0.1, 0.0])

    next_state = system.step(state, control_torque=0.01, dt=0.001)

    assert next_state.shape == (2,)
    assert np.all(np.isfinite(next_state))


def test_unicycle_step_returns_finite_state():
    system = UnicycleModel()
    state = np.array([0.0, 0.0, 0.0])

    next_state = system.step(
        state,
        linear_velocity=1.0,
        angular_velocity=0.1,
        dt=0.1,
    )

    assert next_state.shape == (3,)
    assert np.all(np.isfinite(next_state))
