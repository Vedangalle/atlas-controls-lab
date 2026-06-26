import numpy as np

from src.estimation.kalman import KalmanFilter


def test_kalman_predict_update_shapes():
    dt = 0.1

    A = np.array([
        [1, dt],
        [0, 1],
    ])

    B = np.array([
        [0.5 * dt**2],
        [dt],
    ])

    H = np.array([
        [1, 0],
    ])

    Q = np.eye(2) * 0.01
    R = np.array([[1.0]])

    initial_state = np.array([
        [0.0],
        [0.0],
    ])

    initial_covariance = np.eye(2)

    kf = KalmanFilter(
        A=A,
        B=B,
        H=H,
        Q=Q,
        R=R,
        initial_state=initial_state,
        initial_covariance=initial_covariance,
    )

    predicted_state = kf.predict(np.array([[0.1]]))
    updated_state = kf.update(np.array([[1.0]]))

    assert predicted_state.shape == (2, 1)
    assert updated_state.shape == (2, 1)
    assert kf.covariance.shape == (2, 2)


def test_kalman_state_remains_finite():
    dt = 0.1

    A = np.array([
        [1, dt],
        [0, 1],
    ])

    B = np.array([
        [0.5 * dt**2],
        [dt],
    ])

    H = np.array([
        [1, 0],
    ])

    Q = np.eye(2) * 0.01
    R = np.array([[1.0]])

    kf = KalmanFilter(
        A=A,
        B=B,
        H=H,
        Q=Q,
        R=R,
        initial_state=np.zeros((2, 1)),
        initial_covariance=np.eye(2),
    )

    for _ in range(10):
        kf.predict(np.array([[0.1]]))
        kf.update(np.array([[1.0]]))

    assert np.all(np.isfinite(kf.state))
    assert np.all(np.isfinite(kf.covariance))
