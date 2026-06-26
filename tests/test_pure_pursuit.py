import numpy as np

from src.controllers.pure_pursuit import PurePursuitController


def test_pure_pursuit_returns_valid_commands():
    controller = PurePursuitController(
        lookahead_distance=1.0,
        linear_velocity=0.8,
    )

    path = np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [2.0, 0.0],
        [3.0, 0.0],
    ])

    state = np.array([0.0, 0.0, 0.0])

    linear_velocity, angular_velocity, lookahead_point = controller.update(
        state,
        path,
    )

    assert linear_velocity == 0.8
    assert np.isfinite(angular_velocity)
    assert lookahead_point.shape == (2,)


def test_pure_pursuit_lookahead_point_is_on_path():
    controller = PurePursuitController(
        lookahead_distance=1.0,
        linear_velocity=0.8,
    )

    path = np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [2.0, 0.0],
    ])

    robot_position = np.array([0.0, 0.0])
    lookahead_point = controller.find_lookahead_point(robot_position, path)

    assert any(np.allclose(lookahead_point, point) for point in path)
