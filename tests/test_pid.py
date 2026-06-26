from src.controllers.pid import PIDController


def test_pid_output_decreases_as_error_decreases():
    controller = PIDController(
        kp=2.0,
        ki=0.0,
        kd=0.0,
        dt=0.01,
    )

    large_error_output = controller.update(setpoint=1.0, measurement=0.0)
    small_error_output = controller.update(setpoint=1.0, measurement=0.8)

    assert abs(small_error_output) < abs(large_error_output)


def test_pid_output_limits_are_respected():
    controller = PIDController(
        kp=100.0,
        ki=0.0,
        kd=0.0,
        dt=0.01,
        output_limits=(-5.0, 5.0),
    )

    output = controller.update(setpoint=10.0, measurement=0.0)

    assert output == 5.0


def test_pid_reset_clears_internal_state():
    controller = PIDController(
        kp=1.0,
        ki=1.0,
        kd=1.0,
        dt=0.01,
    )

    controller.update(setpoint=1.0, measurement=0.0)
    controller.reset()

    assert controller.integral == 0.0
    assert controller.previous_error == 0.0
