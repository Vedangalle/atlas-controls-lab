class PIDController:
    """
    Basic PID controller.

    u(t) = Kp * e(t) + Ki * integral(e) + Kd * derivative(e)

    This controller is intentionally written from scratch for clarity.
    """

    def __init__(self, kp, ki, kd, dt, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limits = output_limits

        self.integral = 0.0
        self.previous_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0

    def update(self, setpoint, measurement):
        error = setpoint - measurement

        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        if self.output_limits is not None:
            lower, upper = self.output_limits
            output = max(lower, min(upper, output))

        self.previous_error = error
        return output
