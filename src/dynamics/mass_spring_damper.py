import numpy as np


class MassSpringDamper:
    """
    Mass-spring-damper system.

    Equation:
        m*x_ddot + c*x_dot + k*x = u

    State:
        x[0] = position
        x[1] = velocity
    """

    def __init__(self, mass=1.0, damping=0.5, stiffness=2.0):
        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness

    def dynamics(self, state, control_input):
        position, velocity = state

        acceleration = (
            control_input
            - self.damping * velocity
            - self.stiffness * position
        ) / self.mass

        return np.array([velocity, acceleration])

    def step(self, state, control_input, dt):
        state_dot = self.dynamics(state, control_input)
        return state + state_dot * dt
