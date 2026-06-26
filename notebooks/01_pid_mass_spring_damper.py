import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(str(Path(__file__).resolve().parents[1]))

from src.controllers.pid import PIDController
from src.dynamics.mass_spring_damper import MassSpringDamper


def run_simulation():
    dt = 0.01
    total_time = 10.0
    time = np.arange(0, total_time, dt)

    system = MassSpringDamper(mass=1.0, damping=0.5, stiffness=2.0)
    controller = PIDController(
        kp=18.0,
        ki=2.0,
        kd=5.0,
        dt=dt,
        output_limits=(-50, 50),
    )

    setpoint = 1.0
    state = np.array([0.0, 0.0])

    positions = []
    velocities = []
    controls = []

    for _ in time:
        position = state[0]
        control_input = controller.update(setpoint, position)
        state = system.step(state, control_input, dt)

        positions.append(state[0])
        velocities.append(state[1])
        controls.append(control_input)

    positions = np.array(positions)
    controls = np.array(controls)

    figures_dir = Path("figures")
    figures_dir.mkdir(exist_ok=True)

    plt.figure(figsize=(10, 5))
    plt.plot(time, positions, label="Position")
    plt.axhline(setpoint, linestyle="--", label="Setpoint")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("PID Control of Mass-Spring-Damper System")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "pid_mass_spring_damper_position.png", dpi=200)
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(time, controls, label="Control Input")
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")
    plt.title("PID Control Effort")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "pid_mass_spring_damper_control.png", dpi=200)
    plt.show()

    final_error = setpoint - positions[-1]
    overshoot = max(positions) - setpoint

    print("Simulation complete")
    print(f"Final position: {positions[-1]:.4f} m")
    print(f"Final error: {final_error:.4f} m")
    print(f"Overshoot: {overshoot:.4f} m")


if __name__ == "__main__":
    run_simulation()
