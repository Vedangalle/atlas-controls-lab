import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(str(Path(__file__).resolve().parents[1]))

from src.controllers.pid import PIDController
from src.dynamics.drone_attitude import DroneAttitude1D


def run_simulation():
    dt = 0.001
    total_time = 5.0
    time = np.arange(0, total_time, dt)

    system = DroneAttitude1D(
        inertia=0.02,
        damping=0.01,
    )

    controller = PIDController(
        kp=0.9,
        ki=0.05,
        kd=0.18,
        dt=dt,
        output_limits=(-1.0, 1.0),
    )

    target_angle_rad = np.deg2rad(0.0)
    initial_angle_deg = 20.0

    state = np.array([
        np.deg2rad(initial_angle_deg),
        0.0,
    ])

    angles = []
    angular_velocities = []
    control_torques = []

    for _ in time:
        current_angle = state[0]

        control_torque = controller.update(
            setpoint=target_angle_rad,
            measurement=current_angle,
        )

        state = system.step(state, control_torque, dt)

        angles.append(state[0])
        angular_velocities.append(state[1])
        control_torques.append(control_torque)

    angles = np.array(angles)
    angular_velocities = np.array(angular_velocities)
    control_torques = np.array(control_torques)

    angles_deg = np.rad2deg(angles)
    angular_velocities_deg = np.rad2deg(angular_velocities)

    figures_dir = Path("figures")
    figures_dir.mkdir(exist_ok=True)

    plt.figure(figsize=(10, 5))
    plt.plot(time, angles_deg, label="Roll angle")
    plt.axhline(0.0, linestyle="--", label="Target angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Roll angle (deg)")
    plt.title("PID Drone Roll Attitude Stabilization")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "drone_attitude_pid_angle.png", dpi=200)
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(time, angular_velocities_deg, label="Roll angular velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular velocity (deg/s)")
    plt.title("Drone Roll Angular Velocity Response")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "drone_attitude_pid_angular_velocity.png", dpi=200)
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(time, control_torques, label="Control torque")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (N*m)")
    plt.title("PID Control Torque for Drone Roll Stabilization")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "drone_attitude_pid_control_torque.png", dpi=200)
    plt.show()

    final_angle_deg = angles_deg[-1]
    max_torque = np.max(np.abs(control_torques))

    print("Simulation complete")
    print(f"Initial roll angle: {initial_angle_deg:.2f} deg")
    print(f"Final roll angle: {final_angle_deg:.4f} deg")
    print(f"Maximum control torque: {max_torque:.4f} N*m")


if __name__ == "__main__":
    run_simulation()
