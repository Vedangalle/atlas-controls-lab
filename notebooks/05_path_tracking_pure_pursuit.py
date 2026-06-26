import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(str(Path(__file__).resolve().parents[1]))

from src.controllers.pure_pursuit import PurePursuitController
from src.dynamics.unicycle import UnicycleModel


def generate_reference_path():
    x = np.linspace(0, 20, 400)
    y = 2.0 * np.sin(0.4 * x)
    return np.column_stack((x, y))


def compute_tracking_error(robot_positions, path):
    errors = []

    for position in robot_positions:
        distances = np.linalg.norm(path - position, axis=1)
        errors.append(np.min(distances))

    return np.array(errors)


def run_simulation():
    dt = 0.05
    total_time = 25.0
    time = np.arange(0, total_time, dt)

    path = generate_reference_path()

    robot = UnicycleModel()
    controller = PurePursuitController(
        lookahead_distance=1.2,
        linear_velocity=0.9,
    )

    state = np.array([
        0.0,
        -1.5,
        np.deg2rad(20.0),
    ])

    states = []
    lookahead_points = []
    linear_velocities = []
    angular_velocities = []

    for _ in time:
        linear_velocity, angular_velocity, lookahead_point = controller.update(
            state,
            path,
        )

        state = robot.step(
            state,
            linear_velocity,
            angular_velocity,
            dt,
        )

        states.append(state.copy())
        lookahead_points.append(lookahead_point.copy())
        linear_velocities.append(linear_velocity)
        angular_velocities.append(angular_velocity)

        distance_to_goal = np.linalg.norm(state[:2] - path[-1])
        if distance_to_goal < 0.5:
            break

    states = np.array(states)
    lookahead_points = np.array(lookahead_points)
    linear_velocities = np.array(linear_velocities)
    angular_velocities = np.array(angular_velocities)
    sim_time = np.arange(0, len(states) * dt, dt)

    robot_positions = states[:, :2]
    tracking_errors = compute_tracking_error(robot_positions, path)

    figures_dir = Path("figures")
    figures_dir.mkdir(exist_ok=True)

    plt.figure(figsize=(10, 6))
    plt.plot(path[:, 0], path[:, 1], label="Reference path")
    plt.plot(robot_positions[:, 0], robot_positions[:, 1], label="Robot trajectory")
    plt.scatter(robot_positions[0, 0], robot_positions[0, 1], label="Start")
    plt.scatter(path[-1, 0], path[-1, 1], label="Goal")
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.title("Pure Pursuit Path Tracking")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "pure_pursuit_path_tracking.png", dpi=200)
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(sim_time, tracking_errors, label="Tracking error")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.title("Pure Pursuit Tracking Error")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "pure_pursuit_tracking_error.png", dpi=200)
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(sim_time, angular_velocities, label="Angular velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular velocity (rad/s)")
    plt.title("Pure Pursuit Steering Command")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "pure_pursuit_angular_velocity.png", dpi=200)
    plt.show()

    print("Simulation complete")
    print(f"Number of simulation steps: {len(states)}")
    print(f"Final position: x={states[-1, 0]:.4f} m, y={states[-1, 1]:.4f} m")
    print(f"Final heading: {np.rad2deg(states[-1, 2]):.4f} deg")
    print(f"Mean tracking error: {np.mean(tracking_errors):.4f} m")
    print(f"Max tracking error: {np.max(tracking_errors):.4f} m")
    print(f"Final tracking error: {tracking_errors[-1]:.4f} m")


if __name__ == "__main__":
    run_simulation()
