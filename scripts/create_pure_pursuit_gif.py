import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

sys.path.append(str(Path(__file__).resolve().parents[1]))

from src.controllers.pure_pursuit import PurePursuitController
from src.dynamics.unicycle import UnicycleModel


def generate_reference_path():
    x = np.linspace(0, 20, 400)
    y = 2.0 * np.sin(0.4 * x)
    return np.column_stack((x, y))


def simulate():
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

        if np.linalg.norm(state[:2] - path[-1]) < 0.5:
            break

    return path, np.array(states), np.array(lookahead_points)


def create_animation():
    path, states, lookahead_points = simulate()

    media_dir = Path("media")
    media_dir.mkdir(exist_ok=True)

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(path[:, 0], path[:, 1], label="Reference path")
    trajectory_line, = ax.plot([], [], label="Robot trajectory")
    robot_point, = ax.plot([], [], marker="o", markersize=8, label="Robot")
    lookahead_point, = ax.plot([], [], marker="x", markersize=8, label="Lookahead point")
    heading_line, = ax.plot([], [], linewidth=2, label="Heading")

    ax.set_title("Pure Pursuit Path Tracking")
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.axis("equal")
    ax.grid(True)
    ax.legend(loc="upper right")

    ax.set_xlim(-1, 21)
    ax.set_ylim(-3, 3)

    frame_skip = 4
    frame_indices = range(0, len(states), frame_skip)

    def update(frame_index):
        state = states[frame_index]
        lookahead = lookahead_points[frame_index]

        x, y, heading = state

        trajectory_line.set_data(
            states[:frame_index + 1, 0],
            states[:frame_index + 1, 1],
        )

        robot_point.set_data([x], [y])
        lookahead_point.set_data([lookahead[0]], [lookahead[1]])

        heading_length = 0.7
        heading_x = [
            x,
            x + heading_length * np.cos(heading),
        ]
        heading_y = [
            y,
            y + heading_length * np.sin(heading),
        ]
        heading_line.set_data(heading_x, heading_y)

        return trajectory_line, robot_point, lookahead_point, heading_line

    animation = FuncAnimation(
        fig,
        update,
        frames=frame_indices,
        interval=50,
        blit=True,
    )

    output_path = media_dir / "pure_pursuit_tracking.gif"
    animation.save(output_path, writer=PillowWriter(fps=20))

    plt.close(fig)

    print(f"Saved animation to {output_path}")


if __name__ == "__main__":
    create_animation()
