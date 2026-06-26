import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(str(Path(__file__).resolve().parents[1]))

from src.estimation.kalman import KalmanFilter


def run_simulation():
    np.random.seed(42)

    dt = 0.1
    total_time = 20.0
    time = np.arange(0, total_time, dt)

    true_initial_position = 0.0
    true_initial_velocity = 1.5
    acceleration = 0.05

    true_positions = []
    true_velocities = []
    measurements = []

    position = true_initial_position
    velocity = true_initial_velocity

    measurement_noise_std = 2.0

    for _ in time:
        position = position + velocity * dt + 0.5 * acceleration * dt**2
        velocity = velocity + acceleration * dt

        noisy_measurement = position + np.random.normal(0, measurement_noise_std)

        true_positions.append(position)
        true_velocities.append(velocity)
        measurements.append(noisy_measurement)

    true_positions = np.array(true_positions)
    true_velocities = np.array(true_velocities)
    measurements = np.array(measurements)

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

    Q = np.array([
        [0.01, 0],
        [0, 0.01],
    ])

    R = np.array([
        [measurement_noise_std**2],
    ])

    initial_state = np.array([
        [0.0],
        [0.0],
    ])

    initial_covariance = np.array([
        [10.0, 0],
        [0, 10.0],
    ])

    kf = KalmanFilter(
        A=A,
        B=B,
        H=H,
        Q=Q,
        R=R,
        initial_state=initial_state,
        initial_covariance=initial_covariance,
    )

    estimated_positions = []
    estimated_velocities = []

    control_input = np.array([[acceleration]])

    for measurement in measurements:
        kf.predict(control_input)
        estimated_state = kf.update(np.array([[measurement]]))

        estimated_positions.append(estimated_state[0, 0])
        estimated_velocities.append(estimated_state[1, 0])

    estimated_positions = np.array(estimated_positions)
    estimated_velocities = np.array(estimated_velocities)

    position_rmse = np.sqrt(np.mean((true_positions - estimated_positions) ** 2))
    measurement_rmse = np.sqrt(np.mean((true_positions - measurements) ** 2))

    figures_dir = Path("figures")
    figures_dir.mkdir(exist_ok=True)

    plt.figure(figsize=(10, 5))
    plt.plot(time, true_positions, label="True position")
    plt.scatter(time, measurements, s=10, alpha=0.5, label="Noisy measurements")
    plt.plot(time, estimated_positions, label="Kalman estimate")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Kalman Filter 1D Position Tracking")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "kalman_1d_position_tracking.png", dpi=200)
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(time, true_velocities, label="True velocity")
    plt.plot(time, estimated_velocities, label="Estimated velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Kalman Filter Velocity Estimation")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "kalman_1d_velocity_estimation.png", dpi=200)
    plt.show()

    print("Simulation complete")
    print(f"Measurement RMSE: {measurement_rmse:.4f} m")
    print(f"Kalman estimate RMSE: {position_rmse:.4f} m")
    print(f"Final true position: {true_positions[-1]:.4f} m")
    print(f"Final estimated position: {estimated_positions[-1]:.4f} m")
    print(f"Final true velocity: {true_velocities[-1]:.4f} m/s")
    print(f"Final estimated velocity: {estimated_velocities[-1]:.4f} m/s")


if __name__ == "__main__":
    run_simulation()
