import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.append(str(Path(__file__).resolve().parents[1]))

from src.controllers.lqr import LQRController
from src.dynamics.inverted_pendulum import InvertedPendulum


def run_simulation():
    dt = 0.001
    total_time = 6.0
    time = np.arange(0, total_time, dt)

    system = InvertedPendulum(
        cart_mass=1.0,
        pendulum_mass=0.1,
        length=0.5,
        gravity=9.81,
    )

    Q = np.diag([1.0, 1.0, 80.0, 5.0])
    R = np.array([[0.1]])

    controller = LQRController(system.A, system.B, Q, R)

    initial_angle_deg = 10.0
    state = np.array([
        0.0,
        0.0,
        np.deg2rad(initial_angle_deg),
        0.0,
    ])

    reference = np.zeros(4)

    states = []
    controls = []

    for _ in time:
        control_input = controller.update(state, reference)
        state = system.step(state, control_input, dt)

        states.append(state.copy())
        controls.append(control_input)

    states = np.array(states)
    controls = np.array(controls)

    cart_position = states[:, 0]
    pendulum_angle_deg = np.rad2deg(states[:, 2])

    figures_dir = Path("figures")
    figures_dir.mkdir(exist_ok=True)

    plt.figure(figsize=(10, 5))
    plt.plot(time, pendulum_angle_deg, label="Pendulum angle")
    plt.axhline(0.0, linestyle="--", label="Upright equilibrium")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("LQR Stabilization of Inverted Pendulum")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "lqr_inverted_pendulum_angle.png", dpi=200)
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(time, cart_position, label="Cart position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Cart Position During LQR Stabilization")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "lqr_inverted_pendulum_cart_position.png", dpi=200)
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(time, controls, label="Control force")
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")
    plt.title("LQR Control Effort")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(figures_dir / "lqr_inverted_pendulum_control.png", dpi=200)
    plt.show()

    print("Simulation complete")
    print(f"LQR gain K: {controller.K}")
    print(f"Initial angle: {initial_angle_deg:.2f} deg")
    print(f"Final angle: {pendulum_angle_deg[-1]:.4f} deg")
    print(f"Final cart position: {cart_position[-1]:.4f} m")
    print(f"Max control effort: {np.max(np.abs(controls)):.4f} N")


if __name__ == "__main__":
    run_simulation()
