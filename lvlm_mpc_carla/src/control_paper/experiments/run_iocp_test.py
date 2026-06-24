"""iOCP test: verify Algorithm 1 & 2 with a task-switching scenario.

Scenario:
  - Steps  0–39:  T = IDLE,  normal lane keeping + speed tracking
  - Step     40:  T switches to LANE_RIGHT, but a lead vehicle is only 8 m
                   ahead → the LC safety constraint (d_safe ≥ 10 m) is violated
                   → feasibility check FAILS → iOCP engages
  - Steps 40–N:   iOCP guides ego (slows down) toward feasibility;
                   once feasible, MPC Switcher accepts LC directly.

This exercises:
  - Formula (10)  — horizon feasibility check with previous control sequence
  - Algorithm 1   — iOCP(O_prev, O_target)
  - Algorithm 2   — MPC Switcher (feasible → accept / iOCP / reject)
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from control_paper.mpc_builder import (
    MPCBuilder,
    MPCBuilderConfig,
    SurroundingVehicle,
    TaskCommand,
    TrafficContext,
)
from control_paper.mpc_builder.models import KinematicBicycleParams, kinematic_bicycle_dynamics

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SIM_STEPS = 60           # 3 s
SWITCH_STEP = 20         # switch task at t = 1.0 s
DT = 0.05


def heuristic_warm_start(x: np.ndarray, task: TaskCommand, horizon: int) -> np.ndarray:
    """Heuristic control sequence for MPPI warm-start."""
    y = float(x[1])
    theta = float(x[2])
    v = float(x[3])

    k_y, k_theta = 0.5, 0.3
    delta_heu = float(np.clip(-k_y * y - k_theta * theta, -0.5, 0.5))
    a_heu = float(np.clip((task.target_speed - v) / (horizon * DT), -3.0, 2.0))
    return np.tile(np.array([a_heu, delta_heu], dtype=float), (horizon, 1))


def run_iocp_test() -> dict:
    """Run closed-loop with a mid-simulation task switch that triggers iOCP."""
    builder = MPCBuilder(
        MPCBuilderConfig(
            horizon=20,
            num_samples=256,             # faster for iOCP test
            enable_iocp=True,
            max_iocp_steps=50,
        )
    )

    # Two task commands
    task_idle = TaskCommand(
        lane_action="IDLE",
        target_speed=12.0,
        lane_bounds=(-3.5, 3.5),
        steer_bounds=(-0.5, 0.5),
        accel_bounds=(-3.0, 2.0),
        safe_distance=10.0,
    )
    task_lc = TaskCommand(
        lane_action="LANE_RIGHT",
        target_speed=12.0,
        lane_bounds=(-3.5, 3.5),
        steer_bounds=(-0.5, 0.5),
        accel_bounds=(-3.0, 2.0),
        safe_distance=10.0,              # dˡᶜ_safe
    )

    # Traffic: a lead vehicle that ego will approach
    context = TrafficContext(
        current_lane_center_y=0.0,
        lane_width=3.5,
        surrounding_vehicles=[
            SurroundingVehicle(x=15.0, y=0.0, vx=0.0, vy=0.0),  # stationary, ~5m gap at switch
        ],
    )

    x = np.array([0.0, 0.2, 0.02, 10.0], dtype=float)
    dyn = kinematic_bicycle_dynamics(KinematicBicycleParams(dt=DT))

    traj = [x.copy()]
    controls: list[np.ndarray] = []
    costs: list[float] = []
    rejected_flags: list[bool] = []
    iocp_active: list[bool] = []
    primitive_history: list[list[str]] = []

    u_prev = heuristic_warm_start(x, task_idle, builder.config.horizon)

    for step in range(SIM_STEPS):
        # --- task switch at SWITCH_STEP --------------------------------
        if step < SWITCH_STEP:
            task = task_idle
        else:
            task = task_lc

        result = builder.solve(task, x, u_init=u_prev, context=context)
        u = np.asarray(result.control, dtype=float)

        controls.append(u.copy())
        costs.append(float(result.cost))
        rejected_flags.append(bool(result.rejected))

        # detect iOCP by checking if solve_ocp name contains "iOCP"
        is_iocp = "iOCP" in result.solve_ocp.name
        iocp_active.append(is_iocp)
        primitive_history.append(result.primitive_names)

        # advance dynamics
        x = dyn(x, u)
        traj.append(x.copy())

        u_prev = heuristic_warm_start(x, task, builder.config.horizon)

    traj_arr = np.asarray(traj, dtype=float)
    ctrl_arr = np.asarray(controls, dtype=float)

    return {
        "sim_steps": SIM_STEPS,
        "switch_step": SWITCH_STEP,
        "dt": DT,
        "trajectory": traj_arr.tolist(),
        "controls": ctrl_arr.tolist(),
        "costs": costs,
        "rejected_flags": [int(r) for r in rejected_flags],
        "iocp_active": [int(i) for i in iocp_active],
        "primitive_history": primitive_history,
        "final_state": traj_arr[-1].tolist(),
        "initial_state": traj_arr[0].tolist(),
    }


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

def save_visualizations(output: dict, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    traj = np.asarray(output["trajectory"], dtype=float)
    ctrl = np.asarray(output["controls"], dtype=float)
    costs = np.asarray(output["costs"], dtype=float)
    iocp = np.asarray(output["iocp_active"], dtype=bool)
    dt = output["dt"]
    switch = output["switch_step"]
    t = np.arange(traj.shape[0]) * dt
    t_ctrl = np.arange(ctrl.shape[0]) * dt

    x, y, theta, v = traj[:, 0], traj[:, 1], traj[:, 2], traj[:, 3]
    a, delta = ctrl[:, 0], ctrl[:, 1]

    # --- 4-panel summary -------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    ax = axes[0, 0]
    ax.plot(x, y, linewidth=1.5, label="trajectory")
    ax.scatter([x[0]], [y[0]], color="red", s=80, zorder=5, label="start")
    ax.scatter([x[-1]], [y[-1]], color="blue", s=80, zorder=5, label="end")
    ax.axvline(x[switch], color="orange", linestyle="--", alpha=0.7, label="task switch")
    ax.axhline(0.0, color="green", linestyle="--", label="lane center (0)")
    ax.axhline(-3.5, color="green", linestyle="--", label="target lane center (-3.5)")
    ax.axhline(-3.5, color="gray", linestyle=":")
    ax.axhline(3.5, color="gray", linestyle=":")
    ax.set_title("Trajectory (XY)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, y, linewidth=1.5)
    ax.axvline(switch * dt, color="orange", linestyle="--", alpha=0.7, label="task switch")
    ax.axhline(0.0, color="green", linestyle="--", label="current lane center")
    ax.axhline(-3.5, color="green", linestyle="--", label="target lane center")
    ax.fill_between(t, -3.5, 3.5, color="gray", alpha=0.1, label="lane bounds")
    ax.set_title("Lateral Position")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("y [m]")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(t, v, linewidth=1.5, label="speed")
    ax.axvline(switch * dt, color="orange", linestyle="--", alpha=0.7)
    ax.axhline(12.0, color="green", linestyle="--", label="target (12 m/s)")
    ax.set_title("Speed")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("v [m/s]")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t_ctrl, a, linewidth=1.5, label="accel")
    ax.plot(t_ctrl, delta, linewidth=1.5, label="steer")
    ax.axvline(switch * dt, color="orange", linestyle="--", alpha=0.7)
    ax.set_title("Control Inputs")
    ax.set_xlabel("time [s]")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle("iOCP Test — Task Switch IDLE → LANE_RIGHT (unsafe)", fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_dir / "iocp_test_summary.png", dpi=200)
    plt.close(fig)

    # --- iOCP diagnostic -------------------------------------------------
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axes2[0].plot(t_ctrl, costs, linewidth=1.5, color="tab:purple")
    axes2[0].set_ylabel("MPC cost")
    axes2[0].set_title("Cost per Step")
    axes2[0].grid(True, alpha=0.3)
    axes2[0].axvline(switch * dt, color="orange", linestyle="--")

    axes2[1].fill_between(t_ctrl, 0, iocp.astype(float), step="mid",
                          color="blue", alpha=0.4, label="iOCP active")
    axes2[1].set_ylabel("iOCP")
    axes2[1].set_ylim(-0.1, 1.1)
    axes2[1].legend(loc="upper right")
    axes2[1].grid(True, alpha=0.3)
    axes2[1].axvline(switch * dt, color="orange", linestyle="--")

    rejected = np.asarray(output["rejected_flags"], dtype=bool)
    if rejected.any():
        axes2[2].fill_between(t_ctrl, 0, rejected.astype(float), step="mid",
                              color="red", alpha=0.4, label="rejected")
    axes2[2].set_ylabel("rejected")
    axes2[2].set_xlabel("time [s]")
    axes2[2].set_ylim(-0.1, 1.1)
    axes2[2].grid(True, alpha=0.3)
    axes2[2].axvline(switch * dt, color="orange", linestyle="--")

    fig2.suptitle("iOCP Test — Switcher Diagnostics")
    fig2.tight_layout()
    fig2.savefig(out_dir / "iocp_test_diagnostics.png", dpi=200)
    plt.close(fig2)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    project_root = Path(__file__).resolve().parents[4]
    out_dir = project_root / "results" / "images" / "iocp_test"
    out_dir.mkdir(parents=True, exist_ok=True)

    output = run_iocp_test()

    # summary
    iocp_steps = sum(output["iocp_active"])
    rejected_steps = sum(output["rejected_flags"])
    print(json.dumps({
        "initial_state": output["initial_state"],
        "final_state": output["final_state"],
        "switch_step": output["switch_step"],
        "iocp_steps": iocp_steps,
        "rejected_steps": rejected_steps,
        "y_final": output["trajectory"][-1][1],
        "v_final": output["trajectory"][-1][3],
        "saved_to": str(out_dir),
    }, indent=2, ensure_ascii=False))

    save_visualizations(output, out_dir)

    data_path = out_dir / "iocp_test_data.json"
    with open(data_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2, ensure_ascii=False, default=float)
    print(f"Raw data saved to: {data_path}")


if __name__ == "__main__":
    main()
