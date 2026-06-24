"""Closed-loop MPC Builder demo with receding-horizon MPPI.

Runs N_sim steps in closed loop, plotting the full state / control history
so we can verify lane-keeping and speed-tracking behaviour over time.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from control_paper.mpc_builder import (
    MPCBuilder,
    MPCBuilderConfig,
    TaskCommand,
    TrafficContext,
)
from control_paper.mpc_builder.models import KinematicBicycleParams, kinematic_bicycle_dynamics


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SIM_STEPS = 100          # 100 × 0.05 s = 5 s closed-loop
DT = 0.05


def heuristic_warm_start(x: np.ndarray, task: TaskCommand, horizon: int) -> np.ndarray:
    """Compute a simple heuristic control sequence as MPPI warm-start.

    - acceleration: proportional to speed error, clipped to bounds
    - steering: proportional to lateral + heading error (P-controller style)
    """
    y = float(x[1])
    theta = float(x[2])
    v = float(x[3])

    # Heuristic steering: steer toward lane center, counter heading error
    k_y = 0.5
    k_theta = 0.3
    delta_heu = -k_y * y - k_theta * theta
    delta_heu = float(np.clip(delta_heu, task.steer_bounds[0], task.steer_bounds[1]))

    # Heuristic acceleration: reach target speed over the horizon
    a_heu = (task.target_speed - v) / (horizon * DT)
    a_heu = float(np.clip(a_heu, task.accel_bounds[0], task.accel_bounds[1]))

    U = np.tile(np.array([a_heu, delta_heu], dtype=float), (horizon, 1))
    return U


def run_closed_loop() -> dict:
    """Run MPC Builder in receding-horizon closed-loop simulation."""
    # Paper variances: σa=2.0, σδ=0.01 — now defaults in MPCBuilderConfig
    builder = MPCBuilder(MPCBuilderConfig())

    task = TaskCommand(
        lane_action="IDLE",
        target_speed=12.0,
        lane_bounds=(-3.5, 3.5),
        steer_bounds=(-0.5, 0.5),
        accel_bounds=(-3.0, 2.0),
    )
    context = TrafficContext(current_lane_center_y=0.0)

    # initial state: slight lateral offset, below target speed
    x = np.array([0.0, 0.5, 0.05, 6.0], dtype=float)
    dyn = kinematic_bicycle_dynamics(KinematicBicycleParams(dt=DT))

    traj = [x.copy()]
    controls: list[np.ndarray] = []
    costs: list[float] = []
    rejected_flags: list[bool] = []

    # Use heuristic warm-start for the first step
    u_prev = heuristic_warm_start(x, task, builder.config.horizon)

    for step in range(SIM_STEPS):
        result = builder.solve(task, x, u_init=u_prev, context=context)
        u = np.asarray(result.control, dtype=float)

        controls.append(u.copy())
        costs.append(float(result.cost))
        rejected_flags.append(bool(result.rejected))

        # advance dynamics
        x = dyn(x, u)
        traj.append(x.copy())

        # warm-start for next MPC step: heuristic based on current state
        u_prev = heuristic_warm_start(x, task, builder.config.horizon)

    traj_arr = np.asarray(traj, dtype=float)
    ctrl_arr = np.asarray(controls, dtype=float)

    return {
        "sim_steps": SIM_STEPS,
        "dt": DT,
        "trajectory": traj_arr.tolist(),
        "controls": ctrl_arr.tolist(),
        "costs": costs,
        "rejected_flags": [int(r) for r in rejected_flags],
        "final_state": traj_arr[-1].tolist(),
        "initial_state": traj_arr[0].tolist(),
        "task": {
            "lane_action": task.lane_action,
            "target_speed": task.target_speed,
            "lane_bounds": list(task.lane_bounds),
            "steer_bounds": list(task.steer_bounds),
            "accel_bounds": list(task.accel_bounds),
        },
        "context": {
            "current_lane_center_y": context.current_lane_center_y,
            "lane_width": context.lane_width,
        },
    }


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

def save_visualizations(output: dict, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    traj = np.asarray(output["trajectory"], dtype=float)
    ctrl = np.asarray(output["controls"], dtype=float)
    costs = np.asarray(output["costs"], dtype=float)
    task = output["task"]
    ctx = output.get("context", {})
    dt = output["dt"]
    lane_center = ctx.get("current_lane_center_y", 0.0)
    t = np.arange(traj.shape[0]) * dt         # state times
    t_ctrl = np.arange(ctrl.shape[0]) * dt     # control times

    x, y, theta, v = traj[:, 0], traj[:, 1], traj[:, 2], traj[:, 3]
    a, delta = ctrl[:, 0], ctrl[:, 1]

    # --- Summary 4-panel -------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    ax = axes[0, 0]
    ax.plot(x, y, linewidth=1.5, label="closed-loop traj")
    ax.scatter([x[0]], [y[0]], color="red", s=80, zorder=5, label="start")
    ax.scatter([x[-1]], [y[-1]], color="blue", s=80, zorder=5, label="end")
    ax.axhline(lane_center, color="green", linestyle="--", label="target lane center")
    ax.axhline(task["lane_bounds"][0], color="gray", linestyle=":", label="lane bounds")
    ax.axhline(task["lane_bounds"][1], color="gray", linestyle=":")
    ax.set_title("Closed-loop Trajectory (XY)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, y, linewidth=1.5, label="y")
    ax.axhline(lane_center, color="green", linestyle="--", label="target")
    ax.fill_between(t, task["lane_bounds"][0], task["lane_bounds"][1],
                    color="gray", alpha=0.1, label="lane bounds")
    ax.set_title("Lateral Error")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("y [m]")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(t, v, linewidth=1.5, label="speed")
    ax.axhline(task["target_speed"], color="green", linestyle="--", label="target")
    ax.set_title("Speed Tracking")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("v [m/s]")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t_ctrl, a, linewidth=1.5, label="acceleration")
    ax.plot(t_ctrl, delta, linewidth=1.5, label="steering")
    ax.axhline(task["accel_bounds"][0], color="gray", linestyle=":", label="accel bounds")
    ax.axhline(task["accel_bounds"][1], color="gray", linestyle=":")
    ax.set_title("Control Inputs")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("value")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle("MPC Builder — Closed-loop Demo (MPPI, receding horizon)", fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_dir / "mpc_builder_closed_loop_summary.png", dpi=200)
    plt.close(fig)

    # --- State evolution -------------------------------------------------
    fig2, axes2 = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    labels = ["x [m]", "y [m]", "theta [rad]", "v [m/s]"]
    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
    for i, (ax_i, lbl, col, data) in enumerate(zip(axes2, labels, colors, [x, y, theta, v])):
        ax_i.plot(t, data, linewidth=1.5, color=col, label=lbl)
        ax_i.set_ylabel(lbl)
        ax_i.legend(loc="upper right")
        ax_i.grid(True, alpha=0.3)
    axes2[1].axhline(lane_center, color="green", linestyle="--", alpha=0.7)
    axes2[3].axhline(task["target_speed"], color="green", linestyle="--", alpha=0.7, label="target speed")
    axes2[-1].set_xlabel("time [s]")
    fig2.suptitle("State Evolution (Closed-loop)")
    fig2.tight_layout()
    fig2.savefig(out_dir / "mpc_builder_closed_loop_states.png", dpi=200)
    plt.close(fig2)

    # --- Cost & feasibility ----------------------------------------------
    fig3, axes3 = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes3[0].plot(t_ctrl, costs, linewidth=1.5, color="tab:purple")
    axes3[0].set_ylabel("MPC cost")
    axes3[0].set_title("Cost per MPC Step")
    axes3[0].grid(True, alpha=0.3)

    rejected = output.get("rejected_flags", [])
    if any(rejected):
        axes3[1].fill_between(t_ctrl, 0, rejected, step="mid", color="red", alpha=0.4, label="rejected")
    axes3[1].set_ylabel("rejected flag")
    axes3[1].set_xlabel("time [s]")
    axes3[1].set_ylim(-0.1, 1.1)
    axes3[1].grid(True, alpha=0.3)
    fig3.suptitle("Solver Diagnostics")
    fig3.tight_layout()
    fig3.savefig(out_dir / "mpc_builder_closed_loop_diagnostics.png", dpi=200)
    plt.close(fig3)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    # repo root = lvlm_mpc_carla  (parents[4] from experiments/)
    project_root = Path(__file__).resolve().parents[4]
    out_dir = project_root / "results" / "images" / "demo_run"
    out_dir.mkdir(parents=True, exist_ok=True)

    output = run_closed_loop()

    # print summary to stdout
    summary = {
        "initial_state": output["initial_state"],
        "final_state": output["final_state"],
        "y_final": output["trajectory"][-1][1],
        "v_final": output["trajectory"][-1][3],
        "mean_accel": float(np.mean(np.asarray(output["controls"])[:, 0])),
        "mean_steer": float(np.mean(np.asarray(output["controls"])[:, 1])),
        "mean_cost": float(np.mean(output["costs"])),
        "any_rejected": any(output["rejected_flags"]),
        "saved_to": str(out_dir),
    }
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    save_visualizations(output, out_dir)

    # also save raw data as JSON for later analysis
    data_path = out_dir / "closed_loop_data.json"
    with open(data_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2, ensure_ascii=False, default=float)
    print(f"Raw data saved to: {data_path}")


if __name__ == "__main__":
    main()
