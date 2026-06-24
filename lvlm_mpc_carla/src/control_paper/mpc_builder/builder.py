"""High-level MPC Builder orchestration aligned with the paper.

Implements the blue section of Fig. 2:
  Task Cmd T + TrafficContext → Primitive Assigner → P → MPC Composer → OCP_target
                                                                    ↓
                                                              MPC Switcher → OCP_solve → MPPI
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

from .models import (
    KinematicBicycleParams,
    adaptive_cruise_control_primitive,
    constant_speed_primitive,
    kinematic_bicycle_dynamics,
    lane_change_primitive,
    lane_keep_primitive,
    parallel_vehicle_primitive,
)
from .primitives import ComposedPrimitive, MPCPrimitive, compose_primitives
from .solvers import MPPIConfig, MPPISolver


# =========================================================================
# Traffic context (paper Section V-A)
# =========================================================================

@dataclass
class SurroundingVehicle:
    """State of one surrounding vehicle (Frenet coordinates)."""

    x: float          # longitudinal position [m]
    y: float          # lateral position [m]
    vx: float = 0.0   # longitudinal velocity [m/s]
    vy: float = 0.0   # lateral velocity [m/s]


@dataclass
class TrafficContext:
    """Environment information needed by the Primitive Assigner.

    Paper Section V-A: the assigner uses ego state, lane geometry, and
    surrounding-vehicle positions to select primitives.
    """

    current_lane_center_y: float = 0.0
    lane_width: float = 3.5
    surrounding_vehicles: list[SurroundingVehicle] = field(default_factory=list)

    # convenience helpers
    def lead_vehicle(self, ego_x: float, max_lookahead: float = 200.0) -> SurroundingVehicle | None:
        """Return the closest vehicle ahead of ego in the same lane, or None."""
        best: SurroundingVehicle | None = None
        best_dist = max_lookahead
        for v in self.surrounding_vehicles:
            dx = v.x - ego_x
            if 0 < dx < best_dist:
                best_dist = dx
                best = v
        return best


# =========================================================================
# Task command (paper formula 11)
# =========================================================================

@dataclass
class TaskCommand:
    """Symbolic driving task command.

    Paper eq. (11):  T ∈ {LANE_LEFT, IDLE, LANE_RIGHT}
    Extended with speed target for convenience (used when CS is selected).
    """

    lane_action: str = "IDLE"          # LANE_LEFT | IDLE | LANE_RIGHT
    target_speed: float = 12.0         # used by CS primitive

    # bounds (shared across primitives)
    lane_bounds: tuple[float, float] = (-3.5, 3.5)
    steer_bounds: tuple[float, float] = (-0.5, 0.5)
    accel_bounds: tuple[float, float] = (-3.0, 2.0)

    # ACC / PV parameters
    desired_gap: float = 15.0          # dacc  — target following gap
    safe_distance: float = 10.0        # dacc_safe, dpv_safe, dlc_safe


# =========================================================================
# Configuration & result
# =========================================================================

@dataclass
class MPCBuilderConfig:
    horizon: int = 20
    num_samples: int = 1024
    penalty_weight: float = 100.0
    # MPPI sampling variances (paper Section V-A: σa=2.0, σδ=0.01)
    acceleration_std: float = 2.0
    steering_std: float = 0.01
    temperature: float = 0.1
    enable_iocp: bool = False          # Phase 1: off; Phase 3: on
    max_iocp_steps: int = 50           # nmax in Algorithm 2


@dataclass
class MPCBuilderResult:
    solve_ocp: ComposedPrimitive
    control: np.ndarray
    cost: float
    trajectory: np.ndarray
    feasible: bool
    rejected: bool
    primitive_names: list[str] = field(default_factory=list)


# =========================================================================
# MPC Builder
# =========================================================================

class MPCBuilder:
    """Orchestrates Assigner → Composer → Switcher → MPPI Solver."""

    def __init__(self, config: MPCBuilderConfig | None = None):
        self.config = config or MPCBuilderConfig()
        self.solver = MPPISolver(
            MPPIConfig(
                horizon=self.config.horizon,
                num_samples=self.config.num_samples,
                penalty_weight=self.config.penalty_weight,
                steering_std=self.config.steering_std,
                acceleration_std=self.config.acceleration_std,
                temperature=self.config.temperature,
            )
        )
        self.previous_ocp: ComposedPrimitive | None = None
        self.previous_control_sequence: np.ndarray | None = None
        self.iocp_count = 0

    # ------------------------------------------------------------------
    # Primitive Assigner (paper Section V-A)
    # ------------------------------------------------------------------

    def assign_primitives(
        self,
        task: TaskCommand,
        context: TrafficContext | None = None,
        ego_x: float = 0.0,
    ) -> list[MPCPrimitive]:
        """Select primitives from the pool according to task command and traffic.

        Rules (paper Section V-A):
          Lateral:     T=IDLE → LK,  T=LANE_LEFT/RIGHT → LC
          Longitudinal: lead vehicle ≤ 2·dacc → ACC,  else → CS
          Safety:      every nearby vehicle → PV (up to 6)
        """
        if context is None:
            context = TrafficContext()

        primitives: list[MPCPrimitive] = []

        # --- lateral task primitive ------------------------------------
        if task.lane_action in ("LANE_LEFT", "LANE_RIGHT"):
            target_y = context.current_lane_center_y
            if task.lane_action == "LANE_LEFT":
                target_y += context.lane_width
            else:
                target_y -= context.lane_width

            lead = context.lead_vehicle(ego_x)
            lead_x = lead.x if lead else 100.0

            primitives.append(
                lane_change_primitive(
                    target_lane_y=target_y,
                    y_bounds=task.lane_bounds,
                    steer_bounds=task.steer_bounds,
                    lead_vehicle_x=lead_x,
                    safe_lc_distance=task.safe_distance,
                )
            )
        else:  # IDLE
            primitives.append(
                lane_keep_primitive(
                    lane_center_y=context.current_lane_center_y,
                    y_bounds=task.lane_bounds,
                    steer_bounds=task.steer_bounds,
                )
            )

        # --- longitudinal task primitive -------------------------------
        lead = context.lead_vehicle(ego_x)
        if lead is not None and (lead.x - ego_x) <= 2.0 * task.desired_gap:
            primitives.append(
                adaptive_cruise_control_primitive(
                    lead_vehicle_x=lead.x,
                    desired_gap=task.desired_gap,
                    accel_bounds=task.accel_bounds,
                    safe_distance=task.safe_distance,
                )
            )
        else:
            primitives.append(
                constant_speed_primitive(
                    target_speed=task.target_speed,
                    accel_bounds=task.accel_bounds,
                )
            )

        # --- safety primitives (PV) — adjacent-lane vehicles only -----
        # Lead vehicle is handled by LC/ACC; PV guards lateral neighbours.
        for sv in context.surrounding_vehicles[:6]:
            # Only add PV if the vehicle is in a *different* lateral band
            if abs(sv.y - context.current_lane_center_y) < context.lane_width * 0.4:
                continue   # same lane → handled by ACC / LC
            primitives.append(
                parallel_vehicle_primitive(
                    pv_x=sv.x,
                    pv_y=sv.y,
                    pv_vx=sv.vx,
                    pv_vy=sv.vy,
                    safe_distance=task.safe_distance,
                )
            )

        return primitives

    # ------------------------------------------------------------------
    # Composer
    # ------------------------------------------------------------------

    def compose(self, primitives: Iterable[MPCPrimitive]) -> ComposedPrimitive:
        return compose_primitives(primitives)

    def build_task_ocp(
        self,
        task: TaskCommand,
        context: TrafficContext | None = None,
        ego_x: float = 0.0,
    ) -> ComposedPrimitive:
        primitives = self.assign_primitives(task, context, ego_x=ego_x)
        return self.compose(primitives)

    # ------------------------------------------------------------------
    # Feasibility check (formula 10)
    # ------------------------------------------------------------------

    def check_feasibility(
        self,
        ocp: ComposedPrimitive,
        x0: np.ndarray,
    ) -> bool:
        """Paper formula (10): horizon simulation with previous control sequence.

        Uses {û(1|t−1), …, û(N−1|t−1)} to roll out the *task* OCP dynamics
        and check its constraints at every step.  Returns True only if the
        full roll-out is constraint-satisfying.

        Notes:
          - A small tolerance (1e-6) is used for inequality checks because the
            MPPI solver uses soft/penalty constraints, which can produce
            numerically-tiny violations that are practically feasible.
          - When the task OCP has not changed (same primitive names), the
            previous control sequence was optimised for the same constraints
            and the check is skipped as trivially satisfied.
        """
        if self.previous_control_sequence is None:
            return True   # first step — no previous sequence to check with

        # Same OCP → trivially feasible (previous controls were optimised for it)
        name_a = self.previous_ocp.name if self.previous_ocp else ""
        name_b = ocp.name if ocp else ""
        if name_a == name_b:
            return True

        U_prev = self.previous_control_sequence   # shape (H, m)
        x = np.asarray(x0, dtype=float).reshape(-1)

        # û(0|t−1) was already applied; use û(1) … û(H−1)
        for k in range(len(U_prev) - 1):
            u = U_prev[k + 1]
            x = np.asarray(ocp.dynamics(x, u), dtype=float).reshape(-1)
            g, h = ocp.constraints(x, u)
            # small tolerance: MPPI uses soft constraints → tiny violations
            if not (np.all(g <= 1e-6) and np.allclose(h, 0.0, atol=1e-6)):
                return False
        return True

    # ------------------------------------------------------------------
    # iOCP (Algorithm 1)
    # ------------------------------------------------------------------

    def intermediate_ocp(
        self, previous: ComposedPrimitive, target: ComposedPrimitive
    ) -> ComposedPrimitive:
        """Algorithm 1:  iOCP(O_t,prev, O_t,target).

        - State space:  X_AB = X_A × X_B  (Cartesian product)
        - Dynamics:     f_AB = (f_A, f_B)
        - Cost:         J_AB = J_A + Σ ρ·penalty(g_B, h_B)   (B→penalty)
        - Constraints:  g_AB = g_A,  h_AB = h_A                (A only)
        """
        dim_a = previous.state_dim
        dim_b = target.state_dim
        dim_ab = dim_a + dim_b
        rho = self.config.penalty_weight

        # --- combined dynamics (formula ① + ⑤) -------------------------
        def iocp_dynamics(x: np.ndarray, u: np.ndarray) -> np.ndarray:
            x_a = x[:dim_a]
            x_b = x[dim_a:dim_ab]
            dx_a = np.asarray(previous.dynamics(x_a, u), dtype=float).reshape(-1)
            dx_b = np.asarray(target.dynamics(x_b, u), dtype=float).reshape(-1)
            return np.concatenate([dx_a, dx_b])

        # --- combined cost (formula ⑩ + ⑫) -----------------------------
        def iocp_cost(x: np.ndarray, u: np.ndarray) -> float:
            x_a = x[:dim_a]
            x_b = x[dim_a:dim_ab]
            # J_A — previous OCP cost
            j_a = float(previous.stage_cost(x_a, u))
            # penalty from B's constraints
            g_b, h_b = target.constraints(x_b, u)
            penalty = rho * (
                float(np.sum(np.maximum(g_b, 0.0) ** 2))
                + float(np.sum(h_b ** 2))
            )
            return j_a + penalty

        # --- constraints: A only (formula ⑬ + ⑮) -----------------------
        def iocp_g(x: np.ndarray, u: np.ndarray) -> np.ndarray:
            x_a = x[:dim_a]
            g_a, _ = previous.constraints(x_a, u)
            return g_a

        def iocp_h(x: np.ndarray, u: np.ndarray) -> np.ndarray:
            x_a = x[:dim_a]
            _, h_a = previous.constraints(x_a, u)
            return h_a

        iocp_primitive = MPCPrimitive(
            name=f"iOCP({previous.name},{target.name})",
            state_dim=dim_ab,
            dynamics=iocp_dynamics,
            stage_cost=iocp_cost,
            inequality_constraints=iocp_g,
            equality_constraints=iocp_h,
        )
        return ComposedPrimitive((iocp_primitive,))

    # ------------------------------------------------------------------
    # MPC Switcher (Algorithm 2)
    # ------------------------------------------------------------------

    def switch_ocp(
        self, task_ocp: ComposedPrimitive, x0: np.ndarray
    ) -> tuple[ComposedPrimitive, bool]:
        """Algorithm 2: feasibility → accept / iOCP / reject.

        Side effects:
          - self.previous_ocp is updated ONLY when the target OCP is accepted
            (feasible or iOCP disabled).  During iOCP it stays at the pre-switch
            OCP so that intermediate_ocp never nests iOCP inside iOCP.
          - self.iocp_count tracks consecutive iOCP steps; reset on acceptance.
        """
        if not self.config.enable_iocp:
            self.previous_ocp = task_ocp
            return task_ocp, False

        # First step: no previous OCP to compare against
        if self.previous_ocp is None:
            self.previous_ocp = task_ocp
            self.iocp_count = 0
            return task_ocp, False

        # Check feasibility of target OCP with previous control sequence
        feasible = self.check_feasibility(task_ocp, x0)
        if feasible:
            self.previous_ocp = task_ocp
            self.iocp_count = 0
            return task_ocp, False

        # Infeasible → try iOCP transition (up to nmax consecutive steps)
        # IMPORTANT: do NOT update self.previous_ocp here — keep the
        # pre-switch OCP so that subsequent intermediate_ocp calls do not
        # nest iOCP inside iOCP (which would blow up state dimensions).
        if self.iocp_count < self.config.max_iocp_steps:
            self.iocp_count += 1
            return self.intermediate_ocp(self.previous_ocp, task_ocp), False

        # iOCP limit exceeded → reject, fall back to previous OCP
        self.iocp_count = 0
        return self.previous_ocp, True

    # ------------------------------------------------------------------
    # State augmentation (for PV primitives)
    # ------------------------------------------------------------------

    @staticmethod
    def _augment_x0(x0: np.ndarray, ocp: ComposedPrimitive) -> np.ndarray:
        """Ensure x0 dimension matches ocp.state_dim.

        - Plain ego OCPs (4D): no-op.
        - PV primitives present: append PV states from metadata.
        - iOCP (dim_a + dim_b): replicate ego state to fill the gap.
        """
        x0_flat = np.asarray(x0, dtype=float).reshape(-1)
        if ocp.state_dim <= len(x0_flat):
            return x0_flat

        # Gather known extra states (PV)
        extra: list[float] = []
        for p in ocp.primitives:
            pv_state = p.metadata.get("pv_state")
            if pv_state is not None:
                extra.extend(pv_state)

        # If there's still a gap (e.g. iOCP doubling), replicate x0
        remaining = ocp.state_dim - len(x0_flat) - len(extra)
        while remaining > 0:
            chunk = min(len(x0_flat), remaining)
            extra.extend(x0_flat[:chunk].tolist())
            remaining -= chunk

        return np.concatenate([x0_flat, np.asarray(extra, dtype=float)])

    # ------------------------------------------------------------------
    # Main solve entry point
    # ------------------------------------------------------------------

    def solve(
        self,
        task: TaskCommand,
        x0: np.ndarray,
        u_init: np.ndarray | None = None,
        context: TrafficContext | None = None,
    ) -> MPCBuilderResult:
        x0_flat = np.asarray(x0, dtype=float).reshape(-1)
        ego_x = float(x0_flat[0])  # longitudinal position for Assigner

        task_ocp = self.build_task_ocp(task, context, ego_x=ego_x)

        # Augment x0 if the OCP has PV primitives (state_dim > 4)
        x_aug = self._augment_x0(x0_flat, task_ocp)

        solve_ocp, rejected = self.switch_ocp(task_ocp, x_aug)

        # iOCP may further enlarge the state → augment again if needed
        x_aug = self._augment_x0(x_aug, solve_ocp)

        sol = self.solver.solve(solve_ocp, x_aug, u_init=u_init)

        # Only update previous_ocp when NOT in iOCP mode.
        # switch_ocp already handles previous_ocp updates for accept/reject;
        # during iOCP, previous_ocp stays at the pre-switch OCP to avoid nesting.
        if not solve_ocp.primitives[0].name.startswith("iOCP"):
            self.previous_ocp = solve_ocp

        self.previous_control_sequence = np.asarray(sol["control_sequence"], dtype=float)
        return MPCBuilderResult(
            solve_ocp=solve_ocp,
            control=np.asarray(sol["control"], dtype=float),
            cost=float(sol["cost"]),
            trajectory=np.asarray(sol["trajectory"], dtype=float),
            feasible=bool(sol["feasible"]),
            rejected=rejected,
            primitive_names=[p.name for p in solve_ocp.primitives],
        )
