"""Paper-aligned motion primitives for autonomous driving MPC Builder.

Implements Table I from the paper — six primitives:
  KBM  – ego dynamics (kinematic bicycle model)
  LK   – lane keep
  LC   – lane change
  CS   – constant speed
  ACC  – adaptive cruise control
  PV   – parallel vehicle (safety)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from .primitives import MPCPrimitive

ArrayLike = np.ndarray


# =========================================================================
# Kinematic Bicycle Model (shared by all ego-related primitives)
# =========================================================================

@dataclass(frozen=True)
class KinematicBicycleParams:
    wheelbase: float = 2.7
    dt: float = 0.05


def kinematic_bicycle_dynamics(
    params: KinematicBicycleParams | None = None,
) -> Callable[[ArrayLike, ArrayLike], ArrayLike]:
    """Discrete-time KBM:  Xego = [x, y, θ, v]  →  Xego_next."""
    if params is None:
        params = KinematicBicycleParams()

    def _dyn(x: ArrayLike, u: ArrayLike) -> ArrayLike:
        x = np.asarray(x, dtype=float).reshape(-1)
        u = np.asarray(u, dtype=float).reshape(-1)
        if x.size < 4:
            raise ValueError("Ego state must be [x, y, theta, v].")
        if u.size < 2:
            raise ValueError("Control must be [acceleration, steering].")
        px, py, theta, v = x[:4]
        a, delta = u[:2]
        beta = np.arctan(np.tan(delta) / 2.0)
        px_next = px + v * np.cos(theta + beta) * params.dt
        py_next = py + v * np.sin(theta + beta) * params.dt
        theta_next = theta + (v / params.wheelbase) * np.tan(delta) * params.dt
        v_next = v + a * params.dt
        return np.array([px_next, py_next, theta_next, v_next], dtype=float)

    return _dyn


# =========================================================================
# Table I — Primitive factories
# =========================================================================

# --- KBM: ego dynamics only, no cost, no constraints --------------------

def kinematic_bicycle_primitive(
    params: KinematicBicycleParams | None = None,
) -> MPCPrimitive:
    """Table I "KBM": provides ego dynamics f_KBM, zero cost, no constraints."""
    if params is None:
        params = KinematicBicycleParams()
    return MPCPrimitive(
        name="KBM",
        state_dim=4,
        dynamics=kinematic_bicycle_dynamics(params),
        stage_cost=lambda x, u: 0.0,
        inequality_constraints=lambda x, u: np.zeros(0),
        equality_constraints=lambda x, u: np.zeros(0),
        metadata={"type": "ego_dynamics"},
    )


# --- LK: lane keep ------------------------------------------------------

def lane_keep_primitive(
    lane_center_y: float,
    y_bounds: tuple[float, float],
    steer_bounds: tuple[float, float],
    weights: tuple[float, float, float, float, float] = (10.0, 2.0, 0.5, 0.1, 0.1),
) -> MPCPrimitive:
    """Table I "LK": keep current lane centre, zero heading, small steer.

    J = {‖y‖² + ‖θ‖² + ‖θ̇‖² + ‖δ‖² + ‖δ̇‖²}_Qlk   (θ̇, δ̇ ≈ 0 in discrete)
    g = [ymin - y,  y - ymax,  δmin - δ,  δ - δmax]
    """
    qy, qtheta, _qdtheta, qdelta, _qddelta = weights

    def stage_cost(x: ArrayLike, u: ArrayLike) -> float:
        x = np.asarray(x, dtype=float).reshape(-1)
        u = np.asarray(u, dtype=float).reshape(-1)
        y = float(x[1])
        theta = float(x[2])
        delta = float(u[1])
        return float(qy * (y - lane_center_y) ** 2 + qtheta * theta**2 + qdelta * delta**2)

    def g(x: ArrayLike, u: ArrayLike) -> ArrayLike:
        x = np.asarray(x, dtype=float).reshape(-1)
        u = np.asarray(u, dtype=float).reshape(-1)
        y = float(x[1])
        delta = float(u[1])
        return np.array([
            y_bounds[0] - y,
            y - y_bounds[1],
            steer_bounds[0] - delta,
            delta - steer_bounds[1],
        ], dtype=float)

    return MPCPrimitive(
        name="LK",
        state_dim=4,
        dynamics=kinematic_bicycle_dynamics(KinematicBicycleParams()),
        stage_cost=stage_cost,
        inequality_constraints=g,
        metadata={"type": "lateral_task"},
    )


# --- LC: lane change ----------------------------------------------------

def lane_change_primitive(
    target_lane_y: float,
    y_bounds: tuple[float, float],
    steer_bounds: tuple[float, float],
    lead_vehicle_x: float = 100.0,
    safe_lc_distance: float = 10.0,
    weights: tuple[float, float, float, float, float] = (10.0, 2.0, 0.5, 0.1, 0.1),
) -> MPCPrimitive:
    """Table I "LC": track *target* lane centre with safety distance to lead vehicle.

    J = {‖y - yref‖² + ‖θ‖² + ‖θ̇‖² + ‖δ‖² + ‖δ̇‖²}_Qlc
    g = [ymin - y,  y - ymax,  δmin - δ,  δ - δmax,  dˡᶜ_safe - ‖x - xpv‖]
    """
    qy, qtheta, _qdtheta, qdelta, _qddelta = weights

    def stage_cost(x: ArrayLike, u: ArrayLike) -> float:
        x = np.asarray(x, dtype=float).reshape(-1)
        u = np.asarray(u, dtype=float).reshape(-1)
        y = float(x[1])
        theta = float(x[2])
        delta = float(u[1])
        return float(qy * (y - target_lane_y) ** 2 + qtheta * theta**2 + qdelta * delta**2)

    def g(x: ArrayLike, u: ArrayLike) -> ArrayLike:
        x = np.asarray(x, dtype=float).reshape(-1)
        u = np.asarray(u, dtype=float).reshape(-1)
        y = float(x[1])
        px = float(x[0])
        delta = float(u[1])
        return np.array([
            y_bounds[0] - y,
            y - y_bounds[1],
            steer_bounds[0] - delta,
            delta - steer_bounds[1],
            safe_lc_distance - abs(px - lead_vehicle_x),
        ], dtype=float)

    return MPCPrimitive(
        name="LC",
        state_dim=4,
        dynamics=kinematic_bicycle_dynamics(KinematicBicycleParams()),
        stage_cost=stage_cost,
        inequality_constraints=g,
        metadata={"type": "lateral_task"},
    )


# --- CS: constant speed -------------------------------------------------

def constant_speed_primitive(
    target_speed: float,
    accel_bounds: tuple[float, float],
    weights: tuple[float, float, float] = (0.3, 0.2, 0.05),
) -> MPCPrimitive:
    """Table I "CS": track reference speed.

    J = {‖v - vref‖² + ‖a‖² + ‖ȧ‖²}_Qcs   (ȧ ≈ 0 in discrete)
    g = [amin - a,  a - amax]
    """
    qv, qa, _qda = weights

    def stage_cost(x: ArrayLike, u: ArrayLike) -> float:
        x = np.asarray(x, dtype=float).reshape(-1)
        u = np.asarray(u, dtype=float).reshape(-1)
        v = float(x[3])
        a = float(u[0])
        return float(qv * (v - target_speed) ** 2 + qa * a**2)

    def g(x: ArrayLike, u: ArrayLike) -> ArrayLike:
        u = np.asarray(u, dtype=float).reshape(-1)
        a = float(u[0])
        return np.array([accel_bounds[0] - a, a - accel_bounds[1]], dtype=float)

    return MPCPrimitive(
        name="CS",
        state_dim=4,
        dynamics=kinematic_bicycle_dynamics(KinematicBicycleParams()),
        stage_cost=stage_cost,
        inequality_constraints=g,
        metadata={"type": "longitudinal_task"},
    )


# --- ACC: adaptive cruise control ---------------------------------------

def adaptive_cruise_control_primitive(
    lead_vehicle_x: float,
    desired_gap: float,
    accel_bounds: tuple[float, float],
    safe_distance: float,
    weights: tuple[float, float, float] = (1.0, 0.1, 0.05),
) -> MPCPrimitive:
    """Table I "ACC": keep desired gap to lead vehicle.

    J = {‖x - xpv - dacc‖² + ‖a‖² + ‖ȧ‖²}_Qacc   (ȧ ≈ 0 in discrete)
    g = [amin - a,  a - amax,  dacc_safe - ‖x - xpv‖]
    """
    qx, qa, _qda = weights

    def stage_cost(x: ArrayLike, u: ArrayLike) -> float:
        x = np.asarray(x, dtype=float).reshape(-1)
        u = np.asarray(u, dtype=float).reshape(-1)
        px = float(x[0])
        a = float(u[0])
        gap_error = px - lead_vehicle_x + desired_gap
        return float(qx * gap_error**2 + qa * a**2)

    def g(x: ArrayLike, u: ArrayLike) -> ArrayLike:
        x = np.asarray(x, dtype=float).reshape(-1)
        u = np.asarray(u, dtype=float).reshape(-1)
        px = float(x[0])
        a = float(u[0])
        return np.array([
            accel_bounds[0] - a,
            a - accel_bounds[1],
            safe_distance - abs(px - lead_vehicle_x),
        ], dtype=float)

    return MPCPrimitive(
        name="ACC",
        state_dim=4,
        dynamics=kinematic_bicycle_dynamics(KinematicBicycleParams()),
        stage_cost=stage_cost,
        inequality_constraints=g,
        metadata={"type": "longitudinal_task"},
    )


# --- PV: parallel vehicle (safety) --------------------------------------

def parallel_vehicle_primitive(
    pv_x: float,
    pv_y: float,
    pv_vx: float,
    pv_vy: float,
    safe_distance: float = 5.0,
    dt: float = 0.05,
) -> MPCPrimitive:
    """Table I "PV": safety distance constraint to a surrounding vehicle.

    State space:  Xpv = [xpv, ypv, vpv_x, vpv_y]
    Dynamics:     fpv (constant velocity)
    J = 0
    g = [dpv_safe − √((x−xpv)² + (y−ypv)²)]

    The combined state when composed is  [ego(4), pv(4)] = 8-d.
    """
    _pv_x, _pv_y, pv_vx, pv_vy = pv_x, pv_y, pv_vx, pv_vy
    kbm_params = KinematicBicycleParams(dt=dt)

    def combined_dynamics(x: ArrayLike, u: ArrayLike) -> ArrayLike:
        """Ego: KBM  |  PV: constant velocity."""
        x = np.asarray(x, dtype=float).reshape(-1)
        u = np.asarray(u, dtype=float).reshape(-1)
        ego = x[:4]
        pv = x[4:8]
        ego_next = kinematic_bicycle_dynamics(kbm_params)(ego, u)
        pv_next = np.array([
            pv[0] + pv[2] * dt,
            pv[1] + pv[3] * dt,
            pv[2],
            pv[3],
        ], dtype=float)
        return np.concatenate([ego_next, pv_next])

    def g(x: ArrayLike, u: ArrayLike) -> ArrayLike:
        """Euclidean distance from ego to PV must be ≥ safe_distance."""
        x = np.asarray(x, dtype=float).reshape(-1)
        ego_x, ego_y = float(x[0]), float(x[1])
        pvx, pvy = float(x[4]), float(x[5])
        dist = np.sqrt((ego_x - pvx) ** 2 + (ego_y - pvy) ** 2)
        return np.array([safe_distance - dist], dtype=float)

    return MPCPrimitive(
        name="PV",
        state_dim=8,  # ego(4) + pv(4)
        dynamics=combined_dynamics,
        stage_cost=lambda x, u: 0.0,
        inequality_constraints=g,
        metadata={"type": "safety", "pv_state": (pv_x, pv_y, pv_vx, pv_vy)},
    )
