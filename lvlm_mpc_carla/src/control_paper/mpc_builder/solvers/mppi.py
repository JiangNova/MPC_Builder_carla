"""A lightweight MPPI-style solver for the paper-aligned MPC Builder.

Implements formula (12) from the paper:  C_k = J_k + ρ · 𝟙[g > 0]
Each sample draws an independent noise *sequence* over the horizon,
rolls out the dynamics, and the best sequence is selected.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..primitives import ComposedPrimitive


@dataclass
class MPPIConfig:
    horizon: int = 20
    num_samples: int = 256
    dt: float = 0.05
    acceleration_std: float = 0.8
    steering_std: float = 0.03
    penalty_weight: float = 100.0
    temperature: float = 0.1          # λ for exponential weighting (0 = pure best-sample)
    seed: int | None = 42             # fixed seed for reproducibility
    control_dim: int = 2              # (acceleration, steering)


class MPPISolver:
    """Lightweight MPPI / random-shooting solver.

    For each sample, an independent noise *sequence* is drawn across the
    horizon so the solver can express time-varying controls.  When
    ``temperature > 0`` the final control sequence is an exponentially-
    weighted average of all samples; otherwise the single best sample is
    returned (pure random shooting).
    """

    def __init__(self, config: MPPIConfig | None = None):
        self.config = config or MPPIConfig()
        self._rng = np.random.default_rng(self.config.seed)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def solve(
        self,
        ocp: ComposedPrimitive,
        x0: np.ndarray,
        u_init: np.ndarray | None = None,
    ) -> dict[str, np.ndarray | float | bool]:
        H = self.config.horizon
        K = self.config.num_samples
        m = self.config.control_dim

        # -- build / warm-start the nominal control sequence -------------
        U_nom = self._warm_start(u_init, H, m)

        # -- storage ----------------------------------------------------
        all_costs = np.full(K, np.inf)
        all_noises: list[np.ndarray] = []     # each E_k shape (H, m)
        best_cost = float("inf")
        best_U = U_nom.copy()
        best_traj: list[np.ndarray] = []

        # -- sample loop ------------------------------------------------
        for k in range(K):
            E = self._sample_noise(H, m, self._rng)    # (H, m)
            U_candidate = U_nom + E                    # (H, m)

            cost, traj = self._rollout(ocp, x0, U_candidate)
            all_costs[k] = cost
            all_noises.append(E)

            if cost < best_cost:
                best_cost = cost
                best_U = U_candidate.copy()
                best_traj = traj

        # -- MPPI update (weighted average) when temperature > 0 --------
        if self.config.temperature > 0 and K > 0:
            U_nom = self._weighted_update(U_nom, all_noises, all_costs)
            # re-rollout the blended sequence for a consistent trajectory
            final_cost, final_traj = self._rollout(ocp, x0, U_nom)
            if final_cost < best_cost:
                best_cost = final_cost
                best_U = U_nom.copy()
                best_traj = final_traj

        return {
            "control": best_U[0].copy(),                  # first action u*₀
            "control_sequence": best_U.copy(),            # full sequence U* (H×m)
            "cost": best_cost,
            "trajectory": np.asarray(best_traj, dtype=float),
            "feasible": bool(np.isfinite(best_cost)),
        }

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _warm_start(
        self, u_init: np.ndarray | None, horizon: int, dim: int
    ) -> np.ndarray:
        """Build a (horizon, dim) nominal sequence from ``u_init``.

        * ``None``        → zero sequence
        * 1-D ``(dim,)``  → tile across horizon
        * 2-D ``(H, dim)`` → pad / truncate to ``horizon``
        """
        if u_init is None:
            return np.zeros((horizon, dim), dtype=float)

        u = np.asarray(u_init, dtype=float)
        if u.ndim == 1:
            return np.tile(u.reshape(1, -1), (horizon, 1))
        # 2-D array
        h_in = u.shape[0]
        if h_in >= horizon:
            return u[:horizon].copy()
        # pad by repeating the last control
        tail = np.tile(u[-1:], (horizon - h_in, 1))
        return np.vstack([u, tail])

    def _sample_noise(
        self, horizon: int, dim: int, rng: np.random.Generator
    ) -> np.ndarray:
        """Independent Gaussian noise for each time step."""
        std = np.array([self.config.acceleration_std, self.config.steering_std])
        return rng.normal(0.0, std[:dim], size=(horizon, dim))

    def _rollout(
        self,
        ocp: ComposedPrimitive,
        x0: np.ndarray,
        U: np.ndarray,
    ) -> tuple[float, list[np.ndarray]]:
        """Simulate the open-loop trajectory and return (total_cost, traj)."""
        x = np.asarray(x0, dtype=float).reshape(-1)
        total = 0.0
        traj = [x.copy()]

        for t in range(U.shape[0]):
            u = U[t]
            x = np.asarray(ocp.dynamics(x, u), dtype=float).reshape(-1)
            stage_cost = float(ocp.stage_cost(x, u))
            g, h = ocp.constraints(x, u)
            penalty = self.config.penalty_weight * (
                float(np.sum(np.maximum(g, 0.0) ** 2)) + float(np.sum(h**2))
            )
            total += stage_cost + penalty
            traj.append(x.copy())

        return total, traj

    def _weighted_update(
        self,
        U_nom: np.ndarray,
        noises: list[np.ndarray],
        costs: np.ndarray,
    ) -> np.ndarray:
        """Exponential-weighted MPPI update:  U += Σ w_k · E_k."""
        beta = np.min(costs)
        weights = np.exp(-(costs - beta) / self.config.temperature)
        total_w = np.sum(weights)
        if total_w <= 0:
            return U_nom

        weights /= total_w
        U_new = U_nom.copy()
        for w, E in zip(weights, noises):
            U_new += w * E
        return U_new
