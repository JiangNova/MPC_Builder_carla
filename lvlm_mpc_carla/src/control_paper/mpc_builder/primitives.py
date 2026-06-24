"""MPC primitive definitions aligned with the paper.

This module provides a compact, reusable representation of MPC primitives
and the composition operator used by MPC Builder.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Iterable, Mapping

import numpy as np

ArrayLike = np.ndarray
DynamicsFn = Callable[[ArrayLike, ArrayLike], ArrayLike]
CostFn = Callable[[ArrayLike, ArrayLike], float]
ConstraintFn = Callable[[ArrayLike, ArrayLike], np.ndarray]


@dataclass(frozen=True)
class MPCPrimitive:
    """Reusable MPC primitive.

    The paper defines a primitive as a tuple of state space, dynamics, cost,
    inequality constraints, and equality constraints. This dataclass captures
    those components in a lightweight, composable form.
    """

    name: str
    state_dim: int
    dynamics: DynamicsFn
    stage_cost: CostFn
    inequality_constraints: ConstraintFn = field(default=lambda x, u: np.zeros(0))
    equality_constraints: ConstraintFn = field(default=lambda x, u: np.zeros(0))
    metadata: Mapping[str, object] = field(default_factory=dict)

    def evaluate_constraints(self, x: ArrayLike, u: ArrayLike) -> tuple[np.ndarray, np.ndarray]:
        g = np.asarray(self.inequality_constraints(x, u), dtype=float).reshape(-1)
        h = np.asarray(self.equality_constraints(x, u), dtype=float).reshape(-1)
        return g, h


@dataclass(frozen=True)
class ComposedPrimitive:
    """Composition of multiple MPC primitives.

    The paper combines primitives through a product of state spaces and the sum
    of stage costs. For the lightweight demo in this repository, we evaluate all
    task primitives on the same ego state while using the first primitive as the
    shared dynamics provider.
    """

    primitives: tuple[MPCPrimitive, ...]

    @property
    def name(self) -> str:
        return "+".join(p.name for p in self.primitives)

    @property
    def state_dim(self) -> int:
        """Cartesian product of state spaces (formula 9: X = Xi × Xj).

        When all primitives share the same state (e.g. LK+CS → 4-d ego),
        this is simply 4.  When a PV primitive is present the state
        becomes [ego(4), pv(4)] = 8.
        """
        return max((p.state_dim for p in self.primitives), default=0)

    def dynamics(self, x: ArrayLike, u: ArrayLike) -> ArrayLike:
        """Use the dynamics from the primitive with the largest state
        dimension — this primitive handles the full augmented state
        (e.g. PV includes both KBM for ego and CV for the parallel vehicle).
        """
        if not self.primitives:
            return np.zeros(0)
        best = max(self.primitives, key=lambda p: p.state_dim)
        return np.asarray(best.dynamics(x, u), dtype=float).reshape(-1)

    def stage_cost(self, x: ArrayLike, u: ArrayLike) -> float:
        x = np.asarray(x, dtype=float).reshape(-1)
        return float(sum(primitive.stage_cost(x, u) for primitive in self.primitives))

    def constraints(self, x: ArrayLike, u: ArrayLike) -> tuple[np.ndarray, np.ndarray]:
        x = np.asarray(x, dtype=float).reshape(-1)
        gs: list[np.ndarray] = []
        hs: list[np.ndarray] = []
        for primitive in self.primitives:
            g, h = primitive.evaluate_constraints(x, u)
            if g.size:
                gs.append(g)
            if h.size:
                hs.append(h)
        return (
            np.concatenate(gs) if gs else np.zeros(0),
            np.concatenate(hs) if hs else np.zeros(0),
        )


def compose_primitives(primitives: Iterable[MPCPrimitive]) -> ComposedPrimitive:
    return ComposedPrimitive(tuple(primitives))
