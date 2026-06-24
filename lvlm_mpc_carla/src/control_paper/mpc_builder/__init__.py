"""MPC Builder package.

This package provides paper-aligned primitives, model definitions,
composition utilities, a high-level builder, and a lightweight MPPI-based solver.
"""

from .builder import (
    MPCBuilder,
    MPCBuilderConfig,
    MPCBuilderResult,
    SurroundingVehicle,
    TaskCommand,
    TrafficContext,
)
from .models import (
    KinematicBicycleParams,
    adaptive_cruise_control_primitive,
    constant_speed_primitive,
    kinematic_bicycle_dynamics,
    kinematic_bicycle_primitive,
    lane_change_primitive,
    lane_keep_primitive,
    parallel_vehicle_primitive,
)
from .primitives import ComposedPrimitive, MPCPrimitive, compose_primitives
from .solvers import MPPIConfig, MPPISolver

__all__ = [
    "MPCBuilder",
    "MPCBuilderConfig",
    "MPCBuilderResult",
    "SurroundingVehicle",
    "TaskCommand",
    "TrafficContext",
    "KinematicBicycleParams",
    "adaptive_cruise_control_primitive",
    "constant_speed_primitive",
    "kinematic_bicycle_dynamics",
    "kinematic_bicycle_primitive",
    "lane_change_primitive",
    "lane_keep_primitive",
    "parallel_vehicle_primitive",
    "ComposedPrimitive",
    "MPCPrimitive",
    "compose_primitives",
    "MPPIConfig",
    "MPPISolver",
]
