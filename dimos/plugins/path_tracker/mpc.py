# Extracted from dimos_simple/dimos/navigation/xnavdp/tracking.py (Apache-2.0).
import math

import numpy as np
from scipy.optimize import minimize


def solve_mpc(
    path: np.ndarray,
    horizon: int,
    dt: float,
    desired_v: float,
    v_max: float,
    w_max: float,
) -> np.ndarray:
    points = np.asarray(path, dtype=float)[:, :2]
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("Invalid XNavDP selected path")
    points = np.vstack((np.zeros((1, 2)), points))
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(distance) > 1e-8]
    points, distance = points[keep], distance[keep]
    if len(points) < 2:
        return np.zeros((horizon, 2))
    speed = min(v_max, desired_v)
    arc = np.minimum(np.arange(horizon + 1) * dt * speed, distance[-1])
    reference = np.column_stack(
        [np.interp(arc, distance, points[:, j]) for j in range(2)]
    )

    def objective(flat: np.ndarray) -> tuple[float, np.ndarray]:
        controls = flat.reshape(horizon, 2)
        states = np.zeros((horizon + 1, 3))
        for i, (v, w) in enumerate(controls):
            yaw = states[i, 2]
            states[i + 1] = states[i] + dt * np.array(
                [v * math.cos(yaw), v * math.sin(yaw), w]
            )
        error = states[:, :2] - reference
        cost = 10.0 * np.sum(error**2) + 0.05 * np.sum(controls**2)
        adjoint = np.r_[20.0 * error[-1], 0.0]
        gradient = np.empty_like(controls)
        for i in range(horizon - 1, -1, -1):
            v, _ = controls[i]
            c, s = math.cos(states[i, 2]), math.sin(states[i, 2])
            gradient[i] = 0.1 * controls[i] + dt * np.array(
                [c * adjoint[0] + s * adjoint[1], adjoint[2]]
            )
            adjoint = np.array(
                [
                    adjoint[0],
                    adjoint[1],
                    adjoint[2] + dt * v * (-s * adjoint[0] + c * adjoint[1]),
                ]
            )
            adjoint[:2] += 20.0 * error[i]
        return float(cost), gradient.ravel()

    initial = np.zeros((horizon, 2))
    initial[:, 0] = math.copysign(min(speed, 0.05), points[1, 0])
    initial[:, 1] = np.clip(
        math.atan2(points[1, 1], abs(points[1, 0]) + 1e-8), -w_max, w_max
    )
    result = minimize(
        objective,
        initial.ravel(),
        jac=True,
        method="L-BFGS-B",
        bounds=[(-v_max, v_max), (-w_max, w_max)] * horizon,
        options={"maxiter": 100, "ftol": 1e-8, "gtol": 1e-5},
    )
    if not result.success or not np.isfinite(result.x).all():
        raise RuntimeError(f"XNavDP MPC failed: {result.message}")
    return result.x.reshape(horizon, 2)
