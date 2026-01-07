import numpy as np
import cvxpy as cp


def cbf_row_for_obstacles(p, v_nominal, obs, params):

    drone_radius   = float(params.get("drone_radius", 0.2))
    safety_margin  = float(params.get("safety_margin", 0.0))
    gamma          = float(params.get("gamma", 1.0))

    shape = obs.get("shape", "sphere")  # "sphere" (default) or "cylinder_z"

    # -------------------------
    # Case 1: Spherical obstacle
    # -------------------------
    if shape == "sphere":
        c = np.asarray(obs["c"], dtype=float)
        c_dot = np.asarray(obs.get("c_dot", np.zeros(3)), dtype=float)
        r_obs = float(obs["r_obs"])

        r_safe = r_obs + drone_radius + safety_margin

        # Relative position and velocity
        p_rel     = c - p
        v_rel_nom = c_dot - v_nominal

        d     = np.linalg.norm(p_rel)
        v_mag = np.linalg.norm(v_rel_nom)

        if d < 1e-4 or v_mag < 1e-4:
            return None, None

        sin_phi = min(r_safe / max(d, 1e-4), 0.999)
        cos_phi = np.sqrt(1.0 - sin_phi**2)

        h = float(p_rel @ v_rel_nom + d * v_mag * cos_phi)

        grad_h_p = -(v_rel_nom + v_mag * cos_phi * (p_rel / d))

        A_i = -grad_h_p.reshape(1, 3)
        b_i = gamma * h
        return A_i, b_i

    # -------------------------
    # Case 2: Vertical cylinder (axis aligned with z)
    # -------------------------
    elif shape == "cylinder_z":
        # c is the point on the cylinder axis (e.g., center of the cylinder)
        c = np.asarray(obs["c"], dtype=float)           # 3D: [cx, cy, cz]
        c_dot = np.asarray(obs.get("c_dot", np.zeros(3)), dtype=float)
        r_obs = float(obs["r_obs"])
        z_min = float(obs.get("z_min", -np.inf))
        z_max = float(obs.get("z_max",  np.inf))

        # If drone is outside vertical span (including its radius), ignore this obstacle
        if p[2] < (z_min - drone_radius) or p[2] > (z_max + drone_radius):
            return None, None

        r_safe = r_obs + drone_radius + safety_margin

        # Project to XY plane for radial distance
        p_rel = c - p
        p_rel[2] = 0.0  # ignore vertical component for radial distance

        v_rel_nom = c_dot - v_nominal
        v_rel_nom[2] = 0.0

        d     = np.linalg.norm(p_rel)
        v_mag = np.linalg.norm(v_rel_nom)

        if d < 1e-4 or v_mag < 1e-4:
            return None, None

        sin_phi = min(r_safe / max(d, 1e-4), 0.999)
        cos_phi = np.sqrt(1.0 - sin_phi**2)

        h = float(p_rel @ v_rel_nom + d * v_mag * cos_phi)

        # Gradient wrt p (note p_rel = c - p ⇒ ∂p_rel/∂p = -I; z-part is effectively zero)
        grad_h_p = -(v_rel_nom + v_mag * cos_phi * (p_rel / d))

        A_i = -grad_h_p.reshape(1, 3)
        b_i = gamma * h
        return A_i, b_i

    # Unknown shape → ignore
    else:
        return None, None


def cbf_safe_velocity(p, v_nominal, obstacles, params):

    p         = np.asarray(p, dtype=float).reshape(3,)
    v_nominal = np.asarray(v_nominal, dtype=float).reshape(3,)

    v_max = float(params.get("v_max", 2.0))

    # If no obstacle then just saturate nominal velocity and return
    if obstacles is None or len(obstacles) == 0:
        speed = np.linalg.norm(v_nominal)
        if speed > v_max:
            return v_nominal * (v_max / speed)
        else:
            return v_nominal

    A_list = []
    b_list = []

    for obs in obstacles:
        A_i, b_i = cbf_row_for_obstacles(p, v_nominal, obs, params)
        if A_i is not None:
            A_list.append(A_i)
            b_list.append(b_i)

    # If no active constraints, just saturate nominal
    if len(A_list) == 0:
        speed = np.linalg.norm(v_nominal)
        if speed > v_max:
            return v_nominal * (v_max / speed)
        else:
            return v_nominal

    A = np.vstack(A_list)
    b = np.array(b_list).reshape(-1)

    # QP variable
    u = cp.Variable(3)

    # Objective: stay close to nominal velocity
    obj = cp.Minimize(cp.sum_squares(u - v_nominal))

    # Constraints:
    constraints = []
    constraints.append(A @ u <= b)

    # Velocity bounds
    constraints += [
        u[0] <=  v_max,
        u[0] >= -v_max,
        u[1] <=  v_max,
        u[1] >= -v_max,
        u[2] <=  v_max,
        u[2] >= -v_max,
    ]

    problem = cp.Problem(obj, constraints)

    try:
        problem.solve(solver=cp.OSQP, warm_start=True)
    except Exception:
        # If solver fails, fall back to nominal
        speed = np.linalg.norm(v_nominal)
        if speed > v_max:
            return v_nominal * (v_max / speed)
        else:
            return v_nominal

    if u.value is None:
        # Infeasible or no solution: fall back
        speed = np.linalg.norm(v_nominal)
        if speed > v_max:
            return v_nominal * (v_max / speed)
        else:
            return v_nominal

    v_safe = np.array(u.value).reshape(3,)

    return v_safe
