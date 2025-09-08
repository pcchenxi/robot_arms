import numpy as np
from scipy.spatial.transform import Rotation as R

def so3_log(R):
    """Log map: SO(3) -> R^3 (rotation vector), numerically stable near 0."""
    tr = np.trace(R)
    cos_th = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(cos_th)
    if th < 1e-8:
        return np.zeros(3)
    s = np.sin(th)
    w = (1.0 / (2.0 * s)) * np.array([
        R[2,1] - R[1,2],
        R[0,2] - R[2,0],
        R[1,0] - R[0,1],
    ])
    return w * th

def pose_error_spatial(p_cur, R_cur, p_des, R_des, k_p, k_w, v_cap, w_cap):
    """6D desired EE twist from pose error (spatial/world frame)."""
    e_p = p_des - p_cur
    e_w = so3_log(R_des @ R_cur.T)
    v = np.hstack([k_p * e_p, k_w * e_w])  # [vx,vy,vz, wx,wy,wz]
    # cap linear & angular speeds (smoothness/safety)
    ln = np.linalg.norm(v[:3]);  an = np.linalg.norm(v[3:])
    if ln > v_cap and ln > 1e-12: v[:3] *= (v_cap / ln)
    if an > w_cap and an > 1e-12: v[3:] *= (w_cap / an)
    return v

# ========= numeric Jacobian from your FK =========
def numeric_jacobian_fk(fk, q, h=1e-5):
    """
    Builds a 6 x n spatial Jacobian by central-differencing FK.
    fk(q) must return (p (3,), R (3x3)) in the world frame.
    """
    n = len(q)
    J = np.zeros((6, n))
    p0, R0 = fk(q)
    for i in range(n):
        dq = np.zeros_like(q); dq[i] = h
        p_plus,  R_plus  = fk(q + dq)
        p_minus, R_minus = fk(q - dq)
        # linear part
        J[0:3, i] = (p_plus - p_minus) / (2*h)
        # angular part: local logs around current pose to reduce bias
        w_plus  = so3_log(R_plus  @ R0.T)
        w_minus = so3_log(R_minus @ R0.T)
        J[3:6, i] = (w_plus - w_minus) / (2*h)
    return J

# ========= damped least-squares (Whitney + damping) =========
def dls(J, v, lam=0.08):
    """
    qdot = J^T (J J^T + lam^2 I)^-1 v
    lam tuned for 10 Hz; raise if near singular.
    """
    JJt = J @ J.T
    A = JJt + (lam**2) * np.eye(J.shape[0])
    return J.T @ np.linalg.solve(A, v)

# ========= optional rate/limit helpers =========
def clamp_joint_vel(qdot, qdot_limits):
    if qdot_limits is None: return qdot
    scale = 1.0
    for i, lim in enumerate(qdot_limits):
        if lim is not None and lim > 0:
            scale = max(scale, abs(qdot[i]) / lim)
    return qdot / scale if scale > 1.0 else qdot

def slew_limit(qdot, qdot_prev, a_max, dt):
    """Limit acceleration: |qdot - qdot_prev| <= a_max * dt (elementwise)."""
    if qdot_prev is None or a_max is None: return qdot
    dq = qdot - qdot_prev
    max_step = np.asarray(a_max) * dt
    dq = np.clip(dq, -max_step, max_step)
    return qdot_prev + dq

# ========= main control tick =========
def resolved_rate_motion_control(
    fk_function, q, p_des, R_des, # -> q, J, p_cur, R_cur, p_des, R_des    , R_des and R_cur are rotation matrices
    # 10 Hz tuned gains and caps:
    k_p=0.6,           # m/s per m error
    k_w=1.2,           # rad/s per rad error
    v_cap=0.15,        # max EE linear speed (m/s)
    w_cap=0.8,         # max EE angular speed (rad/s)
    lam=0.08,          # DLS damping
    h=1e-5,            # FD step (rad)
    qdot_limits=None,  # list/array of per-joint rad/s caps, e.g., 0.8
    qdot_prev=None,    # previous commanded qdot
    a_max=None,        # per-joint accel cap (rad/s^2), e.g., 3.0
    dt=0.1             # control period (s) for 10 Hz
):
    # p_cur, R_cur = fk_function(q)
    v_ee = pose_error_spatial(p_cur, R_cur, p_des, R_des, k_p, k_w, v_cap, w_cap)
    # J = numeric_jacobian_fk(fk_function, q, h=h)
    qdot = dls(J, v_ee, lam=lam)

    # scale joint velocity to respect limits
    qdot = clamp_joint_vel(qdot, qdot_limits)
    # qdot = slew_limit(qdot, qdot_prev, a_max, dt)
    return qdot, v_ee

def joint_p_control(q_current, q_target, kp=1.0, qdot_limits=5.0):
    """Simple joint-space P controller with optional velocity limits."""
    # convert to degrees

    qdot = kp * (q_target - q_current)  # joint-space proportional velocity

    # Velocity and optional accel limits
    if qdot_limits is not None:
        scale = max(1.0, np.max(np.abs(qdot)/(np.array(qdot_limits)+1e-12)))
        if scale > 1.0: qdot /= scale


def _saturate(vec: np.ndarray, max_norm: float) -> np.ndarray:
    """Scale vec to have at most max_norm (leave direction intact)."""
    n = np.linalg.norm(vec)
    if n <= 1e-12 or max_norm <= 0.0:
        return np.zeros_like(vec)
    if n > max_norm:
        return vec * (max_norm / n)
    return vec

def ee_p_control(trans_current, quat_current, trans_target, quat_target, kp=1.0, kw=1.0, v_max=0.2, w_max=10.0):
    # --- Position control (P) ---
    p_cur = np.asarray(trans_current, dtype=float).reshape(3)
    p_tgt = np.asarray(trans_target, dtype=float).reshape(3)
    pos_err = p_tgt - p_cur                         # [m]
    v_cmd = kp * pos_err                            # [m/s]
    # v_cmd = _saturate(v_cmd, v_max)

    # --- Orientation control (P on rotation vector) ---
    # Build rotations (SciPy expects quaternion in (x,y,z,w) order)
    R_cur = R.from_quat(np.asarray(quat_current, dtype=float))
    R_tgt = R.from_quat(np.asarray(quat_target, dtype=float))

    # Rotation that takes current to target
    R_err = R_tgt * R_cur.inv()
    # Rotation vector (axis * angle), in radians
    rotvec = R_err.as_rotvec()                      # [rad] direction * angle
    w_cmd = kw * rotvec                             # [rad/s]

    # # Saturate angular speed by magnitude (convert limit deg/s -> rad/s)
    # w_max_rad = np.deg2rad(float(w_max))
    # w_cmd = _saturate(w_cmd, w_max_rad)

    return v_cmd, w_cmd
