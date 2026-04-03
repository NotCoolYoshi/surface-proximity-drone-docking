# params.py
# Surface-Proximate Drone Docking Project — Sean Dai
# All simulation parameters are kept in params.py.
#
# Revision history:
#   v1.0  Phase 1 baseline
#   v1.1  d_safe reduced 0.50 -> 0.80 after margin calibration
#   v1.2  T_max doubled to 39.24; kp_x/kd_x/kd_th tuned
#   v1.3  Phase 2: CBF gains, x_retreat, retreat_speed added
#   v1.4  Phase 2 redesign: detection_range added (10m LiDAR gate)
#         d_safe set to 2.0m (CBF activation boundary)
#         vn_cbf_thresh, omega_cbf_thresh added for permissive CBF
#         x_retreat set to wall_x - d_safe (retreat to safe boundary)

PARAMS = {

    # ── Vehicle ───────────────────────────────────────────────────────────────
    'm':          1.0,      # total mass (kg)
    'I':          0.015,    # pitch moment of inertia (kg*m^2)
                            # 0.015 is correct Phase 1/2 estimate for 500mm H-frame
    'g':          9.81,     # gravitational acceleration (m/s^2)
    'T_max':      39.24,    # max thrust = 4 * m * g (N)
                            # deliberately 4*m*g to give Mu headroom at hover
    'theta_max':  0.436,    # max allowable tilt = 25 degrees (rad)

    # ── World geometry ────────────────────────────────────────────────────────
    'wall_x':     2.0,      # wall position on x-axis (m)
    'z_min':      0.3,      # minimum flight altitude (m)
    'z_max':      2.1,      # maximum flight altitude (m)
    'room_width': 2.16,     # garage width (m) — reference only
    'room_depth': 2.54,     # garage depth (m) — reference only

    # ── Wall disturbance model ─────────────────────────────────────────────────
    # REPLACE with measured values after Phase 3 system ID experiment.
    'alpha':      0.15,     # thrust modification strength
    'beta':       1.5,      # thrust modification decay rate (m^-1)
    'lam':        2.0,      # disturbance force decay rate (m^-1)
    'c0':         0.8,      # pressure attraction (N)        — REPLACE
    'c1':         0.3,      # viscous damping (N*s/m)        — REPLACE
    'c2':         0.1,      # nonlinear drag (N*s^2/m^2)     — REPLACE

    # ── Cascaded PD controller gains ──────────────────────────────────────────
    # Empirically validated in Phase 1. Ratio kp_th/kp_x = 7.5x (stable).
    'kp_z':       8.0,
    'kd_z':       4.0,
    'kp_x':       2.0,
    'kd_x':       10.0,
    'kp_th':      15.0,
    'kd_th':      15.0,

    # ── Proximity zones ───────────────────────────────────────────────────────
    # Three concentric zones define system behavior:
    #
    #   detection_range (10 m): LiDAR/camera detects wall. System arms.
    #                           CBF monitoring begins. No intervention yet.
    #
    #   d_safe (2.0 m):         CBF zone begins. Permissive corrections
    #                           active. Drone may still approach freely.
    #                           Abort retreat targets this boundary.
    #
    #   d_min (0.05 m):         Hard physical clearance. Drone must never
    #                           cross this. CBF enforces absolutely.
    #
    'detection_range': 10.0, # LiDAR detection range (m)
                              # System arms when d <= detection_range
    'd_safe':          2.0,   # CBF activation boundary (m)
                              # = wall_x - d_safe = 0.0 m from start
                              # In simulation: CBF activates when x > 0.0
                              # i.e. when drone is within 2m of wall
    'd_min':           0.05,  # hard minimum clearance (m)

    # ── Dockability Margin thresholds ─────────────────────────────────────────
    'vn_max':     1.5,      # max allowable approach speed for Mv margin (m/s)
    'delta_max':  0.349,    # max allowable tilt = 20 degrees (rad)
    'omega_max':  2.0,      # max allowable angular rate (rad/s)
    'r_max':      2.0,      # disturbance residual threshold (m/s^2)

    # ── State machine thresholds ───────────────────────────────────────────────
    'M_safe':     0.6,      # M < M_safe  -> surface mode (CBF arms)
    'M_dock':     0.4,      # M < M_dock  -> docking mode
    'M_abort':    0.15,     # M < M_abort -> abort and retreat

    # ── CBF filter parameters ─────────────────────────────────────────────────
    # cbf_gamma_d/v: class-K gains for barrier conditions
    # vn_cbf_thresh: closing velocity threshold for permissive correction
    #   Only apply braking when closing faster than this.
    #   More permissive than vn_max (which governs the Mv sub-margin).
    #   Set to 0.8 m/s — allows deliberate 0.1-0.4 m/s approach freely.
    # omega_cbf_thresh: angular rate threshold for damping correction
    #   Only suppress oscillations above 60% of omega_max.
    #   Allows normal attitude maneuvers without interference.
    'cbf_gamma_d':      0.5,
    'cbf_gamma_v':      0.5,
    'cbf_gamma_th':     1.0,    # reserved for future QP formulation
    'cbf_gamma_u':      1.0,    # reserved for future QP formulation
    'vn_cbf_thresh':    0.8,    # closing speed threshold for CBF braking (m/s)
    'omega_cbf_thresh': 1.2,    # angular rate threshold for CBF damping (rad/s)
                                # = 0.6 * omega_max

    # ── Abort retreat ─────────────────────────────────────────────────────────
    # During abort, drone retreats to d_safe boundary (2m from wall).
    # x_retreat = wall_x - d_safe = 2.0 - 2.0 = 0.0
    # But since drone starts at x=0.5 in simulation, use a practical value.
    # In hardware: x_retreat should be computed as wall_x - d_safe
    # dynamically from the LiDAR reading.
    'x_retreat':      0.0,    # retreat target x (m) = wall_x - d_safe
                               # drone retreats until d = d_safe = 2m
    'retreat_speed':  0.10,   # retreat speed (m/s) — smooth, not abrupt

    # ── Integrator settings ────────────────────────────────────────────────────
    'dt_max':     0.005,    # max RK45 step size (s)
    't_span':     (0, 20),  # default simulation duration (s)

    # ── Experiment reference speeds ────────────────────────────────────────────
    'v_approach_slow':   0.05,
    'v_approach_medium': 0.10,
    'v_approach_fast':   0.20,

    # ── Hardware reference ─────────────────────────────────────────────────────
    'wheelbase_mm':       500,
    'prop_diameter_mm':   127,
    'prop_clearance_mm':   30,
    'sensor_baseline_mm':  60,
    'magnet_diameter_mm':  30,
}


# ── Derived quantities ────────────────────────────────────────────────────────

def hover_thrust(params=PARAMS):
    return params['m'] * params['g']

def half_life_distance(params=PARAMS):
    import math
    return math.log(2) / params['lam']

def gamma_at_distance(d, params=PARAMS):
    import math
    return 1.0 + params['alpha'] * math.exp(-params['beta'] * d)


# ── Self-check ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import math
    p = PARAMS
    print('=' * 60)
    print('params.py self-check')
    print('=' * 60)

    T_hover = p['m'] * p['g']
    print(f"  mass              : {p['m']:.2f} kg")
    print(f"  hover thrust      : {T_hover:.2f} N")
    print(f"  T_max             : {p['T_max']:.2f} N  (= 4*m*g, Mu={1-T_hover/p['T_max']:.2f} at hover)")

    ratio = p['kp_th'] / p['kp_x']
    print(f"  kp_th / kp_x      : {ratio:.1f}x  [{'OK' if 3<=ratio<=8 else 'WARNING'}]")

    order_ok = p['M_safe'] > p['M_dock'] > p['M_abort'] > 0
    print(f"  M thresholds      : {p['M_safe']} > {p['M_dock']} > {p['M_abort']}  [{'OK' if order_ok else 'ERROR'}]")

    d_ok = p['d_min'] < p['d_safe'] <= p['wall_x']
    print(f"  d ordering        : d_min={p['d_min']} < d_safe={p['d_safe']} <= wall_x={p['wall_x']}  [{'OK' if d_ok else 'ERROR'}]")
    
    zone_ok = p['d_safe'] < p['detection_range']
    print(f"  zone ordering     : d_safe={p['d_safe']}m < detection={p['detection_range']}m  [{'OK' if zone_ok else 'ERROR'}]")

    print(f"  CBF braking thresh: vn > {p['vn_cbf_thresh']} m/s closing (permissive)")
    print(f"  CBF damp thresh   : |omega| > {p['omega_cbf_thresh']} rad/s")

    all_ok = order_ok and d_ok and zone_ok
    print('=' * 60)
    print('All checks passed.' if all_ok else 'One or more checks FAILED.')