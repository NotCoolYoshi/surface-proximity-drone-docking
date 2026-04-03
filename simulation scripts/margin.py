# margin.py
import numpy as np


def sat(x):
    return float(np.clip(x, 0.0, 1.0))


def compute_margins(state, T_cmd, T_predicted, params):
    x, z, vx, vz, theta, theta_dot = state

    d = params['wall_x'] - x
    vn = -vx

    # Sub-margin 1: Clearance
    Md = sat((d - params['d_min']) /
             (params['d_safe'] - params['d_min']))

    # Sub-margin 2: Approach velocity
    Mv = sat(1.0 - max(0.0, -vn) / params['vn_max'])

    # Sub-margin 3: Alignment (tilt)
    Mdelta = sat(1.0 - abs(theta) / params['delta_max'])

    # Sub-margin 4: Angular rate
    Momega = sat(1.0 - abs(theta_dot) / params['omega_max'])

    # Sub-margin 5: Control authority
    Mu = sat(1.0 - T_cmd / params['T_max'])

    # Sub-margin 6: Disturbance residual
    residual = abs(T_predicted - T_cmd) / params['m']
    Mr = sat(1.0 - residual / params['r_max'])

    M = min(Md, Mv, Mdelta, Momega, Mu, Mr)

    subs = {
        'Md': Md,
        'Mv': Mv,
        'Mdelta': Mdelta,
        'Momega': Momega,
        'Mu': Mu,
        'Mr': Mr,
    }
    return M, subs