# dynamics.py
# Equations of motion for the 2D planar quadrotor with wall disturbance model.
# Called by scipy.integrate.solve_ivp at each integration step.
#
# SIGN CONVENTION:
#   x increases toward the wall (wall at wall_x).
#   vn = -vx  (positive = receding from wall, negative = closing)
#   ax = -(T_eff/m)*sin(theta) + f_wall/m
#      positive theta → ax negative → decelerates wall approach
#
# WALL BOUNDARY NOTE:
#   The integrator (RK45) holds its own state vector and ignores any local
#   variable changes made here. Hard position clamping is therefore performed
#   in run_sim.py via the SafeCtrl wrapper, NOT here. Do not add a clamp to
#   this file — it will silently fail to take effect.

import numpy as np


def derivatives(t, state, controller, params, wall_on=True):
    """
    Compute state derivatives for the 2D drone model.

    Parameters
    ----------
    t          : float     Current time (s). Required by solve_ivp signature.
    state      : array(6)  [x, z, vx, vz, theta, theta_dot]
    controller : object    Object with .compute(state, params) -> (T, tau)
    params     : dict      Simulation parameters from params.py
    wall_on    : bool      If False, disables all wall disturbance terms.

    Returns
    -------
    dstate : list(6)  [vx, vz, ax, az, theta_dot, alpha_rot]
    """
    x, z, vx, vz, theta, theta_dot = state
    m  = params['m']
    I  = params['I']
    g  = params['g']
    wx = params['wall_x']

    # Signed distance to wall. Clamped to 1e-3 to prevent division by zero
    # if the integrator steps the drone exactly to the wall surface.
    d  = max(wx - x, 1e-3)
    vn = -vx   # wall-normal velocity (positive = receding)

    # Get control inputs from controller
    T_nom, tau = controller.compute(state, params)

    # ── Thrust modification (wall proximity effect) ──────────────────────
    # Gamma(d) = 1 + alpha * exp(-beta * d)
    # Increases effective thrust as rotor approaches surface.
    if wall_on:
        Gamma = 1.0 + params['alpha'] * np.exp(-params['beta'] * d)
    else:
        Gamma = 1.0
    T_eff = T_nom * Gamma

    # ── Wall disturbance force ────────────────────────────────────────────
    # Three terms: pressure attraction, viscous damping, nonlinear drag.
    # f_wall = -(c0*e^(-lam*d) + c1*e^(-lam*d)*vn + c2*e^(-lam*d)*vn*|vn|)
    if wall_on:
        e      = np.exp(-params['lam'] * d)
        f_wall = -(params['c0'] * e
                 + params['c1'] * e * vn
                 + params['c2'] * e * vn * abs(vn))
    else:
        f_wall = 0.0

    # ── Equations of motion ───────────────────────────────────────────────
    ax        = -(T_eff / m) * np.sin(theta) + f_wall / m
    az        =  (T_eff / m) * np.cos(theta) - g
    alpha_rot = tau / I

    return [vx, vz, ax, az, theta_dot, alpha_rot]


def compute_wall_force(state, params):
    """
    Return the instantaneous wall disturbance force (N) for logging.
    Separated from derivatives() so the logger can call it independently.
    """
    x, _, vx, _, _, _ = state
    d  = max(params['wall_x'] - x, 1e-3)
    vn = -vx
    e  = np.exp(-params['lam'] * d)
    return -(params['c0'] * e
           + params['c1'] * e * vn
           + params['c2'] * e * vn * abs(vn))


def compute_gamma(state, params):
    """Return the thrust modification factor Gamma(d) for logging."""
    x = state[0]
    d = max(params['wall_x'] - x, 1e-3)
    return 1.0 + params['alpha'] * np.exp(-params['beta'] * d)