print("Script started")
# run_sim.py
# Entry point for all simulation experiments.
#
# Revision history:
#   v1.3  Duplicate init removed, SafeCtrl moved to module level,
#         abort threshold aligned, wall clamp added
#   v1.4  Abort retreat redesigned: targets d_safe boundary dynamically
#         rather than a fixed x_retreat coordinate.
#         CBF filter now receives mode string for mode-aware corrections.
#         Detection range gate added — state machine only arms within
#         detection_range of wall.

import numpy as np
from scipy.integrate import solve_ivp
from params import PARAMS
from dynamics import derivatives, compute_wall_force, compute_gamma
from controller import CascadedPD
from margin import compute_margins
from logger import Logger
from cbf import CBFFilter
from state_machine import StateMachine


# ── SafeCtrl helper ───────────────────────────────────────────────────────────
class SafeCtrl:
    """Wraps pre-computed (T_safe, tau_safe) for derivatives()."""
    def __init__(self):
        self.T   = 0.0
        self.tau = 0.0

    def compute(self, state, params):
        return self.T, self.tau


# ── Main run function ─────────────────────────────────────────────────────────

def run(x_des, z_des, t_span=None, state0=None,
        wall_on=True, cbf_on=True, name='run', notes=''):
    """
    Run a single simulation experiment.

    Parameters
    ----------
    x_des   : float or callable  Target x position (m)
    z_des   : float              Target z position (m)
    t_span  : tuple              (t0, tf)
    state0  : array(6)           Initial state
    wall_on : bool               Enable wall disturbance
    cbf_on  : bool               Enable CBF safety filter
    name    : str                Filename prefix
    notes   : str                Description saved with data
    """
    p = PARAMS.copy()
    if t_span is not None:
        p['t_span'] = t_span

    if state0 is None:
        state0 = [1.5, 1.5, 0.0, 0.0, 0.0, 0.0]

    x_des_val = x_des(0) if callable(x_des) else x_des

    # ── Initialise objects ────────────────────────────────────────────────────
    ctrl            = CascadedPD(x_des_val, z_des)
    log             = Logger()
    cbf             = CBFFilter(p)
    sm              = StateMachine(p)
    safe_ctrl       = SafeCtrl()
    abort_triggered = [False]

    def ode(t, state):
        x, z, vx, vz, theta, theta_dot = state

        # ── Step 1: update moving target ──────────────────────────────────────
        if callable(x_des):
            ctrl.update_target(x_des(t), z_des)

        # ── Step 2: nominal control ───────────────────────────────────────────
        T_nom, tau_nom = ctrl.compute(state, p)

        # ── Step 3: wall effects ──────────────────────────────────────────────
        fwall = compute_wall_force(state, p) if wall_on else 0.0
        Gamma = compute_gamma(state, p)      if wall_on else 1.0
        T_eff = T_nom * Gamma

        # ── Step 4: Dockability Margin ────────────────────────────────────────
        # Only compute and use M when within detection range.
        # Outside detection range, M = 1.0 (fully safe, no wall detected).
        d = p['wall_x'] - x
        if d <= p['detection_range']:
            M, subs = compute_margins(state, T_nom, T_eff, p)
        else:
            M = 1.0
            subs = {k: 1.0 for k in
                    ['Md', 'Mv', 'Mdelta', 'Momega', 'Mu', 'Mr']}

        # ── Step 5: State machine ─────────────────────────────────────────────
        # State machine only active within detection range.
        if d <= p['detection_range']:
            mode = sm.update(M, state, p)
        else:
            mode = 'free'
            sm.mode = 'free'   # reset state machine when out of range

        # ── Step 6: Abort handling ────────────────────────────────────────────
        # DESIGN INTENT:
        #   free    mode : CBF off. Controller navigates freely.
        #   surface mode : CBF on. Permissive corrections only. NO retreat.
        #   docking mode : CBF on. Tighter speed limiting.    NO retreat.
        #   abort   mode : Controller OVERRIDDEN. Retreat to d_safe boundary.
        #                  This is the ONLY mode where CBF takes authority.
        #
        # When cbf_on=False the abort handler is disabled entirely so the
        # No-CBF run is a true unprotected baseline. Mode is capped at
        # 'docking' so the log accurately reflects no action was taken.

        if not cbf_on:
            if mode == 'abort':
                mode = 'docking'
                sm.mode = 'docking'
        else:
            if mode == 'abort':
                abort_triggered[0] = True

            if abort_triggered[0]:
                # Hard override: retreat to d_safe boundary.
                # x_retreat = wall_x - d_safe = 0.0 m in this setup.
                # CBF slows the approach in surface/docking but NEVER
                # commands retreat — only abort does that.
                x_retreat_target = p['wall_x'] - p['d_safe']
                ctrl.update_target(x_retreat_target, z_des)
                T_nom, tau_nom = ctrl.compute(state, p)

                # Recover once M is back above M_safe AND at safe distance.
                m_recovered = M >= p['M_safe'] + StateMachine.HYSTERESIS
                d_recovered = d >= p['d_safe'] - 0.05
                if m_recovered and d_recovered:
                    abort_triggered[0] = False

        # ── Step 7: CBF filter ────────────────────────────────────────────────
        # In all non-abort active modes: permissive corrections only.
        # In abort: adds smooth retreat torque on top of position controller.
        # CBF receives current mode so it applies the correct layer of correction.
        if cbf_on and mode in ['surface', 'docking', 'abort']:
            T_safe, tau_safe, cbf_active = cbf.filter(
                state, T_nom, tau_nom, p, mode=mode, M=M)
        else:
            T_safe, tau_safe, cbf_active = T_nom, tau_nom, False

        # ── Step 8: Log ───────────────────────────────────────────────────────
        log.log(t, state, T_safe, tau_safe, T_nom,
                fwall, Gamma, M, subs, mode, cbf_active, p)

        # ── Step 9: Derivatives ───────────────────────────────────────────────
        safe_ctrl.T   = T_safe
        safe_ctrl.tau = tau_safe
        return derivatives(t, state, safe_ctrl, p, wall_on=wall_on)

    # ── Run integrator ────────────────────────────────────────────────────────
    print(f'[run_sim] Starting: {name} wall={wall_on} cbf={cbf_on} '
          f'target=({x_des_val:.2f}, {z_des:.2f}) '
          f't={p["t_span"][0]}..{p["t_span"][1]} s')

    solve_ivp(
        ode,
        p['t_span'],
        state0,
        method='RK45',
        max_step=p['dt_max'],
        dense_output=False,
    )

    # ── Post-integration wall clamp ───────────────────────────────────────────
    x_limit = p['wall_x'] - p['d_min'] - 1e-3
    log.x = [min(xi, x_limit) for xi in log.x]
    log.d = [p['wall_x'] - xi for xi in log.x]

    log.save(name, p, notes=notes)
    return log


# ── Experiments ───────────────────────────────────────────────────────────────

if __name__ == '__main__':

    # ── PHASE 1 EXPERIMENTS (completed, passing) ──────────────────────────────

    # Experiment 1: Free-air hover (no wall)
    # run(x_des=1.0, z_des=1.5,
    #     t_span=(0, 15),
    #     state0=[1.2, 1.5, 0.0, 0.0, 0.0, 0.0],
    #     wall_on=False,
    #     name='exp1_hover_nowall',
    #     notes='Free-air hover verification.')

    # Experiment 2: Hover near wall
    # run(x_des=1.80, z_des=1.5,
    #     t_span=(0, 20),
    #     state0=[1.6, 1.5, 0.0, 0.0, 0.0, 0.0],
    #     wall_on=True,
    #     name='exp2_hover_nearwall',
    #     notes='Hover at d=0.20 m.')

    # Experiment 3: Constant-speed approach
    # v_approach = 0.05
    # x_start = 1.50
    # x_stop  = PARAMS['wall_x'] - PARAMS['d_min'] - 0.02
    # run(x_des=lambda t: min(x_stop, x_start + v_approach * t),
    #     z_des=1.5, t_span=(0, 25),
    #     state0=[1.5, 1.5, 0.0, 0.0, 0.0, 0.0],
    #     wall_on=True, name='exp3_approach_0p05ms')

    # ── PHASE 2 EXPERIMENTS ───────────────────────────────────────────────────

    # ── Experiment 4: Disturbance strength sensitivity ────────────────────────
    # Goal: show that stronger wall disturbances produce lower M at the same
    # wall distance, and that the disturbance residual sub-margin Mr is the
    # most sensitive indicator of disturbance strength.
    #
    # Three runs with identical approach trajectory, varying c0/c1/c2 only:
    #   Run A — no disturbance  (c0=c1=c2=0)
    #   Run B — default         (c0=0.8, c1=0.3, c2=0.1)
    #   Run C — double strength (c0=1.6, c1=0.6, c2=0.2)
    #
    # Success criterion: stronger disturbance → lower M at same distance.
    # Mr drops earliest and furthest in Run C. Overlay M(t) from all three.
    #
    # To run, uncomment the three blocks below and comment out Experiment 7.
    #
    # import copy
    # v_approach = 0.05
    # x_start = 1.50
    # x_stop  = PARAMS['wall_x'] - PARAMS['d_min'] - 0.02
    #
    # # Run A: no disturbance
    # p_nodist = copy.deepcopy(PARAMS)
    # p_nodist.update({'c0': 0.0, 'c1': 0.0, 'c2': 0.0})
    # run(x_des=lambda t: min(x_stop, x_start + v_approach * t),
    #     z_des=1.5, t_span=(0, 30),
    #     state0=[x_start, 1.5, 0.0, 0.0, 0.0, 0.0],
    #     wall_on=True, cbf_on=False,
    #     name='exp4_nodist',
    #     notes='Exp4 Run A: no wall disturbance.')
    #
    # # Run B: default disturbance
    # run(x_des=lambda t: min(x_stop, x_start + v_approach * t),
    #     z_des=1.5, t_span=(0, 30),
    #     state0=[x_start, 1.5, 0.0, 0.0, 0.0, 0.0],
    #     wall_on=True, cbf_on=False,
    #     name='exp4_default',
    #     notes='Exp4 Run B: default disturbance (c0=0.8, c1=0.3, c2=0.1).')
    #
    # # Run C: double disturbance — temporarily patch PARAMS then restore
    # p_strong = copy.deepcopy(PARAMS)
    # p_strong.update({'c0': 1.6, 'c1': 0.6, 'c2': 0.2})
    # run(x_des=lambda t: min(x_stop, x_start + v_approach * t),
    #     z_des=1.5, t_span=(0, 30),
    #     state0=[x_start, 1.5, 0.0, 0.0, 0.0, 0.0],
    #     wall_on=True, cbf_on=False,
    #     name='exp4_strong',
    #     notes='Exp4 Run C: double disturbance (c0=1.6, c1=0.6, c2=0.2).')
    # NOTE: run() uses PARAMS.copy() internally, so passing a patched dict
    # requires a small refactor to accept an optional params override, or
    # temporarily mutate PARAMS before each call and restore afterward.

    # ── Experiment 5: State machine transition verification ───────────────────
    # Goal: confirm the state machine transitions through free → surface →
    # docking → abort in the correct order and at the correct M thresholds
    # as the drone performs a slow deliberate approach. No CBF — this isolates
    # the state machine and margin estimator from the safety filter.
    #
    # A slow ramp (0.03 m/s) is used so the drone lingers in each mode long
    # enough to log a clear dwell time. The plot_modes() function in
    # visualize.py colours the background by mode and overlays M(t).
    #
    # Success criteria:
    #   • Mode sequence is exactly free → surface → docking → abort (no skips).
    #   • Each transition fires at the correct M threshold (±0.01 tolerance).
    #   • Dwell time in surface and docking modes is > 2 s each.
    #
    # To run, uncomment the block below and comment out Experiment 7.
    #
    # v_approach = 0.03                          # slow enough to see all modes
    # x_start    = 0.0                           # start at d_safe boundary
    # x_stop     = PARAMS['wall_x'] - PARAMS['d_min'] - 0.02
    # run(x_des=lambda t: min(x_stop, x_start + v_approach * t),
    #     z_des=1.5,
    #     t_span=(0, 60),                        # 60 s — slow approach needs time
    #     state0=[x_start, 1.5, 0.0, 0.0, 0.0, 0.0],
    #     wall_on=True,
    #     cbf_on=False,                          # isolate state machine only
    #     name='exp5_state_machine',
    #     notes='Exp5: slow 0.03 m/s approach to verify mode transition sequence.')
    # # After running, plot with:
    # # from visualize import load, plot_modes
    # # from params import PARAMS
    # # import glob
    # # d = load(sorted(glob.glob('data/exp5_state_machine_*.npz'))[-1])
    # # plot_modes(d, params=PARAMS, fname='exp5_modes.png')

    # ── Experiment 6: Progressive speed limiting verification ─────────────────
    # Goal: verify that Layer 3a of the CBF (progressive speed limiting) slows
    # the drone proportionally as M decreases, without commanding retreat.
    # This is a targeted test of the CBF mechanism before the full CBF vs
    # No-CBF comparison in Experiment 7.
    #
    # Two runs with a fixed target at x_stop:
    #   Run A — CBF off  (drone accelerates freely, M collapses)
    #   Run B — CBF on   (Layer 3a limits speed, M stays higher)
    #
    # Both runs use cbf_on to verify that only speed limiting fires — abort
    # should NOT trigger because the progressive limit prevents M from
    # reaching M_abort during a normal PD-controlled approach from x=1.0.
    # Start at x=1.0 (d=1.0m, Md=0.50 → surface mode from t=0) so CBF is
    # armed immediately and Layer 3a is exercised from the first timestep.
    #
    # Success criteria:
    #   • Run B closing speed stays below the progressive limit at all times.
    #   • Run B min(M) > Run A min(M).
    #   • Abort does NOT fire in Run B (no retreat, just speed modulation).
    #   • CBF active steps > 0 in Run B during the approach phase.
    #
    # To run, uncomment the two blocks below and comment out Experiment 7.
    #
    # x_start_exp6 = 1.0
    # x_stop_exp6  = PARAMS['wall_x'] - PARAMS['d_min'] - 0.02
    #
    # # Run A: no CBF — baseline, drone accelerates freely
    # run(x_des=x_stop_exp6,
    #     z_des=1.5,
    #     t_span=(0, 25),
    #     state0=[x_start_exp6, 1.5, 0.0, 0.0, 0.0, 0.0],
    #     wall_on=True,
    #     cbf_on=False,
    #     name='exp6_no_cbf',
    #     notes='Exp6 Run A: no CBF, free acceleration toward wall.')
    #
    # # Run B: CBF on — Layer 3a progressive limiting active
    # run(x_des=x_stop_exp6,
    #     z_des=1.5,
    #     t_span=(0, 25),
    #     state0=[x_start_exp6, 1.5, 0.0, 0.0, 0.0, 0.0],
    #     wall_on=True,
    #     cbf_on=True,
    #     name='exp6_cbf',
    #     notes='Exp6 Run B: CBF on, Layer 3a progressive speed limiting.')

    # ── EXPERIMENT 7: CBF vs No-CBF ───────────────────────────────────────────
    #
    # Goal: show that the permissive CBF
    #   (a) does not interfere during a controlled moderate approach, AND
    #   (b) provides braking corrections when approach speed exceeds the CBF
    #       threshold (vn_cbf_thresh = 0.8 m/s), preventing M from collapsing.
    #
    # Design rationale:
    #   • Start at x=0.5 (d=1.5m, inside CBF zone, Md≈0.74 → surface mode).
    #     This keeps the drone within d_safe from t=0 so CBF is armed.
    #   • Use a FIXED target at x_stop (no ramp). The PD controller accelerates
    #     the drone from rest; it naturally reaches ~0.9-1.0 m/s during the
    #     mid-approach, which exceeds vn_cbf_thresh=0.8 m/s and triggers
    #     Layer 3a braking. A ramp at 0.1 m/s never reaches that threshold.
    #   • 35 s is long enough to complete the approach and observe divergence.
    #
    x_start_exp7 = 0.5
    x_stop_exp7  = PARAMS['wall_x'] - PARAMS['d_min'] - 0.02   # = 1.93 m

    # No CBF — approach with wall disturbance only
    run(x_des=x_stop_exp7,
        z_des=1.5,
        t_span=(0, 35),
        state0=[x_start_exp7, 1.5, 0.0, 0.0, 0.0, 0.0],
        wall_on=True,
        cbf_on=False,
        name='exp7_no_cbf',
        notes='Experiment 7: fixed-target approach without CBF.')

    # With CBF — same approach, permissive safety filter active
    run(x_des=x_stop_exp7,
        z_des=1.5,
        t_span=(0, 35),
        state0=[x_start_exp7, 1.5, 0.0, 0.0, 0.0, 0.0],
        wall_on=True,
        cbf_on=True,
        name='exp7_cbf',
        notes='Experiment 7: fixed-target approach with permissive CBF.')