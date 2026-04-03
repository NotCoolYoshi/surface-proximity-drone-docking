# cbf.py
# Control Barrier Function safety filter — layered proximity design.
#
# VERSION 4: Implements the correct layered safety architecture.
#
# ── DESIGN PHILOSOPHY ────────────────────────────────────────────────────────
#
# The CBF is NOT a flight controller. It is a safety supervisor that watches
# over the nominal controller and only intervenes when genuinely necessary.
# It should be as invisible as possible during normal operation.
#
# ── FOUR OPERATING LAYERS ─────────────────────────────────────────────────────
#
#   Layer 1 — Open air (d > detection_range = 10 m):
#     CBF fully disabled. System returns nominal input unchanged.
#     No wall has been detected; there is nothing to protect against.
#
#   Layer 2 — Monitoring zone (d_safe < d <= detection_range):
#     CBF still disabled. Wall has been detected but drone is not yet
#     within the influence zone. M is computed and logged but zero
#     intervention is applied.
#
#   Layer 3 — CBF zone (d_min < d <= d_safe, mode != abort):
#     CBF active but PERMISSIVE. Applies gradual torque corrections only
#     when wall aerodynamic effects are causing genuine instability.
#     Does NOT restrict intentional approach. Does NOT command retreat.
#     Does NOT override position. Corrections are small and additive.
#
#   Layer 4 — Abort (mode == abort):
#     Hard override. Drone retreats to safe distance (d = d_safe).
#     This is the ONLY mode where the CBF takes position authority
#     away from the controller. Retreat is smooth, not instantaneous.
#
# ── WHAT THE CBF CORRECTS IN LAYER 3 ──────────────────────────────────────────
#
#   1. Excessive closing velocity (vn << -vn_max_cbf):
#      Applies gentle braking torque. Does not stop approach, only slows it.
#
#   2. Wall-induced angular rate oscillations (|theta_dot| > omega_cbf_thresh):
#      Applies damping torque to suppress oscillatory instability.
#      This is the primary wall effect — rotors near surfaces cause roll/pitch
#      oscillations that grow with proximity.
#
#   3. Thrust saturation warning (T approaching T_max):
#      Applies small level-flight torque to reduce horizontal acceleration
#      demand and free up thrust authority for altitude hold.
#
# ── WHAT THE CBF DOES NOT DO IN LAYER 3 ──────────────────────────────────────
#   - Does not prevent the drone from getting closer to the wall
#   - Does not command retreat
#   - Does not override the pilot/controller's position target
#   - Does not prevent M from decreasing (M decreasing is normal and expected)
#
# ── SIGN CONVENTION ──────────────────────────────────────────────────────────
#   d   = wall_x - x    (positive when drone is left of wall)
#   vn  = -vx           (positive = receding, negative = closing)
#   ax  = -(T/m)*sin(theta)
#   To decelerate approach: theta > 0 (positive) → ax < 0

import numpy as np


class CBFFilter:

    def __init__(self, params):
        self.params = params

    def filter(self, state, T_nom, tau_nom, params, mode='surface', M=1.0):
        """
        Apply layered CBF safety correction.

        Parameters
        ----------
        state   : array  [x, z, vx, vz, theta, theta_dot]
        T_nom   : float  Nominal thrust from controller (N)
        tau_nom : float  Nominal torque from controller (N*m)
        params  : dict   Simulation parameters from params.py
        mode    : str    Current state machine mode:
                         'free' | 'surface' | 'docking' | 'abort'

        Returns
        -------
        T_safe     : float  Safe thrust (N)
        tau_safe   : float  Safe torque (N*m)
        cbf_active : bool   True if CBF applied any correction
        """
        x, z, vx, vz, theta, theta_dot = state

        m              = params['m']
        I              = params['I']
        wx             = params['wall_x']
        d_min          = params['d_min']
        d_safe         = params['d_safe']
        detection_range = params['detection_range']
        vn_max         = params['vn_max']
        T_max          = params['T_max']
        omega_max      = params['omega_max']
        gd             = params['cbf_gamma_d']

        d     = wx - x    # distance to wall
        vn    = -vx       # wall-normal velocity
        T_ref = max(T_nom, 0.1)

        # ── Layer 1 & 2: Detection gate ───────────────────────────────────────
        # If wall has not been detected (d > detection_range) or drone is
        # still outside the CBF zone (d > d_safe), do nothing at all.
        if d > d_safe:
            return T_nom, tau_nom, False

        # ── Layer 4: Abort — hard retreat to d_safe ───────────────────────────
        # This is the only mode where CBF takes position authority.
        # Apply a smooth torque to tilt drone away from wall and let the
        # position controller (which has been redirected to x_retreat) handle
        # the actual retreat trajectory. The CBF here adds extra push beyond
        # what the position controller commands.
        if mode == 'abort':
            # The position controller has already been redirected to x_retreat by
            # run_sim.py. It generates the correct positive theta_des for retreat.
            # Adding extra torque here fights the inner loop: once theta overshoots
            # theta_des, the inner loop produces negative tau to correct; the CBF
            # adding positive tau wins, theta grows without bound → 4x overshoot.
            # Correct behaviour: let the controller manage the retreat unmodified.
            # Only flag cbf_active=True so the logger can identify abort periods.
            return T_nom, tau_nom, True

        # ── Layer 3: Permissive CBF — additive corrections only ───────────────
        # Drone is inside d_safe and NOT in abort. The controller's intended
        # direction is always preserved. The CBF never commands retreat here.
        # Three corrections are applied additively when triggered:
        #
        #   3a. Progressive speed limiting:
        #       As M decreases toward M_abort, the permitted closing speed
        #       scales down proportionally. At M_safe the full vn_max is
        #       allowed; at M_abort the limit reaches zero. This ensures the
        #       drone slows gradually as it approaches — it does NOT stop it.
        #
        #   3b. Angular rate damping:
        #       Wall aerodynamics cause oscillatory roll/pitch. When |theta_dot|
        #       exceeds omega_cbf_thresh, a damping torque is added to suppress
        #       the oscillation without changing position or approach direction.
        #
        #   3c. Thrust saturation guard:
        #       When thrust is near T_max (Gamma boost near wall), a small
        #       level-flight torque reduces horizontal demand and frees up
        #       thrust headroom for altitude hold.

        tau_correction = 0.0
        T_correction   = 0.0
        any_active     = False

        # ── Correction 3a: Progressive speed limiting ─────────────────────────
        # Compute how far M is above the abort threshold as a fraction of the
        # full operating range [M_abort, M_safe]. This fraction scales the
        # permitted closing speed from vn_max (at M_safe) down to 0 (at M_abort).
        # Any closing speed above this scaled limit gets a proportional braking
        # torque. The controller can still close — just not faster than the limit.
        M_safe_val  = params["M_safe"]
        M_abort_val = params["M_abort"]
        M_range     = M_safe_val - M_abort_val           # total operating range
        margin_frac = float(np.clip((M - M_abort_val) / M_range, 0.0, 1.0))
        v_limit = vn_max * margin_frac                   # permitted closing speed

        closing_speed = max(0.0, -vn)                    # only closing counts
        if closing_speed > v_limit:
            excess = closing_speed - v_limit
            # Braking: need ax < 0 to decelerate. With ax=-(T/m)*sin(theta),
            # a positive delta_theta produces negative ax → decelerates.
            ax_req = gd * excess
            delta_theta = float(np.clip(ax_req * m / T_ref,
                                        -params["theta_max"],
                                         params["theta_max"]))
            tau_correction += I * params["kp_th"] * 0.4 * delta_theta
            any_active = True

        # ── Correction 3b: Angular rate damping ──────────────────────────────
        omega_cbf_thresh = params.get("omega_cbf_thresh", omega_max * 0.6)
        if abs(theta_dot) > omega_cbf_thresh:
            excess_rate = abs(theta_dot) - omega_cbf_thresh
            damp_tau    = -np.sign(theta_dot) * I * params["kd_th"] * excess_rate * 0.5
            tau_correction += damp_tau
            any_active = True

        # ── Correction 3c: Thrust saturation guard ────────────────────────────
        if (1.0 - T_nom / T_max) < 0.25:
            tau_correction += -theta * I * params["kp_th"] * 0.1
            any_active = True

        # ── Apply corrections ─────────────────────────────────────────────────
        if not any_active:
            return T_nom, tau_nom, False

        tau_safe = tau_nom + tau_correction
        T_safe   = T_nom   + T_correction

        tau_max  = params["theta_max"] * I * params["kp_th"] * 2.0
        tau_safe = float(np.clip(tau_safe, -tau_max, tau_max))
        T_safe   = float(np.clip(T_safe, 0.0, T_max))

        cbf_active = abs(tau_correction) > 0.0001 or abs(T_correction) > 0.001

        return T_safe, tau_safe, cbf_active