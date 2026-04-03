# state_machine.py
# Hybrid docking state machine.
# Reads the Dockability Margin M at each timestep and returns
# the current operating mode as a string.

class StateMachine:

    HYSTERESIS = 0.05

    def __init__(self, params):
        self.mode = 'free'
        self.params = params

    def update(self, M, state, params):
        """
        Evaluate transition conditions and return the current mode.

        Modes:
            'free'    -- nominal controller, no CBF
            'surface' -- CBF active, slow approach continues
            'docking' -- CBF active, hold position
            'abort'   -- retreat to x_retreat, CBF active

        Parameters
        ----------
        M      : float  Current Dockability Margin
        state  : array  Current drone state [x,z,vx,vz,theta,theta_dot]
        params : dict   Simulation parameters from params.py

        Returns
        -------
        mode : str  Current operating mode
        """
        M_safe  = params['M_safe']
        M_dock  = params['M_dock']
        M_abort = params['M_abort']
        H       = self.HYSTERESIS

        if self.mode == 'free':
            if M < M_safe:
                self.mode = 'surface'

        elif self.mode == 'surface':
            if M < M_dock:
                self.mode = 'docking'
            elif M >= M_safe + H:
                self.mode = 'free'

        elif self.mode == 'docking':
            if M < M_abort:
                self.mode = 'abort'
            elif M >= M_dock + H:
                self.mode = 'surface'

        elif self.mode == 'abort':
            if M >= M_safe + H:
                self.mode = 'free'

        return self.mode