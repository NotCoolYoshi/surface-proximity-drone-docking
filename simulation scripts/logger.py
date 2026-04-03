# logger.py
import numpy as np
import os
from datetime import datetime


class Logger:

    def __init__(self):
        self.reset()

    def reset(self):
        self.t = []
        self.x = []
        self.z = []
        self.vx = []
        self.vz = []
        self.theta = []
        self.thdot = []
        self.T = []
        self.tau = []
        self.T_nom = []        # NEW: nominal thrust before CBF
        self.d = []
        self.vn = []
        self.fwall = []
        self.Gamma = []
        self.M = []
        self.Md = []
        self.Mv = []
        self.Mdelta = []
        self.Momega = []
        self.Mu = []
        self.Mr = []
        self.mode = []         # NEW: state machine mode
        self.cbf_active = []   # NEW: CBF intervention flag

    def log(self, t, state, T, tau, T_nom, fwall, Gamma,
            M, subs, mode, cbf_active, params):
        x, z, vx, vz, theta, thdot = state
        self.t.append(t)
        self.x.append(x); self.z.append(z)
        self.vx.append(vx); self.vz.append(vz)
        self.theta.append(theta); self.thdot.append(thdot)
        self.T.append(T); self.tau.append(tau)
        self.T_nom.append(T_nom)
        self.d.append(params['wall_x'] - x)
        self.vn.append(-vx)
        self.fwall.append(fwall)
        self.Gamma.append(Gamma)
        self.M.append(M)
        self.Md.append(subs['Md'])
        self.Mv.append(subs['Mv'])
        self.Mdelta.append(subs['Mdelta'])
        self.Momega.append(subs['Momega'])
        self.Mu.append(subs['Mu'])
        self.Mr.append(subs['Mr'])
        self.mode.append(mode)
        self.cbf_active.append(cbf_active)

    def save(self, name, params, notes=''):
        os.makedirs('data', exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = f'data/{name}_{timestamp}.npz'
        np.savez_compressed(
            path,
            t=self.t, x=self.x, z=self.z,
            vx=self.vx, vz=self.vz,
            theta=self.theta, thdot=self.thdot,
            T=self.T, tau=self.tau,
            T_nom=self.T_nom,
            d=self.d, vn=self.vn,
            fwall=self.fwall, Gamma=self.Gamma,
            M=self.M,
            Md=self.Md, Mv=self.Mv, Mdelta=self.Mdelta,
            Momega=self.Momega, Mu=self.Mu, Mr=self.Mr,
            mode=self.mode,
            cbf_active=self.cbf_active,
            params=str(params),
            notes=notes,
        )
        print(f'[Logger] Saved {len(self.t)} timesteps -> {path}')
        return path