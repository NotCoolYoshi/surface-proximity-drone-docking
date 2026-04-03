# visualize.py
import numpy as np
import matplotlib.pyplot as plt
import os

C = {
    'blue': '#185FA5',
    'teal': '#0F6E56',
    'amber': '#854F0B',
    'red': '#A32D2D',
    'purple': '#534AB7',
    'gray': '#5F5E5A',
    'safe': '#1D9E75',
    'warn': '#EF9F27',
    'danger': '#E24B4A',
}


def load(path):
    data = np.load(path, allow_pickle=True)
    return {k: data[k] for k in data.files}


def plot_trajectory(d, params=None, save=True, fname='trajectory.png'):
    wall_x = float(params['wall_x']) if params else 2.0
    d_min = float(params['d_min']) if params else 0.05
    t = np.array(d['t'])
    x = np.array(d['x'])
    z = np.array(d['z'])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle('Trajectory', fontsize=13, fontweight='bold')

    ax1.plot(t, x, color=C['blue'], lw=1.8, label='x(t)')
    ax1.axhline(wall_x, color=C['red'], ls='--', lw=1.2, label='wall')
    ax1.axhline(wall_x - d_min, color=C['danger'], ls=':', lw=1.2, label='d_min')
    ax1.set_ylabel('horizontal position (m)')
    ax1.legend(fontsize=9); ax1.grid(alpha=0.3)

    ax2.plot(t, z, color=C['teal'], lw=1.8, label='z(t)')
    ax2.set_ylabel('vertical position (m)')
    ax2.set_xlabel('time (s)')
    ax2.legend(fontsize=9); ax2.grid(alpha=0.3)

    plt.tight_layout()
    os.makedirs('plots', exist_ok=True)
    if save:
        plt.savefig(f'plots/{fname}', dpi=150)
        print(f'[visualize] Saved plots/{fname}')
    plt.show()


def plot_margins(d, params=None, save=True, fname='margins.png'):
    t = np.array(d['t'])
    M = np.array(d['M'])
    Md = np.array(d['Md'])
    Mv = np.array(d['Mv'])
    Mdelta = np.array(d['Mdelta'])
    Momega = np.array(d['Momega'])
    Mu = np.array(d['Mu'])
    Mr = np.array(d['Mr'])

    M_safe = float(params['M_safe']) if params else 0.5
    M_dock = float(params['M_dock']) if params else 0.4
    M_abort = float(params['M_abort']) if params else 0.2

    sub_colors = [C['blue'], C['teal'], C['amber'],
                  C['red'], C['purple'], C['gray']]
    sub_labels = ['Md (clearance)', 'Mv (velocity)',
                  'Mdelta (alignment)', 'Momega (ang. rate)',
                  'Mu (authority)', 'Mr (residual)']
    sub_data = [Md, Mv, Mdelta, Momega, Mu, Mr]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig.suptitle('Dockability Margin', fontsize=13, fontweight='bold')

    for s, col, lbl in zip(sub_data, sub_colors, sub_labels):
        ax1.plot(t, s, color=col, alpha=0.75, lw=1.5, label=lbl)
    ax1.set_ylabel('sub-margin value')
    ax1.set_ylim(-0.05, 1.08)
    ax1.legend(ncol=3, fontsize=8)
    ax1.grid(alpha=0.3)

    ax2.fill_between(t, M, 0, alpha=0.12, color=C['blue'])
    ax2.plot(t, M, color=C['blue'], lw=2.2, label='M (overall)')
    ax2.axhline(M_safe, color=C['warn'], ls='--', lw=1.2,
                label=f'M_safe={M_safe}')
    ax2.axhline(M_dock, color=C['amber'], ls='--', lw=1.2,
                label=f'M_dock={M_dock}')
    ax2.axhline(M_abort, color=C['danger'], ls='--', lw=1.2,
                label=f'M_abort={M_abort}')
    ax2.set_ylabel('M'); ax2.set_xlabel('time (s)')
    ax2.set_ylim(-0.05, 1.08)
    ax2.legend(fontsize=9); ax2.grid(alpha=0.3)

    plt.tight_layout()
    os.makedirs('plots', exist_ok=True)
    if save:
        plt.savefig(f'plots/{fname}', dpi=150)
        print(f'[visualize] Saved plots/{fname}')
    plt.show()


def plot_disturbance(d, save=True, fname='disturbance.png'):
    dist = np.array(d['d'])
    fwall = np.array(d['fwall'])
    Gamma = np.array(d['Gamma'])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    fig.suptitle('Wall disturbance model', fontsize=13, fontweight='bold')

    ax1.plot(dist, fwall, color=C['red'], lw=1.8)
    ax1.axhline(0, color='gray', lw=0.8, ls='--')
    ax1.set_ylabel('f_wall (N)')
    ax1.set_title('disturbance force vs distance')
    ax1.grid(alpha=0.3)

    ax2.plot(dist, Gamma, color=C['amber'], lw=1.8)
    ax2.axhline(1.0, color='gray', lw=0.8, ls='--', label='no wall effect')
    ax2.set_ylabel('Gamma (thrust modifier)')
    ax2.set_xlabel('wall distance d (m)')
    ax2.set_title('thrust modification vs distance')
    ax2.legend(fontsize=9); ax2.grid(alpha=0.3)

    plt.tight_layout()
    os.makedirs('plots', exist_ok=True)
    if save:
        plt.savefig(f'plots/{fname}', dpi=150)
        print(f'[visualize] Saved plots/{fname}')
    plt.show()


def plot_hover_error(d, x_des, z_des, save=True, fname='hover_error.png'):
    t = np.array(d['t'])
    ex = np.abs(np.array(d['x']) - x_des)
    ez = np.abs(np.array(d['z']) - z_des)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    fig.suptitle('Hover position error', fontsize=13, fontweight='bold')

    for ax, err, lbl, col in [(ax1, ex, 'x error (m)', C['blue']),
                               (ax2, ez, 'z error (m)', C['teal'])]:
        ax.plot(t, err, color=col, lw=1.8)
        ax.axhspan(0, 0.05, alpha=0.15, color=C['safe'],
                   label='< 5 cm threshold')
        ax.set_ylabel(lbl); ax.legend(fontsize=9); ax.grid(alpha=0.3)
    ax2.set_xlabel('time (s)')

    plt.tight_layout()
    os.makedirs('plots', exist_ok=True)
    if save:
        plt.savefig(f'plots/{fname}', dpi=150)
        print(f'[visualize] Saved plots/{fname}')
    plt.show()

def plot_modes(d, params=None, save=True, fname='modes.png'):
    """
    Plot state machine modes and CBF intervention over time.
    Top panel: M(t) with colored background bands showing active mode.
    Bottom panel: T_nom vs T_safe to show CBF thrust correction.
    """
    t          = np.array(d['t'])
    M          = np.array(d['M'])
    T_safe     = np.array(d['T'])
    T_nom      = np.array(d['T_nom'])
    modes      = d['mode']
    cbf_active = np.array(d['cbf_active'])

    M_safe  = float(params['M_safe'])  if params else 0.6
    M_dock  = float(params['M_dock'])  if params else 0.4
    M_abort = float(params['M_abort']) if params else 0.15

    # Mode colors
    mode_colors = {
        'free':    '#1D9E75',   # green
        'surface': '#EF9F27',   # yellow
        'docking': '#E08020',   # orange
        'abort':   '#E24B4A',   # red
    }

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle('State Machine Modes and CBF Intervention',
                 fontsize=13, fontweight='bold')

    # Top panel: M(t) with mode background bands
    prev_t = t[0]
    prev_mode = str(modes[0])
    for i in range(1, len(t)):
        curr_mode = str(modes[i])
        if curr_mode != prev_mode or i == len(t) - 1:
            color = mode_colors.get(prev_mode, '#aaaaaa')
            ax1.axvspan(prev_t, t[i], alpha=0.15, color=color)
            prev_t   = t[i]
            prev_mode = curr_mode

    ax1.plot(t, M, color=C['blue'], lw=2.0, label='M (overall)')
    ax1.axhline(M_safe,  color=C['warn'],   ls='--', lw=1.2,
                label=f'M_safe={M_safe}')
    ax1.axhline(M_dock,  color=C['amber'],  ls='--', lw=1.2,
                label=f'M_dock={M_dock}')
    ax1.axhline(M_abort, color=C['danger'], ls='--', lw=1.2,
                label=f'M_abort={M_abort}')

    # Add mode legend patches
    import matplotlib.patches as mpatches
    patches = [mpatches.Patch(color=mode_colors[m], alpha=0.4, label=m)
               for m in ['free', 'surface', 'docking', 'abort']]
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles=handles + patches, fontsize=8, ncol=4)
    ax1.set_ylabel('M')
    ax1.set_ylim(-0.05, 1.08)
    ax1.grid(alpha=0.3)

    # Bottom panel: T_nom vs T_safe
    ax2.plot(t, T_nom,  color=C['gray'],  lw=1.5, ls='--',
             label='T_nom (nominal)')
    ax2.plot(t, T_safe, color=C['blue'],  lw=1.8,
             label='T_safe (CBF filtered)')
    ax2.fill_between(t, T_nom, T_safe,
                     where=cbf_active.astype(bool),
                     alpha=0.3, color=C['danger'],
                     label='CBF active')
    ax2.set_ylabel('Thrust (N)')
    ax2.set_xlabel('time (s)')
    ax2.legend(fontsize=9)
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    os.makedirs('plots', exist_ok=True)
    if save:
        plt.savefig(f'plots/{fname}', dpi=150)
        print(f'[visualize] Saved plots/{fname}')
    plt.show()