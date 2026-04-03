# surface-proximity-drone-docking
Turns out flying a drone near a wall is pretty sketch and the aerodynamics get weird fast. So I've been working on building a safety layer using Control Barrier Functions and a stability metric I designed from scratch. Sim passes 19/19 checks and 50 Monte Carlo trials. Still working on refining the CBF to be less drastic before moving to hardware.

This project still has a ton of work, I've only been implementing software testing in virtual environments without any actual hardware testing yet. Once I have hardware testing, I'll be posting videos of my progress. If anyone reading this has knowledge in coding for UAV control systems, I'd love to hear any feedback or suggestions! I'm still a beginner and this is probably my first project of such a large caliber (spent 100+ hours on this and had a lot of all nighter crash outs haha). 

If you're wondering where all of this content came at once, it's because I never got to uploading this project to github (I know, sacrilege), only updating information to a discord server I created just for myself to document everything (again, more sacrilege sorry). I only got to uploading this project now. I have a lot more information to share beyond this repository but its all a cluttered mess in that discord server, but feel free to ask me any questions about this project. 
---

## Table of Contents

- [Motivation](#motivation)
- [Key Contributions](#key-contributions)
- [System Architecture](#system-architecture)
- [Mathematical Formulation](#mathematical-formulation)
- [Simulation Results](#simulation-results)
- [Verification](#verification)
- [Design Iterations](#design-iterations)
- [File Structure](#file-structure)
- [Running the Simulation](#running-the-simulation)
- [Roadmap](#roadmap)
- [References](#references)

---

## Motivation
A while ago, I saw a really cool video of a drone that was able to dock onto surfaces to drill holes developed by engineering students at ETH Zurich (https://www.youtube.com/watch?v=kGuKkIqlAqg). It was pretty awesome to see how drones could be used for construction applications through surface docking mechanisms. However, it also led me to wonder how UAV controllers could get drones to attach to surfaces accurately. 

Flying a drone near a wall is aerodynamically chaotic. Rotors operating close to a surface generate recirculating airflow, asymmetric pressure gradients, and proximity-induced thrust modifications that conventional PID controllers are not designed to handle. The closer the drone gets, the worse it gets and at docking distances (< 0.3m), the disturbances are strong enough to destabilize the vehicle entirely.

Existing drone docking systems solve this with mechanical guides or vision alignment, but almost none of them account for the nonlinear aerodynamic behavior that emerges during close-proximity flight. If a controller assumes calm air and the air isn't calm, you either crash or you never dock.

This project addresses that gap with two contributions: a real-time **stability metric** that quantifies how close the drone is to losing control authority, and a **safety-constrained control layer** that keeps the drone stable without restricting intentional approach.

---

## Key Contributions

**1. Dockability Margin M (novel metric)**
A scalar ∈ [0, 1] computed in real time from six normalized safety sub-margins:

| Sub-margin | Symbol | What it measures |
|---|---|---|
| Clearance | M_d | Distance to wall vs. minimum safe distance |
| Approach velocity | M_v | Closing speed vs. maximum allowable |
| Alignment | M_δ | Body-frame docking axis tilt error |
| Angular rate | M_ω | Oscillatory instability from wall aerodynamics |
| Control authority | M_u | Remaining thrust headroom before saturation |
| Disturbance residual | M_r | Discrepancy between predicted and effective thrust |

M = min{M_d, M_v, M_δ, M_ω, M_u, M_r}

A value near 1.0 indicates safe, stable operation. A value approaching 0 indicates the drone is near loss of control. **This metric is the original contribution of this project** that provides a unified, interpretable scalar for surface-proximate flight safety which did not previously exist. 

**2. Layered CBF-QP Safety Filter**
A Control Barrier Function safety filter with four operating layers:

- **Layer 1/2 (open air / monitoring):** CBF fully inactive outside the 2m detection zone. No intervention.
- **Layer 3 (permissive corrections):** Inside 2m, applies three additive torque corrections only when genuinely needed — progressive speed limiting, angular rate damping, and thrust saturation guard. Never restricts intentional approach. Never commands retreat.
- **Layer 4 (abort only):** Hard override when M < M_abort = 0.15. Controller target redirected to the d_safe boundary. This is the *only* mode where the CBF takes position authority.

**3. Progressive Speed Limiting (Layer 3a)**
The permitted closing speed scales linearly with M:

```
v_limit = v_max × (M - M_abort) / (M_safe - M_abort)
```

At M = M_safe the drone can close at full v_max. As M drops toward M_abort the limit tapers to zero. This prevents M from collapsing before abort fires — the drone naturally slows as it approaches, so abort is only needed for genuine instability.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        run_sim.py                            │
│                    (simulation entry point)                  │
└──────────┬──────────────────────────────────────────────────┘
           │ state vector [x, z, vx, vz, θ, θ̇]
           ▼
┌──────────────────┐    ┌──────────────────┐
│   dynamics.py    │    │  controller.py   │
│  equations of    │◄───│  cascaded PD     │
│  motion + wall   │    │  outer: position │
│  disturbance     │    │  inner: attitude  │
└──────────┬───────┘    └──────────────────┘
           │                      ▲
           │ T_eff, f_wall, Γ(d)  │ T_safe, τ_safe
           ▼                      │
┌──────────────────┐    ┌─────────┴────────┐
│    margin.py     │    │     cbf.py       │
│  Dockability     │───►│  CBF safety      │
│  Margin M and    │    │  filter — layers │
│  six sub-margins │    │  1 through 4     │
└──────────┬───────┘    └──────────────────┘
           │ M                    ▲
           ▼                      │ mode string
┌──────────────────┐              │
│ state_machine.py │──────────────┘
│  free / surface  │
│  docking / abort │
└──────────────────┘
           │
           ▼
┌──────────────────┐
│    logger.py     │
│  logs all state, │
│  M, mode, CBF    │
│  to .npz file    │
└──────────────────┘
```

---

## Mathematical Formulation

### 2D Planar Drone Dynamics

State vector: **x** = [x, z, vx, vz, θ, θ̇]

**Equations of motion:**

```
ẋ  = vx
ż  = vz
mv̇x = −T_eff · sin(θ) + f_wall / m
mv̇z =  T_eff · cos(θ) − g
Iθ̈  = τ
```

**Wall-modified thrust:**

```
T_eff(d) = T · Γ(d)
Γ(d)     = 1 + α · exp(−β·d)
```

**Wall disturbance force:**

```
f_wall = −[c₀·e^(−λd) + c₁·e^(−λd)·vn + c₂·e^(−λd)·vn|vn|]
```

where d = wall_x − x is the signed distance to the wall and vn = −vx is the wall-normal velocity.

### Dockability Margin

```
M = min{M_d, M_v, M_δ, M_ω, M_u, M_r}

M_d = sat((d − d_min) / (d_safe − d_min))
M_v = sat(1 − max(0, −vn) / vn_max)
M_δ = sat(1 − |θ| / δ_max)
M_ω = sat(1 − |θ̇| / ω_max)
M_u = sat(1 − T_cmd / T_max)
M_r = sat(1 − |T_eff − T_cmd| / (m · r_max))

where sat(x) = min(1, max(0, x))
```

### CBF Safety Condition

For each safety constraint h_i(x) ≥ 0, the CBF condition requires:

```
ḣ_i(x, u) + α_i(h_i(x)) ≥ 0
```

where α_i is a class-K function, typically α_i(h) = γ·h. The safe control input is obtained via:

```
min_u  ‖u − u_nom‖²
s.t.   ḣ_i(x, u) + γ·h_i(x) ≥ 0  ∀i
```

This ensures safety constraints are enforced while minimally modifying the nominal controller output.

---

## Simulation Results

### Experiment 7 — CBF vs No-CBF Wall Approach

Both runs start at x = 0.5m and target the wall at x = 1.93m. The No-CBF run has zero safety action. The CBF run has the full four-layer filter active.

![Experiment 7 — Corrected Design](results/exp7_corrected_design.png)

**Key observations:**
- Blue (CBF) and red (No-CBF) trajectories are **numerically identical** before abort fires (max diff = 0.00094m) — confirming CBF does not interfere with intentional approach
- CBF abort fires at t ≈ 7s, retreats 0.97m, M recovers above M_safe
- No-CBF drone oscillates against the wall indefinitely with min(M) = 0.060
- CBF min(M) = 0.137 — stays near M_abort due to unavoidable 200ms RK45 integration latency

### Monte Carlo — 50 Trials, Random Initial Conditions

50 trials with randomised starting position (x ∈ [0.10, 1.50]m), velocity (vx ∈ [−0.30, 0.50] m/s), altitude, tilt, and angular rate. Each trial run twice — CBF on and CBF off.

![Monte Carlo Milestone](results/monte_carlo_milestone.png)

| Metric | Result |
|---|---|
| CBF wall violations (d < d_min) | **0 / 50** |
| No-CBF closest approach | 0.148m |
| CBF closest approach | 0.271m |
| Abort fired | 50 / 50 trials |
| M recovered above M_safe post-abort | 50 / 50 trials |
| Trials ending in abort mode | 0 / 50 |

**Phase 2 milestone achieved: d ≥ d_min in all CBF trials.**

---

## Verification

Full 19-check system verification run against final simulation:

```
================================================================
FULL SYSTEM VERIFICATION
================================================================

── Block 1: Phase 1 Fundamentals ──────────────────────────────
[1.1] Hover thrust ≈ m*g (t<1s, pre-wall):   ✓ PASS  T=9.677 N
[1.2] f_wall attractive at d<0.5m:            ✓ PASS  mean=−0.420 N
[1.3] Gamma > 1.0 at d<0.5m:                 ✓ PASS  min=1.071
[1.4] M decreases during approach:            ✓ PASS  0.744 → 0.239

── Block 2: Detection and Zone Gating ─────────────────────────
[2.1] CBF inactive outside d_safe=2.0m:       ✓ PASS  interventions=0
[2.2] Mode=free outside d_safe:               ✓ PASS
[2.3] No free→abort jump:                     ✓ PASS

── Block 3: CBF Surface/Docking Behavior ───────────────────────
[3.1] CBF corrects in surface/docking:        ✓ PASS  steps=1057
[3.2] Approach matches No-CBF pre-abort:      ✓ PASS  max_diff=0.00094m
[3.3] Closing speed within progressive limit: ✓ PASS  max_overshoot=0.050 m/s

── Block 4: Abort Behavior ─────────────────────────────────────
[4.1] Abort fires in CBF run:                 ✓ PASS  steps=3863
[4.2] Abort absent in No-CBF run:             ✓ PASS  steps=0
[4.3] CBF min(M) ≥ M_abort−0.02 (latency):   ✓ PASS  min(M)=0.137
[4.4] No-CBF min(M) < M_abort (unsafe):      ✓ PASS  min(M)=0.060
[4.5] Drone retreats during abort:            ✓ PASS  retreat=0.975m
[4.6] No theta runaway:                       ✓ PASS  max=29.2°/s
[4.7] Retreat bounded (x > −1.0m):           ✓ PASS  min_x=−0.045m

── Block 5: Recovery After Abort ───────────────────────────────
[5.1] M recovers above M_safe post-abort:     ✓ PASS  max_post=0.754
[5.2] Not in abort at end of sim:             ✓ PASS  final_mode=free

================================================================
RESULT: 19/19 checks passed
================================================================
```

---

## Design Iterations

This project went through several significant redesigns. Documenting them here because the debugging process is as important as the final result.

**CBF abort layer sign error**
The original abort handler added a torque correction `τ_abort = −γ · proximity · 0.3`. The intent was to push the drone away from the wall. The dynamics convention is `ax = −(T/m)·sin(θ)`, which means a positive theta produces negative ax (deceleration of wall approach). The negative sign in `τ_abort` was therefore driving theta *negative* — pushing the drone *into* the wall harder during abort. At close range it overpowered the position controller entirely. Fixed by removing the extra torque and letting the position controller handle retreat unmodified once redirected to `x_retreat`.

**M_ABORT_BUFFER overreach**
To handle RK45 integration latency (the integrator takes ~200ms of substeps before a controller redirect takes physical effect), an early trigger buffer was added: fire abort when `M < M_abort + 0.045`. This fired the retreat handler when `M = 0.195` — well inside docking mode during a deliberate, controlled approach. The correct fix was to implement progressive speed limiting in Layer 3a so M never falls that fast in the first place, eliminating the need for a buffer at all.

**Ramp vs. fixed target in Experiment 7**
The initial experiment design used a 0.1 m/s ramp target. The CBF's Layer 3a threshold is 0.8 m/s. The ramp approach never reached that speed, so CBF and No-CBF produced numerically identical results and the experiment showed nothing. Switching to a fixed target lets the PD controller accelerate naturally, reaching ~0.9 m/s mid-approach and triggering Layer 3a.

---

## File Structure

```
surface-proximate-drone-docking/
├── README.md
├── docs/
│   ├── proposal.pdf            research proposal with full mathematical derivation
│   ├── phase1_guide.pdf        Phase 1 implementation guide
│   └── figures/                all key plots
├── simulation/
│   ├── params.py               all parameters with inline documentation
│   ├── dynamics.py             equations of motion + wall disturbance model
│   ├── controller.py           cascaded PD controller
│   ├── margin.py               Dockability Margin and six sub-margins
│   ├── cbf.py                  CBF safety filter — four-layer architecture
│   ├── state_machine.py        hybrid state machine (free/surface/docking/abort)
│   ├── logger.py               per-timestep data logging to .npz
│   ├── run_sim.py              simulation entry point — all experiments
│   ├── visualize.py            standard plot functions
│   └── monte_carlo.py          50-trial Monte Carlo robustness test
├── results/
│   ├── exp7_corrected_design.png
│   └── monte_carlo_milestone.png
└── hardware/                   Phase 3 — in progress
    └── README.md
```

---

## Running the Simulation

**Requirements**
```
python >= 3.10
numpy >= 1.24
scipy >= 1.10
matplotlib >= 3.7
```

Install:
```bash
pip install numpy scipy matplotlib
```

**Run Experiment 7 (CBF vs No-CBF wall approach):**
```bash
cd simulation
python run_sim.py
```

**Run Monte Carlo (50 trials, Phase 2 milestone):**
```bash
cd simulation
python monte_carlo.py
```

**Run full 19-check system verification:**
```bash
cd simulation
python run_sim.py
# then paste the verification script from docs/verification.py
```

Data is saved to `simulation/data/` as timestamped `.npz` files. Plots are saved to `simulation/plots/`.

---

## Roadmap

| Phase | Status | Description |
|---|---|---|
| Phase 1 | ✅ Complete | Python simulation — dynamics, controller, Dockability Margin |
| Phase 2 | ✅ Complete | CBF safety filter, state machine, Monte Carlo validation |
| Phase 3 | 🔧 In Progress | H-frame hardware build, PX4 integration, first flights |
| Phase 4 | 📋 Planned | Docking scatter validation vs. ETH Zurich ±10cm baseline |

**Phase 3 hardware stack:**
- Holybro X500 V2 frame (500mm wheelbase)
- Holybro Pixhawk 6X flight controller
- NVIDIA Jetson Orin Nano (companion computer)
- Livox Mid-360 LiDAR (wall detection + plane fitting)
- Benewake TFmini Plus × 4 (docking face distance sensors)
- Intel RealSense D455 (alignment and fiducial detection)

**Phase 4 target:** Beat the ETH Zurich perching drone benchmark of ±10cm docking scatter across 30 trials.

---

## References

1. ETH Zurich Perching Drone — benchmark target: ±10cm docking scatter
2. Ames, A.D., et al. — *Control Barrier Functions: Theory and Applications* (2019)
3. Lee, T., et al. — *Geometric Tracking Control of a Quadrotor UAV on SE(3)* (2010)
4. PX4 Autopilot — https://px4.io
5. Holybro X500 V2 — https://holybro.com

---

*Surface-Proximate Drone Docking Project — Sean Dai, Arizona*
*Phases 1–2 complete. Hardware build in progress.*
