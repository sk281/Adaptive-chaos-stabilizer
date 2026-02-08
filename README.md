# Safe Chaos Nudge – Lightweight Layered Chaos Controller

A simple, fast, real-time controller that tames chaotic systems (like the Lorenz attractor) without heavy math, training, or compute.

### What it does
It keeps error from exploding in chaotic trajectories by:
- Watching error growth **per component** (x, y, z separately)
- Using **bidirectional layers** (positive/negative nudges tracked separately)
- Applying **strong nudges early** (when error is growing fast) then **tapering them down** (weak nudges later to avoid overshooting and flipping the system)
- Adding **opposite-side decay** (reduces layers on the non-active side)
- Hard **caps** to prevent dangerous big corrections
- Optional **look-ahead cut** (estimates how much the next part of the path will naturally cancel error, then only cuts the leftover)

### Results on noisy Lorenz path
- Uncontrolled: error explodes to ~1.12
- This controller (with look-ahead): error stays at ~0.49 (~56% reduction)
- Hybrid with OGY (when error is low): error locks down to ~0.18 (~84% reduction)

All with **tiny code**, **no training**, and **real-time speed** (microseconds per step).

### Why it's different / useful
Most chaos controllers either:
- Overshoot and flip the system (too aggressive)
- Do almost nothing (too weak)
- Or are heavy/complex (ML, big MPC, pre-computed orbits)

This one is **safe-first**: strong when needed, gentle when risky, easy to understand and deploy on anything (robots, embedded devices, simulations).
Coulnt make the graphs right!
### Quick start
```bash
pip install numpy scipy matplotlib
python lorenz_sim.py