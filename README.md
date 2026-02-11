# Chaos Nudge Hybrid – Layered Controller + OGY

A hand-built, real-time controller for chaotic systems (tested on Lorenz attractor).

**What it does**:
- Per-axis error growth detection
- Bidirectional layers (positive/negative tracked separately)
- Decreasing gain taper (strong early → weak late to prevent overshoot)
- Look-ahead cut (estimates next curve cancels some error → cut only leftover)
- Switches to simple OGY when error norm < 1.5–2.0 for long-term locking

**Current results** (100 time units, noisy paths):
- Uncontrolled: error explodes (~10–40 final)
- Hybrid: ~55–65% reduction (final error ~15–20, no consistent lock-in)

OGY rarely activates (error stays high). Still safe (no flips), but nudge strength needs tuning for better performance.

**Limitations & Next Steps**
Built from intuition and manual tuning — no formal stats, ablation studies, or large-scale optimization.  
Performance varies by seed/parameters and may differ on other systems.

If you're good with statistics, ML, or dynamical systems, this is a great candidate for:
- Running thousands of paths (mean/std/confidence intervals)
- Ablation studies on each feature (layers, taper, look-ahead, etc.)
- Correlation analysis (curve intensity, velocity, turbulence, dt growth, remainders) vs error reduction
- Comparison to other lightweight methods (Pyragas, adaptive gain, etc.)

Contributions welcome — especially from stats/ML people who want to quantify what each part actually does.

MIT license — fork, extend, publish, whatever you like.

**Quick start**
```bash
pip install numpy scipy matplotlib
python chaos_nudge_hybrid.py