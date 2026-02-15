# Chaos Nudge Hybrid

A lightweight, hand-crafted, real-time controller for chaotic systems — tested on the Lorenz attractor.

Built from pure intuition: detect growth per axis, apply layered nudges that taper off to avoid overshoot, look ahead to let upcoming curves do some natural cancellation, and switch to OGY when close enough for long-term locking.

**Features**
- Per-axis error growth detection (x, y, z separately)
- Bidirectional layers (positive and negative counters tracked independently)
- Decreasing gain taper (strong nudges early → weak late to prevent flips)
- Opposite-side decay + hard nudge caps (safety first)
- Look-ahead cut (estimates next curve cancels some error → only cut the leftover)
- Velocity damping (stronger when error changes fast)
- Dynamic asymmetry (cut harder in the "bad" direction when curve × velocity is high)
- OGY hybrid (activates at error norm < ~0.6, lighter kicks for stability)

**Current results** (100 time units, noisy starting points)
- Uncontrolled: error explodes (often 20–40+ final)
- Hybrid: ~50–65% reduction in average runs (final error ~15–25, sometimes flattens low after OGY)
- Best runs: up to ~75–85% when OGY locks early

OGY does not always trigger (depends on path/seed — error may stay high).  
Still safe (no flips, no blow-ups), but nudge strength needs tuning for consistency.

**Limitations & Next Steps**
This was built from trial-and-error and intuition — no formal statistics, ablation studies, or large-scale optimization.  
Performance varies by random seed, parameters, and system — it may behave differently on other chaotic systems.

If you're good with statistics, machine learning, or dynamical systems, this is a great candidate for:
- Running thousands of paths to compute mean/std/confidence intervals
- Ablation studies on each feature (layers, taper, look-ahead, asymmetry, etc.)
- Correlation analysis (curve intensity, velocity, turbulence proxies, dt growth, remainders) vs error reduction
- Comparison to other lightweight methods (Pyragas delay, adaptive gain, etc.)

Contributions welcome — especially from stats/ML people who want to quantify how much each part actually contributes.

MIT license — fork, extend, publish, whatever you want.
,

### Extra Idea: Component-to-Group Inverse Influence Score (CG-Influence)

While building this controller I came up with a simple way to map asymmetric influence between groups of variables.

**Definition**  
For component x in group A and group B:

CG-Influence_{B→A}(x) = ∑_{y ∈ B, y ≠ x} w_y · corr(x, y)  
(with ∑ w_y = 1, default uniform)

Symmetrically for CG-Influence_{A→B}(y).

**Key points**:
- Asymmetric (B→A ≠ A→B)
- Component-to-group (whole group B's influence on single x)
- Bounded [-1, 1]
- Cheap to compute (just correlations + weighted sum)
- Perturbable (change a score, re-run, see effects)

Use it to spot leverage points (high score = high sensitivity), track changes over time, or test "what if this influence doubles?"

No formal proofs or large-scale tests — just an intuition tool that helped me understand influence flow in the system.

If anyone wants to formalize it, prove properties, or apply it elsewhere — feel free.
**Quick start**
```bash
pip install numpy scipy matplotlib
python chaos_nudge_hybrid.py


