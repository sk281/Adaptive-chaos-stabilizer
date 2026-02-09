# Chaos Nudge Hybrid – Your Layered Controller + OGY

A hand-built real-time controller for chaotic systems (Lorenz tested).

**What it does**:
- Per-axis error growth detection
- Bidirectional layers (pos/neg separate)
- Decreasing gain taper (strong early → weak late)
- Look-ahead cut (estimates next curve cancels some error → cut less)
- Switches to simple OGY when error norm < 1.5–2.0 for locking


Tune the settings for better results, Ogy not working properly
**Current results** (100 time units, noisy path):
- Uncontrolled: error explodes (~10–40 final)
- Hybrid: ~55–65% reduction (final error ~15–20, no lock-in yet)

OGY not activating in most runs (error stays high).  
Still safe (no flips), but nudge strength needs tuning for better performance.



**To run**:
```bash
pip install numpy scipy matplotlib
python chaos_nudge_hybrid.py