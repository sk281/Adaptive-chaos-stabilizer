# Safe Chaos Nudge - Bidirectional Layered Controller

Lightweight real-time stabilizer for chaotic systems (e.g. Lorenz attractor).

Features:
- Per-component error growth detection
- Bidirectional positive/negative layers
- Decreasing gain taper (strong early, weak late to avoid overshoot)
- Opposite-side decay + hard caps

Achieves ~90–96% error reduction over 150–500 time units in Lorenz.

## Quick run
```bash
pip install numpy scipy matplotlib
python lorenz_sim.py
See plots/ for results.
MIT license — feel free to fork/use/modify.