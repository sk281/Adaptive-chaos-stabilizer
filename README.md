# Chaos Control Nudge

A simple, interpretable heuristic for delaying divergence in chaotic dynamical systems.

## Core Idea
- **Every tick**: Small sign-reversed nudge (±0.01 × t) when error growth direction changes (push back when accelerating, allow breathing room when stabilizing).
- **Periodic resets**: Stronger nudges at fixed intervals (e.g., every 20n light, every 45n strong) to handle accumulated drift.

## Results (from tests)
- Lorenz attractor (t=150–300): ~95–96% error reduction vs baseline
- Restricted 3-body: similar ~95% reduction with per-body error handling
- Outperforms standard adaptive step-size in long runs

## Usage
See `Controled_Lorenz.py` for the implementation.

## Status
Scouting prototype — not building a product.  
Open to passive equity/partnership if someone wants to productize for real applications (space, medical, turbulence, etc.). DM or email me.

License: MIT