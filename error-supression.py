import numpy as np
import matplotlib.pyplot as plt

# ============================
# Common parameters
# ============================
steps = 4000
X = 0.02
noise_max = 5 * X
rng = np.random.default_rng(1)

# ============================================================
# MODEL 1: VECTOR ROUNDING ERROR (the "previous one")
# ============================================================
# This is what caused confusion:
# - error is a vector
# - norm can decrease due to cancellation

e_unc = np.zeros(3)
e_ctrl = np.zeros(3)

err_unc_vec = np.zeros(steps)
err_ctrl_vec = np.zeros(steps)

for k in range(steps):
    noise = rng.uniform(-noise_max, noise_max, size=3)

    # uncontrolled vector accumulation
    e_unc += noise

    # controlled: subtract X in the direction of the error vector
    if np.linalg.norm(e_ctrl) > 0:
        bleed = X * e_ctrl / np.linalg.norm(e_ctrl)
    else:
        bleed = np.zeros(3)

    e_ctrl += noise - bleed

    err_unc_vec[k] = np.linalg.norm(e_unc)
    err_ctrl_vec[k] = np.linalg.norm(e_ctrl)

# ============================================================
# MODEL 2: SCALAR MAGNITUDE BUDGET (the "correct" one)
# ============================================================
# This matches your actual mental model:
# - error magnitude never cancels
# - always grows
# - control removes exactly X per tick

E_unc = 0.0
E_ctrl = 0.0

err_unc_scalar = np.zeros(steps)
err_ctrl_scalar = np.zeros(steps)

for k in range(steps):
    injection = rng.uniform(X, noise_max)

    # uncontrolled
    E_unc += injection

    # controlled
    E_ctrl += injection
    E_ctrl = max(E_ctrl - X, 0.0)

    err_unc_scalar[k] = E_unc
    err_ctrl_scalar[k] = E_ctrl

# ============================================================
# PLOTS
# ============================================================

# ---- Plot 1: Vector model ----
plt.figure(figsize=(10, 5))
plt.plot(err_unc_vec, color="red", label="Uncontrolled (vector norm)")
plt.plot(err_ctrl_vec, color="blue", label="Controlled (−X along direction)")
plt.axhline(X, color="black", linestyle="--", alpha=0.6, label="X")
plt.title("Vector Rounding Error (Norm Can Go Up or Down)")
plt.xlabel("Tick")
plt.ylabel("‖error vector‖")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# ---- Plot 2: Scalar model ----
plt.figure(figsize=(10, 5))
plt.plot(err_unc_scalar, color="red", label="Uncontrolled (monotonic)")
plt.plot(err_ctrl_scalar, color="blue", label="Controlled (−X per tick)")
plt.axhline(X, color="black", linestyle="--", alpha=0.6, label="X")
plt.title("Scalar Rounding Error (Pure Accumulated Magnitude)")
plt.xlabel("Tick")
plt.ylabel("Accumulated error magnitude")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# ============================================================
# Final values
# ============================================================
print("FINAL VALUES")
print(f"Vector uncontrolled:  {err_unc_vec[-1]:.3f}")
print(f"Vector controlled:    {err_ctrl_vec[-1]:.3f}")
print(f"Scalar uncontrolled:  {err_unc_scalar[-1]:.3f}")
print(f"Scalar controlled:    {err_ctrl_scalar[-1]:.3f}")
