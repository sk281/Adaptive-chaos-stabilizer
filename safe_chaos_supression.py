import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Lorenz parameters
sigma = 10.0
rho = 28.0
beta = 8.0 / 3.0

def lorenz(t, y):
    x, y_, z = y
    return np.array([sigma * (y_ - x),
                     x * (rho - z) - y_,
                     x * y_ - beta * z])

# Your ChaosController class (with look-ahead cut)
class ChaosController:
    def __init__(self, base_strength=0.01, growth_threshold=0.05,
                 opposite_decay=0.85, max_layers=10, max_nudge=0.05,
                 lookahead_cancel_frac=0.3):  # fraction next curve cancels
        self.base_strength = base_strength
        self.growth_threshold = growth_threshold
        self.opposite_decay = opposite_decay
        self.max_layers = max_layers
        self.max_nudge = max_nudge
        self.lookahead_cancel_frac = lookahead_cancel_frac
        
        self.positive_layers = np.zeros(3)
        self.negative_layers = np.zeros(3)
        self.prev_error = None

    def compute_nudge(self, current_error, dt, next_curve_intensity=0.0):
        if self.prev_error is None:
            self.prev_error = current_error.copy()
            return np.zeros(3)

        growth = (current_error - self.prev_error) / dt
        nudge = np.zeros(3)

        # Expected natural cancel from next curve (your idea)
        expected_cancel = self.lookahead_cancel_frac * next_curve_intensity
        safe_error = np.maximum(current_error - expected_cancel, 0)

        for i in range(3):
            if abs(growth[i]) > self.growth_threshold:
                if growth[i] > 0:
                    self.positive_layers[i] = min(self.positive_layers[i] + 1, self.max_layers)
                    self.negative_layers[i] *= self.opposite_decay
                else:
                    self.negative_layers[i] = min(self.negative_layers[i] + 1, self.max_layers)
                    self.positive_layers[i] *= self.opposite_decay

            pos_extra = max(0.1, 1.0 - (self.positive_layers[i] - 1) * 0.1)
            pos_strength = self.base_strength * (1 + pos_extra)

            neg_extra = max(0.1, 1.0 - (self.negative_layers[i] - 1) * 0.1)
            neg_strength = self.base_strength * (1 + neg_extra)

            if growth[i] > 0:
                nudge[i] = -pos_strength * dt
            elif growth[i] < 0:
                nudge[i] = +neg_strength * dt

            nudge[i] = np.clip(nudge[i], -self.max_nudge * dt, self.max_nudge * dt)

        self.prev_error = current_error.copy()
        return nudge

# Simple OGY implementation (linear perturbation toward 2-period UPO)
class SimpleOGY:
    def __init__(self, A, B, u_star, epsilon=0.01):
        self.A = A  # Jacobian at UPO
        self.B = B  # Control input matrix (identity here)
        self.u_star = u_star  # UPO fixed point
        self.epsilon = epsilon  # max nudge size

    def compute_ogy_nudge(self, current_state):
        delta = current_state - self.u_star
        # Simple linear OGY: u = - (C delta) where C = (B^T A)^{-1} B^T or similar
        # Here we use pseudo-inverse approximation for simplicity
        nudge = -np.linalg.pinv(self.B.T @ self.A) @ (self.B.T @ delta)
        nudge = np.clip(nudge, -self.epsilon, self.epsilon)
        return nudge

# Pre-detected 2-period UPO for Lorenz (approximate from literature)
upo_point = np.array([np.sqrt(beta*(rho-1)), np.sqrt(beta*(rho-1)), rho-1])  # one point of 2-period orbit
A_jacobian = np.array([[ -sigma, sigma, 0 ],
                       [ rho - upo_point[2], -1, -upo_point[0] ],
                       [ upo_point[1], upo_point[0], -beta ]])  # approx Jacobian
B = np.eye(3)  # control on all states

ogy = SimpleOGY(A_jacobian, B, upo_point, epsilon=0.01)

# Full hybrid controller
def hybrid_controller(t, state, controller, threshold=0.5):
    error = state - np.array([np.sqrt(beta*(rho-1)), np.sqrt(beta*(rho-1)), rho-1])  # distance to UPO
    norm_error = np.linalg.norm(error)

    if norm_error < threshold:
        # OGY mode
        nudge = ogy.compute_ogy_nudge(state)
    else:
        # Your layered nudge mode
        nudge = controller.compute_nudge(error, 0.01, next_curve_intensity=0.5)  # dummy intensity

    return nudge

# Simulation parameters
t_span = (0, 100)
t_eval = np.linspace(t_span[0], t_span[1], 10000)
y0 = [1.0, 1.0, 1.0] + np.random.randn(3) * 0.01

controller = ChaosController(base_strength=0.01, growth_threshold=0.05,
                             opposite_decay=0.85, max_layers=10, max_nudge=0.05,
                             lookahead_cancel_frac=0.3)

# Run uncontrolled
sol_unc = solve_ivp(lorenz, t_span, y0, t_eval=t_eval, method='RK45')

# Run hybrid
def hybrid_ode(t, y):
    nudge = hybrid_controller(t, y, controller)
    dy = lorenz(t, y) + nudge
    return dy

sol_hybrid = solve_ivp(hybrid_ode, t_span, y0, t_eval=t_eval, method='RK45')

# Correct error: distance from INITIAL STATE (y0), not from uncontrolled
y0_array = np.array(y0)[:, np.newaxis]  # shape (3,1) for broadcasting
error_unc = np.linalg.norm(sol_unc.y - y0_array, axis=0)
error_hybrid = np.linalg.norm(sol_hybrid.y - y0_array, axis=0)

# Plot
plt.figure(figsize=(12, 6))
plt.plot(t_eval, error_unc, label='Uncontrolled', color='red', alpha=0.7)
plt.plot(t_eval, error_hybrid, label='Hybrid (your nudge + OGY)', color='green', linewidth=2)
plt.xlabel('Time')
plt.ylabel('L2 Error from initial state')
plt.title('Hybrid Chaos Control: Your Nudge + OGY')
plt.legend()
plt.grid(True)
plt.yscale('log')  # optional: log scale makes long-term flattening easier to see
plt.show()

final_reduction = 100 * (1 - error_hybrid[-1] / error_unc[-1])
print(f"Final error reduction: {final_reduction:.1f}%")
print(f"Hybrid final error: {error_hybrid[-1]:.3f}")
print(f"Uncontrolled final error: {error_unc[-1]:.3f}")