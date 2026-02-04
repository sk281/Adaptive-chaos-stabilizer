import numpy as np
from scipy.integrate import solve_ivp

# Lorenz system parameters
sigma = 10.0
rho = 28.0
beta = 8.0 / 3.0

def lorenz(t, state):
    x, y, z = state
    dx = sigma * (y - x)
    dy = x * (rho - z) - y
    dz = x * y - beta * z
    return [dx, dy, dz]

# Simulation parameters
t_span = (0, 150)
t_eval = np.linspace(t_span[0], t_span[1], 15000)

# Initial conditions
y0_true = np.array([1.0, 1.0, 1.0])
y0_perturbed = y0_true + np.array([0.0001, 0.0, 0.0])

# Solve true trajectory
sol_true = solve_ivp(lorenz, t_span, y0_true, method='RK45', t_eval=t_eval, rtol=1e-10, atol=1e-10)

# Controlled integration
def controlled_lorenz(t, state, prev_error, t_prev, correction_history):
    deriv = np.array(lorenz(t, state))
    
    current_error = prev_error if prev_error is not None else 0.0
    t_prev_return = t
    
    dt = t - t_prev if t_prev is not None else 0
    if dt > 0:
        idx = min(int(t / (t_span[1] / len(t_eval))), len(t_eval) - 1)
        current_error = np.linalg.norm(state - sol_true.y[:, idx])
        error_growth = (current_error - prev_error) / dt if prev_error is not None else 0
        
        shift = 0.0
        if error_growth > 0:
            shift = -0.01 * t
        elif error_growth < 0:
            shift = +0.01 * t
        
        if abs(t % 20) < 1e-3:
            shift *= 1.5
        if abs(t % 45) < 1e-3:
            shift *= 2.4
        
        state += shift * np.ones(3)
        correction_history.append((t, shift, current_error))
    
    return deriv, current_error, t_prev_return

# Step-by-step with control
state = y0_perturbed.copy()
t_current = 0.0
prev_error = 0.0
t_prev = None
controlled_path = [state.copy()]
correction_history = []
error_history_controlled = [0.0]

while t_current < t_span[1]:
    dt_step = 0.01
    sol_step = solve_ivp(lorenz, [t_current, t_current + dt_step], state,
                         method='RK45', rtol=1e-10, atol=1e-10)
    
    state = sol_step.y[:, -1]
    t_current = sol_step.t[-1]
    
    deriv, current_error, t_prev = controlled_lorenz(t_current, state, prev_error, t_prev, correction_history)
    prev_error = current_error
    
    controlled_path.append(state.copy())
    error_history_controlled.append(current_error)

controlled_path = np.array(controlled_path)
error_history_controlled = np.array(error_history_controlled)

# Compare to baseline
sol_perturbed = solve_ivp(lorenz, t_span, y0_perturbed, method='RK45', t_eval=t_eval, rtol=1e-10, atol=1e-10)
baseline_error = np.linalg.norm(sol_true.y - sol_perturbed.y, axis=0)[-1]
controlled_error = error_history_controlled[-1]

print(f"Baseline final L2 error: {baseline_error:.2f}")
print(f"Controlled final L2 error: {controlled_error:.2f}")
print(f"Reduction: {100 * (1 - controlled_error / baseline_error):.1f}%")