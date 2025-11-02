# rack_detection_revised.py
import numpy as np
import matplotlib.pyplot as plt

# Simulated rack alignment using depth sensor data
# Assume robot has left/right depth sensors giving distance in cm
np.random.seed(0)

left_dist  = 80.0  # cm
right_dist = 60.0  # cm
target_diff = 0.0  # perfect alignment
kp, ki, kd = 0.4, 0.05, 0.2

integral = 0
prev_error = 0
history = []

for i in range(30):
    error = right_dist - left_dist  # +ve means right is farther → rotate right
    integral += error
    derivative = error - prev_error
    control = kp*error + ki*integral + kd*derivative  # PID output
    prev_error = error

    # Simulate applying control (robot rotates slightly)
    left_dist  += control * 0.15 + np.random.normal(0,0.5)
    right_dist -= control * 0.15 + np.random.normal(0,0.5)

    history.append((i, left_dist, right_dist, error))
    print(f"Step {i}: Error={error:.2f}, Control={control:.2f}")

# Plot alignment progress
iters = [h[0] for h in history]
left_vals = [h[1] for h in history]
right_vals = [h[2] for h in history]

plt.figure(figsize=(7,4))
plt.plot(iters, left_vals, label="Left Distance (cm)", linewidth=2)
plt.plot(iters, right_vals, label="Right Distance (cm)", linewidth=2)
plt.axhline(y=np.mean([left_vals[-1], right_vals[-1]]), color='gray', linestyle='--', label="Final Alignment Line")
plt.title("Rack Alignment using PID Control (Simulated Depth Sensors)")
plt.xlabel("Iteration")
plt.ylabel("Distance from Rack (cm)")
plt.legend()
plt.grid(True)
plt.show()
