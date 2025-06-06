import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# ---- Simplified Motor Transfer Function: G_motor(s) = k / (tau*s + 1)
k = 0.0314
tau = 0.02
G_motor = ctrl.TransferFunction([k], [tau, 1])

# ---- Platform Dynamics
Kx = 4.13
Kz = 1.06
G_theta_to_x = ctrl.TransferFunction([Kx], [1, 0, 0])
G_theta_to_z = ctrl.TransferFunction([Kz], [1, 0, 0])

# ---- PIDX Controller (X-Axis)
Kp_x = 20
Ki_x = 2
Kd_x = 100
PIDX = ctrl.TransferFunction([Kd_x, Kp_x, Ki_x], [1, 0])

# ---- PIDY Controller (Y-Axis)
Kp_y = 10
Ki_y = 0.4
Kd_y = 120
PIDY = ctrl.TransferFunction([Kd_y, Kp_y, Ki_y], [1, 0])

# ---- Open-Loop System with PID
G_open_x = PIDX * G_motor * G_theta_to_x
G_open_z = PIDY * G_motor * G_theta_to_z

# ---- Closed-Loop Systems
G_closed_x = ctrl.feedback(G_open_x, 1)
G_closed_z = ctrl.feedback(G_open_z, 1)

# ---- Step Responses
t1, y1 = ctrl.step_response(G_closed_x)
t2, y2 = ctrl.step_response(G_closed_z)

# ---- Step Info
info_x = ctrl.step_info(G_closed_x)
info_z = ctrl.step_info(G_closed_z)

# ---- Step Plot with Metrics
plt.figure()
plt.plot(t1, y1, label="X-Axis (Closed-Loop)")
plt.plot(t2, y2, label="Y-Axis (Closed-Loop)")
plt.title("Closed-Loop Step Response with PID")
plt.xlabel("Time [s]")
plt.ylabel("Ball Position [m]")
plt.grid(True)
plt.legend()

x_metrics = (
    f"X-Axis:\nOvershoot: {info_x['Overshoot']:.2f}%\n"
    f"Settling Time: {info_x['SettlingTime']:.3f}s\n"
    f"SS Value: {info_x['SteadyStateValue']:.3f}"
)
z_metrics = (
    f"Y-Axis:\nOvershoot: {info_z['Overshoot']:.2f}%\n"
    f"Settling Time: {info_z['SettlingTime']:.3f}s\n"
    f"SS Value: {info_z['SteadyStateValue']:.3f}"
)

plt.text(0.6 * max(t1), 0.9 * max(y1), x_metrics, fontsize=9, bbox=dict(facecolor='white', alpha=0.6))
plt.text(0.6 * max(t2), 0.4 * max(y2), z_metrics, fontsize=9, bbox=dict(facecolor='white', alpha=0.6))
plt.show()

# ---- Root Locus Plots
plt.figure()
ctrl.root_locus(G_open_x)
plt.title("Root Locus - X Axis (PID Open-Loop)")
plt.grid(True)

plt.figure()
ctrl.root_locus(G_open_z)
plt.title("Root Locus - Y Axis (PID Open-Loop)")
plt.grid(True)
plt.show()
