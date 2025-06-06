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
G_theta_to_x = ctrl.TransferFunction([Kx], [1, 0, 0])  # 4.13 / s²
G_theta_to_z = ctrl.TransferFunction([Kz], [1, 0, 0])  # 1.06 / s²

# ---- Combined Open-Loop Systems
G_open_x = G_motor * G_theta_to_x
G_open_z = G_motor * G_theta_to_z

# ---- Step Responses
t1, y1 = ctrl.step_response(G_open_x)
t2, y2 = ctrl.step_response(G_open_z)

plt.figure()
plt.plot(t1, y1, label="X-Axis")
plt.plot(t2, y2, label="Y-Axis")
plt.title("Open-Loop Step Response: Motor Input → Ball Position (Simplified Model)")
plt.xlabel("Time [s]")
plt.ylabel("Ball Position [m]")
plt.grid(True)
plt.legend()
plt.show()

# ---- Root Locus Plots
plt.figure()
ctrl.root_locus(G_open_x)
plt.title("Root Locus - X Axis (Simplified Model)")
plt.grid(True)

plt.figure()
ctrl.root_locus(G_open_z)
plt.title("Root Locus - Y Axis (Simplified Model)")
plt.grid(True)
plt.show()
