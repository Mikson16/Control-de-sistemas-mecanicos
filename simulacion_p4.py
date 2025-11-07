# from scipy.integrate import odeint
# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation

# # --- 1. Modelo de la Planta (Sin cambios) ---


# def model(X, t, tau):
#     # ... (el código de tu modelo va aquí, sin cambios) ...
#     theta, theta_dot, phi, phi_dot = X  # desempaquetar los datos
#     # parameters
#     m_a = 0.095  # Kg Masa del brazo rotatorio
#     r = 0.085  # m Longitud del brazo rotatorio
#     j_a = 2.288e-4  # Kg.m^2 Inercia del brazo rotatorio
#     m_p = 0.024  # Kg Masa del pendulo
#     l = 0.129  # m mitad del largo del pendulo, distancia del extremo al centro de masa
#     j_p = 0.00532  # 1.331e-4  # Kg.m^2 Inercia del pendulo
#     g = -9.81  # m/s^2 Aceleracion de la gravedad
#     # Ecuaciones dinamicas
#     v = (
#         (
#             m_p * l * r * j_p * phi_dot**2 * np.cos(theta)
#             + m_p**2 * l**2 * r * g
#             - 2 * j_p**2 * theta_dot * phi_dot
#         )
#         * np.sin(theta)
#         * np.cos(theta)
#         - m_p * l * r * j_p * theta_dot**2 * np.sin(theta)
#         + j_p * tau
#     ) / (
#         j_a * j_p
#         + j_p**2 * np.sin(theta) ** 2
#         - m_p**2 * l**2 * r**2 * np.cos(theta) ** 2
#     )
#     theta_2dot = (
#         phi_dot**2 * np.cos(theta) * np.sin(theta)
#         + ((m_p * l * g) / j_p) * np.sin(theta)
#         + ((m_p * l * r) / j_p) * v * np.cos(theta)
#     )
#     phi_2dot = v
#     dt_dtheta_phi = [
#         theta_dot,
#         theta_2dot,
#         phi_dot,
#         phi_2dot,
#     ]  # devolver derivadas de primer orden
#     return dt_dtheta_phi


# # --- 2. Parámetros de Simulación y Controlador ---


# # ... (parámetros de simulación y planta sin cambios) ...
# r = 0.085
# l = 0.129
# Ts = 0.002
# T = 5
# t = np.arange(0, T + Ts, Ts)
# N = len(t)
# X0 = [np.deg2rad(45), 0.0, np.deg2rad(90), 0.0]

# # Parámetros del controlador (GANANCIAS DE EJEMPLO - NECESITAN AJUSTE)
# # Las ganancias Ki y Kd anteriores eran para Kp=0.
# # Al añadir Kp, estas deben cambiar.
# Kp = 0.6917  # <--- AÑADIDO: Ganancia Proporcional (valor de ejemplo)
# Ki = 4.906  # (Valor anterior, probablemente necesita ajuste)
# Kd = 0.024381  # (Valor anterior, probablemente necesita ajuste)

# # Referencia: A qué ángulo queremos que vaya el brazo phi
# phi_ref = np.deg2rad(45.0)

# # Límites del actuador (Torque)
# tau_max = 5.0  # (Nm) - ¡Valor supuesto!

# # --- 3. Bucle de Simulación de Lazo Cerrado ---

# sol = np.zeros((N, 4))
# tau_history = np.zeros(N)
# sol[0, :] = X0

# # Inicializar estados del controlador
# i_k = 0.0  # Estado del integrador
# # <--- AÑADIDO: Estado anterior de 'phi' para el derivativo
# phi_k_prev = X0[2]

# print("Iniciando simulación de lazo cerrado (PID)...")

# for k in range(N - 1):
#     # 1. Leer el estado actual
#     X_k = sol[k, :]
#     theta_k, theta_dot_k, phi_k, phi_dot_k = X_k

#     # 2. Calcular la ley de control (PID Discreto con "Derivative on Measurement")

#     # Error actual
#     e_k = phi_ref - phi_k

#     # Término Proporcional
#     p_k = Kp * e_k  # <--- AÑADIDO

#     # Término Integral
#     i_k = i_k + Ki * Ts * e_k

#     # Término Derivativo (sobre la medición 'phi_k', no sobre el error 'e_k')
#     d_k = -(Kd / Ts) * (phi_k - phi_k_prev)  # <--- CAMBIO

#     # Señal de control total (tau)
#     tau = p_k + i_k + d_k  # <--- CAMBIO (se añade p_k)

#     # 3. Saturar el actuador
#     tau_sat = np.clip(tau, -tau_max, tau_max)

#     # 4. Guardar estados del controlador para el siguiente paso
#     phi_k_prev = phi_k  # <--- AÑADIDO

#     # Anti-Windup
#     if tau != tau_sat:
#         i_k = tau_sat - p_k - d_k

#     tau_history[k] = tau_sat

#     # 5. Simular UN paso adelante
#     t_span = [t[k], t[k + 1]]
#     X_next = odeint(model, X_k, t_span, args=(tau_sat,))
#     sol[k + 1, :] = X_next[1, :]
#     # print("Estamod en ", k, "/", N)

# tau_history[N - 1] = tau_history[N - 2]
# print("Simulación completada.")

# # --- 4. Graficar los Resultados (Sin cambios) ---
# fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

# # ... (código de gráficos sin cambios) ...
# ax[0].plot(t, np.rad2deg(sol[:, 0]), "b", label="theta (péndulo)")
# ax[0].plot(t, np.rad2deg(sol[:, 2]), "r", label="phi (brazo)")
# ax[0].plot(t, np.rad2deg(np.full_like(t, phi_ref)), "r--", label="Ref. phi")
# ax[0].set_ylabel("Angulo (grados)")
# ax[0].legend(loc="best")
# ax[0].grid()
# ax[1].plot(t, np.rad2deg(sol[:, 1]), "b", label="theta_dot (péndulo)")
# ax[1].plot(t, np.rad2deg(sol[:, 3]), "r", label="phi_dot (brazo)")
# ax[1].set_ylabel("Velocidad (grados/s)")
# ax[1].legend(loc="best")
# ax[1].grid()
# ax[2].plot(t, tau_history, "k", label="tau (Torque)")
# ax[2].set_xlabel("Tiempo (s)")
# ax[2].set_ylabel("Torque (Nm)")
# ax[2].legend(loc="best")
# ax[2].grid()

# plt.tight_layout()
# plt.show()

from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt

# --- 1. Modelo de la Planta (modificado con la nueva EDO) ---


def model(X, t, tau):
    theta, theta_dot = X  # desempaquetar el estado

    # Parámetros físicos
    m_p = 0.024  # Kg  masa del péndulo
    r = 0.085  # m   longitud del brazo
    l = 0.129  # m   mitad del largo del péndulo
    J_a = 2.288e-4  # Kg·m^2  inercia del brazo
    J_p = 0.00532  # Kg·m^2  inercia del péndulo
    g = -9.81  # m/s^2 gravedad

    # Coeficientes de la ecuación
    A = J_p - (m_p**2 * r**2 * l**2) / J_a
    B = m_p * g * l
    C = (m_p * r * l) / J_a

    # Ecuación: A*θ¨ - B*θ = C*τ  -> θ¨ = (B*θ + C*τ) / A
    theta_2dot = (B * theta + C * tau) / A

    return [theta_dot, theta_2dot]


# --- 2. Parámetros de simulación ---
Ts = 0.002
T = 4
t = np.arange(0, T + Ts, Ts)
N = len(t)

# Estado inicial [θ, θ̇]
X0 = [np.deg2rad(2), 0.0]

# Parámetros del controlador PID
Kp = 0.6917 * 0.7
Ki = 4.906 * 0.0
Kd = 0.024381 * 1.01

# Referencia
theta_ref = np.deg2rad(0.0)
tau_max = 1000.0  # Nm

# --- 3. Bucle de simulación de lazo cerrado ---
sol = np.zeros((N, 2))
tau_history = np.zeros(N)
sol[0, :] = X0

i_k = 0.0
theta_k_prev = X0[0]
print("Iniciando simulación")
for k in range(N - 1):
    theta_k, theta_dot_k = sol[k, :]

    # Error
    e_k = theta_ref - theta_k

    # Control PID
    p_k = Kp * e_k
    i_k += Ki * Ts * e_k
    d_k = -(Kd / Ts) * (theta_k - theta_k_prev)
    tau = p_k + i_k + d_k
    tau_sat = np.clip(tau, -tau_max, tau_max)

    theta_k_prev = theta_k
    tau_history[k] = tau_sat

    # Integrar un paso
    t_span = [t[k], t[k + 1]]
    X_next = odeint(model, sol[k, :], t_span, args=(tau_sat,))
    sol[k + 1, :] = X_next[1, :]

tau_history[-1] = tau_history[-2]

# --- 4. Graficar resultados ---
fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

ax[0].plot(t, np.rad2deg(sol[:, 0]), "b", label="theta (péndulo)")
ax[0].plot(t, np.rad2deg(np.full_like(t, theta_ref)), "r--", label="Ref. theta")
ax[0].set_ylabel("Ángulo (°)")
ax[0].legend()
ax[0].grid()

ax[1].plot(t, np.rad2deg(sol[:, 1]), "g", label="theta_dot (°/s)")
ax[1].set_ylabel("Velocidad (°/s)")
ax[1].legend()
ax[1].grid()

ax[2].plot(t, tau_history, "k", label="Torque (Nm)")
ax[2].set_xlabel("Tiempo (s)")
ax[2].set_ylabel("Torque (Nm)")
ax[2].legend()
ax[2].grid()

plt.tight_layout()
plt.show()
