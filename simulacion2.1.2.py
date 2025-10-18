from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def model(X, t, tau):

    theta, theta_dot, phi, phi_dot = X  # desempaquetar los datos

    # parameters
    m_a = 0.095  # Kg Masa del brazo rotatorio
    r = 0.085  # m Longitud del brazo rotatorio
    j_a = 2.288e-4  # Kg.m^2 Inercia del brazo rotatorio
    m_p = 0.024  # Kg Masa del pendulo
    l = 0.129  # m mitad del largo del pendulo, distancia del extremo al centro de masa
    j_p = 1.331e-4  # Kg.m^2 Inercia del pendulo
    g = 9.81  # m/s^2 Aceleracion de la gravedad

    # Ecuaciones dinamicas
    v = (
        (
            m_p * l * r * j_p * phi_dot**2 * np.cos(theta)
            + m_p**2 * l**2 * r * g
            - 2 * j_p**2 * theta_dot * phi_dot
        )
        * np.sin(theta)
        * np.cos(theta)
        - m_p * l * r * j_p * theta_dot**2 * np.sin(theta)
        + j_p * tau
    ) / (
        j_a * j_p
        + j_p**2 * np.sin(theta) ** 2
        - m_p**2 * l**2 * r**2 * np.cos(theta) ** 2
    )

    theta_2dot = (
        phi_dot**2 * np.cos(theta) * np.sin(theta)
        + ((m_p * l * g) / j_p) * np.sin(theta)
        + ((m_p * l * r) / j_p) * v * np.cos(theta)
    )

    phi_2dot = v

    dt_dtheta_phi = [
        theta_dot,
        theta_2dot,
        phi_dot,
        phi_2dot,
    ]  # devolver derivadas de primer orden
    return dt_dtheta_phi


r = 0.085  # m Longitud del brazo rotatorio
l = 0.129  # m mitad del largo del pendulo, distancia del extremo al centro de masa
# Condiciones iniciales
Ts = 0.002  # segundos
T = 10  # Tiempo de simulacion
X0 = [np.deg2rad(45), 0.0, np.deg2rad(90), 0.0]  # [theta, theta_dot, phi, phi_dot]
t = np.arange(0, T + Ts, Ts)  # vector de tiempo (inicio, fin, paso)
tau = 0.0  # torque constante aplicado al brazo rotatorio

sol = odeint(model, X0, t, args=(tau,))  # resolver ODE
# print(sol.shape)
# Ahora graficamos los resultados
fig, ax = plt.subplots(2, 1, figsize=(8, 6))
ax[0].plot(t, sol[:, 0], "b", label="theta (rad)")
ax[0].plot(t, sol[:, 2], "r", label="phi (rad)")
ax[0].set_xlabel("Tiempo (s)")
ax[0].set_ylabel("Angulo (rad)")
ax[0].legend(loc="best")
ax[0].grid()


ax[1].plot(t, sol[:, 1], "b", label="theta_dot (rad/s)")
ax[1].plot(t, sol[:, 3], "r", label="phi_dot (rad/s)")
ax[1].set_xlabel("Tiempo (s)")
ax[1].set_ylabel("Velocidad angular (rad/s)")
ax[1].legend(loc="best")
ax[1].grid()
plt.tight_layout()
plt.show()

# Animacion del sistema
fig2, ax2 = plt.subplots(figsize=(6, 6))
ax2.set_xlim(-(r + 0.3), r + 0.3)
ax2.set_ylim(-(r + 0.3), r + 0.3)
ax2.set_aspect("equal")
ax2.grid()
(brazo_line,) = ax2.plot([], [], "o-", lw=4, color="blue", label="Brazo")
(pendulo_line,) = ax2.plot([], [], "o-", lw=2, color="red", label="Péndulo")
ax2.legend()


def init():
    brazo_line.set_data([], [])
    pendulo_line.set_data([], [])
    return brazo_line, pendulo_line


# Para submuestrear
paso = 3
indices = np.arange(0, len(t), paso)


def animate(i):
    idx = indices[i]
    # Posición del extremo del brazo
    x_brazo = r * np.cos(sol[idx, 0])
    y_brazo = r * np.sin(sol[idx, 0])
    # Posición del extremo del péndulo
    x_pendulo = x_brazo + l * np.sin(sol[idx, 2]) * np.cos(sol[idx, 0])
    y_pendulo = y_brazo - l * np.cos(sol[idx, 2])
    # Brazo: desde el origen al extremo
    brazo_line.set_data([0, x_brazo], [0, y_brazo])
    # Péndulo: desde extremo del brazo al extremo del péndulo
    pendulo_line.set_data([x_brazo, x_pendulo], [y_brazo, y_pendulo])
    return brazo_line, pendulo_line


ani = animation.FuncAnimation(
    fig2, animate, init_func=init, frames=len(indices), interval=30, blit=True
)
plt.show()
