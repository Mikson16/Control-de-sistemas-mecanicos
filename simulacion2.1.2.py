from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt


def model():

    # parameters
    m_a = 0.095  # Kg Masa del brazo rotatorio
    r = 0.085  # m Longitud del brazo rotatorio
    j_a = 2.288 * np.e - 4  # Kg.m^2 Inercia del brazo rotatorio
    m_p = 0.024  # Kg Masa del pendulo
    l = 0.129  # m mitad del largo del pendulo, distancia del extremo al centro de masa
    j_p = 1.331 * np.e - 4  # Kg.m^2 Inercia del pendulo
    g = 9.81  # m/s^2 Aceleracion de la gravedad

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
        + ((m_p * l * g) / j_p) * sin(theta)
        + ((m_p * l * r) / j_p) * v * np.cos(theta)
    )

    phi_2dot = v
