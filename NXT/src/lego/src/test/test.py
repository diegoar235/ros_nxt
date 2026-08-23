import numpy as np
import matplotlib.pyplot as plt


def Z_parametrizado_vel_max(x0, xf, vmax):

    D = xf - x0

    if abs(D) < 1e-12:
        coef = np.array([x0, 0.0, 0.0, 0.0, 0.0, 0.0])
        T = 0.0
        return coef, T

    if vmax <= 0:
        raise ValueError("vmax debe ser mayor que cero")

    # Para el quintico normalizado, la velocidad máxima es:
    # vmax = 1.875 * |D| / T
    T = 1.875 * abs(D) / vmax

    a0 = x0
    a1 = 0.0
    a2 = 0.0
    a3 = 10.0 * D / T**3
    a4 = -15.0 * D / T**4
    a5 = 6.0 * D / T**5

    coef = np.array([a0, a1, a2, a3, a4, a5])

    return coef, T


def evaluar_polinomio(coef, t):
    z = np.zeros_like(t, dtype=float)

    for i, c in enumerate(coef):
        z += c * t**i

    return z


def derivar_coeficientes(coef):
    derivada = []

    for i in range(1, len(coef)):
        derivada.append(i * coef[i])

    return np.array(derivada)


# ==========================
# Ejemplo de uso
# ==========================

x0 = 0.0
xf = 0.40
vmax = 10

coef, T = Z_parametrizado_vel_max(x0, xf, vmax)

print("Coeficientes:")
print(coef)

print("Tiempo total T:")
print(T)

# Vector de tiempo
t = np.linspace(0, T, 500)

# Posición
z = evaluar_polinomio(coef, t)

# Velocidad
coef_vel = derivar_coeficientes(coef)
vel = evaluar_polinomio(coef_vel, t)

# Aceleración
coef_acc = derivar_coeficientes(coef_vel)
acc = evaluar_polinomio(coef_acc, t)

# ==========================
# Gráficas
# ==========================

plt.figure()
plt.plot(t, z, label="Posición z(t)")
plt.xlabel("Tiempo [s]")
plt.ylabel("Posición [m]")
plt.title("Trayectoria de posición")
plt.grid(True)
plt.legend()

plt.figure()
plt.plot(t, vel, label="Velocidad dz/dt")
plt.axhline(vmax, linestyle="--", label="vmax")
plt.axhline(-vmax, linestyle="--", label="-vmax")
plt.xlabel("Tiempo [s]")
plt.ylabel("Velocidad [m/s]")
plt.title("Velocidad")
plt.grid(True)
plt.legend()

plt.figure()
plt.plot(t, acc, label="Aceleración d2z/dt2")
plt.xlabel("Tiempo [s]")
plt.ylabel("Aceleración [m/s²]")
plt.title("Aceleración")
plt.grid(True)
plt.legend()

plt.show()
