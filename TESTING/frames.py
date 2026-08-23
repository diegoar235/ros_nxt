import numpy as np

def rot_x(theta):
    """
    Matriz de rotación alrededor del eje X
    theta en radianes
    """
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta),  np.cos(theta)]
    ])

def rot_y(theta):
    """
    Matriz de rotación alrededor del eje Y
    theta en radianes
    """
    return np.array([
        [ np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def rot_z(theta):
    """
    Matriz de rotación alrededor del eje Z
    theta en radianes
    """
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1]
    ])

Vector = np.array([1,1,0])

print(Vector, "Rotado en X", np.round(rot_x(np.pi/2)@Vector,2))
print(Vector, "Rotado en X, pero luego post rotado en Z", np.round(rot_z(np.pi/2)@Vector,2))