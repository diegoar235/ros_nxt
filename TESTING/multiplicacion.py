import numpy as np
import plotly.graph_objects as go


def rot_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1]
    ])


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



def vector_trace(v, name="vector"):
    """
    Crea una línea 3D desde el origen hasta el vector v
    """
    return go.Scatter3d(
        x=[0, v[0]],
        y=[0, v[1]],
        z=[0, v[2]],
        mode="lines+markers",
        name=name,
        line=dict(width=6),
        marker=dict(size=4)
    )


def axes_traces(length=1.5):
    """
    Ejes X, Y, Z
    """
    return [
        go.Scatter3d(
            x=[0, length], y=[0, 0], z=[0, 0],
            mode="lines+text",
            text=["", "X"],
            name="X"
        ),
        go.Scatter3d(
            x=[0, 0], y=[0, length], z=[0, 0],
            mode="lines+text",
            text=["", "Y"],
            name="Y"
        ),
        go.Scatter3d(
            x=[0, 0], y=[0, 0], z=[0, length],
            mode="lines+text",
            text=["", "Z"],
            name="Z"
        )
    ]


# Vector inicial
v0 = np.array([1.0, 0.0, 0.5])

# Cantidad de pasos de animación
N = 100
thetas = np.linspace(0, 2*np.pi, N)

# Frames de animación
frames = []

for i, theta in enumerate(thetas):
    v = rot_x(theta) @ v0

    frames.append(
        go.Frame(
            data=[
                vector_trace(v, name="Vector animado")
            ],
            name=str(i)
        )
    )


# Figura inicial
fig = go.Figure(
    data=[
        vector_trace(v0, name="Vector animado"),
        *axes_traces(length=1.5)
    ],
    frames=frames
)

fig.update_layout(
    title="Vector 3D animado con Plotly",
    scene=dict(
        xaxis=dict(range=[-1.5, 1.5]),
        yaxis=dict(range=[-1.5, 1.5]),
        zaxis=dict(range=[-1.5, 1.5]),
        aspectmode="cube"
    ),
    updatemenus=[
        dict(
            type="buttons",
            buttons=[
                dict(
                    label="Play",
                    method="animate",
                    args=[
                        None,
                        dict(
                            frame=dict(duration=40, redraw=True),
                            fromcurrent=True
                        )
                    ]
                ),
                dict(
                    label="Pause",
                    method="animate",
                    args=[
                        [None],
                        dict(
                            frame=dict(duration=0, redraw=False),
                            mode="immediate"
                        )
                    ]
                )
            ]
        )
    ]
)

fig.show()