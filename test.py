import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def crear_plano_desde_tres_puntos(p1, p2, p3, size=1.0):
    """
    Dibuja un plano definido por tres puntos en 3D.

    Parámetros:
    - p1, p2, p3: arrays de forma (3,), puntos en 3D
    - size: escala del plano generado
    """
    p1, p2, p3 = map(np.array, (p1, p2, p3))
    
    # Vectores en el plano
    u = p2 - p1
    v = p3 - p2
    
    angulo_entre_vectores = calcular_angulo_entre_vectores(u, v)
    print("Ángulo entre vectores u y v:", angulo_entre_vectores, "grados")

    # Crear la malla del plano
    uu, vv = np.meshgrid(np.linspace(-size, size, 10), np.linspace(-size, size, 10))
    puntos = p1[np.newaxis, np.newaxis, :] + uu[..., np.newaxis]*u + vv[..., np.newaxis]*v
    x, y, z = puntos[..., 0], puntos[..., 1], puntos[..., 2]
    
    # Vector desde p3 en el plano con ángulo 30 grados respecto a v
    w = vector_en_plano_con_angulo(u, v, p3, -angulo_entre_vectores)

    # Visualización
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x, y, z, alpha=0.5, color='lightblue', label='Plano')
    
    # Dibujar los puntos
    ax.scatter(*p1, color='red', label='p1')
    ax.scatter(*p2, color='green', label='p2')
    ax.scatter(*p3, color='blue', label='p3')
    
    # Dibujar vectores u y v
    ax.quiver(*p1, *u, color='orange', label='u')
    ax.quiver(*p2, *v, color='purple', label='v')
    
    # Vector normal
    n = np.cross(u, v)
    ax.quiver(*p1, *n, color='black', label='normal')
    
    # Vector en el plano con ángulo 30° desde v, origen en p3
    ax.quiver(*p3, *w, color='magenta', label='Vector 30° desde v en p3', linewidth=2)

    ax.legend()
    ax.set_title("Plano a partir de 3 puntos y vector en plano")
    set_axes_equal(ax)
    plt.show()

def vector_en_plano_con_angulo(u, v, origen, angulo_grados):
    """
    Calcula un vector en el plano definido por u y v con origen en 'origen',
    que forme un ángulo 'angulo_grados' respecto a v.
    """
    u = np.array(u)
    v = np.array(v)
    origen = np.array(origen)

    # Normalizar v
    v_unit = v / np.linalg.norm(v)

    # Proyección de u sobre v
    u_proj = np.dot(u, v_unit) * v_unit
    u_ort = u - u_proj
    u_ort_unit = u_ort / np.linalg.norm(u_ort)

    # Convertir ángulo a radianes
    theta = np.deg2rad(angulo_grados)

    # Vector combinado en el plano
    w = np.cos(theta) * v_unit + np.sin(theta) * u_ort_unit

    return w

def set_axes_equal(ax):
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d()
    ])
    center = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:,1] - limits[:,0]))
    ax.set_xlim3d([center[0] - radius, center[0] + radius])
    ax.set_ylim3d([center[1] - radius, center[1] + radius])
    ax.set_zlim3d([center[2] - radius, center[2] + radius])

def calcular_angulo_entre_vectores(u, v):
    dot_product = np.dot(u, v)
    dot_product = np.clip(dot_product, -1.0, 1.0)
    angulo_rad = np.arccos(dot_product / (np.linalg.norm(u)*np.linalg.norm(v)))
    angulo_deg = np.degrees(angulo_rad)
    return angulo_deg

if __name__ == "__main__":
    p1 = [0, 0, 0]
    p2 = [0, 0, 1]
    p3 = [1, 2, 3.5]
          

    crear_plano_desde_tres_puntos(p1, p2, p3)