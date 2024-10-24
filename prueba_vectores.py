import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Función para rotar un vector usando la fórmula de Rodrigues
def rotar_alrededor_vector(vector, eje, angulo):
    # Convertir el ángulo de grados a radianes
    theta = np.radians(angulo)
    
    # Normalizar el eje de rotación por si acaso no es unitario
    eje = eje / np.linalg.norm(eje)
    
    # Aplicar la fórmula de Rodrigues
    vector_rotado = (vector * np.cos(theta) +
                     np.cross(eje, vector) * np.sin(theta) +
                     eje * np.dot(eje, vector) * (1 - np.cos(theta)))
    
    return vector_rotado

# Función para generar 10 vectores alrededor del vector unitario con una diferencia angular de 10 grados
def generar_vectores_alrededor(vector_unitario, k, num_vectores=10, angulo_diferencia=10):
    vecinos = []
    
    # Generar los vecinos rotando en múltiplos del ángulo de diferencia
    for i in range(1, num_vectores + 1):
        angulo = i * angulo_diferencia
        vecino_rotado = rotar_alrededor_vector(np.array([1, 0, 0]), vector_unitario, angulo) * k
        vecinos.append(vecino_rotado)
    
    return vecinos

# Función para visualizar los vectores usando matplotlib
def visualizar_vectores(vector_original, vecinos):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Graficar el vector original (eje de rotación)
    ax.quiver(0, 0, 0, vector_original[0], vector_original[1], vector_original[2], color='r', label="Vector Original (Eje)")

    # Graficar los vecinos rotados
    for i, vecino in enumerate(vecinos):
        ax.quiver(0, 0, 0, vecino[0], vecino[1], vecino[2], color='b', label=f"Vecino {i+1}")

    # Configuración del gráfico
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

# Generar un vector unitario aleatorio (el eje de rotación)
vector_unitario = np.random.randn(3)
vector_unitario /= np.linalg.norm(vector_unitario)  # Normalizar para que sea unitario
k = 1  # Longitud del movimiento

# Generar 10 vecinos alrededor del vector unitario con una diferencia angular de 10 grados
vecinos = generar_vectores_alrededor(vector_unitario, k, num_vectores=10, angulo_diferencia=10)

# Visualizar los vectores
visualizar_vectores(vector_unitario, vecinos)
