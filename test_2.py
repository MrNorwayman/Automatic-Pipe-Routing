import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Función para rotar un vector alrededor de un eje arbitrario
def rotar_vector(vector, eje, angulo):
    eje = eje / np.linalg.norm(eje)  # Normalizamos el eje
    cos_a = np.cos(angulo)
    sin_a = np.sin(angulo)
    matriz_rotacion = np.array([
        [cos_a + (1 - cos_a) * eje[0] * eje[0], (1 - cos_a) * eje[0] * eje[1] - sin_a * eje[2], (1 - cos_a) * eje[0] * eje[2] + sin_a * eje[1]],
        [(1 - cos_a) * eje[1] * eje[0] + sin_a * eje[2], cos_a + (1 - cos_a) * eje[1] * eje[1], (1 - cos_a) * eje[1] * eje[2] - sin_a * eje[0]],
        [(1 - cos_a) * eje[2] * eje[0] - sin_a * eje[1], (1 - cos_a) * eje[2] * eje[1] + sin_a * eje[0], cos_a + (1 - cos_a) * eje[2] * eje[2]]
    ])
    return np.dot(matriz_rotacion, vector)

# Función para generar planos y devolver puntos finales
def generar_planos_y_puntos_finales(punto1, punto2, num_planos=72, angulo_rotacion=5):
    # Vector inicial entre los dos puntos
    vector_inicial = punto2 - punto1
    magnitud_vector_inicial = np.linalg.norm(vector_inicial)  # Calcular la magnitud del vector inicial
    
    vecinos_en_planos = []
    puntos_finales = []

    # Generamos un eje perpendicular arbitrario para rotarlo en el plano
    eje_perpendicular = np.cross(vector_inicial, np.array([1, 0, 0]))  # Eje arbitrario de rotación
    if np.linalg.norm(eje_perpendicular) == 0:  # Si el eje es nulo (vector paralelo al eje arbitrario)
        eje_perpendicular = np.array([0, 1, 0])
    
    # Normalizar el eje perpendicular
    eje_perpendicular = eje_perpendicular / np.linalg.norm(eje_perpendicular)

    for i in range(num_planos):
        # Crear un nuevo vector en el plano rotando el vector inicial alrededor del eje_perpendicular
        nuevo_vector = rotar_vector(vector_inicial, eje_perpendicular, i * np.radians(angulo_rotacion))
        
        # Asegurarse de que el nuevo vector tenga la misma magnitud que el vector inicial
        nuevo_vector = (nuevo_vector / np.linalg.norm(nuevo_vector)) * magnitud_vector_inicial
        
        # Ajustar el origen del nuevo vector al final del vector inicial
        origen_nuevo_vector = punto1 + vector_inicial  # El nuevo vector comienza en el final del vector inicial
        fin_nuevo_vector = origen_nuevo_vector + nuevo_vector  # Extremo del nuevo vector
        
        # Almacenar el nuevo vector
        vecinos_en_planos.append((origen_nuevo_vector, fin_nuevo_vector))
        puntos_finales.append(fin_nuevo_vector)  # Guardar el punto final

    return tuple(puntos_finales), vecinos_en_planos  # Devolver los puntos finales como una tupla

# Generar dos puntos en 3D
punto1 = np.array([0, 0, 0])  # Primer punto
punto2 = np.array([1, 1, 0.5])  # Segundo punto

# Llamar a la función y obtener los puntos finales
puntos_finales, vecinos_en_planos = generar_planos_y_puntos_finales(punto1, punto2)