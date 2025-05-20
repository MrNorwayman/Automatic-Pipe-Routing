#Creacion de codigo para busqueda de caminos en 2D. La idea es optimizar al maximo esta version simplificada en 2D para definir la mejor estructura posible y luego integrar la version 3D.

#Se va a partir de dos vectores. Se van a graficar todos los puntos usados, tanto los definitivos como todas las demas opciones usadas. Se van a implementar algunos obstaculos para el control de colisiones.

# 20.98 segundos - Estandar
# 21.36 segundos - Cambio estructura nodo -> Nodo -1.2%
# 19.86 segundos - Cambio calculo colisiones -> Colisiones +5.6%
# 18.98 segundos - Cambio calculo colisiones y distancia -> Distancia +4.6%
# 19.98 segundos - Cambio calculo colisiones y distancia y reconstruccion del camino -> Reconstruccion camino -4.6%
# 19.00 segundos - Cambio calculo colisiones y distancia y creacion de vecinos -> Creacion vecinos +0.0%
# 25.90 segundos - Cambio calculo colisiones y distancia y algoritmo delta -> Heuristica -36.0%

# TIEMPO FINAL OPTIMIZADO -> 17.91 segundos (mejor marca, variacion de 2 segundos entre pruebas)

import twoD_functions, twoD_functions, numpy as np, matplotlib.pyplot as plt, time

print("Algoritmo Delta-Star para busqueda de caminos en 2D")
start_time = time.time()

intervalo_lineal = 5
recta_minima = 25
radio_curvatura = 10
intervalo_angular = np.deg2rad(1)
curva_maxima = np.deg2rad(45)

inicio = np.array([0.0, 0.0])
objetivo = np.array([-26.0, -25.0])

Nodo_inicio = twoD_functions.Nodo(inicio, 0, 0, 0.001, 0, 0, 0)
Nodo_objetivo = twoD_functions.Nodo(objetivo, 0, 0, 1, 0, 0, 0)

distancia_a_obstaculo = 10
obstaculos = [np.array([0, 20]),
              np.array([20, 20]),
              np.array([35, 10]),
              np.array([40, 10]),
              np.array([45, 10]),
              np.array([50, 10]),
              np.array([55, 10])]

camino, explorados = twoD_functions.delta_star( Nodo_inicio,
                                    Nodo_objetivo,
                                    obstaculos,
                                    distancia_a_obstaculo,
                                    intervalo_lineal,
                                    recta_minima,
                                    radio_curvatura,
                                    intervalo_angular,
                                    curva_maxima)

print(f"Tiempo total de ejecución: {(time.time() - start_time):.2f} segundos")

print(len(explorados))
camino_np = np.array(camino)
explorados_np= np.array(list(explorados))
plt.figure()
for obs in obstaculos:
    plt.plot(obs[0], obs[1], 'ro')  # Dibuja cada obstáculo como un punto rojo
plt.plot(explorados_np[:, 0], explorados_np[:, 1], 'k.', markersize=1, label='Nodos explorados')
plt.plot(camino_np[:, 0], camino_np[:, 1], 'g-', linewidth=2, label='Camino óptimo')
plt.plot(objetivo[0], objetivo[1], 'rx', label='Objetivo')
plt.plot(camino_np[0, 0], camino_np[0, 1], 'bo', label='Inicio')
plt.legend()
plt.axis('equal')
plt.title('Camino generado por Delta-Star')
plt.grid(True)
plt.show()