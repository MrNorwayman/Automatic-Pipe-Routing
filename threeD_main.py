#Creacion de codigo para busqueda de caminos en 2D. La idea es optimizar al maximo esta version simplificada en 2D para definir la mejor estructura posible y luego integrar la version 3D.

#Se va a partir de dos vectores. Se van a graficar todos los puntos usados, tanto los definitivos como todas las demas opciones usadas. Se van a implementar algunos obstaculos para el control de colisiones.

################# CAMBIO A 3D #################
#   -> threeD_Nodo.py se añade segundo angulo phi       
#   -> threeD_functions.py los cambios estan especificados en ese fichero 
#   -> threeD_main.py grandes cambios.
################# CAMBIO A 3D #################


import threeD_functions, threeD_functions, numpy as np, matplotlib.pyplot as plt, time

start_time = time.time()

intervalo_lineal = 5
recta_minima = 25
radio_curvatura = 10
intervalo_angular = np.deg2rad(1)
curva_maxima = np.deg2rad(180)


# CAMBIO 3D -> Se añade tercer valor de espacio
inicio = np.array([0.0, 0.0, 0.0])
objetivo = np.array([-25.0, 20.0, 0.0])


# CAMBIO 3D -> Se añade variable extra para angulo theta y phi
Nodo_inicio = threeD_functions.Nodo(inicio, 0, 0, 0, 0, 0.001, 0, 0, 0)
Nodo_objetivo = threeD_functions.Nodo(objetivo, 0, 0, 0, 0, 1, 0, 0, 0)

distancia_a_obstaculo = 10

# CAMBIO 3D -> Se añade tercer valor de espacio
obstaculos = [np.array([0, 20, 0]),
              np.array([20, 20, 0]),
              np.array([35, 10, 0]),
              np.array([40, 10, 0]),
              np.array([45, 10, 0]),
              np.array([50, 10, 0]),
              np.array([10, -20, 0]),
              np.array([55, 10, 0])]

camino, explorados = threeD_functions.delta_star( Nodo_inicio,
                                    Nodo_objetivo,
                                    obstaculos,
                                    distancia_a_obstaculo,
                                    intervalo_lineal,
                                    recta_minima,
                                    radio_curvatura,
                                    intervalo_angular,
                                    curva_maxima)

print(f"Tiempo total de ejecución: {(time.time() - start_time):.2f} segundos")
print(f"{len(explorados)} nodos explorados. {len(camino)} nodos en el camino")


# Convertir listas a arrays de numpy si aún no lo están
camino_np = np.array(camino)
explorados_np = np.array(list(explorados))
objetivo = np.array(objetivo)  # Asegúrate de que sea un array 3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Graficar obstáculos
for obs in obstaculos:
    ax.scatter(obs[0], obs[1], obs[2], c='r', marker='o', label='Obstáculo')

'''
# Graficar nodos explorados
ax.scatter(explorados_np[:, 0], explorados_np[:, 1], explorados_np[:, 2],
           c='k', s=0.1, label='Nodos explorados')
'''

# Graficar camino óptimo
ax.plot(camino_np[:, 0], camino_np[:, 1], camino_np[:, 2],
        c='g', linewidth=2, label='Camino óptimo')

# Graficar punto inicial y objetivo
ax.scatter(objetivo[0], objetivo[1], objetivo[2], c='r', marker='x', label='Objetivo')
ax.scatter(camino_np[0, 0], camino_np[0, 1], camino_np[0, 2], c='b', marker='o', label='Inicio')

# Ajustes finales
ax.set_title('Camino generado por Delta-Star (3D)')
ax.legend()
ax.grid(True)

# Obtener límites máximos y mínimos
x_vals = np.concatenate([camino_np[:, 0], explorados_np[:, 0], np.array([obs[0] for obs in obstaculos])])
y_vals = np.concatenate([camino_np[:, 1], explorados_np[:, 1], np.array([obs[1] for obs in obstaculos])])
z_vals = np.concatenate([camino_np[:, 2], explorados_np[:, 2], np.array([obs[2] for obs in obstaculos])])

max_range = np.array([x_vals.max() - x_vals.min(),
                      y_vals.max() - y_vals.min(),
                      z_vals.max() - z_vals.min()]).max() / 2.0

mid_x = (x_vals.max() + x_vals.min()) * 0.5
mid_y = (y_vals.max() + y_vals.min()) * 0.5
mid_z = (z_vals.max() + z_vals.min()) * 0.5

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)


plt.show()