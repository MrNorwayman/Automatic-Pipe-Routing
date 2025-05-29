
import threeD_functions, threeD_functions, numpy as np, matplotlib.pyplot as plt, time

print("Comenzando ejecución...")
start_time = time.time()

intervalo_lineal = 5
recta_minima = 50
radio_curvatura = 25
intervalo_angular = np.deg2rad(5)
curva_maxima = np.deg2rad(180) - intervalo_angular/2
distancia_a_obstaculo = 10

inicio = np.array([0.0, 0.0, 0.0])
objetivo = np.array([200.0, 100, 500])

vector_inicio = np.array([1, 0, 0])
vector_objetivo = np.array([1, 0, 0])

Nodo_inicio = threeD_functions.Nodo(inicio,
                                    vector_inicio/np.linalg.norm(vector_inicio),
                                    vector_inicio/np.linalg.norm(vector_inicio),
                                    0,
                                    0.0000001,
                                    0,
                                    0)

Nodo_objetivo = threeD_functions.Nodo(objetivo,
                                       vector_objetivo/np.linalg.norm(vector_objetivo),
                                       vector_objetivo/np.linalg.norm(vector_objetivo),      
                                       0,
                                       1,
                                       0,
                                       0)

obstaculos = [np.array([0, 20, 0]),
              np.array([20, 20, 0]),
              np.array([35, 10, 0]),
              np.array([40, 10, 0]),
              np.array([45, 10, 0]),
              np.array([50, 10, 0]),
              np.array([10, -20, 0]),
              np.array([55, 10, 0])]

camino, explorados, costo = threeD_functions.delta_star(Nodo_inicio,
                                    Nodo_objetivo,
                                    obstaculos,
                                    distancia_a_obstaculo,
                                    intervalo_lineal,
                                    recta_minima,
                                    radio_curvatura,
                                    intervalo_angular,
                                    curva_maxima)

print(f"Tiempo total de ejecución: {((time.time() - start_time)/60):.2f} minutos")
print(f"{len(explorados)} nodos explorados. {len(camino)} nodos en el camino")
print(f"Costo total del camino: {costo:.3f}")


# Convertir listas a arrays de numpy si aún no lo están
camino_np = np.array(camino)
explorados_np = np.array(list(explorados))
objetivo = np.array(objetivo)  # Asegúrate de que sea un array 3D

threeD_functions.exportar_camino_a_pts(camino)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Graficar obstáculos
for obs in obstaculos:
    ax.scatter(obs[0], obs[1], obs[2], c='r', marker='o', label='Obstáculo')

'''
# Graficar nodos explorados
ax.scatter(explorados_np[:, 0], explorados_np[:, 1], explorados_np[:, 2],
           c='k', s=0.1, label='Nodos explorados')
#'''

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

ax.set_xlim3d(mid_x - max_range, mid_x + max_range)
ax.set_ylim3d(mid_y - max_range, mid_y + max_range)
ax.set_zlim3d(mid_z - max_range, mid_z + max_range)

plt.show()