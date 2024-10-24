import numpy as np

movimiento1 = np.array([0, 1])
movimiento1 = movimiento1 / np.linalg.norm(movimiento1)

movimiento2 = np.array([np.cos(np.pi/2*1/2), np.sin(np.pi/2*1/2)])
movimiento2 = movimiento2 / np.linalg.norm(movimiento2)

movimiento_curva = np.array(movimiento2) - np.array(movimiento1)
movimiento_curva = movimiento_curva / np.linalg.norm(movimiento_curva)

print(np.cos(np.pi/2*1/2))
print(movimiento1, movimiento2)
print(movimiento_curva)