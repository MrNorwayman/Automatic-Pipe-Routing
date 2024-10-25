import numpy as np

def cartesiano_a_esferico(vector):
    r = np.linalg.norm(vector)
    vector = vector/r
    theta = np.arctan2(vector[0], vector[2])
    if theta < 0:
        theta = np.pi*2 + theta
    phi = np.arccos(vector[1])
    if phi < 0:
        phi = np.pi*2 + phi
    return np.array([r, theta*180/np.pi, phi*180/np.pi])

v1 = [0.1, 1, 0.1]
v2 = [0, 1, 0]

v1 = v1 / np.linalg.norm(v1)
print(cartesiano_a_esferico(v1))
v1[2] = np.pi/2-v1[2]

v2 = [0, 1, 0]
v2 = v2 / np.linalg.norm(v2)
print(cartesiano_a_esferico(v2))


print(np.sqrt(v1[1]**2 + v1[2]**2))