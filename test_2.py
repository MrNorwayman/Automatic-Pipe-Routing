import numpy as np

def cartesiano_a_esferico(vector):
    r = np.linalg.norm(vector)
    theta = np.arctan2(vector[1], vector[0])  # Ángulo acimutal
    phi = np.arccos(vector[2] / r)            # Ángulo polar
    return np.array([r, theta, phi])

def esferico_a_cartesiano(vector):
    r = vector[0]
    theta = vector[1]  # Theta en radianes
    phi = vector[2]    # Phi en radianes
    
    x = r * np.sin(phi) * np.cos(theta)
    y = r * np.sin(phi) * np.sin(theta)
    z = r * np.cos(phi)
    
    return np.array([x, y, z])

v = [0.5,        0.20710678, 0.        ]
print(np.linalg.norm(v))

v2 = cartesiano_a_esferico(v)
print(v2)

v1 = esferico_a_cartesiano(v2)
print(v1)


[array([-7.07106781e-01,  7.07106781e-01,  0]), array([7.07106781e-01, 7.07106781e-01, 0]), array([ 0,  7.07106781e-01, -7.07106781e-01]), array([0, 7.07106781e-01, 7.07106781e-01]), array([0., 1., 0.])]