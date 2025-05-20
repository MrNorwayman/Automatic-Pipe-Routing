import numpy as np, itertools, matplotlib.pyplot as plt
from queue import PriorityQueue
from threeD_Nodo import Nodo


def delta_star(Nodo_inicio,
               Nodo_objetivo,
               obstaculos,
               distancia_a_obstaculo,
               intervalo_lineal,
               recta_minima,
               radio_curvatura,
               intervalo_angular,
               curva_maxima):
    '''##########################################################
    Algoritmo de busqueda de caminos D*
    Variables de entrada:
    -> Nodo_inicio: Nodo de inicio
    -> Nodo_objetivo: Nodo de final
    -> intervalo_lineal: distancia entre los nodos en recta
    -> recta_minima: longitud minima del tramo recto
    -> radio_curvatura: radio de plegado
    -> intervalo_angular: distancia entre los nodos en curva
    -> curva_maxima: Angulo maximo de la curva
    Variables de salida:
    <- movimientos: Lista de posiciones de los nuevos nodos
    ##########################################################'''
    abiertos = PriorityQueue()  #Crea la cola de prioridad
    contador = itertools.count()    #Contador para la cola de prioridad
    Nodo_inicio.heuristica = heuristica(Nodo_inicio.posicion, Nodo_objetivo.posicion, Nodo_inicio.angulo_curva)  #Calcula la heuristica del nodo inicial
    abiertos.put((Nodo_inicio.f(), next(contador), Nodo_inicio))  #Estructura de la cola de prioridad
    # Cuanta menor f(), mayor prioridad
    cerrados = set()    #Conjunto de nodos cerrados

    objetivo = Nodo_objetivo.posicion   #Posicion del objetivo
    intervalo_arco_curva = radio_curvatura / 2 * np.sin(2*intervalo_angular) / np.cos(intervalo_angular)  #Intervalo de arco de curva
    
    explorados = []  #Lista de nodos explorados
    while not abiertos.empty():
        _, _, nodo_actual = abiertos.get()  # Extrae el nodo con menor coste

        pos_tuple = tuple(np.round(nodo_actual.posicion, decimals=5))
        if pos_tuple in cerrados:
            continue
        if cerca_de_obstaculos(nodo_actual.posicion, obstaculos, distancia_a_obstaculo):  # Si el nodo actual esta cerca de un obstaculo
            continue
        cerrados.add(pos_tuple)

        if np.linalg.norm(np.array(nodo_actual.posicion) - np.array(objetivo)) < intervalo_lineal: # Si el nodo actual esta cerca del objetivo:
            camino = reconstruir_camino(nodo_actual)
            # Dibuja camino final

            return camino, explorados

############################ CAMBIO 3D -> Los cambios comienzan a partir de esta linea

        posiciones_vecinos, nuevas_theta, nuevas_phi = generar_vecinos(  nodo_actual,
                                                                intervalo_lineal,
                                                                recta_minima,
                                                                intervalo_arco_curva,
                                                                intervalo_angular,
                                                                curva_maxima)   # Se generan vecinos
        for i, nueva_pos in enumerate(posiciones_vecinos):
            nuevo_theta = nuevas_theta[i]
            nuevo_phi = nuevas_phi[i]

            # Cálculo de tramo recto y ángulo de curva
            if (nuevo_theta == nodo_actual.theta) and (nuevo_phi == nodo_actual.phi):  # Si sigue con los mismos angulos
                longitud_recta = nodo_actual.longitud_recta + intervalo_lineal
                angulo_curva = 0
            else:   # Si cambia el angulo
                longitud_recta = 0
                angulo_curva = nodo_actual.angulo_curva + intervalo_angular

            # Se crea el nuevo nodo vecino
            nuevo_nodo = Nodo(
                posicion=nueva_pos,
                theta=nuevo_theta,
                phi=nuevo_phi,
                theta_padre=nodo_actual.theta,
                phi_padre=nodo_actual.phi,
                longitud_recta=longitud_recta,
                angulo_curva=angulo_curva,
                costo=nodo_actual.costo + np.linalg.norm(np.array(nodo_actual.posicion) - np.array(nueva_pos)),
                heuristica=heuristica(nueva_pos, objetivo, angulo_curva),
                padre=nodo_actual
            )
            
            explorados.append(nueva_pos)  # Añade el nodo a la lista de explorados

            # Se almacena en la cola de prioridad junto con su coste y heurística
            abiertos.put((nuevo_nodo.f(), next(contador), nuevo_nodo))

    return None  # No se encontró camino


def generar_vecinos(Nodo,
                    intervalo_lineal,
                    recta_minima,
                    intervalo_arco_curva,
                    intervalo_angular,
                    curva_maxima):

    '''##########################################################
    Generar vecinos de un nodo
    Variables de entrada:
    -> Nodo: Nodo del que se parte para crear los vecinos
    -> intervalo_lineal: Distancia entre los nodos en recta
    -> recta_minima: Longitud minima de la recta
    -> intervalo_arco_curva: Intervalo en curvas
    -> curva_maxima: Angulo maximo de la curva
    Variables de salida:
    <- posiciones: Lista de posiciones de los nuevos nodos
    <- angulos: Lista de angulos de los nuevos nodos
    ##########################################################'''

    # Continua recto usando los mismos angulos que antes
    theta = Nodo.theta
    phi = Nodo.phi
    theta_padre = Nodo.theta_padre
    phi_padre = Nodo.phi_padre


    if (0 < Nodo.longitud_recta < recta_minima) or (Nodo.angulo_curva > curva_maxima - intervalo_angular * 0.5):
        movimientos = [(theta, phi)]
    elif 0 < Nodo.angulo_curva < curva_maxima:
        # Reflejar el ángulo como en el plano 2D
        movimientos = [
            (theta, phi),
            (2 * theta - theta_padre, 2 * phi - phi_padre)
        ]
    else:
        movimientos = [
            (theta, phi),
            (theta + intervalo_angular, phi),
            (theta - intervalo_angular, phi),
            (theta, phi + intervalo_angular),
            (theta, phi - intervalo_angular)
        ]

    posiciones = []
    nuevas_theta = []
    nuevas_phi = []
    for i, (t, p) in enumerate(movimientos):
        distancia = intervalo_lineal if i == 0 else intervalo_arco_curva
        direccion = np.array([
            np.sin(p) * np.cos(t),
            np.sin(p) * np.sin(t),
            np.cos(p)
        ])
        nueva_pos = Nodo.posicion + distancia * direccion
        posiciones.append(nueva_pos)
        nuevas_theta.append(t)
        nuevas_phi.append(p)

    return posiciones, nuevas_theta, nuevas_phi
    '''
    if (0 < Nodo.longitud_recta < recta_minima) or (Nodo.angulo_curva > curva_maxima - intervalo_angular * 0.5):
        angulos = [Nodo.movimiento]
    elif 0 < Nodo.angulo_curva < curva_maxima:
        angulos = [Nodo.movimiento, 2 * Nodo.movimiento - Nodo.movimiento_padre]
    else:
        angulos = [
            Nodo.movimiento,
            Nodo.movimiento + intervalo_angular,
            Nodo.movimiento - intervalo_angular,
        ]

    posiciones = []
    # Para el primer ángulo usar intervalo_lineal, para el resto intervalo_arco_curva
    for i, angulo in enumerate(angulos):
        distancia = intervalo_lineal if i == 0 else intervalo_arco_curva
        desplazamiento = distancia * np.array([np.cos(angulo), np.sin(angulo)])
        nueva_pos = Nodo.posicion + desplazamiento
        posiciones.append(nueva_pos)

    return posiciones, angulos
    '''



############################ CAMBIO 3D -> Los cambios terminan a partir de esta linea


def reconstruir_camino(nodo):
    '''##########################################################
    Funcion que devuelve el camino calculado por el algoritmo
    Variables de entrada:
    -> nodo: Nodo objetivo
    Variables de salida:
    <- camino: Lista de posiciones del camino
    ##########################################################'''
    camino = []
    current = nodo
    while current is not None:
        camino.append(current.posicion.tolist())
        current = current.padre
    camino.reverse()
    return camino


def heuristica(posicion, objetivo, angulo_curva):    
    '''##########################################################
    Funcion que devuelve la distancia al objetivo. Cuanto mayor sea el valor, menor prioridad tiene
    Variables de entrada:
    -> posicion: Posicion actual
    -> objetivo: Posicion del objetivo
    -> angulo_curva: Angulo de curva
    Variables de salida:
    <- return: Distancia al objetivo
    ##########################################################'''
    posicion = np.asarray(posicion)
    objetivo = np.asarray(objetivo)
    distancia = np.linalg.norm(posicion - objetivo)

    if angulo_curva > 0:
        if angulo_curva == np.deg2rad(90):
            factor = 1
        else:
            factor = 1
    else:
        factor = 1

    return factor * distancia


def cerca_de_obstaculos(posicion, obstaculos, distancia_a_obstaculo):
    '''##########################################################
    Funcion que devuelve True si el nodo esta cerca de un obstaculo
    Variables de entrada:
    -> posicion: Posicion actual
    -> obstaculos: Lista de obstaculos
    -> intervalo_lineal: Distancia entre los nodos en recta
    Variables de salida:
    <- return: True si el nodo esta cerca de un obstaculo
    ##########################################################'''
    posicion = np.asarray(posicion)  # Convertir solo una vez
    return any(
        np.linalg.norm(posicion - np.asarray(obstaculo)) < distancia_a_obstaculo
        for obstaculo in obstaculos
    )