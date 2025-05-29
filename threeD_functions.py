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

    pos_objetivo = Nodo_objetivo.posicion   #Posicion del objetivo
    intervalo_arco_curva = radio_curvatura / 2 * np.sin(2*intervalo_angular) / np.cos(intervalo_angular)  #Intervalo de arco de curva
    
    cont_explorados = 1

    explorados = []  #Lista de nodos explorados
    while not abiertos.empty():
        _, _, nodo_actual = abiertos.get()  # Extrae el nodo con menor coste

        pos_tuple = tuple(np.round(nodo_actual.posicion, decimals=5))
        if pos_tuple in cerrados:
            continue
        if cerca_de_obstaculos(nodo_actual.posicion, obstaculos, distancia_a_obstaculo):  # Si el nodo actual esta cerca de un obstaculo
            continue
        cerrados.add(pos_tuple)

        if np.linalg.norm(np.array(nodo_actual.posicion) - np.array(pos_objetivo)) < intervalo_lineal: # Si el nodo actual esta cerca del objetivo:
            
            print(nodo_actual)
            print(nodo_actual.posicion)
            camino = reconstruir_camino(nodo_actual)
            costo = nodo_actual.costo
            # Dibuja camino final

            return camino, explorados, costo

        # Se generan Nodos vecinos
        contador, explorados, abiertos = generar_vecinos(nodo_actual,
                                                            intervalo_lineal,
                                                            recta_minima,
                                                            intervalo_arco_curva,
                                                            intervalo_angular,
                                                            curva_maxima,
                                                            pos_objetivo,
                                                            contador,
                                                            explorados,
                                                            abiertos)
        
        
        if len(explorados) > cont_explorados*1000000:
            print(f"Iteración: {cont_explorados} millones")
            cont_explorados += 1
    print("No se encontró camino")
    return None  # No se encontró camino


def generar_vecinos(Nodo_actual,
                    intervalo_lineal,
                    recta_minima,
                    intervalo_arco_curva,
                    intervalo_angular,
                    curva_maxima,
                    objetivo,
                    contador,
                    explorados,
                    abiertos):

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

    # Siempre esta la opcion de seguir recto
    nueva_pos = Nodo_actual.posicion + intervalo_lineal * Nodo_actual.vector
    nuevo_nodo = Nodo(
        posicion=nueva_pos,
        vector=Nodo_actual.vector,
        vector_padre=Nodo_actual.vector,
        longitud_recta=Nodo_actual.longitud_recta + intervalo_lineal,
        angulo_curva=0,
        costo=Nodo_actual.costo + intervalo_lineal,
        heuristica=heuristica(nueva_pos, objetivo, 0),
        padre=Nodo_actual
    )
    explorados.append(Nodo_actual.posicion)  # Añade el nodo a la lista de explorados
    abiertos.put((nuevo_nodo.f(), next(contador), nuevo_nodo))  # Se almacena en la cola de prioridad junto con su coste y heurística

    if 0 < Nodo_actual.angulo_curva < curva_maxima:
        # Continua curva creando un plano entre las posiciones actual, padre y abuelo
        vector = vector_en_plano_desde_tres_puntos(Nodo_actual.vector_padre, Nodo_actual.vector, Nodo_actual.posicion)
        nueva_pos = Nodo_actual.posicion + intervalo_arco_curva * vector

        nuevo_nodo = Nodo(
            posicion=nueva_pos,
            vector=vector,
            vector_padre=Nodo_actual.vector,
            longitud_recta=0,
            angulo_curva=Nodo_actual.angulo_curva + intervalo_angular,
            costo=Nodo_actual.costo + intervalo_arco_curva,
            heuristica=heuristica(nueva_pos, objetivo, Nodo_actual.angulo_curva + intervalo_angular),
            padre=Nodo_actual
        )
        explorados.append(Nodo_actual.posicion)  # Añade el nodo a la lista de explorados
        abiertos.put((nuevo_nodo.f(), next(contador), nuevo_nodo))  # Se almacena en la cola de prioridad junto con su coste y heurística


    elif (Nodo_actual.longitud_recta > recta_minima):
        # Crea el anillo de puntos a partir del ultimo nodo
        centro_anillo = Nodo_actual.posicion + Nodo_actual.vector * (intervalo_arco_curva * np.cos(intervalo_angular))
        anillo = generar_anillo_en_extremo(Nodo_actual.vector, centro_anillo, radio=intervalo_arco_curva*np.sin(intervalo_angular), num_puntos=int(360/np.degrees(intervalo_angular)))
        for nueva_pos in anillo:
            nuevo_vector = nueva_pos - Nodo_actual.posicion

            nuevo_nodo = Nodo(
                posicion=nueva_pos,
                vector=nuevo_vector/np.linalg.norm(nuevo_vector),
                vector_padre=Nodo_actual.vector,
                longitud_recta=0,
                angulo_curva=intervalo_angular,
                costo=Nodo_actual.costo + intervalo_arco_curva,
                heuristica=heuristica(nueva_pos, objetivo, intervalo_angular),
                padre=Nodo_actual
            )
            explorados.append(Nodo_actual.posicion)  # Añade el nodo a la lista de explorados
            abiertos.put((nuevo_nodo.f(), next(contador), nuevo_nodo))  # Se almacena en la cola de prioridad junto con su coste y heurística
    
    return contador, explorados, abiertos


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
        camino.append(current.posicion.tolist())  # Usar directamente el nodo actual
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


def vector_en_plano_desde_tres_puntos(u, v, nodo_pos, size=1.0):
    # Vectores en el plano
    angulo_entre_vectores = calcular_angulo_entre_vectores(u, v)
    
    # Vector desde p3 en el plano con ángulo 30 grados respecto a v
    w = vector_en_plano_con_angulo(u, v, nodo_pos, -angulo_entre_vectores)
    return w / np.linalg.norm(w)


def calcular_angulo_entre_vectores(u, v):
    dot_product = np.dot(u, v)
    dot_product = np.clip(dot_product, -1.0, 1.0)
    angulo_rad = np.arccos(dot_product / (np.linalg.norm(u)*np.linalg.norm(v)))
    angulo_deg = np.degrees(angulo_rad)
    return angulo_deg


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
    # Verificar si u y v son colineales
    norm_u_ort = np.linalg.norm(u_ort)
    if norm_u_ort < 1e-8:
        # u y v son colineales: simplemente rota v en el plano (en realidad no hay un plano definido distinto)
        return v_unit  # o puedes retornar directamente np.cos(theta) * v_unit si quisieras cambiar la magnitud
    u_ort_unit = u_ort / norm_u_ort
    # Convertir ángulo a radianes
    theta = np.deg2rad(angulo_grados)
    # Vector combinado en el plano
    w = np.cos(theta) * v_unit + np.sin(theta) * u_ort_unit

    return w


def generar_anillo_en_extremo(vector, posicion, radio=0.1, num_puntos=36):

    """
    Genera un anillo de puntos en el plano perpendicular al vector,
    centrado en su extremo.
    Parámetros:
    - vector: array-like de forma (3,), vector 3D
    - radio: radio del anillo
    - num_puntos: número de puntos en el anillo
    Retorna:
    - puntos: array (num_puntos, 3), coordenadas 3D de los puntos del anillo
    """
    vector = np.array(vector)
    vector_unit = vector / np.linalg.norm(vector)

    # Encontrar dos vectores ortogonales al vector dado
    if np.allclose(vector_unit, [0, 0, 1]):
        ort1 = np.array([1, 0, 0])
    else:
        ort1 = np.cross(vector_unit, [0, 0, 1])
        ort1 /= np.linalg.norm(ort1)
    ort2 = np.cross(vector_unit, ort1)

    # Centro del anillo (extremo del vector)
    centro = posicion
    # Generar puntos sobre el anillo
    puntos = []
    for a in np.linspace(0, 2*np.pi, num_puntos, endpoint=False):
        punto = centro + radio * (np.cos(a) * ort1 + np.sin(a) * ort2)
        puntos.append(punto)

    return np.array(puntos)


def exportar_camino_a_pts(camino, nombre_archivo='camino.pts'):
    """
    Exporta una lista de puntos 3D a un archivo .pts.
    
    Parámetros:
    - camino: lista de listas o arrays [x, y, z]
    - nombre_archivo: nombre del archivo de salida
    """
    with open(nombre_archivo, 'w') as f:
        for punto in camino:
            f.write(f"{punto[0]} {punto[1]} {punto[2]}\n")
