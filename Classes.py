import threading
import ctypes
import open3d as o3d
import numpy as np
import time
import re
import heapq

'''
-----------------------DEFINICION DE CLASES----------------------
'''
### TUBERIA ###
class Tubo:
    '''Inicializar objeto TUBO.
        Datos entrada:  datos -> Datos de la tuberia extraidos del STP
                        intervalo -> Distancia entre nodo y nodo
                        size_region -> Tamaño de las divisiones de puntos del STL
                        tramo_recto_min -> Tramo recto minimo establecido por el tubo
                        tramo_recto_min_corte -> Tramo recto minimo posterior al corte de extremo

        Retorno:        None '''
    def __init__(self, datos, intervalo, size_region, tramo_recto_min = 70, tramo_recto_min_corte = 40, intervalo_angular = 30):  #Inicializa el tubo

        self.tramo_recto_min = tramo_recto_min
        self.tramo_recto_min_corte = tramo_recto_min_corte
        self.intervalo = intervalo
        self.size_region = size_region
        self.intervalo_angular = intervalo_angular
        self.puntos = np.empty((0,3))    #Puntos de la tuberia actuales
        self.no_puntos = np.empty((0,3)) #Puntos de la tuberia que se han probado y no sirven
        self.analizar_datos(datos)
        return None


    '''Añadir punto a el tubo
        Datos entrada:  punto_nuevo -> Nuevo punto para añadir

        Retorno:        Numpy array de puntos                                                                      '''
    def add_punto(self, punto_nuevo):   #Inserta un punto
        self.puntos = np.append(self.puntos, punto_nuevo)
        return self.puntos
    

    '''Analiza los datos y los añade a variables del objeto
    Datos entrada:  dato -> Datos extraidos del STP

    Retorno:        None                                                                      '''
    def analizar_datos(self, dato):

        if dato[0].split("_")[1] == "138":
            radio = 28.57/2
            self.radio_curva = 57
        else:
            radio = 12.7/2
            self.radio_curva = 10

        tolerancia = dato[0].split("_")[2]
        inicio = dato[1]
        final = dato[4]

        self.tolerancia = float(tolerancia)
        self.radio = float(radio)

        #Orientacion del eje z es dato[2] y dato[5]
        print(f"\nVectores inicio: {dato[2], dato[3]}\nVectores final: {dato[5], dato[6]}\n")
        self.inicio = inicio
        self.vector_inicio = dato[2]
        self.final = final
        self.vector_final = dato[5]

        print(f"Inicio: {self.inicio}\nFinal: {self.final}\nTolerancia: {self.tolerancia}   Radio: {self.radio}")
        return None

    def actualizar_obstaculos(self, obstaculos):
        self.obstaculos = obstaculos

    def quitar_ultimo(self):    #Elimina el ultimo punto
        try:
            self.puntos = self.puntos[:-1]
            return self.puntos
        
        except:
            ctypes.windll.user32.MessageBoxW(0, "Number of points in vector is 0", "Error", 16)
            return None
        
    def quitar_puntos(self, puntos_a_quitar):
        self.puntos = self.puntos[~puntos_a_quitar]
        return self.puntos
    
    def crear_tuberia(self):
        Al = Algoritmo()
        camino = Al.d_star(inicio=self.inicio,
                           final=self.final,
                           tolerancia=self.tolerancia,
                           radio=self.radio,
                           intervalo=self.intervalo,
                           obstaculos=self.obstaculos,
                           size_region=self.size_region,
                           vector_incio=self.vector_inicio,
                           vector_final=self.vector_final,
                           tramo_recto_min=self.tramo_recto_min,
                           tramo_recto_min_corte=self.tramo_recto_min_corte,
                           intervalo_angular=self.intervalo_angular,
                           radio_curvatura=self.radio_curva)
        nube_puntos_tubo = np.empty((0, 3))
        for punto in camino:
            esfera = puntos_a_esfera(punto, self.radio, 10000)
            nube_puntos_tubo = np.vstack((nube_puntos_tubo, esfera))
        
        return nube_puntos_tubo
###############

### STL ###
class STL:
    def __init__(self, archivo_stl): #Inicializa el STL
        self.archivo_stl = archivo_stl
        return None

    def stl_solido(self):
        mesh_stl = o3d.io.read_triangle_mesh(self.archivo_stl)  #Abrir STL
        mesh_stl.compute_vertex_normals()
        return mesh_stl

    def fragmentacion(self, num_puntos, size_regiones):    #Crea los fragmentos de la nube de puntos
        try:
            start_time = time.time()

            self.num_puntos = num_puntos
            self.size_regiones = size_regiones
            
            self.convertir_a_puntos()
            self.encontrar_max_y_min()
            self.crear_regiones_np()
            print("Tiempo en ejecutar", self.num_puntos, "puntos y", len(self.fragmentos), "fragmentos ->", (time.time() - start_time)/60, "minutos\n")

            return self.fragmentos
        
        except:
            ctypes.windll.user32.MessageBoxW(0, "Fragmetation failure", "Error", 16)
            quit()
            return None
        
    def add_tubo(self, nube_tubo):
        np.append(self.stl_points, nube_tubo)
        self.encontrar_max_y_min()
        self.crear_regiones_np()

        return self.fragmentos

    def convertir_a_puntos(self):   #Convierte el STL en nube de puntos y en array
        try:
            mesh_stl = o3d.io.read_triangle_mesh(self.archivo_stl)  #Abrir STL
            mesh_stl.compute_vertex_normals()
            self.stl_points_cloud = mesh_stl.sample_points_uniformly(number_of_points = self.num_puntos)    #Convertir a nube de puntos
            self.stl_points = np.asarray(self.stl_points_cloud.points)  #Convertir a matriz numpy
            self.stl_points_list = self.stl_points.tolist()
            return None

        except:
            ctypes.windll.user32.MessageBoxW(0, "No file or error at opening file \nClosing program", "Error", 16)
            quit()
            return None

    def encontrar_max_y_min(self):  # Obtiene los valores más altos y bajos de toda la nube de puntos
        # Convertir los puntos a un array de NumPy
        puntos_array = np.array(self.stl_points)

        # Encontrar el valor mínimo y máximo en cada eje (x, y, z)
        self.min_vector = np.floor(np.min(puntos_array, axis=0)) - 1  # Redondear hacia abajo y restar 1
        self.max_vector = np.ceil(np.max(puntos_array, axis=0)) + 1   # Redondear hacia arriba y sumar 1

        print("Vectores min y max: ", self.min_vector, self.max_vector, "\n")
        return None

    def crear_regiones_np(self):    #Crea las regiones y almacena los puntos en ellas
        x = np.arange(self.min_vector[0], self.max_vector[0], self.size_regiones)
        y = np.arange(self.min_vector[1], self.max_vector[1], self.size_regiones)
        z = np.arange(self.min_vector[2], self.max_vector[2], self.size_regiones)
        
        # Inicializar las regiones
        self.fragmentos = []
        
        # Convertir los puntos en un array de NumPy para aprovechar las operaciones vectorizadas
        puntos_restantes = self.stl_points

        # Iterar sobre las coordenadas de las regiones
        for xi in x:
            for yi in y:
                for zi in z:
                    region_inicio = np.array([xi, yi, zi])
                    region_final = region_inicio + self.size_regiones
                    
                    # Encontrar los puntos dentro de esta región de forma vectorizada
                    mask = np.all((puntos_restantes >= region_inicio) & (puntos_restantes < region_final), axis=1)
                    puntos_en_region = puntos_restantes[mask]
                    
                    # Agregar la región si contiene puntos
                    if len(puntos_en_region) > 0:
                        self.fragmentos.append([region_inicio + self.size_regiones/2, puntos_en_region])
                    
                    # Remover los puntos asignados a la región
                    puntos_restantes = puntos_restantes[~mask]      
        return None
    
    def previsualizacion_puntos(self): #Muestra una previsualizacion de la fragmentacion
        pcd = o3d.geometry.PointCloud()
        visualizacion = np.empty((0, 3))
        for i in range(len(self.fragmentos)):
                visualizacion = np.vstack([visualizacion, self.fragmentos[i][1]])   

        pcd.points = o3d.utility.Vector3dVector(np.asarray(visualizacion))

        o3d.visualization.draw_geometries([pcd])
###########

### NUBE DE PUNTOS ###
class Nube_de_Puntos:
    def __init__(self): #Inicializa la nube de puntos
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(np.empty((0,3)))
        return None

    def add_punto(self, puntos_a_add, color=""): #Apendiza puntos con color (gris predefinido)
        puntos_actuales = np.asarray(self.pcd.points)
        puntos_totales = np.vstack((puntos_actuales, puntos_a_add))
        self.pcd.points = o3d.utility.Vector3dVector(puntos_totales)
        
        if color != "":
            colores_actuales = np.asarray(self.pcd.colors)
            colores_a_add = np.tile(color, (len(puntos_a_add), 1))
            colores_totales = np.vstack((colores_actuales, colores_a_add))
            self.pcd.colors = o3d.utility.Vector3dVector(colores_totales)
        
        return self.pcd.points
    
    def add_stl(self, stl):
        self.mesh = o3d.io.read_triangle_mesh(stl)
        self.mesh.compute_vertex_normals()

    def quitar_puntos(self, puntos_a_quitar):
        self.pcd.points = self.pcd.points[~puntos_a_quitar]

        return self.pcd.points
    
    def visualizar(self):
        window_thread = threading.Thread(target=self.mantener_abierto)
        window_thread.start()

    def mantener_abierto(self):
        vis = o3d.visualization.Visualizer()
        
        vis.create_window()
        vis.add_geometry(self.pcd)
        if self.mesh:
            vis.add_geometry(self.mesh)

        while True:
            vis.update_geometry(self.pcd)
            if self.mesh:
                vis.poll_events()
            vis.poll_events()
            vis.update_renderer()

            time.sleep(0.03)
######################

### NODO ###
class Nodo:
    def __init__(self,
                 posicion,
                 intervalo,
                 intervalo_angular,
                 movimiento=None,
                 g=float('inf'),
                 f=float('inf'),
                 padre=None,
                 dif_mov=None,
                 ang_curva = 0,
                 lar_recta = 0,
                 posicion_padre = None):
        
        self.g = g  # Costo acumulado desde el nodo inicial
        self.f = f
        self.movimiento = movimiento
        self.angulo_curva = ang_curva
        self.largo_recta = lar_recta
        self.dif_mov = dif_mov
        self.esfericas = cartesiano_a_esferico(movimiento)
        #Diferencial de movimiento, para saber si es una recta o una curva
        self.posicion = posicion
        self.posicion_padre = posicion_padre
        
        self.padre = padre  # Nodo padre en el camino

        if np.all(dif_mov == None):
            return None
        
        else:
            #Es una recta
            if np.all(dif_mov == [0, 0, 0]):
                self.largo_recta += intervalo
                self.angulo_curva = 0
                self.g = self.g*0.9
                return
        
            else:
                self.largo_recta = 0
                self.angulo_curva += intervalo_angular
                self.g = self.g *1

    def __lt__(self, otro):
        return self.g < otro.g
############

### D STAR ###
class Algoritmo:
    def __init__(self):
        self.angulo_curva = 0
        self.angulo_max = 180

    def heuristica(self, nodo1, nodo2):  # Heurística euclidiana en 3D
        return np.linalg.norm(np.array(nodo1) - np.array(nodo2))

    def generar_vecinos(self, nodo):   # Genera los vecinos inmediatos en 3D (26 direcciones: ejes y diagonales)

        movimientos = []
        if ((nodo.largo_recta < self.tramo_recto_min) and (nodo.largo_recta > 0)):    
            movimientos.append(np.array(nodo.movimiento) * self.intervalo)

        #Generacion de vecinos cambiando los angulos polares
        else:
            esfericas = []
            esfericas.append([nodo.esfericas[0], nodo.esfericas[1] + self.intervalo_angular, nodo.esfericas[2]])
            esfericas.append([nodo.esfericas[0], nodo.esfericas[1] - self.intervalo_angular, nodo.esfericas[2]])
            esfericas.append([nodo.esfericas[0], nodo.esfericas[1], nodo.esfericas[2] + self.intervalo_angular])
            esfericas.append([nodo.esfericas[0], nodo.esfericas[1], nodo.esfericas[2] - self.intervalo_angular])

            for esferica in esfericas:
                movimientos.append(esferico_a_cartesiano(esferica) * self.mag_curva)


            movimientos.append(np.array(nodo.movimiento) * self.intervalo)

        return [tuple(np.array(nodo.posicion) + np.array(mov)) for mov in movimientos]

    def es_cercano_a_obstaculo(self, vecino_pos, obstaculos, size_region_sqrt3, tol_rad):    # Función para comprobar si un punto está cerca de un obstáculo dentro de una tolerancia
        for region in obstaculos:
            if np.linalg.norm(np.array(vecino_pos) - np.array(region[0])) < size_region_sqrt3:   #Se usa 1.8 por ser un valor similar a sqrt(3)
                for obstaculo in region[1]:
                    if np.linalg.norm(np.array(vecino_pos) - np.array(obstaculo)) < tol_rad:
                        return True
        return False

    def interpolar(self, v1, v2, intervalo):
        v1 = np.array(v1)
        v2 = np.array(v2)
        # Calcular la distancia total correctamente
        distancia_total = np.linalg.norm(v2 - v1)
        # Calcular el número de puntos interpolados
        n_puntos = int(distancia_total // intervalo)
        # Generar los puntos interpolados
        puntos_interpolados = [v1 + i * intervalo * (v2 - v1) / distancia_total for i in range(n_puntos + 1)]
        # Retornar los puntos
        return puntos_interpolados

    def d_star(self,
               inicio,
               final,
               tolerancia,
               radio,
               intervalo,
               obstaculos,
               size_region,
               vector_incio,
               vector_final,
               tramo_recto_min,
               tramo_recto_min_corte,
               intervalo_angular = 30,
               radio_curvatura = 13):   # Algoritmo D* simplificado

        self.tramo_recto_min = tramo_recto_min
        self.intervalo = intervalo
        self.vector_inicio = vector_incio
        self.vector_final = vector_final
        self.intervalo_angular = np.deg2rad(intervalo_angular)

        self.inicio = np.array(inicio) + np.array(self.vector_inicio)*self.tramo_recto_min
        self.final = np.array(final) + np.array(self.vector_final)* tramo_recto_min_corte
        self.mag_curva = 2 * radio_curvatura * np.sin(self.intervalo_angular)
        print(f"Magnitud de curva: {self.mag_curva}")

        mapa = {}
        nodo_inicio = Nodo(posicion=self.inicio,
                           intervalo=self.intervalo,
                           intervalo_angular=intervalo_angular,
                           movimiento=self.vector_inicio,
                           g=0,
                           f=0,
                           lar_recta=self.tramo_recto_min,
                           ang_curva=0)
        
        mapa[tuple(inicio)] = nodo_inicio
        tol_rad = tolerancia + radio

        abierta = []
        heapq.heappush(abierta, nodo_inicio)

        iteracion = 0
        size_region_factor = 1.8*size_region

        while abierta:
            iteracion += 1  # Incrementar el número de iteración
            nodo_actual = heapq.heappop(abierta)

            # Imprimir información de la iteración actual
            if iteracion % 100 == 0:
                print(f"\nIteración {iteracion}: Costo acumulado {nodo_actual.g}")

            # Comprobar si estamos lo suficientemente cerca del objetivo (con tolerancia)
            if np.linalg.norm(np.array(nodo_actual.posicion) - np.array(self.final)) < 1.8*self.intervalo:
                print(f"\n¡Objetivo alcanzado!\nFueron necesarias {iteracion} iteraciones\nCosto acumulado: {nodo_actual.g}")
                camino = []
                camino_nodos = []
                while nodo_actual is not None:
                    camino_nodos.append(nodo_actual)
                    camino.append(nodo_actual.posicion)
                    nodo_actual = nodo_actual.padre
                camino = camino[::-1]

                for index, nodo in enumerate(camino_nodos):
                    print(f"Nodo {index}, movimiento {nodo.movimiento}")
                    if np.all(nodo.dif_mov == [0, 0, 0]):
                        print(f"Recto. ------ Largo de recta: {nodo.largo_recta}")
                    else:
                        print(f"Curvo. {nodo.dif_mov}")

                camino.extend(self.interpolar(inicio, self.inicio, self.intervalo))
                camino.extend(self.interpolar(final, self.final, self.intervalo))
                return camino # Devuelve el camino en el orden correcto

            # Generar vecinos del nodo actual
            for vecino_pos in self.generar_vecinos(nodo_actual):
                # Comprobar si el vecino está cerca de algún obstáculo (dentro de la tolerancia)
                if self.es_cercano_a_obstaculo(vecino_pos, obstaculos, size_region_factor, tol_rad):
                    continue  # Saltar si está demasiado cerca de un obstáculo

                #Calcula la nueva heuristica
                nuevo_f = nodo_actual.f + self.heuristica(nodo_actual.posicion, vecino_pos)
                nuevo_g = np.linalg.norm(np.array(vecino_pos) - np.array(self.final)) + nuevo_f*0.99
                
                if vecino_pos not in mapa:
                    #Calcula movimiento y diferencial de movimiento para el nuevo Nodo
                    movimiento = np.array(vecino_pos) - np.array(nodo_actual.posicion)
                    movimiento = movimiento / np.linalg.norm(movimiento)
                    dif_mov = nodo_actual.movimiento - movimiento

                    #Crea los nuevos nodos vecinos
                    vecino = Nodo(posicion=vecino_pos,
                                  intervalo=self.intervalo,
                                  intervalo_angular=intervalo_angular,
                                  movimiento=movimiento,
                                  g=nuevo_g,
                                  f=nuevo_f,
                                  padre=nodo_actual,
                                  dif_mov=dif_mov,
                                  ang_curva=nodo_actual.angulo_curva,
                                  lar_recta=nodo_actual.largo_recta,
                                  posicion_padre=nodo_actual.posicion)                    
                    
                    mapa[vecino_pos] = vecino
                    heapq.heappush(abierta, vecino)
                else:
                    vecino = mapa[vecino_pos]
                    if nuevo_g < vecino.g:
                        vecino.g = nuevo_g
                        vecino.padre = nodo_actual
                        heapq.heappush(abierta, vecino)

        print("No se encontró un camino.")
        return []  # Si no se encuentra camino, devuelve una lista vacía

    def imprimir_camino(self, camino):
        # Convertir a float
        camino_legible = [(float(p[0]), float(p[1]), float(p[2])) for p in camino]
        print("Camino encontrado:")
        for paso in camino_legible:
            print(paso)
##############

'''
-----------------------DEFINICION DE FUNCIONES----------------------
'''
### INFORMACION STP ###
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


def buscar_y_extraer(archivo):
    with open(archivo, "r") as file:
        lineas = file.readlines()

    matriz_puertos = []

    for num_linea, linea in enumerate(lineas):
        if "START_" in linea:

            valor_eje = []

            indice = linea.split(",")[0].split("'")[1][6::]
            valor_eje.append(indice)
            i=3
            while i > 0:
                coordenadas = extraer_valores_cartesianos(lineas[num_linea - i])
                valor_eje.append(coordenadas)
                i -= 1

            end_line = False
            for num_linea, linea in enumerate(lineas):
                if ("END_" + indice) in linea:
                    match = re.search(r"(END_)\d+", linea)
                    end_line = True
                    i = 3

                    while i > 0:
                        coordenadas = extraer_valores_cartesianos(lineas[num_linea - i])
                        valor_eje.append(coordenadas)
                        i -= 1
            
            if end_line == False:
                ctypes.windll.user32.MessageBoxW(0, "Not enough END ports in STP file", "Error", 16)

            matriz_puertos.append(valor_eje)
    if matriz_puertos == []:
        ctypes.windll.user32.MessageBoxW(0, "There is no ports in the STP file", "Error", 16)
        quit()
    
    return matriz_puertos  

def extraer_valores_cartesianos(linea):

    if linea[4] == "C":
        valores = linea[24:-4].split(",")

    if linea[4] == "D":
        valores = linea[18:-4].split(",")
    
    return [float(valor) for valor in valores]

def puntos_a_esfera(punto_central, radio, num_puntos):
    puntos = []
    
    for _ in range(num_puntos):
        # Generar un punto aleatorio en coordenadas esféricas
        theta = np.random.uniform(0, 2 * np.pi)  # Ángulo azimutal
        phi = np.random.uniform(0, np.pi)  # Ángulo polar

        # Convertir coordenadas esféricas a cartesianas
        x = punto_central[0] + radio * np.sin(phi) * np.cos(theta)
        y = punto_central[1] + radio * np.sin(phi) * np.sin(theta)
        z = punto_central[2] + radio * np.cos(phi)

        puntos.append([x, y, z])
    
    return np.array(puntos)
#######################