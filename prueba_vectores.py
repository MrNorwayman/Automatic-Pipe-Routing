    def generar_vecinos_antiguo(self, nodo, k):   # Genera los vecinos inmediatos en 3D (6 direcciones básicas)
        movimientos = [
            (k, 0, 0), (-k, 0, 0),
            (0, k, 0), (0, -k, 0),
            (0, 0, k), (0, 0, -k)
        ]
        return [tuple(np.array(nodo.posicion) + np.array(mov)) for mov in movimientos]


    def generar_vecinos_nuevos(self, nodo, k):
        if nodo.largo_recta < self.tramo_recto_min:
            movimientos = [np.array(nodo.movimiento)*k]
        movimientos = [
            (k, 0, 0), (-k, 0, 0),
            (0, k, 0), (0, -k, 0),
            (0, 0, k), (0, 0, -k)
        ]
        return [tuple(np.array(nodo.posicion) + np.array(mov)) for mov in movimientos]


#Ini Polar
    def d_star_polares(self, inicio, final, tolerancia, diametro, intervalo, obstaculos, size_region):   # Algoritmo D* simplificado

        self.intervalo = intervalo

        mapa = {}
        nodo_inicio = Nodo(inicio, self.intervalo, self.intervalo_angular, self.mov_inicio, g=0, dif_mov=np.array([0, 0, 0]), lar_recta=self.recta_minima)
        mapa[tuple(inicio)] = nodo_inicio
        tol_diam = tolerancia + diametro

        abierta = []
        heapq.heappush(abierta, nodo_inicio)

        iteracion = 0
        size_region_factor = 1.8*size_region

        while abierta:
            iteracion += 1  # Incrementar el número de iteración
            nodo_actual = heapq.heappop(abierta)

            # Imprimir información de la iteración actual
            if iteracion % 100 == 0:
                print(f"Iteración {iteracion}: Costo acumulado {nodo_actual.g}")

            # Comprobar si estamos lo suficientemente cerca del objetivo (con tolerancia)
            if np.linalg.norm(np.array(nodo_actual.posicion) - np.array(final)) < intervalo:
                print("¡Objetivo alcanzado!")
                camino = []
                while nodo_actual is not None:
                    camino.append(nodo_actual.posicion)
                    nodo_actual = nodo_actual.padre
                return camino[::-1]  # Devuelve el camino en el orden correcto



            ### --------- Generar vecinos --------- ###
            #Obliga a seguir recto
            if (nodo_actual.largo_recta < self.recta_minima) or (nodo_actual.angulo_curva > self.angulo_max):
                nuevos_vecinos = self.seguir_recto(nodo_actual, intervalo)

            else:
                nuevos_vecinos = []
                nuevos_vecinos.append(self.seguir_recto(nodo_actual))
                nuevos_vecinos.append(self.seguir_curva(nodo_actual))
                nuevos_vecinos.append(self.generar_vecinos_polares(nodo_actual))



            for vecino_pos in nuevos_vecinos:
                # Comprobar si el vecino está cerca de algún obstáculo (dentro de la tolerancia)
                if self.es_cercano_a_obstaculo(vecino_pos, obstaculos, size_region_factor, tol_diam):
                    continue  # Saltar si está demasiado cerca de un obstáculo

                nuevo_g = nodo_actual.g + self.heuristica(nodo_actual.posicion, vecino_pos)
                
                if vecino_pos not in mapa:
                    movimiento = np.array(nodo_actual.posicion) - np.array(vecino_pos)
                    dif_mov = np.array(nodo_actual.dif_mov) - np.array(nodo_actual.movimiento)
                    print(movimiento, dif_mov)
                    vecino = Nodo(vecino_pos, self.intervalo, self.intervalo_angular, movimiento, g=nuevo_g, padre=nodo_actual, dif_mov=dif_mov)
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

    def seguir_recto(self, nodo):  #No varia el vector de direccion
        self.largo_recta += self.intervalo
        return tuple(np.array(nodo.posicion) + np.array(nodo.movimiento))

    def seguir_curva(self, nodo):
        return tuple(np.array(nodo.posicion) + np.array(nodo.dif_mov))

    def generar_vecinos_polares(self, posicion, nodo, incremento=25, incremento_phi=np.pi/8):

        movimientos = [

        ]
        return [tuple(np.array(posicion) + np.array(mov)) for mov in movimientos]
#Fin Polar