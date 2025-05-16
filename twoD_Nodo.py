class Nodo:
    '''##########################################################
    El nodo representa cada posibilidad que puede seguir la tuberia.
    Variables de entrada:
    -> posicion: Posicion del nodo en el espacio.
    -> movimiento: Angulo de coordenadas polares.
    -> movimiento_padre: Angulo de coordenadas polares del nodo padre.
    -> longitud_recta: Longitud del tramo recto del que forma parte.
    -> angulo_curva: Angulo de la curva que forma parte.
    -> costo: Costo acumulado de la tuberia
    -> heuristica: Costo estimado de la tuberia
    -> padre: Nodo padre del que se parte para crear este nodo
    ##########################################################'''

    def __init__(self, posicion, movimiento, movimiento_padre, longitud_recta, angulo_curva, costo, heuristica=0, padre=None):
        # Variables del movimiento
        self.posicion = posicion
        self.movimiento = movimiento
        self.movimiento_padre = movimiento_padre
        self.longitud_recta = longitud_recta
        self.angulo_curva = angulo_curva
        self.heuristica = heuristica
        self.padre = padre

        # Variables de coste
        self.costo = costo

    def f(self):
        return 1 * self.costo + 1 * self.heuristica