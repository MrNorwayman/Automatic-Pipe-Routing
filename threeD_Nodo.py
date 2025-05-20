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


    # CAMBIO 3D -> Se añade variable extra para angulo phi
    def __init__(self,
                 posicion,
                 theta,
                 phi,
                 theta_padre,
                 phi_padre,
                 longitud_recta,
                 angulo_curva,
                 costo,
                 heuristica=0,
                 padre=None):
        # Variables del movimiento
        self.posicion = posicion
        self.theta = theta
        self.phi = phi
        self.theta_padre = theta_padre
        self.phi_padre = phi_padre
        self.longitud_recta = longitud_recta
        self.angulo_curva = angulo_curva
        self.heuristica = heuristica
        self.padre = padre

        # Variables de coste
        self.costo = costo

    def f(self):
        return 1 * self.costo + 1 * self.heuristica