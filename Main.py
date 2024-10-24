import numpy as np
import Classes
import time

def main():
    #Variables de gestion de puntos

    #Costo aproximado de 380
    size_region = 100
    num_puntos = 1000000
    intervalo = 7.5
    tramo_recto_minimo = 60
    tramo_recto_min_corte = 30

    #Extraccion de datos del STP
    datos = Classes.buscar_y_extraer("STP.stp")

    #Creacion de objeto de tuberia
    Tuberias = []
    for dato in datos:
        Tubo = Classes.Tubo(dato,
                            intervalo,
                            size_region,
                            tramo_recto_min=tramo_recto_minimo,
                            tramo_recto_min_corte=tramo_recto_min_corte)
        Tuberias.append(Tubo)

    # Crear objeto STL y fragmentar
    Maquina = Classes.STL("STL.stl")
    obstaculos = Maquina.fragmentacion(num_puntos, size_region)
    #Maquina.previsualizacion_puntos()

    Tuberias[0].actualizar_obstaculos(obstaculos)

    # Crear visualizador
    Visualizador = Classes.Nube_de_Puntos()
    Visualizador.add_stl(Maquina.archivo_stl)
    Visualizador.visualizar()

    print("Inicio D*\n")
    # Definir puntos de inicio y final

    #Crear tuberia
    for index, tubo in enumerate(Tuberias):
        nube_puntos_tubo =tubo.crear_tuberia()
        Visualizador.add_punto(nube_puntos_tubo, [0, 1, index])

if __name__ == "__main__":

    time_start = time.time()

    main()

    print("Tiempo total de procesamiento", (time.time() - time_start)/60, "minutos\n")
    
