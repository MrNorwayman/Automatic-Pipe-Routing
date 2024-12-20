import numpy as np
import Classes
import time

def main():
    #Variables de gestion de puntos

    #Costo aproximado de 380
    size_region = 100
    num_puntos = 50000
    intervalo = 5
    tramo_recto_minimo = 50
    tramo_recto_min_corte = 10

    #Extraccion de datos del STP
    datos = Classes.buscar_y_extraer("C:/Users/GARCIAMA44/OneDrive - Carrier Corporation/Escritorio/TRABAJO/CODIGOS/TUBERIA/STP.stp")

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
    Maquina = Classes.STL("C:/Users/GARCIAMA44/OneDrive - Carrier Corporation/Escritorio/TRABAJO/CODIGOS/TUBERIA/STL.stl")
    #Maquina.previsualizacion_puntos()
    
    # Crear visualizador
    Visualizador = Classes.Nube_de_Puntos()
    Visualizador.add_stl(Maquina.archivo_stl)
    Visualizador.visualizar()

    print("Inicio D*\n")
    # Definir puntos de inicio y final

    #Crear tuberia
    flag_tubo = False

    for tubo in Tuberias:
        if flag_tubo == False:
            flag_tubo = True
            obstaculos = Maquina.fragmentacion(num_puntos, size_region)
            #Maquina.previsualizacion_puntos()
        else:
            obstaculos = Maquina.add_tubo(nube_puntos_tubo)
            Maquina.previsualizacion_puntos()
        
        tubo.actualizar_obstaculos(obstaculos)
        nube_puntos_tubo = tubo.crear_tuberia()
        Visualizador.add_punto(nube_puntos_tubo, np.random.rand(3).tolist())
        


if __name__ == "__main__":

    time_start = time.time()

    main()

    print("Tiempo total de procesamiento", (time.time() - time_start)/60, "minutos\n")
    
