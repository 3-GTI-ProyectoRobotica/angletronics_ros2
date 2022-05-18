import cv2
import numpy as np
from matplotlib import pyplot as plt

#Load the original larger image and the template image

amarillo = cv2.imread("amarilloJpg.jpg")
azul = cv2.imread("azulJpg.jpg")
verde = cv2.imread("verdeJpg.jpg")
foto = cv2.imread("test_cristal.jpg")
listaIconosReciclar = [amarillo,azul,verde]
posicionEntontrada = -1
for n in listaIconosReciclar:
    posicionEntontrada += 1

    #Apply template matching
    res = cv2.matchTemplate(n,foto,0)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    if min_loc != None:
        print("imagen encontrada en "+str(min_loc))

        texto = "aqui"
        posicion = min_loc
        fuente = cv2.FONT_HERSHEY_SIMPLEX
        color = (0,0,255)

        cv2.putText(foto,texto,posicion,fuente,1,color,2)

        #Definimos los parámetros que necesita la función rectangle()
        posicion_ini = max_loc
        posicion_fin = min_loc
        color = (0,0,255)

        cv2.rectangle(foto,posicion_ini,posicion_fin,color,4)

        cv2.imshow("My Image",foto)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        break
    
if posicionEntontrada == 0:       
    print("contenedor amarillo")
elif posicionEntontrada == 1:
    print("contenedor azul")
elif posicionEntontrada == 2:
    print("contenedor verde")
else:
    print("contenedor no encontrado")

cv2.imshow("My Image",foto)
cv2.waitKey(0)
cv2.destroyAllWindows()