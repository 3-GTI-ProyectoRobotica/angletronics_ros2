import rclpy
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from datetime import datetime
import insertar_registro_reciclaje


class Ros2OpenCVImageConverter(Node):   

    def __init__(self):

        super().__init__('Ros2OpenCVImageConverter')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/image',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
    def camera_callback(self,data):

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        tratamiento_de_imagen(cv_image)
      

def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = Ros2OpenCVImageConverter()
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")
    
    cv2.destroyAllWindows() 
    


def tratamiento_de_imagen(foto):
    print("Dimensiones de la imagen:{}x{}".format(foto.shape[0],foto.shape[1] ))
    now = datetime.now()
    dt_string = now.strftime("%d_%m_%Y_%H:%M:%S")

    nombre = "src/angletronics_ros2/angletronics_ros2_capture_image/angletronics_ros2_capture_image/plantillas_reciclaje/"
    icono = cv2.imread(nombre+"verde.jpg")
    #foto = cv2.imread("plantillas_reciclaje/test_3.jpg")

    res = cv2.matchTemplate(icono,foto,0)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    
    #detectamos si existe el icono de reciclaje
    if min_loc[0]>100 and min_loc[1]>100:
        print("Se ha encontrado un icono de reciclaje! Procesando....")
        hsv = cv2.cvtColor(foto,cv2.COLOR_BGR2HSV)

        #elegimos que mascara utilizar
        #primero probamos con la azul
        mask_azul = cv2.inRange(hsv, (100, 150, 0), (140, 255,255))
        res = cv2.bitwise_and(foto, foto, mask= mask_azul)
        todo_negro = True
        color =""
        #detectamos de que contenedor se trata
        for i in range(res.shape[0]):
            for j in range(res.shape[1]):
                pixel = res[i,j,:]
                if pixel[0] > 0 or pixel[1] > 0 or pixel[2] > 0:
                    todo_negro = False
                    color = "azul"

        if todo_negro:
            mask_verde = cv2.inRange(hsv, (36, 100, 100), (100, 255,255))
            res = cv2.bitwise_and(foto, foto, mask= mask_verde)
            for i in range(res.shape[0]):
                for j in range(res.shape[1]):
                    pixel = res[i,j,:]
                    if pixel[0] > 0 or pixel[1] > 0 or pixel[2] > 0:
                        todo_negro = False
                        color = "verde"

        if todo_negro:
            mask_amarillo = cv2.inRange(hsv, (15,100,100), (36, 255,255))
            res = cv2.bitwise_and(foto, foto, mask= mask_amarillo)
            for i in range(res.shape[0]):
                for j in range(res.shape[1]):
                    pixel = res[i,j,:]
                    if pixel[0] > 0 or pixel[1] > 0 or pixel[2] > 0:
                        todo_negro = False
                        color = "amarillo"

        if(color!=""):
            #pintamos el rectangulo
            minima_x = 1000000000000000
            maxima_x = 0
            minima_y = 1000000000000000
            maxima_y = 0

            for i in range(res.shape[0]):
                for j in range(res.shape[1]):
                    pixel = res[i,j,:]
                    
                    if pixel[0] > 0:
                        if pixel[1] > 0:
                            if pixel[2] > 0:
                                #print(str(i)+","+str(j))
                                if i > maxima_x:
                                    maxima_x = i
                                if i < minima_x:
                                    minima_x = i
                                if j > maxima_y:
                                    maxima_y = j
                                if j < minima_y:
                                    minima_y = j  

            ##DIbujamos el rectangulo que englobe los pixeles
            #Obtenemos los pixeles con el visualizador de imagenes 
            #Esquina superior izquierda (55,139)
            #Esquina inferior derecha (289,494)
            color_cuadro = (0,0,255)

            x1 = minima_y
            y1 = maxima_x
            x2 = maxima_y
            y2 = minima_x

            image_rec = cv2.rectangle(foto, (x1,y1), (x2,y2), color_cuadro, 2)

            nombre= "../Escritorio/GIT/angletronics_web/src/public/assets/fotos_reciclaje/test{}_{}.jpg".format(dt_string,color)
            #nombre = "../Escritorio/GIT/angletronics_web/src/public/assets/fotos_reciclaje/test"+dt_string+"_"+color+".jpg"
            cv2.imwrite(nombre,image_rec)

            #Hacer INSERT EN LA BASE DE DATOS
            #INSERT INTO `registros_reciclaje`(`imagen`,`clasificacion`,`id_usuario`,`fecha`) VALUES ('ruta','vidrio',15,'2020-09-16 15:14:24')
            ruta_abs = os.path.abspath(nombre)
            if color == "azul":
                clasificacion = "Carton"
            elif color == "amarillo":
                clasificacion = "Plastico"
            else:
                clasificacion = "Vidrio"
            #id usuario
            id_usuario = -1
            #tratamiento de fecha
            nueva = dt_string.replace('_','-',2)
            fecha_mod = nueva.replace('_',' ',1)
            insertar_registro_reciclaje.insertar_registro(ruta_abs,clasificacion,id_usuario,fecha_mod)

            #Mostramos una imagen
            cv2.imshow("Image window",image_rec) #el primer parámetro es el nombre de la ventana, que puedo poner el que quiera
            cv2.waitKey(1) #aprieta una tecla
    else:
        print("No se ha encontrado nigún icono de reciclaje") 

if __name__ == '__main__':
    main()
