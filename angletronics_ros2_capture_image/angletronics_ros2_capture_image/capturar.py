import rclpy
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from datetime import datetime


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
        

        print("Dimensiones de la imagen:{}x{}".format(cv_image.shape[0],cv_image.shape[1] ))
        #cv2.imshow("Image window", cv_image)
        #guardamos la imagen
        #------------------------Aplicar tratmiento de imagenes----------------
        now = datetime.now()
        dt_string = now.strftime("%d_%m_%Y_%H:%M:%S")
        nombre = "../Escritorio/GIT/angletronics_web/src/public/assets/fotos_reciclaje/test"+dt_string+".jpg"
        #nombre = "test"+dt_string+".jpg"
        cv2.imwrite(nombre,cv_image)
        #Mostramos una imagen
        cv2.imshow("Image window",cv_image) #el primer parámetro es el nombre de la ventana, que puedo poner el que quiera
        cv2.waitKey(1) #aprieta una tecla 

def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = Ros2OpenCVImageConverter()
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")
    
    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()
