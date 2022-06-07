"""
@autor: Juan Carlos Hernandez 
@Fecha: 06/2022
Fichero que controla la lógica sobre la captura de imagenes del robot 
"""
import rclpy
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from datetime import datetime

from .predict import Tratamiento_de_imagen_yolo

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
    ruta = "/home/juanc/turtlebot3_ws/src/angletronics_ros2/angletronics_ros2_yolo/img_robot/"
    nombre = "test{}.jpg".format(dt_string)
    cv2.imwrite(ruta+nombre,foto)
    Tratamiento_de_imagen_yolo(ruta,nombre)

if __name__ == '__main__':
    main()
