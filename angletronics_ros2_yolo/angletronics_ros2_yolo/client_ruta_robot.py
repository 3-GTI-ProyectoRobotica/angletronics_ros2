"""
@autor: Alberto Valls y Juan Carlos Hernández
@Fecha: 06/2022
Fichero que controla la llamada al servicio de movimiento del robot
"""
#importamos el mensaje
from angletronics_ros2_custom_interface.srv import RutaReciclajeMsg
#importamos la bib ROS2 para python
import rclpy
from rclpy.node import Node
#importamos la bib sys para poder usar los arg de entrada
import sys

#definimos la clase cliente
class ClientAsync(Node):

    def __init__(self):
        #inicializa el nodo cliente
        super().__init__('movement_client')
        #crea el objeto cliente
        self.client = self.create_client(RutaReciclajeMsg, 'movement')
        #cada segundo revisa si el servicio esta activo
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('el servicio no esta activo, prueba de nuevo...')
        
        #crea el mensaje 
        self.req = RutaReciclajeMsg.Request()

    def send_request(self, parametro):
        # usa sys.argv para tener acceso a los argumentos introducidos en la
        # llamada al programa por consola
        self.req.contenedor = parametro
        #envia la peticion del servicio
        self.future = self.client.call_async(self.req)