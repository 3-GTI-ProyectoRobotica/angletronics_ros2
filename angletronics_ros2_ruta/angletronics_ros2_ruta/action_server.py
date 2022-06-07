# server_action.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from angletronics_ros2_custom_interface.srv import RutaReciclajeMsg
from angletronics_ros2_custom_interface.msg import RutaReciclajeParams
from geometry_msgs.msg import Twist
import time

class MyActionServer(Node):
    reciclaje_params = RutaReciclajeParams

    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('movement_server')
        timer_period = 2
        # crea un timer 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio
        self.srv = self.create_service(RutaReciclajeMsg, 'movement', self.my_first_service_callback)
        # declara el objeto publisher pasando como parametros
        # tipo de mensaje
        # nombre del topic
        # tamaño de la cola
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        #declaracion de parametros variables
        self.declare_parameter('contenedor', "desconocido")
        self.reciclaje_params.contenedor = self.get_parameter('contenedor').get_parameter_value().string_value

    def timer_callback(self):
        # cada timer_period segundos cargamos los parámetros en el mensaje
        self.get_logger().info('El contenedor tiene la direccion : %s' %self.reciclaje_params.contenedor)


    def my_first_service_callback(self, request, response):
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response

        # crea un mensaje tipo Twist 
        msg = Twist()

        if request.move == "derecha":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para hacer un giro a la derecha
            msg.linear.x = 0.1
            msg.angular.z = -0.5
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Girando hacia la derecha')
            # devuelve la respuesta
            response.success = True
        elif request.move == "izquierda":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para hacer un giro a la izquierda
            msg.linear.x = 0.1
            msg.angular.z = 0.5
            # publica el mensaje
            self.publisher.publish(msg)   
            # imprime mensaje informando del movimiento
            self.get_logger().info('Girando hacia la izquierda')
            # devuelve la respuesta
            response.success = True
        elif request.move == "delante":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para moverse hacia delante
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            # publica el mensaje
            self.publisher.publish(msg)   
            # imprime mensaje informando del movimiento
            self.get_logger().info('Hacia delante')
            # devuelve la respuesta
            response.success = True
        elif request.move == "atras":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para moverse hacia atras
            msg.linear.x = -0.1
            msg.angular.z = 0.0
            # publica el mensaje
            self.publisher.publish(msg)   
            # imprime mensaje informando del movimiento
            self.get_logger().info('Hacia atras')
            # devuelve la respuesta
            response.success = True
        elif request.move == "parar":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para parar el robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # publica el mensaje
            self.publisher.publish(msg)   
            # imprime mensaje informando del movimiento
            self.get_logger().info('Parando')
            # devuelve la respuesta
            response.success = True
        else:
            # estado de la respuesta
            # si no se ha dado ningun caso anterior
            response.success = False

        # devuelve la respuesta
        return response


def main(args=None):
    rclpy.init(args=args)

    my_action_server = MyActionServer()

    rclpy.spin(my_action_server)

if __name__=='__main__':
    main()
