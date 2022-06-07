# Importar mensajes
from geometry_msgs.msg import Twist
from angletronics_ros2_custom_interface.srv import MyMoveMsg,CircleMoveMsg,RutaReciclajeMsg
from custom_interface.msg import CircleParams
#importar  biblioteca Python ROS2
import rclpy
from rclpy.node import Node
from .movement import NavToPose
from geometry_msgs.msg import PoseStamped 

class Service(Node):
    circle_params = CircleParams

    def __init__(self):
        #constructor con el nombre del nodo
        super().__init__('movement_server') 
        # declara el objeto servicio pasando como parametros
        # tipo de mensaje
        # nombre del servicio
        # callback del servicio

        self.srv = self.create_service(MyMoveMsg, 'movement', self.my_first_service_callback)
        self.srv2 = self.create_service(CircleMoveMsg, 'circle_movement', self.callback_circle_movement)
        self.srv3 = self.create_service(RutaReciclajeMsg, 'reciclaje_movement', self.callback_reciclaje)

        #declaracion de parametros variables
        self.declare_parameter('radio', 1.0)
        self.declare_parameter('velocidad', 0.22)
        self.declare_parameter('direccion', "izquierda")
        self.declare_parameter('contenedor', "desconocido")

        self.circle_params.radio = self.get_parameter('radio').get_parameter_value().double_value
        self.circle_params.velocidad = self.get_parameter('velocidad').get_parameter_value().double_value
        self.circle_params.direccion = self.get_parameter('direccion').get_parameter_value().string_value




        #declara el objeto publisher pasando como parametros
        # tipo de mensaje
        # nombre del topic
        # tamaño de la cola

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)


    def callback_reciclaje(self,request,response):
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response

        action_client = NavToPose()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = action_client.get_clock().now().to_msg()
        
    
        
        if request.contenedor == "verde":
            goal_pose.pose.position.x = -2.78
            goal_pose.pose.position.y = -1.07
            goal_pose.pose.orientation.w = 1.0
            
            response.success = True
        elif request.contenedor == "amarillo":
            goal_pose.pose.position.x = -2.50
            goal_pose.pose.position.y = -1.06
            goal_pose.pose.orientation.w = 1.0

            response.success = True


        elif request.contenedor == "azul":
            goal_pose.pose.position.x = -3.80
            goal_pose.pose.position.y = -1.06
            goal_pose.pose.orientation.w = 1.0
            
            response.success = True
        
        if(response.success):
            self.get_logger().info('Desplazando a contenedor:'+request.contenedor)
            future = action_client.send_goal(goal_pose) # se para secs como argumento
            rclpy.spin(action_client)
        
        return response

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

    def calcular_velocidad_angular(velocidad, radio):
        return velocidad/radio
    
    def callback_circle_movement(self, request, response):
        # recibe los parametros de esta clase
        #  recibe el mensaje request
        # devuelve el mensaje response

        # crea un mensaje tipo Twist
        msg = Twist()
        if request.direccion == "girar":
            v_angular = self.calcular_velocidad_angular(self.circle_params.velocidad, self.circle_params.radio)
            if self.circle_params.direccion == "derecha":
                v_angular = -v_angular
            
            msg.linear.x = self.circle_params.velocidad
            msg.angular.z = v_angular
            # publica el mensaje
            self.publisher.publish(msg)
            # imprime mensaje informando del movimiento
            self.get_logger().info('Girando...')
            # devuelve la respuesta
            response.success = True
        elif request.direccion == "parar":
            # rellena el mensaje msg con la velocidad angular y lineal
            # necesaria para parar el robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # publica el mensaje
            self.publisher.publish(msg)   
            # imprime mensaje informando del movimiento
            self.get_logger().info('Parando...')
            # devuelve la respuesta
            response.success = True
        
        return response

def main(args=None):
    # inicializa la comunicacion ROS2
    rclpy.init(args=args)
    # creamos el nodo
    service = Service()
    try:
        #dejamos abierto el servicio
        rclpy.spin(service)
    except KeyboardInterrupt:
        service.get_logger().info('Cerrando el nodo service')
    finally:
        #destruimos el nodo
        service.destroy_node()
        #cerramos la comunicacion
        rclpy.shutdown()



#definimos el ejecutable
if __name__=='__main__':
    main()