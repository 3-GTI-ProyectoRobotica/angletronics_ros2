import rclpy
# importamos las librerias ROS2 de python 
from rclpy.node import Node
# importamos los mensajes tipo PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
#importamos librerias para la calidad
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

# creamos una clase pasándole como parámetro el Nodo
class SimplePublisher(Node):

    def __init__(self):
        # Constructor de la clase
        # ejecutamos super() para inicializar el Nodo
        # introducimos le nombre del nodo como parámetro
        super().__init__('simple_publisher')
        

        # creamos el objeto publisher

        #definimos que tipo de calidad queremos
        #qos_profile = QoSProfile()
        
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        # que publicara en el topic /initialpose 
        # añadimos el perfil de calidad que hemos añadido
        #añadimos que sea de tipo PoseWithCovarianceStamped
        self.publisher_= self.create_publisher(PoseWithCovarianceStamped,'initialpose',qos_profile)
        
        # definimos un periodo para publicar periodicamente
        timer_period = 0.5
        # creamos un timer con dos parametros:
        # - el periodo (0.5 seconds)
        # - la funcion a realizar  (timer_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        # creamos el mensaje tipo Twist
        msg = PoseWithCovarianceStamped()
        # define la velocidad lineal en el eje x 
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 1.0
        msg.pose.pose.position.z = 1.0
        msg.pose.pose.orientation.w = 1.0
        msg.header.frame_id = "map"
        # Publicamos el mensaje en el topic
        self.publisher_.publish(msg)
        # Mostramos el mensaje por el terminal
        self.get_logger().info('Hola' + str(msg))
        
        
            
def main(args=None):
    # inicializa la comunicación
    rclpy.init(args=args)
    # declara el constructor del nodo 
    simple_publisher = SimplePublisher()
    # para parar el programa habrá que matar el node (ctrl+c)
    rclpy.spin(simple_publisher)
    # destruye en nodo
    simple_publisher.destroy_node()
    # se cierra la comunicacion ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()