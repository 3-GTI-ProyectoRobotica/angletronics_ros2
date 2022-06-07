"""
@autor: Juan Carlos Hernandez 
@Fecha: 06/2022
Fichero que controla el tratamiento y la clasificación de las imágenes
"""
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import cv2
from .insertar_registro_reciclaje import Insertar_registro
import os
from datetime import datetime
import time
from absl import app, flags, logging
from absl.flags import FLAGS
import numpy as np
import tensorflow as tf
from .models import (
    YoloV3, YoloV3Tiny
)
from .dataset import transform_images, load_tfrecord_dataset
from .utils import draw_outputs
from .client_ruta_robot import ClientAsync

def Tratamiento_de_imagen_yolo(ruta,nombre):

    current_path = "/home/juanc/turtlebot3_ws/src/angletronics_ros2/angletronics_ros2_yolo"
    classes_path = current_path+'/new_names.names' #path to classes file
    weights = current_path+'/angletronics_ros2_yolo/checkpoints/yolov3_train_22.tf' #path to weights file
    tiny = False #yolov3 or yolov3-tiny
    size = 516 #resize images to
    #image = ruta + "carton21.jpg"
    #image = ruta + "cristal44.jpg"
    image  = ruta+nombre
    tfrecord = None #tfrecord instead of image
    output = '/home/juanc/Escritorio/GIT/angletronics_web/src/public/assets/fotos_reciclaje/' #path to output image
    num_classes = 3 #number of classes in the model

    physical_devices = tf.config.experimental.list_physical_devices('GPU')
    
    if len(physical_devices) > 0:
        tf.config.experimental.set_memory_growth(physical_devices[0], True)

    if tiny:
        yolo = YoloV3Tiny(classes=num_classes)
    else:
        yolo = YoloV3(classes=num_classes)

    yolo.load_weights(weights).expect_partial()
    logging.info('weights loaded')

    class_names = [c.strip() for c in open(classes_path).readlines()]
    logging.info('classes loaded')

    if tfrecord:
        dataset = load_tfrecord_dataset(
            tfrecord, classes_path, size)
        dataset = dataset.shuffle(512)
        img_raw, _label = next(iter(dataset.take(1)))
    else:
        img_raw = tf.image.decode_image(
            open(image, 'rb').read(), channels=3)

    img = tf.expand_dims(img_raw, 0)
    img = transform_images(img, size)

    t1 = time.time()
    boxes, scores, classes, nums = yolo(img)
    t2 = time.time()
    logging.info('time: {}'.format(t2 - t1))

    logging.info('detections:')
    color=""
    print("Analizando...")
    for i in range(nums[0]):

        
        if(class_names[int(classes[0][i])] == 'botella plastico'):
            color = "amarillo"
        elif(class_names[int(classes[0][i])] == 'pote cristal'):
            color = "verde"
        elif(class_names[int(classes[0][i])] == 'rollo papel'):
            color="azul"
        logging.info('\t{}, {}, {}'.format(class_names[int(classes[0][i])],
                                           np.array(scores[0][i]),
                                           np.array(boxes[0][i])))
    
    if(len(color)>0):
        now = datetime.now()
        dt_string = now.strftime("%d_%m_%Y_%H:%M:%S")
        new_name = "test{}_{}.jpg".format(dt_string,color)

        ruta_abs = ruta+new_name
        id_usuario = 15
        #tratamiento de fecha
        fecha_mod = now.strftime("%Y-%m-%d %H:%M:%S")
        print("Se ha encontrado un objeto, contenedor:" + color)
        Insertar_registro(ruta_abs,color,id_usuario,fecha_mod)
        llamar_servicio_movimiento(color)
       
        img = cv2.cvtColor(img_raw.numpy(), cv2.COLOR_RGB2BGR)
        img = draw_outputs(img, (boxes, scores, classes, nums), class_names)
        cv2.imwrite(output+new_name, img)


def llamar_servicio_movimiento(color):
    #declara el constructor del objeto cliente
    client = ClientAsync()
    #ejecuta el metodo de peticion de servicio
    client.send_request(color)

    while rclpy.ok():
        #deja el nodo abierto hasta recibir ctrl+c
        rclpy.spin_once(client)
        #si se ha enviado el mensaje future
        if client.future.done():
            try:
                # chequea el mensaje future
                # si se ha enviado una respuesta 
                # la recoge
                response = client.future.result()
            except Exception as e:
                client.get_logger().info('La llamada al servicio ha fallado %r' % (e,))
        else:
            respuesta = response.success

            client.get_logger().info('Respuesta del servicio : {}'.format(respuesta))
        break
    client.destroy_node()
    
