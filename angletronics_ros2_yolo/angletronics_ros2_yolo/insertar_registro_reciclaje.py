"""
@autor: Sergi Sirvent Sempere
@Fecha: 06/2022
Fichero que controla los inserts a la bbdd sobre los registros de reciclaje
"""
import mysql.connector

mydb = mysql.connector.connect(
  host="localhost",
  user="root",
  passwd="",
  database="bd_angle_tronics"
)

mycursor = mydb.cursor()

def Insertar_registro(ruta_imagen,clasificacion,id_usuario,fecha):
  
  sql = "INSERT INTO registros_reciclaje (imagen, clasificacion, id_usuario, fecha) VALUES (%s, %s, %s, %s)"
  val = (ruta_imagen, clasificacion, id_usuario, fecha)
  mycursor.execute(sql, val)

  mydb.commit()

  print(mycursor.rowcount, "registro insertado")