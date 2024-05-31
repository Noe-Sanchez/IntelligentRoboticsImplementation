#!/usr/bin/env python3
import cv2
import numpy as np 
from scipy.spatial.transform import Rotation as R
import math 
import os
import rospy
from geometry_msgs.msg import Point, Twist
import time 


# Nombre del aruco utilizado
aruco_dictionary_name = "DICT_4X4_50"
 
# Diccionario con los diferentes arucos que se pueden utilizar 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
 
# Longitud del aruco en cm
aruco_marker_side_length = 6
 
# Función para convertir ángulos en cuaterniones a ángulos en Euler (Roll, Pitch, Yaw)
def euler_from_quaternion(x, y, z, w):

  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)
      
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)
      
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)
      
  return roll_x, pitch_y, yaw_z # en radians
 

def main():

  #Inicializar nodo y tópico Publisher
  pose_pub = rospy.Publisher("/aruco_pose", Twist, queue_size=0)
  rospy.init_node('aruco_pose_publisher', anonymous=False)


 
  # Checar que el Aruco es válido
  if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
    print("No es posible usar ArUco")
 
  # Cargar parámetros de la cámara obtenidos anteriormente
  root = os.getcwd()
  calibrationDir = os.path.join(root,'calibration.npz')
  calib_data = np.load(calibrationDir)
  camMatrix = calib_data['camMatrix']
  distCoeff = calib_data['distCoeff']
  tvecs = calib_data['tvecs']
  rvecs = calib_data['rvecs']
     
  # Cargar el diccionario de ArUco
  print("Detectando '{}' marcador...".format(aruco_dictionary_name))
  this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
  this_aruco_parameters = cv2.aruco.DetectorParameters()
  detector = cv2.aruco.ArucoDetector(this_aruco_dictionary,this_aruco_parameters)
   
  # Comenzar el video stream
  cap = cv2.VideoCapture(0)
  
  transform_translation_x_last = 0
  transform_translation_y_last = 0
  transform_translation_z_last = 0
  transform_translation_x_current = 0
  transform_translation_y_current = 0
  transform_translation_z_current = 0

  while(True):
  
    ret, frame = cap.read()  

    if not ret:
      break

    # Detecta marcadores de ArUco
    (corners, marker_ids, rejected) = detector.detectMarkers(frame)
       
    # Checa que al menos un marcador ha sido detectado
    if marker_ids is not None:
 
      # Dibuja un cuadrado alrededor de los marcadores detectados
      cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
       
      # Se obtiene el vector de translación y vector de rotación del ArUco con respecto a la cámara
      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        aruco_marker_side_length,
        camMatrix,
        distCoeff)
         
      # Se imprime la pose del ArUco, la cual es con respecto al marco de referencia de la cámara usada
      # Eje x es positivo hacia la derecha
      # Eje y es positivo hacia abajo
      # Eje z es positivo desde la cámara. Profundidad del objeto
      for i, marker_id in enumerate(marker_ids):
        
        # Desglosar vector de translación en sus tres componentes del primer marcador
        transform_translation_x = tvecs[0][0][0]
        transform_translation_y = tvecs[0][0][1]
        transform_translation_z = tvecs[0][0][2]

        # Actualizar pose actual
        transform_translation_x_current = transform_translation_x
        transform_translation_y_current = transform_translation_y
        transform_translation_z_current = transform_translation_z

        # Calcular velocidad
        vx = (transform_translation_x_current - transform_translation_x_last)/0.01
        vy = (transform_translation_y_current - transform_translation_y_last)/0.01
        vz = (transform_translation_z_current - transform_translation_z_last)/0.01

        #Construir y enviar mensaje
        msg = Twist()
        msg.linear.x = vy
        msg.linear.y = -vx
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        pose_pub.publish(msg)

        #Actualizar última posición translacional
        transform_translation_x_last = transform_translation_x_current
        transform_translation_y_last = transform_translation_y_current
        transform_translation_z_last = transform_translation_z_current


        # Guardar la matriz de rotación del primer marcador
        # rotation_matrix = np.eye(4)
        # rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[0][0]))[0]
        # r = R.from_matrix(rotation_matrix[0:3, 0:3])
        # quat = r.as_quat()   
         
        # Formato de cuaternion
        # transform_rotation_x = quat[0] 
        # transform_rotation_y = quat[1] 
        # transform_rotation_z = quat[2] 
        # transform_rotation_w = quat[3] 
         
        # Formato de ángulo de Euler
        # roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
        #                                                transform_rotation_y, 
        #                                                transform_rotation_z, 
        #                                                transform_rotation_w)


        # Actualizar ángulo a grados 
        # roll_x = math.degrees(roll_x)
        # pitch_y = math.degrees(pitch_y)
        # yaw_z = math.degrees(yaw_z)

        # Imprimir pose del ArUco
        print("transform_translation_x: {}".format(transform_translation_x))
        print("transform_translation_y: {}".format(transform_translation_y))
        print("transform_translation_z: {}".format(transform_translation_z))
        # print("roll_x: {}".format(roll_x))
        # # print("pitch_y: {}".format(pitch_y))
        # print("yaw_z: {}".format(yaw_z))
        print()
        
        # Dibuja ejes en el marcador
        cv2.drawFrameAxes(frame, camMatrix, distCoeff, rvecs[0], tvecs[0], aruco_marker_side_length/2, 2)
     
    # Observar resultados
    cv2.imshow('frame',frame)
    #Esperar 10 milisegundos. Si se presiona 1, el video se termina
    if cv2.waitKey(10) & 0xFF==ord('1'):
      break
  
  # Cerrar video stream
  cap.release()
  cv2.destroyAllWindows()



if __name__ == '__main__':
  main()