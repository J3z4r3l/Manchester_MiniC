#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.parametros = cv2.aruco.DetectorParameters_create()
        self.diccionario = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        
        # Parámetros de la cámara simulados (para pruebas)
        self.camera_matrix = np.array([[1000.0, 0.0, 630.0],
                                       [0.0, 1000.0, 630.0],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.array([0.1, -0.25, 0.0, 0.0, 0.0], dtype=np.float32)

        # Tamaño del marcador ArUco en metros
        self.marker_size = 0.13  # Por ejemplo, 5 cm
        
        rospy.init_node('aruco_detector', anonymous=True)
        self.image_sub = rospy.Subscriber('CAMBIAR POR EL TOPICO DE LA CAMARA', Image, self.image_callback)
        
        rospy.spin()

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        esquinas, ids, _ = cv2.aruco.detectMarkers(gray, self.diccionario, parameters=self.parametros)
        
        if ids is not None:
            frame = cv2.aruco.drawDetectedMarkers(frame, esquinas)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(esquinas, self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                aruco_id = ids[i][0]
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Calcular la distancia y el ángulo
                distance = np.linalg.norm(tvec)
                angle = np.arctan2(tvec[0], tvec[2])
                angle_degrees = np.degrees(angle)

                rospy.loginfo(f"Aruco ID: {aruco_id}")
                rospy.loginfo(f"Distancia: {distance:.2f} m")
                rospy.loginfo(f"Ángulo: {angle_degrees:.2f} grados")

                # Dibujar el eje del marcador
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        else:
            rospy.loginfo("No se detectaron Arucos")
        
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        ArucoDetector()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
