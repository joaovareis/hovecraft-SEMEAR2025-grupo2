#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

ALTURA_OBJETO = 14.2
DIST_FOCAL = 813.444712
iContador = 0

mtx = np.array([[813.556992, 0.000000, 328.327845],
[0.000000, 813.444712, 225.690383],
[0.000000, 0.000000, 1.000000]]
)

dist = np.array([0.073695, 0.053370, -0.000830, -0.000563, 0.000000])

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:

    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)


    hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (83, 141, 88), (146, 255, 255))#Ajustar para a cor que for selecionada. Penso em roxo ou laranja
    kernel = np.ones((5, 5), np.uint8)
    img_erosao = cv2.erode(mask,kernel)
    img_dilatada = cv2.dilate(img_erosao, kernel)
                
    # monta os contornos da imagem binária(pós filtro)
    contornos, _ = cv2.findContours(img_dilatada, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    num_pixel_tela = cv2.countNonZero(img_dilatada)

    if contornos:
        maior_contorno = max(contornos, key=cv2.contourArea)
        _ , _ , largura_pixels_obj , altura_pixels_obj = cv2.boundingRect(maior_contorno) #Calcula a atura e a largura do maior retangulo que cabe dentro do objeto filtrado
        M = cv2.moments(maior_contorno)
        #esse moments é mt foda, basicamente um dicionário de várias informações úteis dos contornos

        if num_pixel_tela > 4:
            iContador+=1
            centro_x_objeto = int(M["m10"] / M["m00"])
            centro_y_objeto = int(M["m01"] / M["m00"])
            area_objeto = int(M["m00"])
            #o centroide é calculado com uma média ponderada que basicamente vira 1/2 ou seja 0.5
            #esse m00 é a área calculada mt eficientemente pela própria função moments

            dist_objeto = (ALTURA_OBJETO * DIST_FOCAL)/ altura_pixels_obj

            if(iContador == 10):
                print((f"Distancia ate o objeto {dist_objeto} centimetros")) 
                iContador = 0


    cv2.imshow('Video', img_dilatada)
    if cv2.waitKey(33) & 0xFF == ord('q'):
        break


rospy.init_node('teste')

bridge = CvBridge()
rospy.spin()

cv2.destroyAllWindows()
cap.release()