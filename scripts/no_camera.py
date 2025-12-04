#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
from collections import deque

''' Constantes '''
ALTURA_OBJETO = 50 #colocar um valor
LARGURA_OBJETO = 50
DIST_FOCAL = 813.444712

kernel = np.ones((5, 5), np.uint8)


''' Codigo '''
def hsv(input):

    hsv = cv2.cvtColor(input, cv2.COLOR_BGR2HSV) #converte de BGR para HS

    #mask = cv2.inRange(hsv, (84, 130, 88), (123, 221, 255))#Apagador de quadro
    mask = cv2.inRange(hsv, (90, 50, 50), (130, 255, 255))#Simulação
    #mascara a imagem

    return(mask)

class robo_seguidor:

    def __init__(self):

        rospy.init_node('camera', anonymous=True) 
        #anonymous faz com que o nó tenha nome único

        self.buffer = deque(maxlen = 10)# Tem que ver qual o tamnho ideal
        #cria o buffer que recebe ass imagens da função callback

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imagem_callback)
        #pega a imagem do topico do ros

        self.controle_pub = rospy.Publisher('/info_objeto', Float32MultiArray, queue_size=5)
        #publica no topico de controle

        self.bridge = CvBridge()

        self.imagem_lock = threading.Lock() 
        # lock para evitar conflitos de thread. Acho que isso da certo pq meu computador n tava tankando single threading pra exibir as coisas do gazebo (?)

    def imagem_callback(self, ros_camera):

        self.buffer.append(self.bridge.imgmsg_to_cv2(ros_camera, "bgr8")) 
    
if __name__ == '__main__':
    robo = robo_seguidor()
    taxa = rospy.Rate(30)
    #esse rate vai cair dependendo do fps da camera
    while not rospy.is_shutdown():
        # processa eventos do OpenCV no loop principal, pq aparentemente se for fora isso ferra com o gazebo sem motivo algum
        cv2.waitKey(1)

        imagem_para_processar = None
        #Quando o buffer estava vazio o codigo quebrava, pois nada era passado para imagem_para_processar, por isso precisei atribuir um valor inicial á essa variavel
        
        with robo.imagem_lock:
            if robo.buffer:
                imagem_para_processar = robo.buffer.popleft()
        #Acessa a imagem mais antiga do buffer para processar

        if imagem_para_processar is not None:
            # aplica os filtros e exibe as janelas aqui. Sem esse is not none simplesmente nao abre a janela do opencv mt foda
            
            img_hsv = hsv(imagem_para_processar)
            img_erodida = cv2.erode(img_hsv, kernel)
            img_dilatada = cv2.dilate(img_erodida, kernel)

            cv2.imshow("camera", imagem_para_processar)
            cv2.imshow("filtro", img_dilatada)
            
            #pega a altura e a largura da imagem original
            altura_imagem, largura_imagem, _ = imagem_para_processar.shape
            
            # monta os contornos da imagem binária(pós filtro)
            contornos, _ = cv2.findContours(img_dilatada, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0, float(largura_imagem), float(altura_imagem)]
            #cria a msg

            num_pixel_tela = cv2.countNonZero(img_dilatada)
            
            if contornos:
                maior_contorno = max(contornos, key=cv2.contourArea)
                _ , _ , largura_pixels_obj , altura_pixels_obj = cv2.boundingRect(maior_contorno) #Calcula a atura e a largura do maior retangulo que cabe dentro do objeto filtrado
                M = cv2.moments(maior_contorno)
                #esse moments é mt foda, basicamente um dicionário de várias informações úteis dos contornos

                if num_pixel_tela > 20:
                    centro_x_objeto = int(M["m10"] / M["m00"])
                    centro_y_objeto = int(M["m01"] / M["m00"])
                    area_objeto = int(M["m00"])
                    #o centroide é calculado com uma média ponderada que basicamente vira 1/2 ou seja 0.5
                    #esse m00 é a área calculada mt eficientemente pela própria função moments

                    dist_objeto_alt = (ALTURA_OBJETO * DIST_FOCAL)/ altura_pixels_obj
                    dist_objeto_lar = (LARGURA_OBJETO * DIST_FOCAL)/ largura_pixels_obj

                    dist_objeto = (dist_objeto_lar + dist_objeto_alt)/2
                    
                    msg.data[0] = float(centro_x_objeto)
                    msg.data[1] = float(dist_objeto)        #float(centro_y_objeto)
                    msg.data[2] = float(area_objeto)

            robo.controle_pub.publish(msg)

        taxa.sleep()
        
    cv2.destroyAllWindows()