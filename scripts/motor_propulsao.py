#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import pigpio
import time
import os

#Falta fazer o PID para corrigir o erro entre o real e o aolvo


VEL_LIN_MAX = 0.2
PORCENTAGEM_DE_SUSTENTACAO = 60 #Porcentagem

os.system("sudo pigpiod")
time.sleep(1)
pi = pigpio.pi()


class ESC:

    def __init__(self, pino_da_rasp):
        self.ligado = False
        self.calibrada = False
        self.pino_da_rasp = pino_da_rasp
        self.velocidade_atual = 0
        self.frequencia = 50

    def definir_velocidade(self, velocidade_porcentagem):
        if self.calibrada == True:
            valor_minimo_duty_cycle = 50000
            degrau_porcentagem = 500
            duty_cycle = (degrau_porcentagem * velocidade_porcentagem) + valor_minimo_duty_cycle
            
            pi.hardware_PWM(self.pino_da_rasp, self.frequencia, duty_cycle)

            self.velocidade_atual = velocidade_porcentagem
            self.ligado = True
    
    def armar(self):
        pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 50000)
        time.sleep(3)

    def desligar_esc(self):
        pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 0)
        self.ligado = False

    def flight_forward(self):
        velocidade_porcentagem = (target_vel.linear * 100)/VEL_LIN_MAX
        self.definir_velocidade(velocidade_porcentagem)
    
    def flight_up(self):
        self.definir_velocidade(PORCENTAGEM_DE_SUSTENTACAO)


class dado_vel:
    def __init__(self):
        self.linear = 0
        self.angular = 0
           
    def callback(self ,msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

def error_definition(self):
    erro_vel.linear = target_vel.linear - real_vel.linear
    erro_vel.angular = target_vel.angular - real_vel.angular 


if __name__ == '__main__':

    esc_propulsao = ESC(13)
    esc_susutentacao = ESC(12)

    real_vel = dado_vel()
    target_vel = dado_vel()
    erro_vel = dado_vel()
    
    rospy.init_node('Motor de propulsão')

    sub_pix_vel = rospy.Subscriber("/mavros/local_position/velocity", Twist, real_vel.callback)
    sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, target_vel.callback)

    #Armam os dois motores
    esc_propulsao.armar()
    esc_susutentacao.armar()

    #Enche o saco
    esc_propulsao.flight_up()

    while not rospy.is_shutdown():
        esc_propulsao.flight_forward()
        rospy.sleep(0.05)



    esc_propulsao.desligar_esc()
    pi.stop()




