#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import pigpio
import time
import os

#Falta fazer o PID para corrigir o erro entre o real e o aolvo


VEL_LIN_MAX = 0.2
PORCENTAGEM_DE_SUSTENTACAO = 60 #Porcentagem

KP = 15000.0  
MAX_OUTPUT = 100.0
MIN_OUTPUT = 0.0

os.system("sudo pigpiod")
time.sleep(1)
pi = pigpio.pi()



class PController:
    def __init__(self, Kp, max_out, min_out):
        self.Kp = Kp
        self.max_out = max_out
        self.min_out = min_out

    def calcular_saida(self, erro_atual):

        output = self.Kp * erro_atual
        output_limitado = max(min(output, self.max_out), self.min_out)
        
        return output_limitado


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
    
    def flight_up(self):
        self.definir_velocidade(PORCENTAGEM_DE_SUSTENTACAO)

    def set_thrust_from_p_controller(self, p_output):
        self.definir_velocidade(p_output)


class dado_vel:
    def __init__(self):
        self.linear = 0
        self.angular = 0
           
    def callback(self ,msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

def error_definition(erro_vel, target_vel, real_vel):
    erro_vel.linear = target_vel.linear - real_vel.linear
    erro_vel.angular = target_vel.angular - real_vel.angular 




if __name__ == '__main__':

    esc_propulsao = ESC(13)
    esc_susutentacao = ESC(12)

    real_vel = dado_vel()
    target_vel = dado_vel()
    erro_vel = dado_vel()

    propulsao_p_controller = PController(KP, MAX_OUTPUT, MIN_OUTPUT)
    
    rospy.init_node('Motor de propulsão')

    sub_pix_vel = rospy.Subscriber("/mavros/local_position/velocity", Twist, real_vel.callback)
    sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, target_vel.callback)

    #Armam os dois motores
    esc_propulsao.armar()
    esc_susutentacao.armar()

    #Enche o saco
    esc_susutentacao.flight_up()

    while not rospy.is_shutdown():
        
        error_definition(erro_vel, target_vel, real_vel)
        p_output_linear = propulsao_p_controller.calcular_saida(erro_vel.linear)
        esc_propulsao.set_thrust_from_p_controller(p_output_linear)
        rospy.sleep(0.05)


    esc_susutentacao.desligar_esc()
    esc_propulsao.desligar_esc()
    pi.stop()




