#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import pigpio
import time
import os

erro_angular = 0
erro_linear = 0

VEL_LIN_MAX = 0.2

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

    def calibrar(self):
        pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 50000)
        time.sleep(5)
        pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 0)
        self.calibrada = True
        print('calibração terminada')

    def definir_velocidade(self, velocidade_porcentagem):
        if self.calibrada == True:
            valor_minimo_duty_cycle = 50000
            degrau_porcentagem = 500
            duty_cycle = (degrau_porcentagem *
                          velocidade_porcentagem) + valor_minimo_duty_cycle
            pi.hardware_PWM(self.pino_da_rasp, self.frequencia, duty_cycle)

            self.velocidade_atual = velocidade_porcentagem
            self.ligado = True

    def desligar_esc(self):
        pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 0)
        self.ligado = False

    def flight_foward(self):

        velocidade_porcentagem = (target_vel.linear * 100)/VEL_LIN_MAX
        self.definir_velocidade(velocidade_porcentagem)


class dado_vel:
    def __init__(self):
        self.linear = 0
        self.angular = 0
           
def imu_callback(msg):
    real_vel.linear = msg.linear.x
    real_vel.angular = msg.angular.z

def cmd_vel_callback(msg):
    target_vel.linear = msg.linear.x
    target_vel.angular = msg.angular.z

def error_definition(self):
    erro_linear = target_vel.linear - real_vel.linear
    erro_angular = target_vel.angular - real_vel.angular 


if __name__ == '__main__':

    esc_propulsao = ESC(18)
    esc_propulsao.calibrar()



    real_vel = dado_vel()
    target_vel = dado_vel()


    rospy.init_node('Motor de propulsão')

    sub_pix_vel = rospy.Subscriber("/mavros/local_position/velocity", Twist, imu_callback)
    sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)



