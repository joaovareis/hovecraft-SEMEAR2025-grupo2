import pigpio
import time

pi = pigpio.pi()


# Definições iniciais para PWM
# PWM de hardware só é possivel nos pinos 12,13,14,19
frequencia = 50


class ESC:

    def __init__(self, pino_da_rasp, velocidade_atual=0, ligado=False, calibrada=False):
        self.ligado = ligado
        self.calibrada = calibrada
        self.pino_da_rasp = pino_da_rasp

    def calibrar(self):
        print('iniciando calibração da esc')
        pi.hardware_PWM(self.pino_da_rasp, frequencia, 50000)
        time.sleep(1)
        pi.hardware_PWM(self.pino_da_rasp, frequencia, 100000)
        time.sleep(1)
        pi.hardware_PWM(self.pino_da_rasp, frequencia, 0)
        self.calibrada = True
        print('calibração terminada')

    def definir_velocidade(self, velocidade_porcentagem):
        if self.calibrada == True:
            valor_minimo_duty_cycle = 50000
            degrau_porcentagem = 500
            duty_cycle = (degrau_porcentagem *
                          velocidade_porcentagem) + valor_minimo_duty_cycle
            pi.hardware_PWM(self.pino_da_rasp, frequencia, duty_cycle)

            self.velocidade_atual = velocidade_porcentagem
            self.ligado = True

    def desligar_esc(self, ligado, pino_da_rasp):
        pi.hardware_PWM(pino_da_rasp, frequencia, 0)
        pi.stop()
        self.ligado = False


# Código principal
ESC_propulsao = ESC(12)
ESC_sustentacao = ESC(13)

ESC_propulsao.calibrar()
ESC_sustentacao.calibrar()

while True:
    velocidade_propulsao = int(
        input('velocidade desejada para a ESC de propulsão: '))
    velocidade_sustentacao = int(
        input('velocidade desejada para a ESC de sustentação: '))

    ESC_propulsao.calibrar()
    ESC_sustentacao.calibrar()

    ESC_propulsao.definir_velocidade(velocidade_propulsao)
    ESC_sustentacao.definir_velocidade(velocidade_sustentacao)

    ESC_propulsao.desligar_esc()
    ESC_sustentacao.desligar_esc()


'''#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float64

PINO_ESC = 2 #numero do pino da ESC, Não sei qual é
FREQ_PWM = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(PINO_ESC, GPIO.OUT)

pwm_esc = GPIO.PWM(PINO_ESC, FREQ_PWM)
pwm_esc.start(0)


def pwm_callback(percentage_vel_motor):
    dc_min = 0.05 #motor desligado
    dc_max = 0.1 #motor a 100%

    dc = (dc_min + ((dc_max - dc_min) * percentage_vel_motor.data)) * 100

    pwm_esc.ChangeDutyCycle(dc)


def desligar_motor():
    print("\nDesligando o motor e limpando os pinos GPIO.")
    pwm_esc.stop()
    GPIO.cleanup()

sub_motor_vel = rospy.Subscriber("/control_node", Float64,callback = pwm_callback)

rospy.on_shutdown(desligar_motor)

rospy.spin()
'''
