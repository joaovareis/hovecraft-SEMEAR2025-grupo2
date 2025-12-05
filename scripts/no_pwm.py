# sudo sshfs -o allow_other pi@192.168.0.5:/home/pi/ /mnt/mount/
import pigpio
import time
import os
# python3 ./no_pwm.py


def valor_numerico(x):
    try:
        x = int(x)
        return True
    except:
        return False


os.system("sudo pigpiod")
time.sleep(1)
pi = pigpio.pi()

# Definições iniciais para PWM
# PWM de hardware só é possivel nos pinos 12,14,19,18
frequencia = 50


class ESC:

    def __init__(self, pino_da_rasp):
        self.ligado = False
        self.calibrada = False
        self.pino_da_rasp = pino_da_rasp
        self.velocidade_atual = 0
        self.frequencia = 50

    def calibrar(self):
        print('iniciando calibração da esc')
        pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 100000)
        input('Digite enter se já tiver plugado a bateria')
        pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 50000)
        time.sleep(1)
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


# Código principal
ESC_propulsao = ESC(12)


ESC_propulsao.calibrar()

while True:

    velocidade_propulsao = input(
        'velocidade desejada para a ESC de propulsão: ')

    if valor_numerico(velocidade_propulsao):
        velocidade_propulsao = int(velocidade_propulsao)
        if velocidade_propulsao <= 100:
            ESC_propulsao.definir_velocidade(velocidade_propulsao)
    elif velocidade_propulsao == 'q':
        ESC_propulsao.desligar_esc()
        pi.stop()
        break


# velocidade_sustentacao = input(
# 'velocidade desejada para a ESC de sustentação: ')

# velocidade_sustentacao = int(velocidade_sustentacao)
# ESC_sustentacao.definir_velocidade(velocidade_sustentacao)

# input('calibrar 2')
# ESC_sustentacao.calibrar()

# ESC_sustentacao = ESC(18)