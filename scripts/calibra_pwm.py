#!/usr/bin/env python3
import pigpio
import time
import os

os.system("sudo pigpiod")
time.sleep(1)
pi = pigpio.pi()

frequencia = 50
pino_da_rasp = 12

print('iniciando calibração da esc')
pi.hardware_PWM(pino_da_rasp, frequencia, 100000)
input('Digite enter se já tiver plugado a bateria')
pi.hardware_PWM(pino_da_rasp, frequencia, 50000)
time.sleep(1)
pi.hardware_PWM(pino_da_rasp, frequencia, 0)
print('calibração terminada')

