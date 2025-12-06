import rospy 
from std_msgs.msg import Int16
import pigpio
import os 
import time

class servo:
    def __init__ (self, pi_instance, pino, pwm_max, pwm_min):
        self.pino = pino
        self.max = pwm_max
        self.min = pwm_min
        self.pi = pi_instance

    def desligar(self):
        self.pi.set_servo_pulsewidth(self.pino, 0)

    def calibrar(self):
        try:
            #Gira para o máximo
            self.pi.set_servo_pulsewidth(self.pino, self.max)
            rospy.loginfo("Servo girado para o máximo.")
            rospy.sleep(2)

            #Gira para o meio
            self.pi.set_servo_pulsewidth(self.pino, (self.max + self.min)/2)
            rospy.loginfo("Servo girado para o meio.")
            rospy.sleep(2)

            #Gira para o mínimo
            self.pi.set_servo_pulsewidth(self.pino, self.min)
            rospy.loginfo("Servo girado para o mínimo.")
            rospy.sleep(2)
    
        except rospy.ROSInterruptException:
            rospy.loginfo("Calibração interrompida pelo usuário.")

        finally:
            self.desligar()
        
    def mover_para_angulo(self, angulo):
        if angulo < 0:
            angulo = 0
        elif angulo > 180:
            angulo = 180

        pulso = self.min + (self.max - self.min)*(angulo/180)

        self.pi.set_servo_pulsewidth(self.pino, int(pulso))

    def callback_angulo(self, msg):
        angulo = msg.data
        self.mover_para_angulo(angulo)  

class ESC:
    def __init__(self, pino_rasp, pi_instance):
        self.pino_da_rasp = pino_rasp
        self.frequencia = 50
        self.pi = pi_instance
        self.ligado = False
        self.calibrada = False

    def calibrar_esc (self):
        print('iniciando calibração da esc')
        self.pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 100000)
        input('Digite enter se já tiver plugado a bateria')
        self.pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 50000)
        time.sleep(1)
        self.pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 0)
        self.calibrada = True
        print('calibração terminada')

    def definir_velocidade(self, velocidade_porcentagem):
        if self.calibrada == True:
            valor_minimo_duty_cycle = 50000
            degrau_porcentagem = 500
            duty_cycle = (degrau_porcentagem * velocidade_porcentagem) + valor_minimo_duty_cycle
            self.pi.hardware_PWM(self.pino_da_rasp, self.frequencia, duty_cycle)

            self.velocidade_atual = velocidade_porcentagem
            self.ligado = True

        else:
            print("ESC não calibrada. Por favor, calibre antes de definir a velocidade.")
    
    def desligar_esc(self):
        self.pi.hardware_PWM(self.pino_da_rasp, self.frequencia, 0)
        self.ligado = False

    def callback_definir_velocidade(self, msg):
        velocidade = msg.data
        self.definir_velocidade(velocidade)
    
class node:
    def __init__ (self):
        rospy.init_node('no_pwm', anonymous=True)
        os.system('sudo pigpiod')
        time.sleep(1)
        self.pi = pigpio.pi()
        self.servo_motor = servo(self.pi, 18, 2000, 800)
        self.esc_propulsao = ESC(12, self.pi)
        self.esc_sustentacao = ESC(13, self.pi)

        

    def calibrar (self):
        self.servo_motor.calibrar()
        self.esc_propulsao.calibrar_esc()
        self.esc_sustentacao.calibrar_esc()

    def cleanup(self):
        rospy.loginfo("Encerrando dispositivos...")
        self.servo_motor.desligar()
        self.esc_propulsao.desligar_esc()  
        self.esc_sustentacao.desligar_esc()
        self.pi.stop()

if __name__ == '__main__':
    try:
        pwm_node = node()
        pwm_node.calibrar()

        rospy.Subscriber('/servo_angulo', Int16, pwm_node.servo_motor.callback_angulo)
        rospy.Subscriber('/esc_propulsao_velocidade', Int16, pwm_node.esc_propulsao.callback_definir_velocidade)
        rospy.Subscriber('/esc_sustentacao_velocidade', Int16, pwm_node.esc_sustentacao.callback_definir_velocidade)

        rospy.loginfo("Nó PWM inicializado. Aguardando mensagens...")
        rospy.spin()

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("Nó encerrado.")
    
    finally:
        pwm_node.cleanup()
