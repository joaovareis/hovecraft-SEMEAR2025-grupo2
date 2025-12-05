#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from enum import Enum
import threading

#recebe da camera cx[0], dist-obj[1], area_obj[2], larg_obj[3], altura_obj[4], altura_cam[5], 
#variaveis globais PID e controle a serem definidas pelo usuario

ganho_proporcional = 0.01
ganho_diferencial = 0.001
ganho_integral = 0.0
offset = 75


class maquina_de_estados(Enum):
    procurando = 0
    seguindo = 1

class FSM_robo_seguidor:

    def __init__(self):

        rospy.init_node('controle', anonymous=True)
        rospy.Subscriber('/info_objeto',Float32MultiArray, self.callback_info_objeto)
        self.comando = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #inicia nó controle, subscreve a ele pra receber info da camera; publica em cmd_vel (no futuro em PWM)

        self.dados_do_alvo = None
        self.dados_do_alvo_lock = threading.Lock()
        #isso impede que os dados sejam lidos quando não existem e enquanto são modificados, pra evitar dar bugs

        self.area_total = 0.0
        self.area_minima = 0.0
        self.area_maxima = 0.0
        #cria variável que identifica as áreas totais do vídeo e a area mínima e max que o hover utiliza pra alterar estados

        self.erro_anterior = 0.0
        self.erro_integral = 0.0
        self.tempo_anterior_pid = rospy.Time.now()
        #variáveis utilizadas no PID

        self.estado = maquina_de_estados.procurando
        self.visto_por_ultimo = rospy.Time.now()
        self.tempo_minimo_sem_objeto = 1.0  # segundos

        self.fsm = {
            maquina_de_estados.procurando: (self.procura,  self.transicao_procura),
            maquina_de_estados.seguindo:   (self.segue,    self.transicao_seguindo)
        }
        #estrutura da maquina de estados

    def PID(self, erro_em_x):

        agora = rospy.Time.now()
        dt = (agora - self.tempo_anterior_pid).to_sec()
        self.tempo_anterior_pid = agora
        #calcula o dt utilizado. Acho que na versão final vale a pena estipular o rate pra ter um valor fixo ao invés de calcular a cada ciclo (mais performance)

        erro_proporcional = ganho_proporcional*erro_em_x

        self.erro_integral += erro_em_x*dt
        erro_integral_limitado = max(min(self.erro_integral, 100), -100)
        #somatorio da integral

        if dt == 0:
            erro_diferencial = 0.0
        else:
            erro_diferencial = ganho_diferencial*((erro_em_x - self.erro_anterior)/dt)
        #calcula o coef angular da reta erro por tempo. Se o codigo for mt rapido pode acontecer de dt ser 0

        saida_PID = erro_proporcional + ganho_integral*erro_integral_limitado + erro_diferencial
        self.erro_anterior = erro_em_x
        return(saida_PID)

    #======================================
    #funcao para receber os dados da camera
    #======================================

    def callback_info_objeto(self, msg):
        
        if len(msg.data) < 5:
            rospy.logwarn("Mensagem de câmera incompleta.")
            return

        with self.dados_do_alvo_lock:
            self.dados_do_alvo = msg.data
            self.area_total = msg.data[3] * msg.data[4]
            self.area_minima = self.area_total * 0.01
            self.area_maxima = self.area_total * 0.6
            self.dist_obj = msg.data[1]

            if msg.data[2]>self.area_minima:
                self.visto_por_ultimo = rospy.Time.now()

    #======================================
    #funçoes de estado
    #======================================

    def procura(self):

        twist = Twist()

        twist.angular.z = 0.6
        self.comando.publish(twist)
        #cria a msg twist para girar e publica para procurar o objeto

    def segue(self):

        twist = Twist()

        with self.dados_do_alvo_lock:
            if self.dist_obj < 15:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            elif self.dados_do_alvo[2] < self.area_minima:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            else:
                twist.linear.x = 0.6
                erro_x = self.dados_do_alvo[0] - (int(self.dados_do_alvo[4] / 2) + offset)
                twist.angular.z = max(min(self.PID(erro_x), 1.0), -1.0)
        #com a "chave" checa se o alvo está nos parametros. o primeiro if é pro tempinho que ele tem antes de voltar a girar, pra impedir que ande em circulos pq do PID

        self.comando.publish(twist)


    #======================================
    #  transicao
    #======================================

    def transicao_procura(self):
        with self.dados_do_alvo_lock:
            if self.dados_do_alvo is not None and self.dados_do_alvo[2] > self.area_minima:
                rospy.loginfo("Objeto detectado! Transição para SEGUINDO.")
                return maquina_de_estados.seguindo
            
        return None

    def transicao_seguindo(self):
        with self.dados_do_alvo_lock:
            if self.dados_do_alvo[2] < self.area_minima:
                # Só muda se o alvo sumiu por mais de tempo minimo. impede que ele entre em procurando se tiver instabilidade na analise de imagem
                if (rospy.Time.now() - self.visto_por_ultimo).to_sec() > self.tempo_minimo_sem_objeto:
                    rospy.loginfo("Objeto perdido! Transição para PROCURANDO.")
                    return maquina_de_estados.procurando
                
        return None
    #checa se o alvo está la. se estiver ele começa a seguir. Essa é a unica saida do perdido pra não virar um ping pong entre perdido e procurando

    def run(self):
        rate = rospy.Rate(30)
        #define quantas vezes o codigo é rodado (hz) - provalvemtne isso deve cair la pra 15 dependendo da camera
        while not rospy.is_shutdown():
            if self.estado in self.fsm:
                func_estado, func_transicao = self.fsm[self.estado]
                #resgata o estado e a função de transição no dicionario de fsm no __init__
                func_estado()
                #roda o estado
                novo_estado = func_transicao()
                #roda a func de transicao pra testar se é valido trocar de estado
                if novo_estado is not None:
                    self.estado = novo_estado
                    rospy.loginfo(f"Mudança de estado: {self.estado.name} -> {novo_estado.name}")
                    #troca de estado e informa
            else:
                rospy.logerr(f"Estado desconhecido: {self.estado}")
                self.estado = maquina_de_estados.procurando
                #pra evitar que o código emperre em um estado desconhecido
            rate.sleep()
            #faz com que o codigo não repita se ele for mais rapido de executar que os 30hz

# ================================
# loop while true
# ================================

if __name__ == '__main__':
    robo = FSM_robo_seguidor()
    try:
        robo.run()
    except rospy.ROSInterruptException:
        pass