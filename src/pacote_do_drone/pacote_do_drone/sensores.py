import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

class LeitorCabo(Node):
    def __init__(self):
        super().__init__('leitor_cabo_node')
        
        self.tensao_atual = 0.0
        self.angulo_ponta = 0.0
        
        self.recebeu_tensao = False
        self.recebeu_angulo = False
        
        self.subscription_tensao = self.create_subscription(
            WrenchStamped,
            '/cabo/tensao_drone',
            self.tensao_callback,
            10)
            
        self.subscription_angulos = self.create_subscription(
            JointState,
            '/world/mundo_ic/model/cabo_dinamico/joint_state',
            self.angulos_callback,
            10)
            
        self.timer = self.create_timer(0.5, self.imprimir_dados)

    def tensao_callback(self, msg):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        self.tensao_atual = math.sqrt(fx**2 + fy**2 + fz**2)
        self.recebeu_tensao = True

    def angulos_callback(self, msg):
        try:
            # 1. Filtra a lista para pegar apenas juntas do tipo 'joint_X'
            juntas_cabo = [nome for nome in msg.name if nome.startswith('joint_') and nome != 'joint_ponta']
            
            # 2. Pega a última junta flexível da lista (ex: 'joint_80')
            ultima_junta = juntas_cabo[-1]
            
            # 3. Acha em qual posição do vetor esse nome está
            idx = msg.name.index(ultima_junta)
            
            # 4. Salva o ângulo em graus
            self.angulo_ponta = math.degrees(msg.position[idx])
            
            self.recebeu_angulo = True
            
        except (ValueError, IndexError):
            # Se a lista vier vazia ou der erro, o código apenas ignora e tenta de novo no próximo frame
            pass

    def imprimir_dados(self):
        # Se recebeu qualquer um dos dois dados, plota no terminal
        if self.recebeu_tensao or self.recebeu_angulo:
            self.get_logger().info(
                f'Tensão: {self.tensao_atual:.3f} N  |  '
                f'Ângulo do cabo em relação ao drone: {self.angulo_ponta:.2f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    leitor = LeitorCabo()
    rclpy.spin(leitor)
    leitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()