import rclpy
from rclpy.node import Node
import math

# Importando os tipos de mensagens que o Gazebo envia
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

class LeitorCabo(Node):
    def __init__(self):
        super().__init__('leitor_cabo_node')
        
        # Assinante para a Tensão
        self.subscription_tensao = self.create_subscription(
            WrenchStamped,
            '/tensao_cabo',
            self.tensao_callback,
            10)
            
        # Assinante para os Ângulos
        self.subscription_angulos = self.create_subscription(
            JointState,
            '/angulos_cabo',
            self.angulos_callback,
            10)

    def tensao_callback(self, msg):
        # Pegando as forças nos 3 eixos
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        
        # Calculando a magnitude da tensão
        tensao_resultante = math.sqrt(fx**2 + fy**2 + fz**2)
        
        self.get_logger().info(f'Tensão Resultante: {tensao_resultante:.3f} N')

    def angulos_callback(self, msg):
        # O JointState retorna duas listas: 'name' (nome da junta) e 'position' (ângulo em radianos)
        # Vamos procurar os índices exatos das juntas do cabo
        try:
            idx_x = msg.name.index('cabo_dinamico::joint_1_x')
            idx_y = msg.name.index('cabo_dinamico::joint_1_y')
            
            angulo_x_rad = msg.position[idx_x]
            angulo_y_rad = msg.position[idx_y]
            
            # Convertendo para graus para ficar mais fácil de visualizar
            angulo_x_graus = math.degrees(angulo_x_rad)
            angulo_y_graus = math.degrees(angulo_y_rad)
            
            self.get_logger().info(f'Ângulo X (Roll): {angulo_x_graus:.2f}°, Ângulo Y (Pitch): {angulo_y_graus:.2f}°')
            
        except ValueError:
            # Caso o tópico ainda não tenha carregado os nomes corretos
            pass

def main(args=None):
    rclpy.init(args=args)
    leitor = LeitorCabo()
    rclpy.spin(leitor)
    leitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()