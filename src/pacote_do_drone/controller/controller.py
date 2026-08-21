import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
import math
import json
from pathlib import Path

raiz_pacote = Path(__file__).resolve().parent.parent
caminho_json = raiz_pacote / 'parameters' / 'tether_parameters.json'

with open(caminho_json, 'r') as f:
    params = json.load(f)

num_links = max(3, int(params.get("num_links", 50)))

length = params.get("length", 0.05)

comprimento = length*num_links

class ControladorDrone(Node):
    def __init__(self):
        super().__init__('controlador_pairar_drone')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        
        
        # ==========================================
        # PARÂMETROS DO VOO
        # ==========================================
        self.altura_alvo = 1
        self.x_alvo = 1
        self.y_alvo = 0 
        # ==========================================

        self.z_atual = 0.0
        self.x_atual = 0.0
        self.y_atual = 0.0
        
        self.tensao_drone = 0.0
        self.tensao_carretel = 0.0
        
        # ==========================================
        # VARIÁVEIS PARA O LOG NA TELA
        # ==========================================
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_z = 0.0
        self.erro_xy_atual = 0.0
        self.erro_z_atual = 0.0
        
        # ==========================================
        # VARIÁVEIS DO PID
        # ==========================================
        self.dt = 0.02  # Tempo de cada iteração do controle (20Hz)
        
        # Memória para o termo Derivativo (erro anterior)
        self.erro_x_ant = 0.0
        self.erro_y_ant = 0.0
        
        # Memória para o termo Integral (soma dos erros)
        self.soma_erro_x = 0.0
        self.soma_erro_y = 0.0
        
        # Ganhos do PID
        self.kp = 8.0   
        self.ki = 1 
        self.kd = 5.0   
        # ==========================================

        # Publishers e Subscribers
        self.publisher_ = self.create_publisher(Twist, '/meu_drone/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/meu_drone/odom', self.odom_callback, 10)
        self.sub_tensao_drone = self.create_subscription(WrenchStamped, '/cabo/tensao_drone', self.tensao_drone_callback, 10)
        self.sub_tensao_carretel = self.create_subscription(WrenchStamped, '/cabo/tensao_carretel', self.tensao_carretel_callback, 10)
        
        # TIMER DO PID (Roda rápido: 20 vezes por segundo)
        self.timer_controle = self.create_timer(self.dt, self.timer_callback)
        
        # TIMER DO LOG (Roda devagar: a cada 0.5 segundos)
        self.timer_log = self.create_timer(0.5, self.imprimir_log)
        
        self.get_logger().info(f"Iniciando voo. Subindo para {self.altura_alvo}m e centralizando no carretel...")

    def odom_callback(self, msg):   
        self.x_atual = msg.pose.pose.position.x
        self.y_atual = msg.pose.pose.position.y
        self.z_atual = msg.pose.pose.position.z

    def tensao_drone_callback(self, msg):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        self.tensao_drone = math.sqrt(fx**2 + fy**2 + fz**2)

    def tensao_carretel_callback(self, msg):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        self.tensao_carretel = math.sqrt(fx**2 + fy**2 + fz**2)

    def timer_callback(self):
        msg = Twist()
        
        # --- 1. CONTROLE DE ALTURA (Controlador P) ---
        erro_z = self.altura_alvo - self.z_atual
        kp_z = 2.0  
        comando_z = kp_z * erro_z
        
        if comando_z > 2.0: comando_z = 2.0
        if comando_z < -2.0: comando_z = -2.0
        msg.linear.z = comando_z

        # --- 2. CONTROLE PID PARA X e Y ---
        erro_x = self.x_alvo - self.x_atual
        erro_y = self.y_alvo - self.y_atual
        
        # Cálculo Integral 
        self.soma_erro_x += erro_x * self.dt
        self.soma_erro_y += erro_y * self.dt
        
        # Trava do Integral
        limite_int = 0.4
        self.soma_erro_x = max(-limite_int, min(limite_int, self.soma_erro_x))
        self.soma_erro_y = max(-limite_int, min(limite_int, self.soma_erro_y))
        
        # Cálculo Derivativo
        deriv_x = (erro_x - self.erro_x_ant) / self.dt
        deriv_y = (erro_y - self.erro_y_ant) / self.dt
        
        # Equação do PID
        comando_x = (self.kp * erro_x) + (self.ki * self.soma_erro_x) + (self.kd * deriv_x)
        comando_y = (self.kp * erro_y) + (self.ki * self.soma_erro_y) + (self.kd * deriv_y)
        
        # Atualiza a memória
        self.erro_x_ant = erro_x
        self.erro_y_ant = erro_y
        
        # Trava final de velocidade física
        limite_xy = 1.0
        msg.linear.x = max(-limite_xy, min(limite_xy, comando_x))
        msg.linear.y = max(-limite_xy, min(limite_xy, comando_y))
        msg.angular.z = 0.0
        
        self.publisher_.publish(msg)
        
        # Salva os valores atuais para o print
        self.cmd_x = msg.linear.x
        self.cmd_y = msg.linear.y
        self.cmd_z = msg.linear.z
        self.erro_xy_atual = math.hypot(erro_x, erro_y)
        self.erro_z_atual = erro_z

    def imprimir_log(self):
        # Esta função roda isolada, a cada 0.5 segundos, só para imprimir os dados
        fase_atual = "Pairando" if abs(self.erro_z_atual) < 0.1 and self.erro_xy_atual < 0.1 else "Posicionando"
        
        self.get_logger().info(f"Comandos (m/s) -> X: {self.cmd_x:.2f} | Y: {self.cmd_y:.2f} | Z: {self.cmd_z:.2f}")
        self.get_logger().info(
            f"[{fase_atual}] Pos: (X:{self.x_atual:.2f}, Y:{self.y_atual:.2f}, Z:{self.z_atual:.2f}) | "
            f"Tensões: D={self.tensao_drone:.2f}N / C={self.tensao_carretel:.2f}N"
        )
        self.get_logger().info("-" * 40) # Apenas uma linha para separar no terminal

def main(args=None):
    rclpy.init(args=args)
    node = ControladorDrone()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Parando o drone...")
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
