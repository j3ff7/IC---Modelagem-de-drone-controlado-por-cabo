import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
import math

class ControladorDrone(Node):
    def __init__(self):
        super().__init__('controlador_pairar_drone')
        
        # ==========================================
        # PARÂMETROS DO VOO
        # ==========================================
        self.altura_alvo = 2.0     
        self.x_alvo = 0.0  
        self.y_alvo = 0.0  
        # ==========================================

        self.z_atual = 0.0
        self.x_atual = 0.0
        self.y_atual = 0.0
        
        self.tensao_drone = 0.0
        self.tensao_carretel = 0.0
        
        # ==========================================
        # VARIÁVEIS DO PID
        # ==========================================
        self.dt = 0.05  # Tempo de cada iteração do timer (10Hz)
        
        # Memória para o termo Derivativo (erro anterior)
        self.erro_x_ant = 0.0
        self.erro_y_ant = 0.0
        
        # Memória para o termo Integral (soma dos erros)
        self.soma_erro_x = 0.0
        self.soma_erro_y = 0.0
        
        # Ganhos do PID (Ajuste esses valores se o drone oscilar)
        self.kp = 5.0   # Força de ida ao alvo
        self.ki = 1.0  # Força contra o repuxo do cabo
        self.kd = 3.0   # Freio para não passar do alvo
        # ==========================================

        # Publishers
        self.publisher_ = self.create_publisher(Twist, '/meu_drone/cmd_vel', 10)
        
        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/meu_drone/odom', self.odom_callback, 10)
        self.sub_tensao_drone = self.create_subscription(WrenchStamped, '/cabo/tensao_drone', self.tensao_drone_callback, 10)
        self.sub_tensao_carretel = self.create_subscription(WrenchStamped, '/cabo/tensao_carretel', self.tensao_carretel_callback, 10)
        
        self.timer = self.create_timer(self.dt, self.timer_callback)
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
        
        # Cálculo Integral (acumula o erro no tempo)
        self.soma_erro_x += erro_x * self.dt
        self.soma_erro_y += erro_y * self.dt
        
        # Trava do Integral (Anti-windup) para não acumular força infinita
        limite_int = 2.0
        self.soma_erro_x = max(-limite_int, min(limite_int, self.soma_erro_x))
        self.soma_erro_y = max(-limite_int, min(limite_int, self.soma_erro_y))
        
        # Cálculo Derivativo (taxa de variação do erro)
        deriv_x = (erro_x - self.erro_x_ant) / self.dt
        deriv_y = (erro_y - self.erro_y_ant) / self.dt
        
        # Equação do PID
        comando_x = (self.kp * erro_x) + (self.ki * self.soma_erro_x) + (self.kd * deriv_x)
        comando_y = (self.kp * erro_y) + (self.ki * self.soma_erro_y) + (self.kd * deriv_y)
        
        # Atualiza a memória para a próxima iteração
        self.erro_x_ant = erro_x
        self.erro_y_ant = erro_y
        
        # Trava final de velocidade física
        limite_xy = 1.5
        msg.linear.x = max(-limite_xy, min(limite_xy, comando_x))
        msg.linear.y = max(-limite_xy, min(limite_xy, comando_y))
        msg.angular.z = 0.0
        
        self.get_logger().info(f"Comandos (m/s) -> X: {msg.linear.x:.2f} | Y: {msg.linear.y:.2f} | Z: {msg.linear.z:.2f}")

        self.publisher_.publish(msg)
        
        # --- 3. STATUS LOG ---
        erro_xy = math.hypot(erro_x, erro_y)
        fase_atual = "Pairando" if abs(erro_z) < 0.1 and erro_xy < 0.1 else "Posicionando"

        self.get_logger().info(
            f"[{fase_atual}] Pos: (X:{self.x_atual:.2f}, Y:{self.y_atual:.2f}, Z:{self.z_atual:.2f}) | "
            f"Tensões: D={self.tensao_drone:.2f}N / C={self.tensao_carretel:.2f}N"
        )

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