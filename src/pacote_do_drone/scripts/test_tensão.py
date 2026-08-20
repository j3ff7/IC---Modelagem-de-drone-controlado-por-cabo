import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import math
import json
from pathlib import Path

class ControladorPairar(Node):
    def __init__(self):
        super().__init__('controlador_pairar_drone')
        caminho_json = Path(__file__).resolve().parents[1] / 'tether_parameters.json'
        # ==========================================
        # Comptimento do cabo
        # ==========================================
        with open(caminho_json, 'r') as f:
            params = json.load(f)
            
        self.num_links = params["num_links"]
        self.length    = params["length"]
        self.comprimento_total = self.num_links * self.length
        # ==========================================
        # PARÂMETROS DO VOO
        # ==========================================
        self.altura_alvo = self.comprimento_total     
        self.x_carretel = 0.0  
        self.y_carretel = 0.0  
        # ==========================================

        self.z_atual = 0.0
        self.x_atual = 0.0
        self.y_atual = 0.0
        self.yaw_atual = 0.0  # Rotação do drone
        
        self.tensao_drone = 0.0
        self.tensao_carretel = 0.0
        
        self.fase = "SUBINDO"
        
        # ==========================================
        # PARÂMETROS DO PID (XY) - Configuração de Amortecimento
        # ==========================================
        self.periodo_controle = 0.1
        self.sim_time_ns = None
        self.ultimo_controle_ns = None
        self.kp_xy = 0.5   # Força moderada de aproximação
        self.ki_xy = 0.0   # TOTALMENTE ZERADO para eliminar a divergência temporal
        self.kd_xy = 2.0   # Aumentado para frear as oscilações e o efeito pêndulo

        # Publishers e Subscribers
        self.publisher_ = self.create_publisher(Twist, '/meu_drone/cmd_vel', 10)
        self.sub_clock = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/meu_drone/odom', self.odom_callback, 10)
        self.sub_tensao_drone = self.create_subscription(WrenchStamped, '/cabo/tensao_drone', self.tensao_drone_callback, 10)
        self.sub_tensao_carretel = self.create_subscription(WrenchStamped, '/cabo/tensao_carretel', self.tensao_carretel_callback, 10)
        
        self.timer = self.create_timer(self.periodo_controle, self.timer_callback)
        self.get_logger().info(f"Iniciando voo. Etapa 1: Subindo verticalmente para {self.altura_alvo}m...")

    def clock_callback(self, msg):
        self.sim_time_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec

    def agora_ns(self):
        if self.sim_time_ns is not None:
            return self.sim_time_ns
        return self.get_clock().now().nanoseconds

    def calcular_dt_controle(self):
        agora_ns = self.agora_ns()
        if agora_ns <= 0:
            return None
        if self.ultimo_controle_ns is None:
            self.ultimo_controle_ns = agora_ns
            return None
        dt = (agora_ns - self.ultimo_controle_ns) * 1e-9
        self.ultimo_controle_ns = agora_ns
        if dt <= 1e-6:
            return None
        return min(dt, 0.5)

    def odom_callback(self, msg):
        self.x_atual = msg.pose.pose.position.x
        self.y_atual = msg.pose.pose.position.y
        self.z_atual = msg.pose.pose.position.z
        
        # Extrai o ângulo Yaw a partir dos Quatérnios
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw_atual = math.atan2(siny_cosp, cosy_cosp)

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
        dt = self.calcular_dt_controle()
        if dt is None:
            return

        msg = Twist()
        
        # --- CONTROLE DE ALTURA (Proporcional) ---
        erro_z = self.altura_alvo - self.z_atual
        kp_z = 2.0  
        msg.linear.z = max(-2.0, min(2.0, kp_z * erro_z))

        # --- LÓGICA DE ESTADOS E PID (X e Y) ---
        erro_x = self.x_carretel - self.x_atual
        erro_y = self.y_carretel - self.y_atual
        distancia_xy = math.hypot(erro_x, erro_y)

        if self.fase == "SUBINDO":
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            
            # Mantém o PID zerado até chegar na altura
            self.integral_x = 0.0
            self.integral_y = 0.0
            self.erro_x_ant = erro_x
            self.erro_y_ant = erro_y
            
            if self.z_atual >= (self.altura_alvo * 0.95):
                self.fase = "CENTRALIZANDO"

        elif self.fase in ["CENTRALIZANDO", "PAIRANDO"]:
            # 1. Acúmulo Integral com Anti-Windup estrito
            self.integral_x += erro_x * dt
            self.integral_y += erro_y * dt
            
            limite_int = 0.5  # Reduzido para evitar arremessar o drone
            self.integral_x = max(-limite_int, min(limite_int, self.integral_x))
            self.integral_y = max(-limite_int, min(limite_int, self.integral_y))
            
            # 2. Cálculo Derivativo
            deriv_x = (erro_x - self.erro_x_ant) / dt
            deriv_y = (erro_y - self.erro_y_ant) / dt
            
            # 3. PID em eixos Globais
            comando_x_global = (self.kp_xy * erro_x) + (self.ki_xy * self.integral_x) + (self.kd_xy * deriv_x)
            comando_y_global = (self.kp_xy * erro_y) + (self.ki_xy * self.integral_y) + (self.kd_xy * deriv_y)
            
            # 4. TRADUÇÃO PARA EIXOS LOCAIS (Matriz de Rotação 2D)
            comando_x_local = comando_x_global * math.cos(self.yaw_atual) + comando_y_global * math.sin(self.yaw_atual)
            comando_y_local = -comando_x_global * math.sin(self.yaw_atual) + comando_y_global * math.cos(self.yaw_atual)

            self.erro_x_ant = erro_x
            self.erro_y_ant = erro_y
            
            # 5. Aplica velocidade final
            limite_vel = 1.0 if self.fase == "CENTRALIZANDO" else 0.5
            msg.linear.x = max(-limite_vel, min(limite_vel, comando_x_local))
            msg.linear.y = max(-limite_vel, min(limite_vel, comando_y_local))
            
            # Transições
            if self.fase == "CENTRALIZANDO" and distancia_xy < 0.15:
                self.fase = "PAIRANDO"
            elif self.fase == "PAIRANDO" and distancia_xy > 0.25:
                self.fase = "CENTRALIZANDO"

        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        
        # Log visual
        self.get_logger().info(
            f"[{self.fase}] Pos: X:{self.x_atual:.2f} Y:{self.y_atual:.2f} Z:{self.z_atual:.2f} | "
            f"Tensões: D={self.tensao_drone:.2f}N C={self.tensao_carretel:.2f}N"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ControladorPairar()
    
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
