import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import math
import json
import time
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
        
        
        # ==========================================
        # PARÂMETROS DO VOO
        # ==========================================
        self.declare_parameter('altura_alvo', 1.5)
        self.declare_parameter('x_alvo', 0.0)
        self.declare_parameter('y_alvo', 0.0)
        self.declare_parameter('csv_path', '')
        self.declare_parameter('duracao_teste', 0.0)
        self.declare_parameter('log_csv_periodo', 0.02)
        self.altura_alvo = float(self.get_parameter('altura_alvo').value)
        self.x_alvo = float(self.get_parameter('x_alvo').value)
        self.y_alvo = float(self.get_parameter('y_alvo').value)
        self.csv_path = str(self.get_parameter('csv_path').value).strip()
        self.duracao_teste = max(0.0, float(self.get_parameter('duracao_teste').value))
        self.log_csv_periodo_ns = int(max(0.001, float(self.get_parameter('log_csv_periodo').value)) * 1e9)
        # ==========================================

        self.z_atual = 0.0
        self.x_atual = 0.0
        self.y_atual = 0.0
        self.vx_atual = 0.0
        self.vy_atual = 0.0
        self.vz_atual = 0.0
        self.roll_atual = 0.0
        self.pitch_atual = 0.0
        self.yaw_atual = 0.0
        self.ultimo_odom_ns = None
        self.ultimo_odom_x = None
        self.ultimo_odom_y = None
        self.ultimo_odom_z = None
        
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
        self.periodo_controle = 0.02  # Periodo nominal do timer de controle (50 Hz)
        self.sim_time_ns = None
        self.ultimo_controle_ns = None
        self.inicio_sim_ns = None
        self.inicio_wall_ns = None
        self.ultimo_csv_ns = None
        self.ultimo_dt_sim = 0.0
        self.finalizado = False
        
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
        self.sub_clock = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/meu_drone/odom', self.odom_callback, 10)
        self.sub_tensao_drone = self.create_subscription(WrenchStamped, '/cabo/tensao_drone', self.tensao_drone_callback, 10)
        self.sub_tensao_carretel = self.create_subscription(WrenchStamped, '/cabo/tensao_carretel', self.tensao_carretel_callback, 10)
        
        # TIMER DO PID (Roda rápido: 20 vezes por segundo)
        self.timer_controle = self.create_timer(self.periodo_controle, self.timer_callback)
        
        # TIMER DO LOG (Roda devagar: a cada 0.5 segundos)
        self.timer_log = self.create_timer(0.5, self.imprimir_log)

        self.csv_file = None
        self.csv_writer = None
        if self.csv_path:
            caminho = Path(self.csv_path)
            caminho.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = caminho.open('w', newline='', encoding='utf-8')
            self.csv_writer = csv.DictWriter(
                self.csv_file,
                fieldnames=[
                    't_sim', 'dt_sim', 't_wall', 'rtf',
                    'x_ref', 'y_ref', 'z_ref',
                    'x', 'y', 'z',
                    'erro_x', 'erro_y', 'erro_z',
                    'vx', 'vy', 'vz',
                    'cmd_x', 'cmd_y', 'cmd_z',
                    'roll', 'pitch', 'yaw',
                    'I_x', 'I_y', 'I_z',
                    'tensao_drone', 'tensao_carretel',
                ],
                lineterminator='\n',
            )
            self.csv_writer.writeheader()
        
        self.get_logger().info(f"Iniciando voo. Subindo para {self.altura_alvo}m e centralizando no carretel...")

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
        if self.inicio_sim_ns is None:
            self.inicio_sim_ns = agora_ns
            self.inicio_wall_ns = time.monotonic_ns()
            self.ultimo_csv_ns = agora_ns
        if self.ultimo_controle_ns is None:
            self.ultimo_controle_ns = agora_ns
            return None
        dt = (agora_ns - self.ultimo_controle_ns) * 1e-9
        self.ultimo_controle_ns = agora_ns
        if dt <= 1e-6:
            return None
        self.ultimo_dt_sim = min(dt, 0.25)
        return self.ultimo_dt_sim

    def odom_callback(self, msg):   
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        agora_ns = self.sim_time_ns if self.sim_time_ns is not None else stamp_ns
        if agora_ns <= 0:
            agora_ns = self.agora_ns()
        novo_x = msg.pose.pose.position.x
        novo_y = msg.pose.pose.position.y
        novo_z = msg.pose.pose.position.z

        if self.ultimo_odom_ns is not None:
            dt = (agora_ns - self.ultimo_odom_ns) * 1e-9
            if dt > 1e-6:
                self.vx_atual = (novo_x - self.ultimo_odom_x) / dt
                self.vy_atual = (novo_y - self.ultimo_odom_y) / dt
                self.vz_atual = (novo_z - self.ultimo_odom_z) / dt

        self.ultimo_odom_ns = agora_ns
        self.ultimo_odom_x = novo_x
        self.ultimo_odom_y = novo_y
        self.ultimo_odom_z = novo_z
        self.x_atual = msg.pose.pose.position.x
        self.y_atual = msg.pose.pose.position.y
        self.z_atual = msg.pose.pose.position.z
        self.roll_atual, self.pitch_atual, self.yaw_atual = self.quat_para_rpy(msg.pose.pose.orientation)

    def quat_para_rpy(self, q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

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
        self.soma_erro_x += erro_x * dt
        self.soma_erro_y += erro_y * dt
        
        # Trava do Integral
        limite_int = 0.4
        self.soma_erro_x = max(-limite_int, min(limite_int, self.soma_erro_x))
        self.soma_erro_y = max(-limite_int, min(limite_int, self.soma_erro_y))
        
        # Cálculo Derivativo: para referência constante, d(erro)/dt = -velocidade.
        deriv_x = -self.vx_atual
        deriv_y = -self.vy_atual
        
        # Equação do PID
        comando_x_global = (self.kp * erro_x) + (self.ki * self.soma_erro_x) + (self.kd * deriv_x)
        comando_y_global = (self.kp * erro_y) + (self.ki * self.soma_erro_y) + (self.kd * deriv_y)
        comando_x = comando_x_global * math.cos(self.yaw_atual) + comando_y_global * math.sin(self.yaw_atual)
        comando_y = -comando_x_global * math.sin(self.yaw_atual) + comando_y_global * math.cos(self.yaw_atual)
        
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
        self.escrever_csv(erro_x, erro_y, erro_z)
        self.verificar_fim_teste()

    def tempos_teste(self):
        agora_sim_ns = self.agora_ns()
        if self.inicio_sim_ns is None or self.inicio_wall_ns is None:
            return 0.0, 0.0, 0.0
        t_sim = max(0.0, (agora_sim_ns - self.inicio_sim_ns) * 1e-9)
        t_wall = max(0.0, (time.monotonic_ns() - self.inicio_wall_ns) * 1e-9)
        rtf = t_sim / t_wall if t_wall > 1e-9 else 0.0
        return t_sim, t_wall, rtf

    def escrever_csv(self, erro_x, erro_y, erro_z):
        if self.csv_writer is None:
            return
        agora_ns = self.agora_ns()
        if self.ultimo_csv_ns is not None and agora_ns - self.ultimo_csv_ns < self.log_csv_periodo_ns:
            return
        self.ultimo_csv_ns = agora_ns
        t_sim, t_wall, rtf = self.tempos_teste()
        self.csv_writer.writerow({
            't_sim': f'{t_sim:.6f}',
            'dt_sim': f'{self.ultimo_dt_sim:.6f}',
            't_wall': f'{t_wall:.6f}',
            'rtf': f'{rtf:.6f}',
            'x_ref': f'{self.x_alvo:.6f}',
            'y_ref': f'{self.y_alvo:.6f}',
            'z_ref': f'{self.altura_alvo:.6f}',
            'x': f'{self.x_atual:.6f}',
            'y': f'{self.y_atual:.6f}',
            'z': f'{self.z_atual:.6f}',
            'erro_x': f'{erro_x:.6f}',
            'erro_y': f'{erro_y:.6f}',
            'erro_z': f'{erro_z:.6f}',
            'vx': f'{self.vx_atual:.6f}',
            'vy': f'{self.vy_atual:.6f}',
            'vz': f'{self.vz_atual:.6f}',
            'cmd_x': f'{self.cmd_x:.6f}',
            'cmd_y': f'{self.cmd_y:.6f}',
            'cmd_z': f'{self.cmd_z:.6f}',
            'roll': f'{math.degrees(self.roll_atual):.6f}',
            'pitch': f'{math.degrees(self.pitch_atual):.6f}',
            'yaw': f'{math.degrees(self.yaw_atual):.6f}',
            'I_x': f'{self.soma_erro_x:.6f}',
            'I_y': f'{self.soma_erro_y:.6f}',
            'I_z': '0.000000',
            'tensao_drone': f'{self.tensao_drone:.6f}',
            'tensao_carretel': f'{self.tensao_carretel:.6f}',
        })

    def verificar_fim_teste(self):
        if self.duracao_teste <= 0.0:
            return
        t_sim, _, _ = self.tempos_teste()
        if t_sim >= self.duracao_teste:
            self.publisher_.publish(Twist())
            self.finalizado = True

    def fechar_csv(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None

    def imprimir_log(self):
        # Esta função roda isolada, a cada 0.5 segundos, só para imprimir os dados
        fase_atual = "Pairando" if abs(self.erro_z_atual) < 0.1 and self.erro_xy_atual < 0.1 else "Posicionando"
        t_sim, t_wall, rtf = self.tempos_teste()
        
        self.get_logger().info(
            f"[{fase_atual}] t_sim={t_sim:.2f}s dt={self.ultimo_dt_sim:.3f}s "
            f"t_wall={t_wall:.2f}s RTF={rtf:.2f} | "
            f"ref=({self.x_alvo:.2f},{self.y_alvo:.2f},{self.altura_alvo:.2f}) "
            f"pos=({self.x_atual:.2f},{self.y_atual:.2f},{self.z_atual:.2f}) "
            f"erro=({self.x_alvo-self.x_atual:.2f},{self.y_alvo-self.y_atual:.2f},{self.altura_alvo-self.z_atual:.2f}) "
            f"cmd=({self.cmd_x:.2f},{self.cmd_y:.2f},{self.cmd_z:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ControladorDrone()
    
    try:
        while rclpy.ok() and not node.finalizado:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Parando o drone...")
        node.publisher_.publish(Twist())
        node.fechar_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()