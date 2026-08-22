import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators # NOVO PACOTE
import math
import time
import matplotlib.pyplot as plt

class ControladorEmpuxo(Node):
    def __init__(self):
        super().__init__('controlador_empuxo_raiz')
        
        # Publicador de comandos diretamente para os motores
        self.pub_motores = self.create_publisher(Actuators, '/meu_drone/command/motor_speed', 10)
        self.sub_odom = self.create_subscription(Odometry, '/meu_drone/odom', self.odom_callback, 10)
        
        # Variáveis de Estado
        self.x_atual = self.y_atual = self.z_atual = 0.0
        self.roll_atual = self.pitch_atual = self.yaw_atual = 0.0
        
        # Alvos
        self.x_alvo = 1.0
        self.y_alvo = 0.0
        self.z_alvo = 1.0
        
        # Parâmetros Físicos do Drone (Estimados a partir do SDF)
        self.massa = 1.5 
        self.gravidade = 9.81
        self.peso = self.massa * self.gravidade
        self.k_f = 1.5e-03 # Constante de força do motor
        
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("Controlador Raiz Iniciado: Assumindo controle direto das hélices!")

    def odom_callback(self, msg):   
        self.x_atual = msg.pose.pose.position.x
        self.y_atual = msg.pose.pose.position.y
        self.z_atual = msg.pose.pose.position.z
        
        # Extrai os ângulos da odometria (Quaternions para Euler)
        q = msg.pose.pose.orientation
        self.roll_atual, self.pitch_atual, self.yaw_atual = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

    def euler_from_quaternion(self, x, y, z, w):
        # Matemática para converter o sensor do Gazebo em Ângulos reais
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def timer_callback(self):
        # ----------------------------------------------------
        # 1. CONTROLE DE POSIÇÃO (Gera ângulos desejados e Empuxo)
        # ----------------------------------------------------
        erro_x = self.x_alvo - self.x_atual
        erro_y = self.y_alvo - self.y_atual
        erro_z = self.z_alvo - self.z_atual
        
        # PIDs simplificados de Posição (ganhos baixos para estabilidade inicial)
        kp_z = 1.0
        empuxo_desejado = self.peso + (kp_z * erro_z) 
        
        kp_xy = 0.5
        # Para ir para frente (X positivo), o drone tem que "abaixar o nariz" (Pitch negativo)
        pitch_desejado = -kp_xy * erro_x
        # Para ir para esquerda/direita, rolar
        roll_desejado = kp_xy * erro_y 
        
        # Travas de segurança de ângulo (máximo ~20 graus)
        pitch_desejado = max(-0.35, min(0.35, pitch_desejado))
        roll_desejado = max(-0.35, min(0.35, roll_desejado))

        # ----------------------------------------------------
        # 2. CONTROLE DE ATITUDE (Gera Torques nos eixos)
        # ----------------------------------------------------
        erro_roll = roll_desejado - self.roll_atual
        erro_pitch = pitch_desejado - self.pitch_atual
        
        kp_atitude = 10.0
        cmd_roll = kp_atitude * erro_roll
        cmd_pitch = kp_atitude * erro_pitch

        # ----------------------------------------------------
        # 3. MATRIZ DE MISTURA (Mixer) -> Distribui para as Hélices
        # ----------------------------------------------------
        # Calcula a velocidade base quadrada para manter o drone no ar
        # Empuxo Total = 4 * k_f * velocidade_quadrada
        base_quadrado = empuxo_desejado / (4.0 * self.k_f)
        if base_quadrado < 0: base_quadrado = 0.0
        
        # Distribui o torque somando e subtraindo do empuxo base de cada hélice
        # Motor 0 (Frente-Dir), Motor 1 (Trás-Esq), Motor 2 (Frente-Esq), Motor 3 (Trás-Dir)
        m0_q = base_quadrado - cmd_pitch - cmd_roll
        m1_q = base_quadrado + cmd_pitch + cmd_roll
        m2_q = base_quadrado - cmd_pitch + cmd_roll
        m3_q = base_quadrado + cmd_pitch - cmd_roll
        
        # Converte para velocidade angular (rad/s) garantindo que não dê raiz negativa
        msg = Actuators()
        msg.velocity = [
            float(math.sqrt(max(0.0, m0_q))),
            float(math.sqrt(max(0.0, m1_q))),
            float(math.sqrt(max(0.0, m2_q))),
            float(math.sqrt(max(0.0, m3_q)))
        ]
        
        self.pub_motores.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControladorEmpuxo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
