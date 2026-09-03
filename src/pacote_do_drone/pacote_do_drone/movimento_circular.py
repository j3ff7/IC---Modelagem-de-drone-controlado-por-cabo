import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


def _normalizar_angulo(rad):
    return math.atan2(math.sin(rad), math.cos(rad))


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _rpy_from_quat(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    yaw = _yaw_from_quat(q)
    return roll, pitch, yaw


def _limitar(valor, limite):
    return max(-limite, min(limite, valor))


class ControladorTrajetoriaDrone(Node):
    def __init__(self):
        super().__init__('controlador_trajetoria_drone')

        ancora = self._ler_ancora()
        self.declare_parameter('centro_x', ancora[0])
        self.declare_parameter('centro_y', ancora[1])
        self.declare_parameter('altura', 1.6)
        self.declare_parameter('waypoints', '')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('tolerancia_posicao', 0.18)
        self.declare_parameter('tolerancia_altura', 0.15)
        self.declare_parameter('histerese_chegada', 1.6)
        self.declare_parameter('tempo_estabilizacao', 1.0)
        self.declare_parameter('tempo_hover', 3.0)
        self.declare_parameter('repetir', True)
        self.declare_parameter('controlar_heading', False)
        self.declare_parameter('heading_fixo', 0.0)
        self.declare_parameter('ganho_posicao_xy', 1.2)
        self.declare_parameter('ganho_altura', 1.5)
        self.declare_parameter('ganho_integral_xy', 0.0)
        self.declare_parameter('ganho_integral_z', 0.0)
        self.declare_parameter('ganho_velocidade_xy', 0.8)
        self.declare_parameter('ganho_velocidade_z', 0.45)
        self.declare_parameter('ganho_yaw', 1.5)
        self.declare_parameter('limite_vel_xy', 1.0)
        self.declare_parameter('limite_vel_z', 0.8)
        self.declare_parameter('limite_integral_xy', 1.5)
        self.declare_parameter('limite_integral_z', 1.0)
        self.declare_parameter('limite_yaw_rate', 0.8)
        self.declare_parameter('tolerancia_velocidade', 0.15)
        self.declare_parameter('cmd_vel_frame', 'world')
        self.declare_parameter('odom_twist_frame', 'body')
        self.declare_parameter('usar_velocidade_por_diferenca', True)
        self.declare_parameter('filtro_velocidade', 0.35)
        self.declare_parameter('log_periodo', 1.0)

        self.centro_x = float(self.get_parameter('centro_x').value)
        self.centro_y = float(self.get_parameter('centro_y').value)
        self.altura = float(self.get_parameter('altura').value)
        self.waypoints_param = str(self.get_parameter('waypoints').value).strip()
        self.waypoints_file = str(self.get_parameter('waypoints_file').value).strip()
        self.tolerancia_posicao = float(self.get_parameter('tolerancia_posicao').value)
        self.tolerancia_altura = float(self.get_parameter('tolerancia_altura').value)
        self.histerese_chegada = max(1.0, float(self.get_parameter('histerese_chegada').value))
        self.tempo_estabilizacao = max(0.0, float(self.get_parameter('tempo_estabilizacao').value))
        self.tempo_hover = max(0.0, float(self.get_parameter('tempo_hover').value))
        self.repetir = bool(self.get_parameter('repetir').value)
        self.controlar_heading = bool(self.get_parameter('controlar_heading').value)
        self.heading_fixo = float(self.get_parameter('heading_fixo').value)
        self.ganho_posicao_xy = float(self.get_parameter('ganho_posicao_xy').value)
        self.ganho_altura = float(self.get_parameter('ganho_altura').value)
        self.ganho_integral_xy = float(self.get_parameter('ganho_integral_xy').value)
        self.ganho_integral_z = float(self.get_parameter('ganho_integral_z').value)
        self.ganho_velocidade_xy = float(self.get_parameter('ganho_velocidade_xy').value)
        self.ganho_velocidade_z = float(self.get_parameter('ganho_velocidade_z').value)
        self.ganho_yaw = float(self.get_parameter('ganho_yaw').value)
        self.limite_vel_xy = float(self.get_parameter('limite_vel_xy').value)
        self.limite_vel_z = float(self.get_parameter('limite_vel_z').value)
        self.limite_integral_xy = float(self.get_parameter('limite_integral_xy').value)
        self.limite_integral_z = float(self.get_parameter('limite_integral_z').value)
        self.limite_yaw_rate = float(self.get_parameter('limite_yaw_rate').value)
        self.tolerancia_velocidade = float(self.get_parameter('tolerancia_velocidade').value)
        self.cmd_vel_frame = str(self.get_parameter('cmd_vel_frame').value).lower()
        self.odom_twist_frame = str(self.get_parameter('odom_twist_frame').value).lower()
        self.usar_velocidade_por_diferenca = bool(self.get_parameter('usar_velocidade_por_diferenca').value)
        self.filtro_velocidade = max(0.0, min(1.0, float(self.get_parameter('filtro_velocidade').value)))
        self.log_periodo_ns = int(max(0.05, float(self.get_parameter('log_periodo').value)) * 1e9)
        self.waypoints = self._carregar_waypoints()

        self.x = None
        self.y = None
        self.z = None
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.ultimo_x = None
        self.ultimo_y = None
        self.ultimo_z = None
        self.ultimo_odom_ns = None
        self.yaw = 0.0
        self.indice_waypoint = 0
        self.tempo_estavel = 0.0
        self.tempo_no_alvo = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.ultimo_waypoint_atingido = False
        self.tensao_drone = 0.0
        self.tensao_carretel = 0.0
        self.conexao_forca = (0.0, 0.0, 0.0)
        self.conexao_momento = (0.0, 0.0, 0.0)
        self.conexao_forca_modulo = 0.0
        self.conexao_momento_modulo = 0.0
        self.rotores = {}
        self.sim_time_ns = None
        self.last_log_ns = None
        self.tempo_inicio_ns = None
        self.wall_inicio_ns = None
        self.wall_last_log_ns = None
        self.ultimo_timer_ns = None

        self.publisher_ = self.create_publisher(Twist, '/meu_drone/cmd_vel', 10)
        self.ref_pub = self.create_publisher(PoseStamped, '/meu_drone/ref', 10)
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.create_subscription(Odometry, '/meu_drone/odom', self.odom_callback, 10)
        self.create_subscription(WrenchStamped, '/cabo/tensao_drone', self.tensao_drone_callback, 10)
        self.create_subscription(WrenchStamped, '/cabo/tensao_carretel', self.tensao_carretel_callback, 10)
        self.create_subscription(WrenchStamped, '/cabo/conexao_drone', self.conexao_drone_callback, 10)
        self.create_subscription(
            JointState,
            '/world/mundo_ic/model/sistema_cabo_drone/model/meu_drone/joint_state',
            self.joint_state_callback,
            10,
        )
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            f'Trajetoria por sequencia: {len(self.waypoints)} waypoints, '
            f'centro_ref=({self.centro_x:.2f}, {self.centro_y:.2f}), '
            f'estabilizacao={self.tempo_estabilizacao:.1f} s, hover={self.tempo_hover:.1f} s, repetir={self.repetir}, '
            f'controlar_heading={self.controlar_heading}, heading={math.degrees(self.heading_fixo):.1f} deg, '
            f'cmd_frame={self.cmd_vel_frame}, velocidade_por_diferenca={self.usar_velocidade_por_diferenca}'
        )
        self.get_logger().info(
            f'Ganhos: Kp_xy={self.ganho_posicao_xy:.2f}, Kp_z={self.ganho_altura:.2f}, '
            f'Ki_xy={self.ganho_integral_xy:.2f}, Ki_z={self.ganho_integral_z:.2f}, '
            f'Kd_xy={self.ganho_velocidade_xy:.2f}, Kd_z={self.ganho_velocidade_z:.2f}, '
            f'lim_int_xy/z='
            f'{self.limite_integral_xy:.2f}/{self.limite_integral_z:.2f}, '
            f'lim_cmd_xy/z={self.limite_vel_xy:.2f}/{self.limite_vel_z:.2f}'
        )

    def _ler_ancora(self):
        try:
            caminho_json = os.path.join(
                get_package_share_directory('pacote_do_drone'),
                'tether_parameters.json',
            )
            with open(caminho_json, 'r') as f:
                params = json.load(f)
        except (FileNotFoundError, KeyError):
            params = {}

        return (
            float(params.get('anchor_x', 0.0)),
            float(params.get('anchor_y', 0.0)),
            float(params.get('anchor_z', 0.33)),
        )

    def _normalizar_waypoint(self, item):
        if isinstance(item, dict):
            return (
                float(item.get('x', self.centro_x)),
                float(item.get('y', self.centro_y)),
                float(item.get('z', self.altura)),
            )
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            z = item[2] if len(item) >= 3 else self.altura
            return (float(item[0]), float(item[1]), float(z))
        raise ValueError(f'Waypoint invalido: {item!r}')

    def _carregar_waypoints(self):
        dados = None
        origem = 'padrao'

        if self.waypoints_param:
            try:
                dados = json.loads(self.waypoints_param)
                origem = 'parametro waypoints'
            except json.JSONDecodeError as exc:
                self.get_logger().error(f'Parametro waypoints nao e JSON valido: {exc}')

        if dados is None and self.waypoints_file:
            try:
                caminho = self.waypoints_file
                if not os.path.isabs(caminho):
                    caminho = os.path.join(get_package_share_directory('pacote_do_drone'), caminho)
                with open(caminho, 'r') as f:
                    dados = json.load(f)
                origem = caminho
            except (FileNotFoundError, json.JSONDecodeError, OSError) as exc:
                self.get_logger().error(f'Falha ao ler waypoints_file={self.waypoints_file}: {exc}')

        if isinstance(dados, dict):
            if 'tempo_hover' in dados:
                self.tempo_hover = max(0.0, float(dados['tempo_hover']))
            if 'repetir' in dados:
                self.repetir = bool(dados['repetir'])
            if 'controlar_heading' in dados:
                self.controlar_heading = bool(dados['controlar_heading'])
            if 'heading_fixo' in dados:
                self.heading_fixo = float(dados['heading_fixo'])
            dados = dados.get('waypoints', [])

        if dados:
            try:
                waypoints = [self._normalizar_waypoint(item) for item in dados]
            except (TypeError, ValueError) as exc:
                self.get_logger().error(f'Lista de waypoints invalida em {origem}: {exc}')
                waypoints = []
        else:
            waypoints = []

        if not waypoints:
            waypoints = [
                (self.centro_x + 0.8, self.centro_y, self.altura),
                (self.centro_x - 0.8, self.centro_y, self.altura),
            ]
            origem = 'padrao'

        self.get_logger().info(
            'Waypoints carregados de %s: %s'
            % (origem, ', '.join(f'({x:.2f}, {y:.2f}, {z:.2f})' for x, y, z in waypoints))
        )
        return waypoints

    def _agora_ns(self):
        if self.sim_time_ns is not None:
            return self.sim_time_ns
        return self.get_clock().now().nanoseconds

    def clock_callback(self, msg):
        self.sim_time_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        novo_x = p.x
        novo_y = p.y
        novo_z = p.z
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        now_ns = self.sim_time_ns if self.sim_time_ns is not None else stamp_ns
        if now_ns <= 0:
            now_ns = self.get_clock().now().nanoseconds
        novo_roll, novo_pitch, novo_yaw = _rpy_from_quat(msg.pose.pose.orientation)

        if self.usar_velocidade_por_diferenca and self.ultimo_odom_ns is not None:
            dt = (now_ns - self.ultimo_odom_ns) * 1e-9
            if dt > 1e-4:
                alpha = self.filtro_velocidade
                vx_medida = (novo_x - self.ultimo_x) / dt
                vy_medida = (novo_y - self.ultimo_y) / dt
                vz_medida = (novo_z - self.ultimo_z) / dt
                self.vx = (1.0 - alpha) * self.vx + alpha * vx_medida
                self.vy = (1.0 - alpha) * self.vy + alpha * vy_medida
                self.vz = (1.0 - alpha) * self.vz + alpha * vz_medida
        elif not self.usar_velocidade_por_diferenca:
            v = msg.twist.twist.linear
            if self.odom_twist_frame == 'body':
                c = math.cos(novo_yaw)
                s = math.sin(novo_yaw)
                self.vx = c * v.x - s * v.y
                self.vy = s * v.x + c * v.y
            else:
                self.vx = v.x
                self.vy = v.y
            self.vz = v.z

        self.ultimo_x = novo_x
        self.ultimo_y = novo_y
        self.ultimo_z = novo_z
        self.ultimo_odom_ns = now_ns

        self.x = novo_x
        self.y = novo_y
        self.z = novo_z
        self.roll = novo_roll
        self.pitch = novo_pitch
        self.yaw = novo_yaw

    def tensao_drone_callback(self, msg):
        f = msg.wrench.force
        self.tensao_drone = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)

    def tensao_carretel_callback(self, msg):
        f = msg.wrench.force
        self.tensao_carretel = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)

    def conexao_drone_callback(self, msg):
        f = msg.wrench.force
        m = msg.wrench.torque
        self.conexao_forca = (f.x, f.y, f.z)
        self.conexao_momento = (m.x, m.y, m.z)
        self.conexao_forca_modulo = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
        self.conexao_momento_modulo = math.sqrt(m.x * m.x + m.y * m.y + m.z * m.z)

    def joint_state_callback(self, msg):
        for name, velocity in zip(msg.name, msg.velocity):
            if name.startswith('rotor_') and name.endswith('_joint'):
                self.rotores[name] = velocity

    def _velocidade_no_frame_de_comando(self, vx_mundo, vy_mundo):
        if self.cmd_vel_frame == 'body':
            c = math.cos(self.yaw)
            s = math.sin(self.yaw)
            return (
                c * vx_mundo + s * vy_mundo,
                -s * vx_mundo + c * vy_mundo,
            )
        return vx_mundo, vy_mundo

    def _resetar_integradores(self):
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0

    def _atualizar_integrador(self, erro_x, erro_y, erro_z, dt):
        if self.ganho_integral_xy <= 0.0:
            self.integral_x = 0.0
            self.integral_y = 0.0
        if self.ganho_integral_z <= 0.0:
            self.integral_z = 0.0

        self.integral_x = _limitar(
            self.integral_x + erro_x * dt if self.ganho_integral_xy > 0.0 else 0.0,
            self.limite_integral_xy,
        )
        self.integral_y = _limitar(
            self.integral_y + erro_y * dt if self.ganho_integral_xy > 0.0 else 0.0,
            self.limite_integral_xy,
        )
        self.integral_z = _limitar(
            self.integral_z + erro_z * dt if self.ganho_integral_z > 0.0 else 0.0,
            self.limite_integral_z,
        )

    def timer_callback(self):
        if self.x is None:
            return
        now_ns = self._agora_ns()
        if self.tempo_inicio_ns is None:
            self.tempo_inicio_ns = now_ns
            wall_now_ns = self.get_clock().now().nanoseconds
            self.wall_inicio_ns = wall_now_ns
            self.wall_last_log_ns = wall_now_ns
            self.ultimo_timer_ns = now_ns
            self.last_log_ns = now_ns

        dt_controle = self.timer_period
        if self.ultimo_timer_ns is not None:
            dt_controle = max(0.0, (now_ns - self.ultimo_timer_ns) * 1e-9)
        self.ultimo_timer_ns = now_ns

        alvo_x, alvo_y, alvo_z = self.waypoints[self.indice_waypoint]
        ref_msg = PoseStamped()
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.header.frame_id = 'world'
        ref_msg.pose.position.x = alvo_x
        ref_msg.pose.position.y = alvo_y
        ref_msg.pose.position.z = alvo_z
        ref_msg.pose.orientation.w = 1.0
        self.ref_pub.publish(ref_msg)

        erro_x = alvo_x - self.x
        erro_y = alvo_y - self.y
        erro_xy = math.hypot(erro_x, erro_y)
        erro_z = alvo_z - self.z
        velocidade_xy = math.hypot(self.vx, self.vy)
        self._atualizar_integrador(erro_x, erro_y, erro_z, dt_controle)

        vx_mundo = (
            self.ganho_posicao_xy * erro_x
            + self.ganho_integral_xy * self.integral_x
            - self.ganho_velocidade_xy * self.vx
        )
        vy_mundo = (
            self.ganho_posicao_xy * erro_y
            + self.ganho_integral_xy * self.integral_y
            - self.ganho_velocidade_xy * self.vy
        )
        vx_mundo_bruto = vx_mundo
        vy_mundo_bruto = vy_mundo
        vx_mundo = _limitar(vx_mundo_bruto, self.limite_vel_xy)
        vy_mundo = _limitar(vy_mundo_bruto, self.limite_vel_xy)

        vz_bruto = (
            self.ganho_altura * erro_z
            + self.ganho_integral_z * self.integral_z
            - self.ganho_velocidade_z * self.vz
        )
        vz = _limitar(vz_bruto, self.limite_vel_z)
        erro_yaw = _normalizar_angulo(self.heading_fixo - self.yaw)
        yaw_rate = 0.0
        if self.controlar_heading:
            yaw_rate = _limitar(self.ganho_yaw * erro_yaw, self.limite_yaw_rate)
        vx_cmd, vy_cmd = self._velocidade_no_frame_de_comando(vx_mundo, vy_mundo)
        saturou_xy = abs(vx_mundo) >= self.limite_vel_xy or abs(vy_mundo) >= self.limite_vel_xy
        saturou_z = abs(vz) >= self.limite_vel_z

        msg = Twist()
        msg.linear.x = vx_cmd
        msg.linear.y = vy_cmd
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.publisher_.publish(msg)

        fator_tolerancia = self.histerese_chegada if self.tempo_estavel > 0.0 else 1.0
        chegou_posicao = (
            erro_xy <= self.tolerancia_posicao * fator_tolerancia
            and abs(erro_z) <= self.tolerancia_altura * fator_tolerancia
        )
        chegou_velocidade = velocidade_xy <= self.tolerancia_velocidade and abs(self.vz) <= self.tolerancia_velocidade
        if chegou_posicao and chegou_velocidade:
            self.tempo_estavel += dt_controle
            if self.tempo_estavel >= self.tempo_estabilizacao:
                self.tempo_no_alvo += dt_controle
        else:
            self.tempo_estavel = 0.0
            self.tempo_no_alvo = 0.0
            self.ultimo_waypoint_atingido = False

        if self.tempo_no_alvo >= self.tempo_hover:
            ultimo = self.indice_waypoint >= len(self.waypoints) - 1
            if ultimo and not self.repetir:
                if not self.ultimo_waypoint_atingido:
                    self.get_logger().info('Sequencia concluida. Mantendo controle no ultimo waypoint.')
                self.ultimo_waypoint_atingido = True
                self.tempo_no_alvo = self.tempo_hover
                return
            indice_anterior = self.indice_waypoint
            self.indice_waypoint = 0 if ultimo else self.indice_waypoint + 1
            proximo_x, proximo_y, proximo_z = self.waypoints[self.indice_waypoint]
            self.get_logger().info(
                f'TRANSICAO WP {indice_anterior}->{self.indice_waypoint} | '
                f'pos=({self.x:.2f},{self.y:.2f},{self.z:.2f}) '
                f'ref_ant=({alvo_x:.2f},{alvo_y:.2f},{alvo_z:.2f}) '
                f'ref_nova=({proximo_x:.2f},{proximo_y:.2f},{proximo_z:.2f}) '
                f'err_ant=({erro_x:.2f},{erro_y:.2f},{erro_z:.2f}) '
                f'vel=({self.vx:.2f},{self.vy:.2f},{self.vz:.2f}) '
                f'cmd=({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f},{msg.angular.z:.2f}) '
                f'I_pre_reset=({self.integral_x:.2f},{self.integral_y:.2f},{self.integral_z:.2f})'
            )
            self.tempo_no_alvo = 0.0
            self.tempo_estavel = 0.0
            self._resetar_integradores()
            self.get_logger().info(f'Avancando para waypoint {self.indice_waypoint}/{len(self.waypoints) - 1}')

        if self.last_log_ns is None or now_ns - self.last_log_ns >= self.log_periodo_ns:
            prev_log_ns = self.last_log_ns
            self.last_log_ns = now_ns
            tempo_s = (now_ns - self.tempo_inicio_ns) * 1e-9
            wall_now_ns = self.get_clock().now().nanoseconds
            dt_sim_log = self.log_periodo_ns * 1e-9 if prev_log_ns is None else (now_ns - prev_log_ns) * 1e-9
            dt_wall_log = max(1e-9, (wall_now_ns - (self.wall_last_log_ns or wall_now_ns)) * 1e-9)
            rtf = dt_sim_log / dt_wall_log
            self.wall_last_log_ns = wall_now_ns
            estado = 'hover' if self.tempo_no_alvo > 0.0 else ('estabilizando' if self.tempo_estavel > 0.0 else 'transitando')
            if self.ultimo_waypoint_atingido:
                estado = 'concluido'
            rotores = '-'
            if self.rotores:
                valores = [self.rotores.get(f'rotor_{i}_joint') for i in range(4)]
                if all(valor is not None for valor in valores):
                    rotores = '[' + ','.join(f'{valor:.0f}' for valor in valores) + '] rad/s'
            self.get_logger().info(
                f't={tempo_s:.2f}s | RTF={rtf:.2f} | WP {self.indice_waypoint} | estado={estado} | '
                f'pos=({self.x:.2f},{self.y:.2f},{self.z:.2f}) '
                f'ref=({alvo_x:.2f},{alvo_y:.2f},{alvo_z:.2f}) '
                f'err=({erro_x:.2f},{erro_y:.2f},{erro_z:.2f}) '
                f'vel=({self.vx:.2f},{self.vy:.2f},{self.vz:.2f}) '
                f'estavel={self.tempo_estavel:.1f}/{self.tempo_estabilizacao:.1f}s '
                f'hover={self.tempo_no_alvo:.1f}/{self.tempo_hover:.1f}s '
                f'rpy=({math.degrees(self.roll):.1f}, {math.degrees(self.pitch):.1f}, '
                f'{math.degrees(self.yaw):.1f}/{math.degrees(self.heading_fixo):.1f})deg '
                f'cmd=({msg.linear.x:.2f},{msg.linear.y:.2f},'
                f'{msg.linear.z:.2f},{msg.angular.z:.2f}) '
                f'cmd_xy_raw=({vx_mundo_bruto:.2f},{vy_mundo_bruto:.2f}) '
                f'cmd_z_raw/lim={vz_bruto:.2f}/{self.limite_vel_z:.2f} '
                f'I=({self.integral_x:.2f},{self.integral_y:.2f},'
                f'{self.integral_z:.2f}) '
                f'sat_xy/z={int(saturou_xy)}/{int(saturou_z)} '
                f'tensoes D/C={self.tensao_drone:.2f}/{self.tensao_carretel:.2f} N '
                f'conexao |F|/|M|={self.conexao_forca_modulo:.2f}/{self.conexao_momento_modulo:.3f} '
                f'F=({self.conexao_forca[0]:.2f},{self.conexao_forca[1]:.2f},{self.conexao_forca[2]:.2f}) '
                f'M=({self.conexao_momento[0]:.3f},{self.conexao_momento[1]:.3f},{self.conexao_momento[2]:.3f}) '
                f'rot={rotores}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ControladorTrajetoriaDrone()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info('Parando controlador de trajetoria.')
            node.publisher_.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
