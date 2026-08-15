import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def _normalizar_angulo(rad):
    return math.atan2(math.sin(rad), math.cos(rad))


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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

        self.centro_x = float(self.get_parameter('centro_x').value)
        self.centro_y = float(self.get_parameter('centro_y').value)
        self.altura = float(self.get_parameter('altura').value)
        self.waypoints_param = str(self.get_parameter('waypoints').value).strip()
        self.waypoints_file = str(self.get_parameter('waypoints_file').value).strip()
        self.tolerancia_posicao = float(self.get_parameter('tolerancia_posicao').value)
        self.tolerancia_altura = float(self.get_parameter('tolerancia_altura').value)
        self.histerese_chegada = max(1.0, float(self.get_parameter('histerese_chegada').value))
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
        self.waypoints = self._carregar_waypoints()

        self.x = None
        self.y = None
        self.z = None
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.ultimo_x = None
        self.ultimo_y = None
        self.ultimo_z = None
        self.ultimo_odom_ns = None
        self.yaw = 0.0
        self.indice_waypoint = 0
        self.tempo_no_alvo = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.ultimo_waypoint_atingido = False
        self.tensao_drone = 0.0
        self.tensao_carretel = 0.0
        self.last_log_time = self.get_clock().now()

        self.publisher_ = self.create_publisher(Twist, '/meu_drone/cmd_vel', 10)
        self.create_subscription(Odometry, '/meu_drone/odom', self.odom_callback, 10)
        self.create_subscription(WrenchStamped, '/cabo/tensao_drone', self.tensao_drone_callback, 10)
        self.create_subscription(WrenchStamped, '/cabo/tensao_carretel', self.tensao_carretel_callback, 10)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            f'Trajetoria por sequencia: {len(self.waypoints)} waypoints, '
            f'centro_ref=({self.centro_x:.2f}, {self.centro_y:.2f}), '
            f'hover={self.tempo_hover:.1f} s, repetir={self.repetir}, '
            f'controlar_heading={self.controlar_heading}, heading={math.degrees(self.heading_fixo):.1f} deg, '
            f'cmd_frame={self.cmd_vel_frame}, velocidade_por_diferenca={self.usar_velocidade_por_diferenca}'
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

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        novo_x = p.x
        novo_y = p.y
        novo_z = p.z
        now_ns = self.get_clock().now().nanoseconds
        novo_yaw = _yaw_from_quat(msg.pose.pose.orientation)

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
        self.yaw = novo_yaw

    def tensao_drone_callback(self, msg):
        f = msg.wrench.force
        self.tensao_drone = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)

    def tensao_carretel_callback(self, msg):
        f = msg.wrench.force
        self.tensao_carretel = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)

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

    def _atualizar_integrador(self, erro_x, erro_y, erro_z):
        if self.ganho_integral_xy <= 0.0:
            self.integral_x = 0.0
            self.integral_y = 0.0
        if self.ganho_integral_z <= 0.0:
            self.integral_z = 0.0

        self.integral_x = _limitar(
            self.integral_x + erro_x * self.timer_period if self.ganho_integral_xy > 0.0 else 0.0,
            self.limite_integral_xy,
        )
        self.integral_y = _limitar(
            self.integral_y + erro_y * self.timer_period if self.ganho_integral_xy > 0.0 else 0.0,
            self.limite_integral_xy,
        )
        self.integral_z = _limitar(
            self.integral_z + erro_z * self.timer_period if self.ganho_integral_z > 0.0 else 0.0,
            self.limite_integral_z,
        )

    def timer_callback(self):
        if self.x is None:
            return

        alvo_x, alvo_y, alvo_z = self.waypoints[self.indice_waypoint]

        erro_x = alvo_x - self.x
        erro_y = alvo_y - self.y
        erro_xy = math.hypot(erro_x, erro_y)
        erro_z = alvo_z - self.z
        velocidade_xy = math.hypot(self.vx, self.vy)
        self._atualizar_integrador(erro_x, erro_y, erro_z)

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
        vx_mundo = _limitar(vx_mundo, self.limite_vel_xy)
        vy_mundo = _limitar(vy_mundo, self.limite_vel_xy)

        vz = _limitar(
            self.ganho_altura * erro_z
            + self.ganho_integral_z * self.integral_z
            - self.ganho_velocidade_z * self.vz,
            self.limite_vel_z,
        )
        erro_yaw = _normalizar_angulo(self.heading_fixo - self.yaw)
        yaw_rate = 0.0
        if self.controlar_heading:
            yaw_rate = _limitar(self.ganho_yaw * erro_yaw, self.limite_yaw_rate)
        vx_cmd, vy_cmd = self._velocidade_no_frame_de_comando(vx_mundo, vy_mundo)

        msg = Twist()
        msg.linear.x = vx_cmd
        msg.linear.y = vy_cmd
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.publisher_.publish(msg)

        fator_tolerancia = self.histerese_chegada if self.tempo_no_alvo > 0.0 else 1.0
        chegou_posicao = (
            erro_xy <= self.tolerancia_posicao * fator_tolerancia
            and abs(erro_z) <= self.tolerancia_altura * fator_tolerancia
        )
        chegou_velocidade = velocidade_xy <= self.tolerancia_velocidade and abs(self.vz) <= self.tolerancia_velocidade
        if chegou_posicao and chegou_velocidade:
            self.tempo_no_alvo += self.timer_period
        else:
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
            self.indice_waypoint = 0 if ultimo else self.indice_waypoint + 1
            self.tempo_no_alvo = 0.0
            self._resetar_integradores()
            self.get_logger().info(f'Avancando para waypoint {self.indice_waypoint}/{len(self.waypoints) - 1}')

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds >= 1_000_000_000:
            self.last_log_time = now
            self.get_logger().info(
                f'pos=({self.x:.2f}, {self.y:.2f}, {self.z:.2f}) '
                f'alvo[{self.indice_waypoint}]=({alvo_x:.2f}, {alvo_y:.2f}, {alvo_z:.2f}) '
                f'erro=({erro_x:.2f}, {erro_y:.2f}, {erro_z:.2f}) '
                f'vel=({self.vx:.2f}, {self.vy:.2f}, {self.vz:.2f}) '
                f'hover={self.tempo_no_alvo:.1f}/{self.tempo_hover:.1f} s '
                f'yaw={math.degrees(self.yaw):.1f}/{math.degrees(self.heading_fixo):.1f} deg '
                f'cmd=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}, {msg.angular.z:.2f}) '
                f'tensoes D/C={self.tensao_drone:.2f}/{self.tensao_carretel:.2f} N'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ControladorTrajetoriaDrone()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Parando controlador de trajetoria.')
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
