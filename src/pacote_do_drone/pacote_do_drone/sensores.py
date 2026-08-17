import json
import os
import re

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

from pacote_do_drone.cabo_angulos import (
    calcular_angulos_ancora_drone_graus,
    calcular_angulos_tangente_cabo_graus,
    calcular_angulos_vetor_graus_z_positivo,
    calcular_angulos_vetor_mundo_graus,
    compor_quaternions,
    elevation_saturado,
    extrair_angulos_graus,
    rotacionar_vetor,
)

JOINT_STATE_TOPIC = '/world/mundo_ic/model/sistema_cabo_drone/model/meu_drone/joint_state'
POSE_TOPIC = '/world/mundo_ic/pose/info'
FINAL_SEGMENT_FRAME_TOKENS = ('final_segment',)
TIP_FRAME_TOKENS = ('ponta_cabo',)
CABO_MODEL_FRAME = 'cabo_dinamico'
SEGMENT_RE = re.compile(r'(^|::)segment_(\d+)$')

class LeitorCabo(Node):
    def __init__(self):
        super().__init__('leitor_cabo_node')
        self.declare_parameter('janela_tangente_links', 6)
        self.sim_time_ns = None

        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.subscription_tensao = self.create_subscription(
            WrenchStamped,
            '/cabo/tensao_drone',
            self.tensao_callback,
            10)

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/meu_drone/odom',
            self.odom_callback,
            10)

        self.subscription_poses = self.create_subscription(
            TFMessage,
            POSE_TOPIC,
            self.pose_callback,
            10)

        self.subscription_angulos = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.joint_callback,
            10)

        self.azimuth_pub = self.create_publisher(Float64, '/cabo/azimuth_graus', 10)
        self.elevation_pub = self.create_publisher(Float64, '/cabo/elevation_graus', 10)
        self.azimuth_ancora_pub = self.create_publisher(Float64, '/cabo/azimuth_ancora_graus', 10)
        self.elevation_ancora_pub = self.create_publisher(Float64, '/cabo/elevation_ancora_graus', 10)
        self.azimuth_drone_pub = self.create_publisher(Float64, '/cabo/drone/azimuth_graus', 10)
        self.elevation_drone_pub = self.create_publisher(Float64, '/cabo/drone/elevation_graus', 10)
        self.azimuth_reta_ancora_pub = self.create_publisher(Float64, '/cabo/drone/reta_ancora/azimuth_graus', 10)
        self.elevation_reta_ancora_pub = self.create_publisher(Float64, '/cabo/drone/reta_ancora/elevation_graus', 10)
        self.azimuth_tangente_ancora_pub = self.create_publisher(Float64, '/cabo/ancora/azimuth_graus', 10)
        self.elevation_tangente_ancora_pub = self.create_publisher(Float64, '/cabo/ancora/elevation_graus', 10)
        self.azimuth_joint_pub = self.create_publisher(Float64, '/cabo/azimuth_joint_graus', 10)
        self.elevation_joint_pub = self.create_publisher(Float64, '/cabo/elevation_joint_graus', 10)
        self.last_log_ns = None
        self.orientacao_modelo_cabo = None
        self.orientacao_segmento_final = None
        self.posicao_modelo_cabo = None
        self.posicao_segmento_final = None
        self.posicao_ponta_cabo = None
        self.posicoes_segmentos = {}
        self.usando_tangente_local = False
        self.posicao_ancora = self._ler_posicao_ancora()
        self.offset_sensor_corpo = (0.0, 0.0, -0.05)
        self.janela_tangente_links = max(1, int(self.get_parameter('janela_tangente_links').value))

    def _agora_ns(self):
        if self.sim_time_ns is not None:
            return self.sim_time_ns
        return self.get_clock().now().nanoseconds

    def clock_callback(self, msg):
        self.sim_time_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec

    def _ler_posicao_ancora(self):
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

    def tensao_callback(self, msg):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z

        tensao_resultante = (fx**2 + fy**2 + fz**2) ** 0.5
        self.get_logger().debug(f'Tensao resultante: {tensao_resultante:.3f} N')

    def pose_callback(self, msg):
        for transform in msg.transforms:
            frame = transform.child_frame_id
            t = transform.transform.translation
            q = transform.transform.rotation
            posicao = (t.x, t.y, t.z)
            orientacao = (q.x, q.y, q.z, q.w)
            if frame == CABO_MODEL_FRAME:
                self.orientacao_modelo_cabo = orientacao
                self.posicao_modelo_cabo = posicao
                continue

            match = SEGMENT_RE.search(frame)
            if match:
                self.posicoes_segmentos[int(match.group(2))] = posicao
            elif all(token in frame for token in FINAL_SEGMENT_FRAME_TOKENS):
                self.orientacao_segmento_final = orientacao
                self.posicao_segmento_final = posicao
            elif all(token in frame for token in TIP_FRAME_TOKENS):
                self.posicao_ponta_cabo = posicao

    def _ponto_cabo_mundo(self, posicao):
        if posicao is None:
            return None
        if self.posicao_modelo_cabo is None or self.orientacao_modelo_cabo is None:
            return posicao
        rotacionado = rotacionar_vetor(self.orientacao_modelo_cabo, posicao)
        return tuple(self.posicao_modelo_cabo[i] + rotacionado[i] for i in range(3))

    def _ponto_tangente_cabo(self):
        if self.posicoes_segmentos:
            maior_indice = max(self.posicoes_segmentos)
            indice = max(1, maior_indice - self.janela_tangente_links)
            while indice > 0:
                posicao = self.posicoes_segmentos.get(indice)
                if posicao is not None:
                    return self._ponto_cabo_mundo(posicao), f'segment_{indice}'
                indice -= 1

        return self._ponto_cabo_mundo(self.posicao_segmento_final), 'final_segment'

    def _ponto_tangente_ancora(self):
        if not self.posicoes_segmentos:
            return None, 'indisponivel'

        indice = min(self.janela_tangente_links, max(self.posicoes_segmentos))
        while indice <= max(self.posicoes_segmentos):
            posicao = self.posicoes_segmentos.get(indice)
            if posicao is not None:
                return self._ponto_cabo_mundo(posicao), f'segment_{indice}'
            indice += 1

        return None, 'indisponivel'

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        orientacao_drone = (q.x, q.y, q.z, q.w)
        posicao_drone = (p.x, p.y, p.z)

        azimuth_ancora_deg, elevation_ancora_deg = calcular_angulos_ancora_drone_graus(
            posicao_drone,
            orientacao_drone,
            self.posicao_ancora,
            self.offset_sensor_corpo,
        )
        azimuth_tangente_ancora_deg = None
        elevation_tangente_ancora_deg = None
        posicao_segmento_ancora, frame_tangente_ancora = self._ponto_tangente_ancora()
        if posicao_segmento_ancora is not None:
            vetor_ancora_mundo = tuple(posicao_segmento_ancora[i] - self.posicao_ancora[i] for i in range(3))
            azimuth_tangente_ancora_deg, elevation_tangente_ancora_deg = calcular_angulos_vetor_graus_z_positivo(
                vetor_ancora_mundo,
            )

        posicao_segmento, frame_tangente = self._ponto_tangente_cabo()
        posicao_ponta = self._ponto_cabo_mundo(self.posicao_ponta_cabo)

        if posicao_segmento is not None and posicao_ponta is not None:
            vetor_mundo = tuple(posicao_segmento[i] - posicao_ponta[i] for i in range(3))
            azimuth_deg, elevation_deg = calcular_angulos_vetor_mundo_graus(
                orientacao_drone,
                vetor_mundo,
            )
            origem = f'geometria:{frame_tangente}'
            self.usando_tangente_local = True
        elif self.orientacao_segmento_final is not None:
            orientacao_segmento = self.orientacao_segmento_final
            if self.orientacao_modelo_cabo is not None:
                orientacao_segmento = compor_quaternions(self.orientacao_modelo_cabo, orientacao_segmento)
            azimuth_deg, elevation_deg = calcular_angulos_tangente_cabo_graus(
                orientacao_drone,
                orientacao_segmento,
            )
            origem = 'tangente'
            self.usando_tangente_local = True
        else:
            azimuth_deg, elevation_deg = azimuth_ancora_deg, elevation_ancora_deg
            origem = 'ancora'

        self.azimuth_pub.publish(Float64(data=azimuth_deg))
        self.elevation_pub.publish(Float64(data=elevation_deg))
        self.azimuth_drone_pub.publish(Float64(data=azimuth_deg))
        self.elevation_drone_pub.publish(Float64(data=elevation_deg))
        self.azimuth_ancora_pub.publish(Float64(data=azimuth_ancora_deg))
        self.elevation_ancora_pub.publish(Float64(data=elevation_ancora_deg))
        self.azimuth_reta_ancora_pub.publish(Float64(data=azimuth_ancora_deg))
        self.elevation_reta_ancora_pub.publish(Float64(data=elevation_ancora_deg))
        if azimuth_tangente_ancora_deg is not None and elevation_tangente_ancora_deg is not None:
            self.azimuth_tangente_ancora_pub.publish(Float64(data=azimuth_tangente_ancora_deg))
            self.elevation_tangente_ancora_pub.publish(Float64(data=elevation_tangente_ancora_deg))

        now_ns = self._agora_ns()
        if self.last_log_ns is None or now_ns - self.last_log_ns >= 500_000_000:
            self.last_log_ns = now_ns
            sufixo = ' | perto de vertical' if elevation_saturado(elevation_deg) else ''
            texto_ancora = 'indisponivel'
            if azimuth_tangente_ancora_deg is not None and elevation_tangente_ancora_deg is not None:
                texto_ancora = (
                    f'az={azimuth_tangente_ancora_deg:6.1f} deg '
                    f'el={elevation_tangente_ancora_deg:5.1f} deg ({frame_tangente_ancora})'
                )
            self.get_logger().info(
                f'Drone tangente: az={azimuth_deg:6.1f} deg el={elevation_deg:5.1f} deg ({frame_tangente}) | '
                f'Drone reta->ancora: az={azimuth_ancora_deg:6.1f} deg el={elevation_ancora_deg:5.1f} deg | '
                f'Ancora tangente: {texto_ancora}{sufixo}'
            )

    def joint_callback(self, msg):
        try:
            azimuth_deg, elevation_deg = extrair_angulos_graus(msg)
        except ValueError:
            return

        self.azimuth_joint_pub.publish(Float64(data=azimuth_deg))
        self.elevation_joint_pub.publish(Float64(data=elevation_deg))

def main(args=None):
    rclpy.init(args=args)
    leitor = LeitorCabo()
    rclpy.spin(leitor)
    leitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
