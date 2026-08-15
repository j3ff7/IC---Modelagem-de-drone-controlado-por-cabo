import json
import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

from pacote_do_drone.cabo_angulos import (
    calcular_angulos_ancora_drone_graus,
    calcular_angulos_tangente_cabo_graus,
    compor_quaternions,
    elevation_saturado,
    extrair_angulos_graus,
)

JOINT_STATE_TOPIC = '/world/mundo_ic/model/sistema_cabo_drone/model/meu_drone/joint_state'
POSE_TOPIC = '/world/mundo_ic/pose/info'
FINAL_SEGMENT_FRAME_TOKENS = ('final_segment',)
CABO_MODEL_FRAME = 'cabo_dinamico'

class LeitorCabo(Node):
    def __init__(self):
        super().__init__('leitor_cabo_node')

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
        self.azimuth_joint_pub = self.create_publisher(Float64, '/cabo/azimuth_joint_graus', 10)
        self.elevation_joint_pub = self.create_publisher(Float64, '/cabo/elevation_joint_graus', 10)
        self.last_log_time = self.get_clock().now()
        self.orientacao_modelo_cabo = None
        self.orientacao_segmento_final = None
        self.usando_tangente_local = False
        self.posicao_ancora = self._ler_posicao_ancora()
        self.offset_sensor_corpo = (0.0, 0.0, -0.05)

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
            float(params.get('anchor_y', 0.18)),
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
            q = transform.transform.rotation
            orientacao = (q.x, q.y, q.z, q.w)
            if frame == CABO_MODEL_FRAME:
                self.orientacao_modelo_cabo = orientacao
            elif all(token in frame for token in FINAL_SEGMENT_FRAME_TOKENS):
                self.orientacao_segmento_final = orientacao

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        orientacao_drone = (q.x, q.y, q.z, q.w)

        if self.orientacao_segmento_final is not None:
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
            azimuth_deg, elevation_deg = calcular_angulos_ancora_drone_graus(
                (p.x, p.y, p.z),
                orientacao_drone,
                self.posicao_ancora,
                self.offset_sensor_corpo,
            )
            origem = 'ancora'

        self.azimuth_pub.publish(Float64(data=azimuth_deg))
        self.elevation_pub.publish(Float64(data=elevation_deg))

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds >= 500_000_000:
            self.last_log_time = now
            sufixo = ' | perto de vertical' if elevation_saturado(elevation_deg) else ''
            self.get_logger().info(
                f'azimuth {origem}: {azimuth_deg:7.2f} graus | '
                f'elevation {origem}: {elevation_deg:7.2f} graus{sufixo}'
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
