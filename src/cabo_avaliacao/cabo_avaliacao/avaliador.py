import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

from cabo_avaliacao.cenarios import menor_erro_angular, parametros_caso
from pacote_do_drone.cabo_angulos import compor_quaternions, calcular_angulos_tangente_cabo_graus


POSE_TOPIC = '/world/cabo_avaliacao/pose/info'
FINAL_SEGMENT_FRAME_TOKENS = ('final_segment',)
SENSOR_FRAME_TOKENS = ('sensor_cabo',)
CABO_MODEL_FRAME = 'cabo_dinamico'


class AvaliadorCabo(Node):
    def __init__(self):
        super().__init__('avaliador_cabo')
        self.declare_parameter(
            'caso',
            'e',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('config', '')
        self.declare_parameter('modo_cabo', 'reto')
        self.declare_parameter('tolerancia_graus', 3.0)

        caso_param = self.get_parameter('caso').value
        self.caso = 'n' if caso_param is False else str(caso_param).lower()
        config_param = self.get_parameter('config').value
        self.config_path = str(config_param) if config_param else None
        self.modo_cabo = str(self.get_parameter('modo_cabo').value).lower()
        geometria = 'catenaria' if self.modo_cabo == 'catenaria' else 'reta'
        self.tolerancia = float(self.get_parameter('tolerancia_graus').value)
        self.esperado = parametros_caso(self.caso, self.config_path, geometria=geometria)
        self.last_log_time = self.get_clock().now()
        self.orientacao_sensor = None
        self.orientacao_modelo_cabo = None
        self.orientacao_segmento_final = None

        self.create_subscription(TFMessage, POSE_TOPIC, self.pose_callback, 10)
        self.azimuth_pub = self.create_publisher(Float64, '/cabo_avaliacao/azimuth_graus', 10)
        self.elevation_pub = self.create_publisher(Float64, '/cabo_avaliacao/elevation_graus', 10)
        self.erro_azimuth_pub = self.create_publisher(Float64, '/cabo_avaliacao/erro_azimuth_graus', 10)
        self.erro_elevation_pub = self.create_publisher(Float64, '/cabo_avaliacao/erro_elevation_graus', 10)

        self.get_logger().info(
            f'Caso {self.caso}: az esperado {self.esperado["azimuth_esperado_graus"]:.2f} graus, '
            f'el esperado {self.esperado["elevation_esperado_graus"]:.2f} graus'
        )
        self.get_logger().info(f'Config dos postes: {self.esperado["config_path"]}')

    def pose_callback(self, msg):
        for transform in msg.transforms:
            frame = transform.child_frame_id
            q = transform.transform.rotation
            orientacao = (q.x, q.y, q.z, q.w)

            if frame == CABO_MODEL_FRAME:
                self.orientacao_modelo_cabo = orientacao
            elif all(token in frame for token in FINAL_SEGMENT_FRAME_TOKENS):
                self.orientacao_segmento_final = orientacao
            elif all(token in frame for token in SENSOR_FRAME_TOKENS):
                self.orientacao_sensor = orientacao

        if self.orientacao_sensor is None or self.orientacao_segmento_final is None:
            return

        orientacao_segmento = self.orientacao_segmento_final
        if self.orientacao_modelo_cabo is not None:
            orientacao_segmento = compor_quaternions(self.orientacao_modelo_cabo, orientacao_segmento)

        azimuth, elevation = calcular_angulos_tangente_cabo_graus(
            orientacao_drone=self.orientacao_sensor,
            orientacao_segmento_final=orientacao_segmento,
        )

        erro_az = menor_erro_angular(azimuth, self.esperado['azimuth_esperado_graus'])
        erro_el = elevation - self.esperado['elevation_esperado_graus']

        if not rclpy.ok():
            return

        try:
            self.azimuth_pub.publish(Float64(data=azimuth))
            self.elevation_pub.publish(Float64(data=elevation))
            self.erro_azimuth_pub.publish(Float64(data=erro_az))
            self.erro_elevation_pub.publish(Float64(data=erro_el))
        except RCLError:
            return

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds >= 500_000_000:
            self.last_log_time = now
            status = 'OK' if abs(erro_az) <= self.tolerancia and abs(erro_el) <= self.tolerancia else 'ERRO'
            self.get_logger().info(
                f'{status} | sensor sensor_cabo | az {azimuth:7.2f} '
                f'esper {self.esperado["azimuth_esperado_graus"]:7.2f} erro {erro_az:7.2f} | '
                f'el {elevation:7.2f} esper {self.esperado["elevation_esperado_graus"]:7.2f} '
                f'erro {erro_el:7.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AvaliadorCabo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
