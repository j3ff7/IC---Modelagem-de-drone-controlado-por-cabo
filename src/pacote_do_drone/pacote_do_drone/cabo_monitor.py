import argparse
import csv
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from pacote_do_drone.cabo_angulos import ELEVATION_LIMIT_DEG, elevation_saturado


def _barra(valor, limite, largura=31):
    valor = max(-limite, min(limite, valor))
    centro = largura // 2
    pos = int(round((valor + limite) * (largura - 1) / (2 * limite)))
    chars = [' '] * largura
    chars[centro] = '|'
    chars[pos] = '*'
    return ''.join(chars)


class CaboMonitor(Node):
    def __init__(self, csv_path=None, rate_hz=5.0):
        super().__init__('cabo_monitor')
        self.create_subscription(Float64, '/cabo/azimuth_graus', self._azimuth_callback, 10)
        self.create_subscription(Float64, '/cabo/elevation_graus', self._elevation_callback, 10)
        self.create_subscription(Float64, '/cabo/azimuth_ancora_graus', self._azimuth_ancora_callback, 10)
        self.create_subscription(Float64, '/cabo/elevation_ancora_graus', self._elevation_ancora_callback, 10)
        self.create_subscription(Float64, '/cabo/ancora/azimuth_graus', self._azimuth_tangente_ancora_callback, 10)
        self.create_subscription(Float64, '/cabo/ancora/elevation_graus', self._elevation_tangente_ancora_callback, 10)
        self.create_subscription(Float64, '/cabo/azimuth_joint_graus', self._azimuth_joint_callback, 10)
        self.create_subscription(Float64, '/cabo/elevation_joint_graus', self._elevation_joint_callback, 10)
        self.period_ns = int(1e9 / max(rate_hz, 0.1))
        self.last_print = self.get_clock().now()
        self.azimuth_deg = None
        self.elevation_deg = None
        self.azimuth_ancora_deg = None
        self.elevation_ancora_deg = None
        self.azimuth_tangente_ancora_deg = None
        self.elevation_tangente_ancora_deg = None
        self.azimuth_joint_deg = None
        self.elevation_joint_deg = None
        self.csv_file = None
        self.csv_writer = None

        if csv_path:
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'tempo_s',
                'azimuth_deg',
                'elevation_deg',
                'azimuth_ancora_deg',
                'elevation_ancora_deg',
                'azimuth_tangente_ancora_deg',
                'elevation_tangente_ancora_deg',
                'azimuth_joint_deg',
                'elevation_joint_deg',
                'elevation_saturado',
            ])
            self.get_logger().info(f'Gravando CSV em {csv_path}')

        self.get_logger().info('Lendo angulos do cabo: drone, reta sensor-ancora e ancora.')

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()

    def _azimuth_callback(self, msg):
        self.azimuth_deg = msg.data
        self._print_if_ready()

    def _elevation_callback(self, msg):
        self.elevation_deg = msg.data
        self._print_if_ready()

    def _azimuth_ancora_callback(self, msg):
        self.azimuth_ancora_deg = msg.data
        self._print_if_ready()

    def _elevation_ancora_callback(self, msg):
        self.elevation_ancora_deg = msg.data
        self._print_if_ready()

    def _azimuth_tangente_ancora_callback(self, msg):
        self.azimuth_tangente_ancora_deg = msg.data
        self._print_if_ready()

    def _elevation_tangente_ancora_callback(self, msg):
        self.elevation_tangente_ancora_deg = msg.data
        self._print_if_ready()

    def _azimuth_joint_callback(self, msg):
        self.azimuth_joint_deg = msg.data
        self._print_if_ready()

    def _elevation_joint_callback(self, msg):
        self.elevation_joint_deg = msg.data
        self._print_if_ready()

    def _print_if_ready(self):
        if self.azimuth_deg is None or self.elevation_deg is None:
            return

        saturado = elevation_saturado(self.elevation_deg)
        tempo_s = self.get_clock().now().nanoseconds * 1e-9

        if self.csv_writer:
            self.csv_writer.writerow([
                f'{tempo_s:.6f}',
                f'{self.azimuth_deg:.6f}',
                f'{self.elevation_deg:.6f}',
                '' if self.azimuth_ancora_deg is None else f'{self.azimuth_ancora_deg:.6f}',
                '' if self.elevation_ancora_deg is None else f'{self.elevation_ancora_deg:.6f}',
                '' if self.azimuth_tangente_ancora_deg is None else f'{self.azimuth_tangente_ancora_deg:.6f}',
                '' if self.elevation_tangente_ancora_deg is None else f'{self.elevation_tangente_ancora_deg:.6f}',
                '' if self.azimuth_joint_deg is None else f'{self.azimuth_joint_deg:.6f}',
                '' if self.elevation_joint_deg is None else f'{self.elevation_joint_deg:.6f}',
                int(saturado),
            ])

        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds < self.period_ns:
            return
        self.last_print = now

        aviso = ' SAT' if saturado else ''
        linha = (
            f'Drone tangente: az={self.azimuth_deg:6.1f} deg '
            f'el={self.elevation_deg:5.1f} deg{aviso}'
        )
        if self.azimuth_ancora_deg is not None and self.elevation_ancora_deg is not None:
            linha += (
                f' | Drone reta->ancora: az={self.azimuth_ancora_deg:6.1f} deg '
                f'el={self.elevation_ancora_deg:5.1f} deg'
            )
        if self.azimuth_tangente_ancora_deg is not None and self.elevation_tangente_ancora_deg is not None:
            linha += (
                f' | Ancora tangente: az={self.azimuth_tangente_ancora_deg:6.1f} deg '
                f'el={self.elevation_tangente_ancora_deg:5.1f} deg'
            )
        if self.azimuth_joint_deg is not None and self.elevation_joint_deg is not None:
            linha += f' | Junta: az={self.azimuth_joint_deg:6.1f} deg el={self.elevation_joint_deg:5.1f} deg'
        print(linha, flush=True)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', default=None, help='Arquivo CSV para gravar os angulos.')
    parser.add_argument('--rate', type=float, default=5.0, help='Taxa de impressao no terminal em Hz.')
    parsed, ros_args = parser.parse_known_args(args if args is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = CaboMonitor(csv_path=parsed.csv, rate_hz=parsed.rate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
