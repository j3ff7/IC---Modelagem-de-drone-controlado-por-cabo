import math
import statistics

from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64


def _rpy_from_quat(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _mean(values):
    return statistics.fmean(values) if values else float('nan')


def _std(values):
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def _rms(values):
    return math.sqrt(statistics.fmean(v * v for v in values)) if values else float('nan')


def _min(values):
    return min(values) if values else float('nan')


def _max(values):
    return max(values) if values else float('nan')


class HoverMetrics(Node):
    def __init__(self):
        super().__init__('hover_metrics')
        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 1.0)
        self.declare_parameter('inicio_s', 0.0)
        self.declare_parameter('duracao_s', 10.0)
        self.declare_parameter('janela_final_s', 5.0)
        self.declare_parameter('log_periodo_s', 2.0)
        self.declare_parameter('limite_vel_xy', 0.6)
        self.declare_parameter('limite_vel_z', 0.5)

        self.target = (
            float(self.get_parameter('target_x').value),
            float(self.get_parameter('target_y').value),
            float(self.get_parameter('target_z').value),
        )
        self.inicio_s = max(0.0, float(self.get_parameter('inicio_s').value))
        self.duracao_s = max(0.0, float(self.get_parameter('duracao_s').value))
        self.janela_final_s = max(0.0, float(self.get_parameter('janela_final_s').value))
        self.log_periodo_ns = int(max(0.1, float(self.get_parameter('log_periodo_s').value)) * 1e9)
        self.limite_vel_xy = max(1e-9, float(self.get_parameter('limite_vel_xy').value))
        self.limite_vel_z = max(1e-9, float(self.get_parameter('limite_vel_z').value))

        self.sim_time_ns = None
        self.sim_start_ns = None
        self.wall_start_ns = None
        self.last_log_ns = None
        self.final_impresso = False

        self.pos = None
        self.rpy = (0.0, 0.0, 0.0)
        self.tensao = 0.0
        self.forca = (0.0, 0.0, 0.0)
        self.cmd = (0.0, 0.0, 0.0)
        self.azimuth = None
        self.elevation = None
        self.samples = []

        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.create_subscription(Odometry, '/meu_drone/odom', self.odom_callback, 10)
        self.create_subscription(WrenchStamped, '/cabo/tensao_drone', self.tensao_callback, 10)
        self.create_subscription(Twist, '/meu_drone/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Float64, '/cabo/azimuth_graus', self.azimuth_callback, 10)
        self.create_subscription(Float64, '/cabo/elevation_graus', self.elevation_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f'Metricas hover: target=({self.target[0]:.2f},{self.target[1]:.2f},{self.target[2]:.2f}), '
            f'inicio={self.inicio_s:.1f}s, duracao={self.duracao_s:.1f}s, janela_final={self.janela_final_s:.1f}s'
        )

    def clock_callback(self, msg):
        self.sim_time_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        self.rpy = _rpy_from_quat(msg.pose.pose.orientation)

    def tensao_callback(self, msg):
        f = msg.wrench.force
        self.forca = (f.x, f.y, f.z)
        self.tensao = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)

    def cmd_callback(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.linear.z)

    def azimuth_callback(self, msg):
        self.azimuth = msg.data

    def elevation_callback(self, msg):
        self.elevation = msg.data

    def _tempo(self):
        wall_ns = self.get_clock().now().nanoseconds
        sim_ns = self.sim_time_ns if self.sim_time_ns is not None else wall_ns
        if self.sim_start_ns is None:
            self.sim_start_ns = sim_ns
            self.wall_start_ns = wall_ns
            self.last_log_ns = sim_ns
        return sim_ns, wall_ns, (sim_ns - self.sim_start_ns) * 1e-9

    def timer_callback(self):
        if self.pos is None:
            return
        sim_ns, wall_ns, t = self._tempo()
        rtf = (sim_ns - self.sim_start_ns) / max(1, wall_ns - self.wall_start_ns)

        dx = self.pos[0] - self.target[0]
        dy = self.pos[1] - self.target[1]
        dz = self.pos[2] - self.target[2]
        erro = math.sqrt(dx * dx + dy * dy + dz * dz)

        if t >= self.inicio_s and self.azimuth is not None and self.elevation is not None:
            self.samples.append({
                't': t,
                'x': self.pos[0],
                'y': self.pos[1],
                'z': self.pos[2],
                'erro': erro,
                'erro_z_abs': abs(dz),
                'roll': math.degrees(self.rpy[0]),
                'pitch': math.degrees(self.rpy[1]),
                'yaw': math.degrees(self.rpy[2]),
                'tensao': self.tensao,
                'fx': self.forca[0],
                'fy': self.forca[1],
                'fz': self.forca[2],
                'cmd_x': self.cmd[0],
                'cmd_y': self.cmd[1],
                'cmd_z': self.cmd[2],
                'sat_x': abs(self.cmd[0]) >= 0.98 * self.limite_vel_xy,
                'sat_y': abs(self.cmd[1]) >= 0.98 * self.limite_vel_xy,
                'sat_z': abs(self.cmd[2]) >= 0.98 * self.limite_vel_z,
                'az': self.azimuth,
                'el': self.elevation,
            })

        if sim_ns - self.last_log_ns >= self.log_periodo_ns:
            self.last_log_ns = sim_ns
            self.get_logger().info(
                f'SIM t={t:.1f}s | RTF={rtf:.2f} | pos=({self.pos[0]:.2f},{self.pos[1]:.2f},{self.pos[2]:.2f}) '
                f'| err={erro:.2f}m | T={self.tensao:.2f}N | '
                f'az={self.azimuth if self.azimuth is not None else float("nan"):.1f} | '
                f'el={self.elevation if self.elevation is not None else float("nan"):.1f}'
            )

        if not self.final_impresso and t >= self.inicio_s + self.duracao_s:
            self.final_impresso = True
            self._imprimir_resumo(rtf)

    def _imprimir_resumo(self, rtf):
        if not self.samples:
            self.get_logger().warning('METRICAS hover: nenhuma amostra coletada.')
            return

        limite = self.samples[-1]['t'] - self.janela_final_s
        janela = [s for s in self.samples if s['t'] >= limite] if self.janela_final_s > 0.0 else self.samples
        xs = [s['x'] for s in janela]
        ys = [s['y'] for s in janela]
        zs = [s['z'] for s in janela]
        erros = [s['erro'] for s in janela]
        rolls = [abs(s['roll']) for s in janela]
        pitches = [abs(s['pitch']) for s in janela]
        yaws = [s['yaw'] for s in janela]
        tensoes = [s['tensao'] for s in janela]
        fxs = [s['fx'] for s in janela]
        fys = [s['fy'] for s in janela]
        fzs = [s['fz'] for s in janela]
        cmd_xs = [abs(s['cmd_x']) for s in janela]
        cmd_ys = [abs(s['cmd_y']) for s in janela]
        cmd_zs = [abs(s['cmd_z']) for s in janela]
        azs = [s['az'] for s in janela]
        els = [s['el'] for s in janela]
        total = max(1, len(janela))
        sat_x = 100.0 * sum(1 for s in janela if s['sat_x']) / total
        sat_y = 100.0 * sum(1 for s in janela if s['sat_y']) / total
        sat_z = 100.0 * sum(1 for s in janela if s['sat_z']) / total
        cruzamentos = []
        for limite in (0.20, 0.10, 0.05):
            tempo = next((s['t'] for s in self.samples if s['erro_z_abs'] < limite), None)
            cruzamentos.append('nan' if tempo is None else f'{tempo:.2f}')

        self.get_logger().info(
            'METRICAS hover final | '
            f'amostras={len(janela)} | RTF_med={rtf:.2f} | '
            f't_erro_z<0.20/0.10/0.05={cruzamentos[0]}/{cruzamentos[1]}/{cruzamentos[2]} s | '
            f'pos_mean=({_mean(xs):.3f},{_mean(ys):.3f},{_mean(zs):.3f}) | '
            f'pos_std=({_std(xs):.3f},{_std(ys):.3f},{_std(zs):.3f}) | '
            f'err_mean/rms/max={_mean(erros):.3f}/{_rms(erros):.3f}/{_max(erros):.3f} m | '
            f'roll_max={_max(rolls):.2f} deg | pitch_max={_max(pitches):.2f} deg | '
            f'yaw_mean/std={_mean(yaws):.2f}/{_std(yaws):.2f} deg | '
            f'T_mean/max={_mean(tensoes):.2f}/{_max(tensoes):.2f} N | '
            f'F_mean=({_mean(fxs):.2f},{_mean(fys):.2f},{_mean(fzs):.2f}) N | '
            f'cmd_abs_max=({_max(cmd_xs):.2f},{_max(cmd_ys):.2f},{_max(cmd_zs):.2f}) m/s | '
            f'sat_xyz=({sat_x:.1f},{sat_y:.1f},{sat_z:.1f}) % | '
            f'az_mean/std/min/max={_mean(azs):.2f}/{_std(azs):.2f}/{_min(azs):.2f}/{_max(azs):.2f} deg | '
            f'el_mean/std/min/max={_mean(els):.2f}/{_std(els):.2f}/{_min(els):.2f}/{_max(els):.2f} deg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HoverMetrics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
