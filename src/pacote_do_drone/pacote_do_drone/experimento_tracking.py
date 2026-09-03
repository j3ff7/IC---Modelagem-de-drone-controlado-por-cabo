import csv
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

from PIL import Image, ImageDraw

from pacote_do_drone.cabo_angulos import rotacionar_vetor


POSE_TOPIC = '/world/mundo_ic/pose/info'
TIP_FRAME_TOKENS = ('ponta_cabo',)
FINAL_SEGMENT_FRAME_TOKENS = ('final_segment',)
CABO_MODEL_FRAME = 'cabo_dinamico'
SENSOR_OFFSET_BASE = (0.0, 0.0, -0.05)


def _rpy_from_quat(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _mean(values):
    return sum(values) / len(values) if values else float('nan')


def _rms(values):
    return math.sqrt(_mean([v * v for v in values])) if values else float('nan')


class Plotter:
    def __init__(self, out_dir):
        self.out_dir = Path(out_dir)

    def plot(self, filename, title, xlabel, ylabel, x, series):
        w, h = 1200, 700
        left, right, top, bottom = 90, 40, 70, 90
        colors = [(31, 119, 180), (214, 39, 40), (44, 160, 44), (148, 103, 189)]
        img = Image.new('RGB', (w, h), 'white')
        d = ImageDraw.Draw(img)
        xs = x if x else [0.0]
        ys_all = [v for _, ys in series for v in ys] or [0.0]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys_all), max(ys_all)
        if abs(xmax - xmin) < 1e-9:
            xmax = xmin + 1.0
        if abs(ymax - ymin) < 1e-9:
            ymin -= 1.0
            ymax += 1.0
        ym = 0.08 * (ymax - ymin)
        ymin -= ym
        ymax += ym

        def px(xv):
            return left + (xv - xmin) * (w - left - right) / (xmax - xmin)

        def py(yv):
            return h - bottom - (yv - ymin) * (h - top - bottom) / (ymax - ymin)

        d.rectangle([left, top, w - right, h - bottom], outline=(0, 0, 0))
        d.text((left, 20), title, fill=(0, 0, 0))
        d.text((w // 2 - 60, h - 35), xlabel, fill=(0, 0, 0))
        d.text((10, top + 10), ylabel, fill=(0, 0, 0))
        for i in range(6):
            gx = left + i * (w - left - right) / 5
            gy = top + i * (h - top - bottom) / 5
            d.line([(gx, top), (gx, h - bottom)], fill=(230, 230, 230))
            d.line([(left, gy), (w - right, gy)], fill=(230, 230, 230))
            d.text((gx - 20, h - bottom + 8), f'{xmin + i * (xmax - xmin) / 5:.1f}', fill=(0, 0, 0))
            d.text((8, gy - 7), f'{ymax - i * (ymax - ymin) / 5:.2f}', fill=(0, 0, 0))
        for idx, (label, ys) in enumerate(series):
            points = [(px(xv), py(yv)) for xv, yv in zip(x, ys)]
            if len(points) >= 2:
                d.line(points, fill=colors[idx % len(colors)], width=3)
            lx, ly = left + 20 + 260 * (idx % 3), top + 15 + 22 * (idx // 3)
            d.line([(lx, ly + 7), (lx + 35, ly + 7)], fill=colors[idx % len(colors)], width=4)
            d.text((lx + 42, ly), label, fill=(0, 0, 0))
        img.save(self.out_dir / filename)


class ExperimentoTracking(Node):
    def __init__(self):
        super().__init__('experimento_tracking')
        self.declare_parameter('caso', 'n')
        self.declare_parameter('result_dir', 'results/n')
        self.declare_parameter('duracao_s', 34.0)
        self.declare_parameter('janela_final_s', 5.0)
        self.declare_parameter('sample_period_s', 0.05)

        self.caso = str(self.get_parameter('caso').value)
        self.result_dir = Path(str(self.get_parameter('result_dir').value)).expanduser()
        self.duracao_s = max(0.1, float(self.get_parameter('duracao_s').value))
        self.janela_final_s = max(0.0, float(self.get_parameter('janela_final_s').value))
        self.sample_period_ns = int(max(0.01, float(self.get_parameter('sample_period_s').value)) * 1e9)

        self.result_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.result_dir / 'data.csv'
        self.samples = []
        self.sim_time_ns = None
        self.sim_start_ns = None
        self.wall_start_ns = None
        self.last_sample_ns = None
        self.finished = False

        self.ref = None
        self.pos = None
        self.orientation = None
        self.rpy = (0.0, 0.0, 0.0)
        self.cmd = (0.0, 0.0, 0.0)
        self.tension = 0.0
        self.connection_force = 0.0
        self.connection_moment = 0.0
        self.azimuth = None
        self.elevation = None
        self.final_segment_pos = None
        self.final_segment_quat = None
        self.ponta_cabo_pos = None
        self.cabo_model_pos = None
        self.cabo_model_quat = None
        self.offset_ponta_cabo = self._offset_ponta_final_segment()

        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.create_subscription(PoseStamped, '/meu_drone/ref', self.ref_callback, 10)
        self.create_subscription(Odometry, '/meu_drone/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/meu_drone/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(WrenchStamped, '/cabo/tensao_drone', self.tension_callback, 10)
        self.create_subscription(WrenchStamped, '/cabo/conexao_drone', self.connection_callback, 10)
        self.create_subscription(Float64, '/cabo/azimuth_graus', self.azimuth_callback, 10)
        self.create_subscription(Float64, '/cabo/elevation_graus', self.elevation_callback, 10)
        self.create_subscription(TFMessage, POSE_TOPIC, self.pose_callback, 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(f'Experimento {self.caso}: resultados em {self.result_dir}')

    def clock_callback(self, msg):
        self.sim_time_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec

    def ref_callback(self, msg):
        p = msg.pose.position
        self.ref = (p.x, p.y, p.z)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos = (p.x, p.y, p.z)
        self.orientation = (q.x, q.y, q.z, q.w)
        self.rpy = _rpy_from_quat(q)

    def cmd_callback(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.linear.z)

    def tension_callback(self, msg):
        f = msg.wrench.force
        self.tension = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)

    def connection_callback(self, msg):
        f = msg.wrench.force
        m = msg.wrench.torque
        self.connection_force = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
        self.connection_moment = math.sqrt(m.x * m.x + m.y * m.y + m.z * m.z)

    def azimuth_callback(self, msg):
        self.azimuth = msg.data

    def elevation_callback(self, msg):
        self.elevation = msg.data

    def pose_callback(self, msg):
        for transform in msg.transforms:
            frame = transform.child_frame_id
            t = transform.transform.translation
            q = transform.transform.rotation
            pos = (t.x, t.y, t.z)
            quat = (q.x, q.y, q.z, q.w)
            if frame == CABO_MODEL_FRAME:
                self.cabo_model_pos = pos
                self.cabo_model_quat = quat
            elif all(token in frame for token in FINAL_SEGMENT_FRAME_TOKENS):
                self.final_segment_pos = pos
                self.final_segment_quat = quat
            elif all(token in frame for token in TIP_FRAME_TOKENS):
                self.ponta_cabo_pos = pos

    def _tempo(self):
        wall_ns = self.get_clock().now().nanoseconds
        sim_ns = self.sim_time_ns if self.sim_time_ns is not None else wall_ns
        if self.sim_start_ns is None:
            self.sim_start_ns = sim_ns
            self.wall_start_ns = wall_ns
            self.last_sample_ns = sim_ns
        return sim_ns, wall_ns, (sim_ns - self.sim_start_ns) * 1e-9

    def _offset_ponta_final_segment(self):
        try:
            caminho_sdf = os.path.join(
                get_package_share_directory('pacote_do_drone'),
                'models',
                'cabo.sdf',
            )
            root = ET.parse(caminho_sdf).getroot()
            pose = root.find(".//joint[@name='joint_ponta_cabo']/pose")
            if pose is not None and pose.text:
                return float(pose.text.split()[0])
        except (LookupError, OSError, ET.ParseError, ValueError, IndexError):
            pass

        try:
            caminho_json = os.path.join(
                get_package_share_directory('pacote_do_drone'),
                'tether_parameters.json',
            )
            with open(caminho_json, 'r', encoding='utf-8') as f:
                params = json.load(f)
            return float(params.get('length', 0.05))
        except (LookupError, OSError, ValueError, json.JSONDecodeError):
            return 0.05

    def _ponto_conexao_cabo_mundo(self):
        if self.final_segment_pos is not None and self.final_segment_quat is not None:
            offset = rotacionar_vetor(self.final_segment_quat, (self.offset_ponta_cabo, 0.0, 0.0))
            ponto = tuple(self.final_segment_pos[i] + offset[i] for i in range(3))
        else:
            ponto = self.ponta_cabo_pos
        if ponto is None:
            return None
        if self.cabo_model_pos is not None and self.cabo_model_quat is not None:
            rotacionado = rotacionar_vetor(self.cabo_model_quat, ponto)
            return tuple(self.cabo_model_pos[i] + rotacionado[i] for i in range(3))
        return ponto

    def _sensor_mundo(self):
        if self.pos is None or self.orientation is None:
            return None
        offset = rotacionar_vetor(self.orientation, SENSOR_OFFSET_BASE)
        return tuple(self.pos[i] + offset[i] for i in range(3))

    def timer_callback(self):
        if self.finished or self.pos is None or self.ref is None:
            return
        sim_ns, wall_ns, t = self._tempo()
        if self.last_sample_ns is not None and sim_ns - self.last_sample_ns < self.sample_period_ns:
            return
        self.last_sample_ns = sim_ns
        rtf = (sim_ns - self.sim_start_ns) / max(1, wall_ns - self.wall_start_ns)

        tip = self._ponto_conexao_cabo_mundo()
        sensor = self._sensor_mundo()
        distancia = float('nan')
        if tip is not None and sensor is not None:
            distancia = _norm3(tuple(tip[i] - sensor[i] for i in range(3)))

        ex = self.ref[0] - self.pos[0]
        ey = self.ref[1] - self.pos[1]
        ez = self.ref[2] - self.pos[2]
        self.samples.append({
            't_sim': t,
            'x_ref': self.ref[0], 'y_ref': self.ref[1], 'z_ref': self.ref[2],
            'x': self.pos[0], 'y': self.pos[1], 'z': self.pos[2],
            'error_x': ex, 'error_y': ey, 'error_z': ez,
            'error_3d': math.sqrt(ex * ex + ey * ey + ez * ez),
            'cmd_x': self.cmd[0], 'cmd_y': self.cmd[1], 'cmd_z': self.cmd[2],
            'roll': self.rpy[0], 'pitch': self.rpy[1], 'yaw': self.rpy[2],
            'tension': self.tension,
            'connection_force': self.connection_force,
            'connection_moment': self.connection_moment,
            'connection_distance': distancia,
            'azimuth': self.azimuth if self.azimuth is not None else float('nan'),
            'elevation': self.elevation if self.elevation is not None else float('nan'),
            'rtf': rtf,
        })
        if t >= self.duracao_s:
            self.finished = True
            self._finalizar()
            raise SystemExit(0)

    def _finalizar(self):
        if not self.samples:
            self.get_logger().warning('Sem amostras para salvar.')
            return
        campos = list(self.samples[0].keys())
        with self.csv_path.open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=campos, lineterminator='\n')
            writer.writeheader()
            for row in self.samples:
                writer.writerow({k: f'{v:.6f}' if isinstance(v, float) else v for k, v in row.items()})
        self._gerar_plots()
        self._resumo()

    def _serie(self, nome):
        return [s[nome] for s in self.samples]

    def _gerar_plots(self):
        t = self._serie('t_sim')
        p = Plotter(self.result_dir)
        p.plot('position_x.png', f'{self.caso}: posicao X', 't_sim [s]', 'x [m]', t, [('x_ref', self._serie('x_ref')), ('x', self._serie('x'))])
        p.plot('position_y.png', f'{self.caso}: posicao Y', 't_sim [s]', 'y [m]', t, [('y_ref', self._serie('y_ref')), ('y', self._serie('y'))])
        p.plot('position_z.png', f'{self.caso}: posicao Z', 't_sim [s]', 'z [m]', t, [('z_ref', self._serie('z_ref')), ('z', self._serie('z'))])
        p.plot('position_error.png', f'{self.caso}: erros', 't_sim [s]', 'erro [m]', t, [('error_x', self._serie('error_x')), ('error_y', self._serie('error_y')), ('error_z', self._serie('error_z')), ('error_3d', self._serie('error_3d'))])
        p.plot('controller_commands.png', f'{self.caso}: comandos', 't_sim [s]', 'cmd [m/s]', t, [('cmd_x', self._serie('cmd_x')), ('cmd_y', self._serie('cmd_y')), ('cmd_z', self._serie('cmd_z'))])
        p.plot('tether_angles.png', f'{self.caso}: angulos do tether', 't_sim [s]', 'angulo [deg]', t, [('azimuth', self._serie('azimuth')), ('elevation', self._serie('elevation'))])
        p.plot('connection.png', f'{self.caso}: conexao tether-drone', 't_sim [s]', 'SI', t, [('distancia [m]', self._serie('connection_distance')), ('|F| [N]', self._serie('connection_force')), ('|M| [Nm]', self._serie('connection_moment'))])
        p.plot('trajectory_xy.png', f'{self.caso}: trajetoria XY', 'x [m]', 'y [m]', self._serie('x'), [('trajetoria real', self._serie('y')), ('referencia', self._serie('y_ref'))])

    def _resumo(self):
        limite = self.samples[-1]['t_sim'] - self.janela_final_s
        janela = [s for s in self.samples if s['t_sim'] >= limite] if self.janela_final_s > 0 else self.samples
        err = [s['error_3d'] for s in janela]
        dist = [s['connection_distance'] for s in janela if not math.isnan(s['connection_distance'])]
        self.get_logger().info(
            'RESUMO experimento | '
            f'caso={self.caso} | dir={self.result_dir} | '
            f'err_rms={_rms(err):.3f} m | '
            f'dist_conexao_mean/max={_mean(dist):.5f}/{(max(dist) if dist else float("nan")):.5f} m | '
            f'|F|_mean/max={_mean([s["connection_force"] for s in janela]):.3f}/{max(s["connection_force"] for s in janela):.3f} N | '
            f'|M|_mean/max={_mean([s["connection_moment"] for s in janela]):.6f}/{max(s["connection_moment"] for s in janela):.6f} Nm | '
            f'roll_max={max(abs(s["roll"]) for s in janela):.2f} deg | '
            f'pitch_max={max(abs(s["pitch"]) for s in janela):.2f} deg | '
            f'T_mean/max={_mean([s["tension"] for s in janela]):.3f}/{max(s["tension"] for s in janela):.3f} N | '
            f'RTF_mean={_mean([s["rtf"] for s in janela]):.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentoTracking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
