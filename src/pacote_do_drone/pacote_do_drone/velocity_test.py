import math

from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _quat_mult(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _rotacionar_inverso(q, v):
    rx, ry, rz, _ = _quat_mult(_quat_mult(_quat_conj(q), (v[0], v[1], v[2], 0.0)), q)
    return rx, ry, rz


def _rpy_from_quat(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class VelocityTest(Node):
    def __init__(self):
        super().__init__('velocity_test')
        self.declare_parameter('vx_cmd', 0.0)
        self.declare_parameter('vy_cmd', 0.0)
        self.declare_parameter('vz_cmd', 0.25)
        self.declare_parameter('yaw_rate_cmd', 0.0)
        self.declare_parameter('duracao', 8.0)
        self.declare_parameter('log_periodo', 0.5)
        self.declare_parameter('cabo_joint_limit_rad', 1.4)

        self.vx_cmd = float(self.get_parameter('vx_cmd').value)
        self.vy_cmd = float(self.get_parameter('vy_cmd').value)
        self.vz_cmd = float(self.get_parameter('vz_cmd').value)
        self.yaw_rate_cmd = float(self.get_parameter('yaw_rate_cmd').value)
        self.duracao = max(0.0, float(self.get_parameter('duracao').value))
        self.log_periodo_ns = int(max(0.05, float(self.get_parameter('log_periodo').value)) * 1e9)
        self.cabo_joint_limit_rad = max(1e-6, float(self.get_parameter('cabo_joint_limit_rad').value))

        self.publisher = self.create_publisher(Twist, '/meu_drone/cmd_vel', 10)
        self.create_subscription(Odometry, '/meu_drone/odom', self.odom_callback, 10)
        self.create_subscription(WrenchStamped, '/cabo/conexao_drone', self.conexao_callback, 10)
        self.create_subscription(
            JointState,
            '/world/mundo_ic/model/sistema_cabo_drone/model/meu_drone/joint_state',
            self.joint_state_callback,
            10,
        )
        self.create_subscription(
            JointState,
            '/world/mundo_ic/model/sistema_cabo_drone/model/cabo_dinamico/joint_state',
            self.cabo_joint_state_callback,
            10,
        )

        self.x = None
        self.y = None
        self.z = None
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.az = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.orientation = (0.0, 0.0, 0.0, 1.0)
        self.last_z = None
        self.last_vz = None
        self.last_odom_ns = None
        self.conexao_f_raw = (0.0, 0.0, 0.0)
        self.conexao_m_raw = (0.0, 0.0, 0.0)
        self.conexao_f_mod = 0.0
        self.conexao_m_mod = 0.0
        self.rotores = {}
        self.cabo_joint_min_margin = None
        self.cabo_joint_near_limit = None
        self.inicio_ns = None
        self.last_log_ns = None
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f'Velocity test: cmd=({self.vx_cmd:.2f},{self.vy_cmd:.2f},{self.vz_cmd:.2f},'
            f'{self.yaw_rate_cmd:.2f}), duracao={self.duracao:.1f}s'
        )

    def odom_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        p = msg.pose.pose.position
        q_msg = msg.pose.pose.orientation
        self.orientation = (q_msg.x, q_msg.y, q_msg.z, q_msg.w)
        self.roll, self.pitch, self.yaw = _rpy_from_quat(self.orientation)
        self.x = p.x
        self.y = p.y
        self.z = p.z

        if self.last_odom_ns is not None:
            dt = (now_ns - self.last_odom_ns) * 1e-9
            if dt > 1e-4:
                dz = self.z - self.last_z
                new_vz = dz / dt
                self.az = 0.0 if self.last_vz is None else (new_vz - self.last_vz) / dt
                self.vz = new_vz
                self.last_vz = new_vz

        self.last_z = self.z
        self.last_odom_ns = now_ns

    def conexao_callback(self, msg):
        f = msg.wrench.force
        m = msg.wrench.torque
        self.conexao_f_raw = (f.x, f.y, f.z)
        self.conexao_m_raw = (m.x, m.y, m.z)
        self.conexao_f_mod = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
        self.conexao_m_mod = math.sqrt(m.x * m.x + m.y * m.y + m.z * m.z)

    def joint_state_callback(self, msg):
        for name, velocity in zip(msg.name, msg.velocity):
            if name.startswith('rotor_') and name.endswith('_joint'):
                self.rotores[name] = velocity

    def cabo_joint_state_callback(self, msg):
        margens = []
        for position in msg.position:
            margem = 1.0 - abs(position) / self.cabo_joint_limit_rad
            margens.append(margem)
        if not margens:
            return
        self.cabo_joint_min_margin = min(margens)
        self.cabo_joint_near_limit = sum(1 for margem in margens if margem < 0.05)

    def timer_callback(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if self.inicio_ns is None:
            self.inicio_ns = now_ns
            self.last_log_ns = now_ns

        t = (now_ns - self.inicio_ns) * 1e-9
        msg = Twist()
        if t <= self.duracao:
            msg.linear.x = self.vx_cmd
            msg.linear.y = self.vy_cmd
            msg.linear.z = self.vz_cmd
            msg.angular.z = self.yaw_rate_cmd
        self.publisher.publish(msg)

        if self.x is None or now_ns - self.last_log_ns < self.log_periodo_ns:
            return
        self.last_log_ns = now_ns

        f_body = _rotacionar_inverso(self.orientation, self.conexao_f_raw)
        rotores = '-'
        if self.rotores:
            valores = [self.rotores.get(f'rotor_{i}_joint') for i in range(4)]
            if all(valor is not None for valor in valores):
                rotores = '[' + ','.join(f'{valor:.0f}' for valor in valores) + '] rad/s'
        cabo_juntas = '-'
        if self.cabo_joint_min_margin is not None:
            cabo_juntas = f'min_margin={100.0 * self.cabo_joint_min_margin:.1f}% near_limit={self.cabo_joint_near_limit}'

        self.get_logger().info(
            f't={t:.2f}s z={self.z:.3f} vz={self.vz:.3f} az={self.az:.3f} '
            f'cmd_z_pub={msg.linear.z:.3f} '
            f'rpy=({math.degrees(self.roll):.1f},{math.degrees(self.pitch):.1f},'
            f'{math.degrees(self.yaw):.1f})deg '
            f'F_raw=({self.conexao_f_raw[0]:.2f},{self.conexao_f_raw[1]:.2f},{self.conexao_f_raw[2]:.2f}) '
            f'F_body_est=({f_body[0]:.2f},{f_body[1]:.2f},{f_body[2]:.2f}) '
            f'|F|={self.conexao_f_mod:.2f} |M|={self.conexao_m_mod:.3f} rot={rotores} '
            f'cabo_juntas={cabo_juntas}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VelocityTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
