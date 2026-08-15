import math


AZIMUTH_JOINT = 'cabo_azimuth_joint'
ELEVATION_JOINT = 'cabo_elevation_joint'
ELEVATION_LIMIT_DEG = 90.0
SATURATION_MARGIN_DEG = 2.0


def normalizar_azimuth(angulo_graus):
    normalizado = (angulo_graus + 180.0) % 360.0 - 180.0
    if abs(normalizado + 180.0) < 1e-9:
        return 180.0
    return normalizado


def _quat_mult(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def compor_quaternions(a, b):
    return _quat_mult(a, b)


def _quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def rotacionar_vetor(q, v):
    vx, vy, vz = v
    rx, ry, rz, _ = _quat_mult(_quat_mult(q, (vx, vy, vz, 0.0)), _quat_conj(q))
    return rx, ry, rz


def rotacionar_vetor_inverso(q, v):
    vx, vy, vz = v
    rx, ry, rz, _ = _quat_mult(_quat_mult(_quat_conj(q), (vx, vy, vz, 0.0)), q)
    return rx, ry, rz


def extrair_angulos_graus(msg):
    idx_azimuth = msg.name.index(AZIMUTH_JOINT)
    idx_elevation = msg.name.index(ELEVATION_JOINT)
    return math.degrees(msg.position[idx_azimuth]), math.degrees(msg.position[idx_elevation])


def calcular_angulos_vetor_graus(vetor):
    """Calcula azimuth/elevation usando x=frente, y=esquerda, z=cima."""
    vx, vy, vz = vetor
    horizontal = math.hypot(vx, vy)

    if horizontal < 1e-9 and abs(vz) < 1e-9:
        return 0.0, 0.0

    azimuth_deg = 0.0 if horizontal < 1e-9 else normalizar_azimuth(math.degrees(math.atan2(vy, vx)))
    elevation_deg = math.degrees(math.atan2(-vz, horizontal))
    return azimuth_deg, elevation_deg


def calcular_angulos_ancora_drone_graus(
    posicao_drone,
    orientacao_drone,
    posicao_ancora,
    offset_sensor_corpo=(0.0, 0.0, -0.05),
):
    offset_sensor_mundo = rotacionar_vetor(orientacao_drone, offset_sensor_corpo)
    posicao_sensor = tuple(posicao_drone[i] + offset_sensor_mundo[i] for i in range(3))
    vetor_mundo = tuple(posicao_ancora[i] - posicao_sensor[i] for i in range(3))
    vetor_corpo = rotacionar_vetor_inverso(orientacao_drone, vetor_mundo)
    return calcular_angulos_vetor_graus(vetor_corpo)


def calcular_angulos_tangente_cabo_graus(orientacao_drone, orientacao_segmento_final):
    eixo_cabo_mundo = rotacionar_vetor(orientacao_segmento_final, (-1.0, 0.0, 0.0))
    eixo_cabo_corpo = rotacionar_vetor_inverso(orientacao_drone, eixo_cabo_mundo)
    return calcular_angulos_vetor_graus(eixo_cabo_corpo)


def elevation_saturado(elevation_deg, margin_deg=SATURATION_MARGIN_DEG):
    return abs(elevation_deg) >= ELEVATION_LIMIT_DEG - margin_deg
