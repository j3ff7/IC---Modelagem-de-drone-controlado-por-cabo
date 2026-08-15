import json
import math
from pathlib import Path

from pacote_do_drone.cabo_angulos import calcular_angulos_vetor_graus, normalizar_azimuth

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except ModuleNotFoundError:
    PackageNotFoundError = Exception
    get_package_share_directory = None


CASOS_PADRAO = ('e', 'ne', 'n', 'nw', 'w', 'sw', 's', 'se')
CASOS = {caso: None for caso in CASOS_PADRAO}
CONFIG_PADRAO_FONTE = Path(__file__).resolve().parents[1] / 'config' / 'postes_padrao.json'


def config_padrao():
    if CONFIG_PADRAO_FONTE.exists():
        return CONFIG_PADRAO_FONTE
    if get_package_share_directory is None:
        return CONFIG_PADRAO_FONTE
    try:
        return Path(get_package_share_directory('cabo_avaliacao')) / 'config' / 'postes_padrao.json'
    except PackageNotFoundError:
        return CONFIG_PADRAO_FONTE


def menor_erro_angular(medido_graus, esperado_graus):
    return normalizar_azimuth(medido_graus - esperado_graus)


def _config_path(config_path=None):
    return Path(config_path).expanduser() if config_path else config_padrao()


def carregar_config(config_path=None):
    caminho = _config_path(config_path)
    with caminho.open('r') as f:
        config = json.load(f)

    casos = config.get('casos', {})
    if not casos:
        raise ValueError(f'Arquivo de configuracao sem casos: {caminho}')

    return config


def nomes_casos(config_path=None):
    return tuple(carregar_config(config_path)['casos'].keys())


def _vetor3(valor, nome):
    if len(valor) != 3:
        raise ValueError(f'{nome} deve ter 3 valores')
    return tuple(float(v) for v in valor)


def _poste_xy(caso, config):
    dados_caso = config['casos'][caso]
    if isinstance(dados_caso, dict):
        if 'xy' in dados_caso:
            x, y = dados_caso['xy']
        else:
            x, y = dados_caso['x'], dados_caso['y']
    else:
        x, y = dados_caso
    return float(x), float(y)


def _vetor_mundo_para_sensor(vetor_mundo, sensor_yaw):
    vx, vy, vz = vetor_mundo
    c = math.cos(sensor_yaw)
    s = math.sin(sensor_yaw)
    return (
        c * vx + s * vy,
        -s * vx + c * vy,
        vz,
    )


def _resolver_parametro_catenaria(horizontal, desnivel, comprimento):
    equivalente_horizontal = comprimento * comprimento - desnivel * desnivel
    if horizontal <= 1e-9 or equivalente_horizontal <= horizontal * horizontal:
        return None

    equivalente_horizontal = math.sqrt(equivalente_horizontal)

    def f(a):
        try:
            return 2.0 * a * math.sinh(horizontal / (2.0 * a)) - equivalente_horizontal
        except OverflowError:
            return float('inf')

    lo = 1e-6
    hi = max(horizontal, 1.0)
    while f(hi) > 0.0:
        hi *= 2.0
        if hi > 1e9:
            return None

    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if f(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def parametros_catenaria(horizontal, desnivel, comprimento, z_inicial=0.0):
    a = _resolver_parametro_catenaria(horizontal, desnivel, comprimento)
    if a is None:
        slope = 0.0 if horizontal <= 1e-9 else desnivel / horizontal
        return {
            'tipo': 'reta',
            'slope_final': slope,
            'z': lambda u: z_inicial + slope * u,
        }

    p = horizontal / (2.0 * a)
    r = math.asinh(desnivel / (2.0 * a * math.sinh(p)))
    u0 = horizontal / 2.0 - a * r
    c = z_inicial - a * math.cosh((0.0 - u0) / a)

    return {
        'tipo': 'catenaria',
        'a': a,
        'u0': u0,
        'c': c,
        'slope_final': math.sinh((horizontal - u0) / a),
        'z': lambda u: a * math.cosh((u - u0) / a) + c,
    }


def parametros_caso(caso, config_path=None, geometria='reta'):
    config = carregar_config(config_path)
    caso = str(caso).lower()
    if caso not in config['casos']:
        nomes = ', '.join(config['casos'].keys())
        raise ValueError(f'Caso invalido "{caso}". Use um destes: {nomes}')

    ancora = _vetor3(config.get('ancora', (0.0, 0.0, 0.05)), 'ancora')
    ax, ay, az = ancora
    x, y = _poste_xy(caso, config)
    poste_altura = float(config.get('poste_altura', 1.2))
    cabo_comprimento = float(config.get('cabo_comprimento', 2.0))
    sensor_yaw = math.radians(float(config.get('sensor_yaw_graus', 90.0)))

    topo = (x, y, az + poste_altura)
    horizontal = math.hypot(x - ax, y - ay)
    vetor_sensor_ancora_mundo = (ax - x, ay - y, -poste_altura)
    if geometria == 'catenaria':
        ux = 0.0 if horizontal < 1e-9 else (x - ax) / horizontal
        uy = 0.0 if horizontal < 1e-9 else (y - ay) / horizontal
        catenaria = parametros_catenaria(horizontal, poste_altura, cabo_comprimento, az)
        slope = catenaria['slope_final']
        vetor_sensor_ancora_mundo = (-ux, -uy, -slope)

    vetor_sensor_ancora_local = _vetor_mundo_para_sensor(vetor_sensor_ancora_mundo, sensor_yaw)
    azimuth, elevation = calcular_angulos_vetor_graus(vetor_sensor_ancora_local)
    yaw = math.atan2(y - ay, x - ax)
    pitch = math.atan2(-poste_altura, horizontal)

    return {
        'caso': caso,
        'ancora': ancora,
        'poste_topo': topo,
        'poste_xy': (x, y),
        'raio_xy': horizontal,
        'poste_altura': poste_altura,
        'cabo_comprimento': cabo_comprimento,
        'yaw_cabo': yaw,
        'pitch_cabo': pitch,
        'sensor_yaw': sensor_yaw,
        'azimuth_esperado_graus': azimuth,
        'elevation_esperado_graus': elevation,
        'config_path': str(_config_path(config_path)),
    }
