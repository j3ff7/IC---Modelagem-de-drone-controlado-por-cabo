import argparse
import csv
import math
import os
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from cabo_avaliacao.cenarios import nomes_casos, parametros_caso, parametros_catenaria


MODOS_CABO = ('reto', 'articulado', 'catenaria')


def _cabo_reto_xml(p, ax, ay, az, tx, ty, tz):
    mx = 0.5 * (ax + tx)
    my = 0.5 * (ay + ty)
    mz = 0.5 * (az + tz)
    return f'''
      <link name="cabo_dinamico_final_segment">
        <pose>{mx} {my} {mz} 0 {p['pitch_cabo']} {p['yaw_cabo']}</pose>
        <visual name="cabo_reto">
          <pose>0 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.006</radius><length>{p['cabo_comprimento']}</length></cylinder></geometry>
          <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material>
        </visual>
      </link>'''


def _segmento_cabo_xml(nome, p0, p1):
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx = x1 - x0
    dy = y1 - y0
    dz = z1 - z0
    horizontal = math.hypot(dx, dy)
    comprimento = math.hypot(horizontal, dz)
    yaw = 0.0 if horizontal < 1e-9 else math.atan2(dy, dx)
    pitch = math.atan2(-dz, horizontal)
    mx = 0.5 * (x0 + x1)
    my = 0.5 * (y0 + y1)
    mz = 0.5 * (z0 + z1)
    return f'''
      <link name="{nome}">
        <pose>{mx} {my} {mz} 0 {pitch} {yaw}</pose>
        <visual name="visual">
          <pose>0 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.006</radius><length>{comprimento}</length></cylinder></geometry>
          <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material>
        </visual>
      </link>'''


def _cabo_catenaria_xml(p, ax, ay, az, tx, ty, tz, segmentos=32):
    horizontal_total = math.hypot(tx - ax, ty - ay)
    if horizontal_total < 1e-9:
        return _cabo_reto_xml(p, ax, ay, az, tx, ty, tz)

    ux = (tx - ax) / horizontal_total
    uy = (ty - ay) / horizontal_total
    curva = parametros_catenaria(horizontal_total, tz - az, p['cabo_comprimento'], az)
    z = curva['z']
    links = []
    for i in range(segmentos):
        u0 = horizontal_total * i / segmentos
        u1 = horizontal_total * (i + 1) / segmentos
        p0 = (ax + ux * u0, ay + uy * u0, z(u0))
        p1 = (ax + ux * u1, ay + uy * u1, z(u1))
        nome = 'cabo_dinamico_final_segment' if i == segmentos - 1 else f'cabo_dinamico_segment_{i + 1}'
        links.append(_segmento_cabo_xml(nome, p0, p1))
    return ''.join(links)


def _cabo_articulado_xml(p, ax, ay, az):
    share_drone = get_package_share_directory('pacote_do_drone')
    cabo_sdf_path = os.path.join(share_drone, 'models', 'cabo.sdf')
    root = ET.parse(cabo_sdf_path).getroot()
    model = root.find('model')
    if model is None:
        raise RuntimeError(f'Nenhum <model> encontrado em {cabo_sdf_path}')

    model.set('name', 'cabo_dinamico')

    for tag in ('static', 'pose'):
        existing = model.find(tag)
        if existing is not None:
            model.remove(existing)

    static = ET.Element('static')
    static.text = 'true'
    pose = ET.Element('pose')
    pose.text = f"{ax} {ay} {az} 0 {p['pitch_cabo']} {p['yaw_cabo']}"

    plugin = ET.Element(
        'plugin',
        {
            'filename': 'gz-sim-pose-publisher-system',
            'name': 'gz::sim::systems::PosePublisher',
        },
    )
    for tag, value in (
        ('publish_link_pose', 'true'),
        ('publish_model_pose', 'true'),
        ('use_pose_vector_msg', 'true'),
        ('static_publisher', 'true'),
        ('static_update_frequency', '5'),
        ('update_frequency', '50'),
    ):
        child = ET.SubElement(plugin, tag)
        child.text = value

    model.insert(0, plugin)
    model.insert(0, pose)
    model.insert(0, static)

    return ET.tostring(model, encoding='unicode')


def _sistema_cabo_articulado_xml(p, ax, ay, az):
    return f'''
    <model name="sistema_cabo_articulado">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <publish_model_pose>true</publish_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <static_publisher>true</static_publisher>
        <static_update_frequency>5</static_update_frequency>
        <update_frequency>50</update_frequency>
      </plugin>
{_cabo_articulado_xml(p, ax, ay, az)}
    </model>'''


def gerar_world(caso, modo_cabo='reto', config_path=None):
    modo_cabo = modo_cabo.lower()
    if modo_cabo not in MODOS_CABO:
        raise ValueError(f'modo_cabo invalido "{modo_cabo}". Use: {", ".join(MODOS_CABO)}')

    geometria = 'catenaria' if modo_cabo == 'catenaria' else 'reta'
    p = parametros_caso(caso, config_path, geometria=geometria)
    ax, ay, az = p['ancora']
    px, py = p['poste_xy']
    tx, ty, tz = p['poste_topo']
    cabo_xml = _cabo_reto_xml(p, ax, ay, az, tx, ty, tz)
    cabo_articulado_xml = ''
    if modo_cabo == 'articulado':
        cabo_xml = ''
        cabo_articulado_xml = _sistema_cabo_articulado_xml(p, ax, ay, az)
    elif modo_cabo == 'catenaria':
        cabo_xml = _cabo_catenaria_xml(p, ax, ay, az, tx, ty, tz)

    return f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="cabo_avaliacao">
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>8 8</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>8 8</size></plane></geometry>
          <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="bancada_cabo">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <publish_model_pose>true</publish_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <static_publisher>true</static_publisher>
        <static_update_frequency>5</static_update_frequency>
        <update_frequency>50</update_frequency>
      </plugin>

      <link name="referencial_global">
        <pose>0 0 0.04 0 0 0</pose>
        <visual name="origem">
          <geometry><sphere><radius>0.035</radius></sphere></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
        <visual name="x_frente">
          <pose>0.35 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.012</radius><length>0.7</length></cylinder></geometry>
          <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material>
        </visual>
        <visual name="y_esquerda">
          <pose>0 0.35 0 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.012</radius><length>0.7</length></cylinder></geometry>
          <material><ambient>0.05 0.7 0.05 1</ambient><diffuse>0.05 0.7 0.05 1</diffuse></material>
        </visual>
        <visual name="z_cima">
          <pose>0 0 0.35 0 0 0</pose>
          <geometry><cylinder><radius>0.012</radius><length>0.7</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.9 1</ambient><diffuse>0.05 0.05 0.9 1</diffuse></material>
        </visual>
      </link>

      <link name="ancora_chao">
        <pose>{ax} {ay} {az} 0 0 0</pose>
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia></inertial>
        <visual name="visual"><geometry><sphere><radius>0.04</radius></sphere></geometry><material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material></visual>
        <collision name="collision"><geometry><sphere><radius>0.04</radius></sphere></geometry></collision>
      </link>

      <link name="poste">
        <pose>{px} {py} {tz / 2.0} 0 0 0</pose>
        <inertial><mass>5</mass><inertia><ixx>0.5</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.5</iyy><iyz>0</iyz><izz>0.02</izz></inertia></inertial>
        <visual name="coluna"><geometry><cylinder><radius>0.035</radius><length>{tz}</length></cylinder></geometry><material><ambient>0.1 0.1 0.8 1</ambient><diffuse>0.1 0.1 0.8 1</diffuse></material></visual>
        <collision name="collision"><geometry><cylinder><radius>0.035</radius><length>{tz}</length></cylinder></geometry></collision>
      </link>

      <link name="sensor_cabo">
        <pose>{tx} {ty} {tz} 0 0 {p['sensor_yaw']}</pose>
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia></inertial>
        <visual name="corpo"><geometry><sphere><radius>0.05</radius></sphere></geometry><material><ambient>0.05 0.8 0.2 1</ambient><diffuse>0.05 0.8 0.2 1</diffuse></material></visual>
        <visual name="eixo_x">
          <pose>0.08 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.006</radius><length>0.16</length></cylinder></geometry>
          <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material>
        </visual>
        <visual name="eixo_y">
          <pose>0 0.08 0 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.006</radius><length>0.16</length></cylinder></geometry>
          <material><ambient>0.05 0.7 0.05 1</ambient><diffuse>0.05 0.7 0.05 1</diffuse></material>
        </visual>
        <visual name="eixo_z">
          <pose>0 0 0.08 0 0 0</pose>
          <geometry><cylinder><radius>0.006</radius><length>0.16</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.9 1</ambient><diffuse>0.05 0.05 0.9 1</diffuse></material>
        </visual>
        <collision name="collision"><geometry><sphere><radius>0.05</radius></sphere></geometry></collision>
      </link>

      <joint name="fixa_sensor_poste" type="fixed">
        <parent>poste</parent>
        <child>sensor_cabo</child>
      </joint>
{cabo_xml}
    </model>
{cabo_articulado_xml}
  </world>
</sdf>
'''


def escrever_world(caso, saida_dir=None, modo_cabo='reto', config_path=None):
    saida = Path(saida_dir or '/tmp/cabo_avaliacao_worlds')
    saida.mkdir(parents=True, exist_ok=True)
    world_path = saida / f'cabo_poste_{caso.lower()}_{modo_cabo.lower()}.sdf'
    world_path.write_text(gerar_world(caso, modo_cabo, config_path))
    return str(world_path)


def escrever_tabela_esperada(saida_dir=None, config_path=None, modo_cabo='reto'):
    saida = Path(saida_dir or '/tmp/cabo_avaliacao_worlds')
    saida.mkdir(parents=True, exist_ok=True)
    tabela_path = saida / 'angulos_esperados.csv'
    with tabela_path.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'caso',
            'poste_x',
            'poste_y',
            'poste_z',
            'sensor_x_frente',
            'sensor_y_esquerda',
            'azimuth_deg',
            'elevation_deg',
            'config_path',
        ])
        geometria = 'catenaria' if modo_cabo == 'catenaria' else 'reta'
        for caso in nomes_casos(config_path):
            p = parametros_caso(caso, config_path, geometria=geometria)
            tx, ty, tz = p['poste_topo']
            writer.writerow([
                caso,
                f'{tx:.6f}',
                f'{ty:.6f}',
                f'{tz:.6f}',
                'norte',
                'oeste',
                f'{p["azimuth_esperado_graus"]:.6f}',
                f'{p["elevation_esperado_graus"]:.6f}',
                p['config_path'],
            ])
    return str(tabela_path)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--caso', default='e')
    parser.add_argument('--modo-cabo', choices=MODOS_CABO, default='reto')
    parser.add_argument('--config', default=None)
    parser.add_argument('--saida', default='/tmp/cabo_avaliacao_worlds')
    parser.add_argument('--todos', action='store_true')
    parsed = parser.parse_args(args)

    casos = nomes_casos(parsed.config) if parsed.todos else [parsed.caso]
    for caso in casos:
        print(escrever_world(caso, parsed.saida, parsed.modo_cabo, parsed.config))
    print(escrever_tabela_esperada(parsed.saida, parsed.config, parsed.modo_cabo))


if __name__ == '__main__':
    main()
