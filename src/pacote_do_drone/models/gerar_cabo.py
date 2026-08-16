import json
import os
import math

base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
caminho_json = os.path.join(base_dir, 'tether_parameters.json')
pasta_models = os.path.join(base_dir, 'models')
pasta_worlds = os.path.join(base_dir, 'worlds')
caminho_sdf = os.path.join(pasta_models, 'cabo.sdf')
caminho_world = os.path.join(pasta_worlds, 'my_world.sdf')
caminho_carretel_sdf = os.path.join(pasta_models, 'carretel', 'carretel.sdf')
caminho_drone_sdf = os.path.join(pasta_models, 'meu_drone', 'meu_drone.sdf')

os.makedirs(pasta_models, exist_ok=True)
os.makedirs(pasta_worlds, exist_ok=True)

with open(caminho_json, 'r') as f:
    params = json.load(f)

num_links = params['num_links']
length = params['length']
radius = params['radius']
mass = params['mass']
dummy_mass = float(params.get('dummy_mass', 0.0001))
root_mass = float(params.get('root_mass', 0.0005))
tip_mass = float(params.get('tip_mass', 0.0005))
drone_x = params['drone_x']
drone_y = params['drone_y']
drone_z = params['drone_z']
initial_shape = str(params.get('initial_shape', 'straight')).lower()
joint_damping = float(params.get('joint_damping', 0.08))
joint_friction = float(params.get('joint_friction', 0.002))
joint_spring_stiffness = float(params.get('joint_spring_stiffness', 0.02))
segment_collision = bool(params.get('segment_collision', True))

comprimento_total = num_links * length
ancora_x = float(params.get('anchor_x', 0.0))
ancora_y = float(params.get('anchor_y', 0.0))
ancora_z = float(params.get('anchor_z', 0.33))

yaw_base = math.atan2(drone_y - ancora_y, drone_x - ancora_x)
initial_end_x = params.get('initial_end_x')
initial_end_y = params.get('initial_end_y')
initial_end_z = params.get('initial_end_z')
if initial_end_x is not None and initial_end_y is not None and initial_end_z is not None:
    drone_spawn_x = float(initial_end_x)
    drone_spawn_y = float(initial_end_y)
    drone_spawn_z = float(initial_end_z)
    yaw_base = math.atan2(drone_spawn_y - ancora_y, drone_spawn_x - ancora_x)
else:
    spawn_distance = comprimento_total
    drone_spawn_x = ancora_x + spawn_distance * math.cos(yaw_base)
    drone_spawn_y = ancora_y + spawn_distance * math.sin(yaw_base)
    drone_spawn_z = ancora_z

span_horizontal = math.hypot(drone_spawn_x - ancora_x, drone_spawn_y - ancora_y)
span_vertical = drone_spawn_z - ancora_z
distancia_extremos = math.hypot(span_horizontal, span_vertical)


def _comprimento_poli_horizontal(amplitude):
    comprimento = 0.0
    anterior = (0.0, 0.0, 0.0)
    for i in range(1, num_links + 1):
        s = i / num_links
        ponto = (
            span_horizontal * s,
            amplitude * math.sin(math.pi * s),
            span_vertical * s,
        )
        comprimento += math.dist(anterior, ponto)
        anterior = ponto
    return comprimento


def _resolver_amplitude_horizontal():
    if comprimento_total + 1e-9 < distancia_extremos:
        raise ValueError(
            f'Cabo curto demais: L={comprimento_total:.4f} m < distancia={distancia_extremos:.4f} m'
        )
    if abs(comprimento_total - distancia_extremos) < 1e-9:
        return 0.0

    baixo = 0.0
    alto = max(0.1, span_horizontal)
    while _comprimento_poli_horizontal(alto) < comprimento_total:
        alto *= 2.0
        if alto > 100.0:
            raise ValueError('Nao foi possivel encontrar amplitude horizontal para o cabo.')

    for _ in range(80):
        meio = 0.5 * (baixo + alto)
        if _comprimento_poli_horizontal(meio) < comprimento_total:
            baixo = meio
        else:
            alto = meio

    return 0.5 * (baixo + alto)


def _calcular_geometria_inicial():
    if initial_shape in ('straight', 'reto'):
        theta = math.atan2(-span_vertical, span_horizontal) if distancia_extremos > 1e-9 else 0.0
        pontos = [(0.0, 0.0, 0.0)]
        for i in range(1, num_links + 1):
            s = i / num_links
            pontos.append((span_horizontal * s, 0.0, span_vertical * s))
        return pontos, 'reta', 0.0

    if initial_shape not in ('sine', 'senoidal', 'sine_slack', 'arco', 'horizontal_sine', 'senoidal_xy'):
        raise ValueError(f'initial_shape invalido: {initial_shape}')

    if span_horizontal <= 1e-9:
        raise ValueError('initial_shape horizontal requer distancia horizontal nao nula.')

    amplitude = _resolver_amplitude_horizontal()
    pontos = []
    for i in range(num_links + 1):
        s = i / num_links
        pontos.append((
            span_horizontal * s,
            amplitude * math.sin(math.pi * s),
            span_vertical * s,
        ))

    descricao = f'senoidal horizontal amplitude_y={amplitude:.4f} m'
    if amplitude == 0.0:
        descricao = 'senoidal horizontal sem folga'
    return pontos, descricao, amplitude


posicoes_iniciais, descricao_inicial, amplitude_lateral = _calcular_geometria_inicial()
segment_lengths = []
pitches_segmentos = []
yaws_segmentos = []
for p0, p1 in zip(posicoes_iniciais[:-1], posicoes_iniciais[1:]):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    horizontal = math.hypot(dx, dy)
    segment_lengths.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    pitches_segmentos.append(math.atan2(-dz, horizontal) if horizontal > 1e-12 else 0.0)
    yaws_segmentos.append(math.atan2(dy, dx) if horizontal > 1e-12 else 0.0)

comprimento_geometrico = sum(segment_lengths)
erro_comprimento = comprimento_geometrico - comprimento_total
erro_fechamento = math.dist(posicoes_iniciais[-1], (span_horizontal, 0.0, span_vertical))
z_min_local = min(p[2] for p in posicoes_iniciais)
z_min_mundo = ancora_z + z_min_local
folga_geometrica = comprimento_total - distancia_extremos
deltas_juntas_y = []
for i, theta in enumerate(pitches_segmentos):
    theta_anterior = 0.0 if i == 0 else pitches_segmentos[i - 1]
    deltas_juntas_y.append(theta - theta_anterior)
max_delta_joint_y = max((abs(delta) for delta in deltas_juntas_y), default=0.0)
max_delta_joint_z = max((abs(delta) for delta in [
    math.atan2(
        math.sin(yaws_segmentos[i] - (0.0 if i == 0 else yaws_segmentos[i - 1])),
        math.cos(yaws_segmentos[i] - (0.0 if i == 0 else yaws_segmentos[i - 1])),
    )
    for i in range(len(yaws_segmentos))
]), default=0.0)

print(f'Alvo original (JSON): ({drone_x:.4f}, {drone_y:.4f}, {drone_z:.4f})')
print(f'Spawn inicial do drone: ({drone_spawn_x:.4f}, {drone_spawn_y:.4f}, {drone_spawn_z:.4f})')
print(f'Cabo: {num_links} elos x {length:.4f} m = {comprimento_total:.4f} m')
print(
    f'Massas do cabo: segmentos={num_links * mass:.4f} kg, '
    f'auxiliares={num_links * dummy_mass + root_mass + tip_mass:.4f} kg, '
    f'total={num_links * mass + num_links * dummy_mass + root_mass + tip_mass:.4f} kg'
)
print(
    f'Inicializacao do cabo: {descricao_inicial}; '
    f'amplitude_lateral={amplitude_lateral:.4f} m; '
    f'comprimento_geom={comprimento_geometrico:.4f} m; erro_comprimento={erro_comprimento:.6f} m; '
    f'dist_extremos={distancia_extremos:.4f} m; folga={folga_geometrica:.4f} m; '
    f'z_min={z_min_mundo:.4f} m; erro_fechamento={erro_fechamento:.6f} m; '
    f'max_delta_joint_y={max_delta_joint_y:.4f} rad; '
    f'max_delta_joint_z={max_delta_joint_z:.4f} rad'
)
print(
    f'Juntas internas: damping={joint_damping:.4f}, friction={joint_friction:.4f}, '
    f'spring={joint_spring_stiffness:.4f}, '
    f'tau_spring_inicial_max={0.0:.4f} Nm, colisoes_segmentos={segment_collision}'
)
indices_debug = sorted(set([0, num_links // 4, num_links // 2, 3 * num_links // 4, num_links]))
for indice in indices_debug:
    px, py, pz = posicoes_iniciais[indice]
    print(f'Ponto inicial cabo[{indice:02d}]: x={px:.4f} y={py:.4f} z={ancora_z + pz:.4f}')
if z_min_mundo < radius:
    print(
        'AVISO: a curva inicial do cabo fica abaixo do raio do cabo em relacao ao solo. '
        f'z_min={z_min_mundo:.4f} m, raio={radius:.4f} m.'
    )

ixx_segment = 0.5 * mass * radius ** 2
iyy_segment = (1.0 / 12.0) * mass * (3 * radius ** 2 + length ** 2)
izz_segment = iyy_segment
joint_limit_rad = max(0.7, max_delta_joint_y, max_delta_joint_z) + 0.3
collision_xml_template = '''
      <collision name="collision">
        <pose>{half_length} 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{collision_length}</length></cylinder></geometry>
      </collision>'''

sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="cabo_flexivel">
    <link name="raiz_cabo">
      <inertial>
        <mass>{root_mass}</mass>
        <inertia>
          <ixx>8e-7</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>8e-7</iyy><iyz>0</iyz><izz>8e-7</izz>
        </inertia>
      </inertial>
      <collision name="col_raiz"><geometry><sphere><radius>0.01</radius></sphere></geometry></collision>
      <visual name="vis_raiz"><geometry><sphere><radius>0.01</radius></sphere></geometry><material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material></visual>
    </link>
'''

parent_link = 'raiz_cabo'
for i in range(1, num_links + 1):
    segment_name = 'final_segment' if i == num_links else f'segment_{i}'
    dummy_name = f'dummy_{i}'
    seg_length = segment_lengths[i - 1]
    parent_offset = 0.0 if i == 1 else segment_lengths[i - 2]
    theta_segmento = pitches_segmentos[i - 1]
    yaw_segmento = yaws_segmentos[i - 1]
    theta_anterior = 0.0 if i == 1 else pitches_segmentos[i - 2]
    yaw_anterior = 0.0 if i == 1 else yaws_segmentos[i - 2]
    pitch_delta = theta_segmento - theta_anterior
    yaw_delta = math.atan2(
        math.sin(yaw_segmento - yaw_anterior),
        math.cos(yaw_segmento - yaw_anterior),
    )

    sensor_xml = ''
    collision_xml = ''
    if segment_collision:
        collision_xml = collision_xml_template.format(
            half_length=seg_length / 2.0,
            radius=radius,
            collision_length=seg_length * 0.9,
        )
    if i == 1:
        sensor_xml = '''
        <sensor name="sensor_tensao_carretel" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/tensao_carretel</topic>
        </sensor>'''
    elif i == num_links:
        sensor_xml = '''
        <sensor name="sensor_tensao_drone" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/tensao_drone</topic>
        </sensor>'''

    sdf += f'''
    <joint name="joint_{i}_y" type="revolute">
      <pose relative_to="{parent_link}">{parent_offset} 0 0 0 0 0</pose>
      <parent>{parent_link}</parent>
      <child>{dummy_name}</child>
      <axis>
        <xyz>0 1 0</xyz>
        <initial_position>{pitch_delta}</initial_position>
        <limit><lower>-{joint_limit_rad}</lower><upper>{joint_limit_rad}</upper><effort>5</effort><velocity>10</velocity></limit>
        <dynamics>
          <damping>{joint_damping}</damping>
          <friction>{joint_friction}</friction>
          <spring_reference>{pitch_delta}</spring_reference>
          <spring_stiffness>{joint_spring_stiffness}</spring_stiffness>
        </dynamics>
      </axis>{sensor_xml}
    </joint>
    <link name="{dummy_name}">
      <pose relative_to="joint_{i}_y">0 0 0 0 0 0</pose>
      <inertial><mass>{dummy_mass}</mass><inertia><ixx>1e-7</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-7</iyy><iyz>0</iyz><izz>1e-7</izz></inertia></inertial>
    </link>
    <joint name="joint_{i}_z" type="revolute">
      <pose relative_to="{dummy_name}">0 0 0 0 0 0</pose>
      <parent>{dummy_name}</parent>
      <child>{segment_name}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <initial_position>{yaw_delta}</initial_position>
        <limit><lower>-{joint_limit_rad}</lower><upper>{joint_limit_rad}</upper><effort>5</effort><velocity>10</velocity></limit>
        <dynamics>
          <damping>{joint_damping}</damping>
          <friction>{joint_friction}</friction>
          <spring_reference>{yaw_delta}</spring_reference>
          <spring_stiffness>{joint_spring_stiffness}</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="{segment_name}">
      <pose relative_to="joint_{i}_z">0 0 0 0 0 0</pose>
      <visual name="visual">
        <pose>{seg_length / 2.0} 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{seg_length}</length></cylinder></geometry>
        <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material>
      </visual>
{collision_xml}
      <inertial>
        <pose>{seg_length / 2.0} 0 0 0 0 0</pose>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx_segment}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy_segment}</iyy><iyz>0</iyz><izz>{izz_segment}</izz>
        </inertia>
      </inertial>
    </link>
'''
    parent_link = segment_name

sdf += '''
    <link name="ponta_cabo">
      <pose relative_to="final_segment">{final_length} 0 0 0 0 0</pose>
      <inertial>
        <mass>{tip_mass}</mass>
        <inertia>
          <ixx>1e-7</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-7</iyy><iyz>0</iyz><izz>1e-7</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><sphere><radius>0.008</radius></sphere></geometry>
        <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material>
      </visual>
    </link>
    <joint name="joint_ponta_cabo" type="fixed">
      <pose relative_to="final_segment">{final_length} 0 0 0 0 0</pose>
      <parent>final_segment</parent>
      <child>ponta_cabo</child>
    </joint>
  </model>
</sdf>
'''.format(
    final_length=segment_lengths[-1] if segment_lengths else length,
    tip_mass=tip_mass,
)

with open(caminho_sdf, 'w') as f:
    f.write(sdf)

print('cabo.sdf gerado com cadeia revoluta estavel.')

world = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="mundo_ic">
    <gravity>0 0 -9.81</gravity>

    <physics name="1ms" type="ignored">
      <max_step_size>0.0004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-forcetorque-system" name="gz::sim::systems::ForceTorque"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
      </link>
    </model>

    <model name="meu_carretel">
      <static>true</static>
      <link name="visual_link">
        <visual name="base_visual">
          <pose>0 0 0.01 0 0 0</pose>
          <geometry><box><size>0.4 0.7 0.02</size></box></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
        <visual name="haste_esq_visual">
          <pose>0 0.23 0.15 0 0 0</pose>
          <geometry><box><size>0.06 0.06 0.30</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.8 1</diffuse></material>
        </visual>
        <visual name="haste_dir_visual">
          <pose>0 -0.23 0.15 0 0 0</pose>
          <geometry><box><size>0.06 0.06 0.30</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.8 1</diffuse></material>
        </visual>
        <visual name="cilindro_visual">
          <pose>0 0 0.26 1.570796 0 0</pose>
          <geometry><cylinder><radius>0.07</radius><length>0.40</length></cylinder></geometry>
          <material><ambient>0.5 0.3 0.3 1</ambient><diffuse>0.5 0.3 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="sistema_cabo_drone">
      <pose>0 0 0 0 0 0</pose>

      <link name="ancora_cabo">
        <pose>{ancora_x} {ancora_y} {ancora_z} 0 0 0</pose>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1</iyy><iyz>0</iyz><izz>1</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry><sphere><radius>0.015</radius></sphere></geometry>
          <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material>
        </visual>
      </link>

      <joint name="fixa_ancora_cabo_mundo" type="fixed">
        <parent>world</parent>
        <child>ancora_cabo</child>
      </joint>

      <include>
        <uri>file://{caminho_sdf}</uri>
        <name>cabo_dinamico</name>
        <pose>{ancora_x} {ancora_y} {ancora_z} 0 0 {yaw_base}</pose>
        <static>false</static>
      </include>

      <include>
        <uri>file://{caminho_drone_sdf}</uri>
        <name>meu_drone</name>
        <pose>{drone_spawn_x} {drone_spawn_y} {drone_spawn_z} 0 0 {yaw_base}</pose>
      </include>

      <joint name="ancora_carretel_cabo" type="fixed">
        <parent>ancora_cabo</parent>
        <child>cabo_dinamico::raiz_cabo</child>
      </joint>

      <joint name="cabo_drone_joint" type="fixed">
        <parent>cabo_dinamico::ponta_cabo</parent>
        <child>meu_drone::cabo_sensor_link</child>
      </joint>
    </model>
  </world>
</sdf>
'''

with open(caminho_world, 'w') as f:
    f.write(world)

print('my_world.sdf atualizado.')
