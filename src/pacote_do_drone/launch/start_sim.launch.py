import os
import json # <-- Adicionado
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_launch(context, nome):
    return LaunchConfiguration(nome).perform(context).strip().lower() in ('1', 'true', 'sim', 'yes', 'on')


def _float_launch(context, nome, default):
    valor = LaunchConfiguration(nome).perform(context).strip()
    return default if valor == '' else float(valor)


def _int_launch(context, nome, default):
    valor = LaunchConfiguration(nome).perform(context).strip()
    return default if valor == '' else int(valor)


def _payload_xml(massa):
    return f'''
      <link name="payload_teste">
        <pose relative_to="meu_drone::base_link">0 0 0 0 0 0</pose>
        <inertial>
          <mass>{massa}</mass>
          <inertia><ixx>0.0005</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0005</iyy><iyz>0</iyz><izz>0.0005</izz></inertia>
        </inertial>
        <visual name="visual">
          <geometry><sphere><radius>0.035</radius></sphere></geometry>
          <material><ambient>0.1 0.8 0.1 1</ambient><diffuse>0.1 0.8 0.1 1</diffuse></material>
        </visual>
      </link>
      <joint name="payload_teste_joint" type="fixed">
        <parent>meu_drone::base_link</parent>
        <child>payload_teste</child>
      </joint>'''


def _pendulo_xml(n_links, massa_total, comprimento_total, attach_link):
    n_links = max(1, n_links)
    massa_link = massa_total / n_links
    comprimento_link = comprimento_total / n_links
    ixx = (1.0 / 12.0) * massa_link * comprimento_link * comprimento_link
    xml = ''
    parent = f'meu_drone::{attach_link}'
    for i in range(n_links):
        nome = f'pendulo_link_{i + 1}'
        joint = f'pendulo_joint_{i + 1}'
        pose_link = f'0 0 -{0.5 * comprimento_link} 0 0 0' if i == 0 else f'0 0 -{comprimento_link} 0 0 0'
        pose_joint = '0 0 0 0 0 0' if i == 0 else f'0 0 -{0.5 * comprimento_link} 0 0 0'
        joint_type = 'ball' if i == 0 else 'revolute'
        axis_xml = ''
        if i > 0:
            axis_xml = '''
        <axis>
          <xyz>0 1 0</xyz>
          <limit><lower>-3.14159</lower><upper>3.14159</upper><effort>5</effort><velocity>10</velocity></limit>
          <dynamics><damping>0.02</damping><friction>0.001</friction></dynamics>
        </axis>'''
        sensor_xml = ''
        if i == 0:
            sensor_xml = '''
        <sensor name="sensor_pendulo_conexao" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/conexao_drone</topic>
        </sensor>'''
        xml += f'''
      <link name="{nome}">
        <pose relative_to="{parent}">{pose_link}</pose>
        <inertial>
          <pose>0 0 0 0 0 0</pose>
          <mass>{massa_link}</mass>
          <inertia><ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{ixx}</iyy><iyz>0</iyz><izz>1e-5</izz></inertia>
        </inertial>
        <visual name="visual">
          <geometry><cylinder><radius>0.006</radius><length>{comprimento_link}</length></cylinder></geometry>
          <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.006</radius><length>{comprimento_link}</length></cylinder></geometry>
        </collision>
      </link>
      <joint name="{joint}" type="{joint_type}">
        <pose relative_to="{parent}">{pose_joint}</pose>
        <parent>{parent}</parent>
        <child>{nome}</child>{axis_xml}{sensor_xml}
      </joint>'''
        parent = nome
    return xml


def _criar_mundo_diagnostico(context, pkg_share, params):
    models_path = os.path.join(pkg_share, 'models')
    caminho_drone_sdf = os.path.join(models_path, 'meu_drone', 'meu_drone.sdf')
    caminho_cabo_sdf = os.path.join(models_path, 'cabo.sdf')
    caminho_world = os.path.join(pkg_share, 'worlds', 'my_world_diagnostico.sdf')

    num_links = int(params.get('num_links', 40))
    length = float(params.get('length', 0.05))
    comprimento_total = num_links * length
    ancora_x = float(params.get('anchor_x', 0.0))
    ancora_y = float(params.get('anchor_y', 0.0))
    ancora_z = float(params.get('anchor_z', 0.33))
    drone_x = float(params.get('drone_x', 1.5))
    drone_y = float(params.get('drone_y', 0.0))
    drone_z = float(params.get('drone_z', ancora_z))

    yaw_base = math.atan2(drone_y - ancora_y, drone_x - ancora_x)
    if all(k in params for k in ('initial_end_x', 'initial_end_y', 'initial_end_z')):
        spawn_x_default = float(params['initial_end_x'])
        spawn_y_default = float(params['initial_end_y'])
        spawn_z_default = float(params['initial_end_z'])
        yaw_base = math.atan2(spawn_y_default - ancora_y, spawn_x_default - ancora_x)
    else:
        spawn_x_default = ancora_x + comprimento_total * math.cos(yaw_base)
        spawn_y_default = ancora_y + comprimento_total * math.sin(yaw_base)
        spawn_z_default = drone_z if 'drone_z' in params else ancora_z

    spawn_x = _float_launch(context, 'spawn_x', spawn_x_default)
    spawn_y = _float_launch(context, 'spawn_y', spawn_y_default)
    spawn_z = _float_launch(context, 'spawn_z', spawn_z_default)
    spawn_yaw = _float_launch(context, 'spawn_yaw', yaw_base)
    usar_cabo = _bool_launch(context, 'usar_cabo')
    prender_ancora = _bool_launch(context, 'prender_ancora')
    tether_mode = LaunchConfiguration('tether_mode').perform(context).strip().lower()
    tether_attach_link = LaunchConfiguration('tether_attach_link').perform(context).strip()
    arquitetura_modelos = LaunchConfiguration('arquitetura_modelos').perform(context).strip().lower()
    if arquitetura_modelos not in ('nested', 'world'):
        print(f"AVISO: arquitetura_modelos={arquitetura_modelos!r} invalida. Usando nested.")
        arquitetura_modelos = 'nested'
    if tether_attach_link not in ('cabo_sensor_link', 'base_link'):
        print(f"AVISO: tether_attach_link={tether_attach_link!r} invalido. Usando cabo_sensor_link.")
        tether_attach_link = 'cabo_sensor_link'
    conexao_cabo_drone = str(params.get('connection_type', 'fixed')).strip().lower()
    if conexao_cabo_drone not in ('fixed', 'ball'):
        print(f"AVISO: connection_type={conexao_cabo_drone!r} invalido. Usando fixed.")
        conexao_cabo_drone = 'fixed'
    if tether_mode not in ('completo', 'desacoplado', 'carga', 'pendulo'):
        print(f"AVISO: tether_mode={tether_mode!r} invalido. Usando completo.")
        tether_mode = 'completo'

    cabo_xml = ''
    juntas_cabo_xml = ''
    diagnostico_xml = ''
    if tether_mode == 'carga':
        diagnostico_xml = _payload_xml(_float_launch(context, 'massa_diagnostico', 0.30))
    elif tether_mode == 'pendulo':
        diagnostico_xml = _pendulo_xml(
            _int_launch(context, 'pendulo_links', 1),
            _float_launch(context, 'massa_diagnostico', 0.30),
            _float_launch(context, 'comprimento_diagnostico', 0.50),
            tether_attach_link,
        )

    if usar_cabo and tether_mode in ('completo', 'desacoplado'):
        junta_ancora_xml = ''
        if prender_ancora:
            junta_ancora_xml = '''
      <joint name="ancora_carretel_cabo" type="fixed">
        <parent>ancora_cabo</parent>
        <child>cabo_dinamico::raiz_cabo</child>
      </joint>'''
        cabo_xml = f'''
      <include>
        <uri>file://{caminho_cabo_sdf}</uri>
        <name>cabo_dinamico</name>
        <pose>{ancora_x} {ancora_y} {ancora_z} 0 0 {yaw_base}</pose>
        <static>false</static>
      </include>'''
        if tether_mode == 'desacoplado':
            juntas_cabo_xml = f'''
{junta_ancora_xml}'''
        else:
            juntas_cabo_xml = f'''
{junta_ancora_xml}
      <joint name="cabo_drone_joint" type="fixed">
        <parent>cabo_dinamico::ponta_cabo</parent>
        <child>meu_drone::{tether_attach_link}</child>
        <sensor name="sensor_conexao_drone" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/conexao_drone</topic>
        </sensor>
      </joint>'''
        if tether_mode != 'desacoplado' and conexao_cabo_drone == 'ball':
            juntas_cabo_xml = f'''
{junta_ancora_xml}
      <joint name="cabo_drone_joint" type="ball">
        <parent>cabo_dinamico::ponta_cabo</parent>
        <child>meu_drone::{tether_attach_link}</child>
        <sensor name="sensor_conexao_drone" type="force_torque">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <topic>/cabo/conexao_drone</topic>
        </sensor>
      </joint>'''
    if arquitetura_modelos == 'world':
        juntas_cabo_xml = juntas_cabo_xml.replace(
            '<parent>ancora_cabo</parent>',
            '<parent>ancora_model::ancora_cabo</parent>',
        )

    ancora_link_xml = f'''
      <link name="ancora_cabo">
        <pose>{ancora_x} {ancora_y} {ancora_z} 0 0 0</pose>
        <inertial>
          <mass>10</mass>
          <inertia><ixx>1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1</iyy><iyz>0</iyz><izz>1</izz></inertia>
        </inertial>
        <visual name="visual">
          <geometry><sphere><radius>0.015</radius></sphere></geometry>
          <material><ambient>0.9 0.05 0.05 1</ambient><diffuse>0.9 0.05 0.05 1</diffuse></material>
        </visual>
      </link>'''
    drone_include_xml = f'''
      <include>
        <uri>file://{caminho_drone_sdf}</uri>
        <name>meu_drone</name>
        <pose>{spawn_x} {spawn_y} {spawn_z} 0 0 {spawn_yaw}</pose>
      </include>'''
    if arquitetura_modelos == 'world':
        if diagnostico_xml:
            print('AVISO: arquitetura_modelos=world ignora modos carga/pendulo nesta versao diagnostica.')
        corpo_simulacao_xml = f'''
    <model name="ancora_model">
      <static>true</static>
{ancora_link_xml}
    </model>
{cabo_xml}
{drone_include_xml}
{juntas_cabo_xml}'''
    else:
        corpo_simulacao_xml = f'''
    <model name="sistema_cabo_drone">
      <pose>0 0 0 0 0 0</pose>
{ancora_link_xml}

      <joint name="fixa_ancora_cabo_mundo" type="fixed">
        <parent>world</parent>
        <child>ancora_cabo</child>
      </joint>
{cabo_xml}
{drone_include_xml}
{diagnostico_xml}
{juntas_cabo_xml}
    </model>'''

    world = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="mundo_ic">
    <physics name="fast" type="ignored">
      <max_step_size>0.0004</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="gz-sim-forcetorque-system" name="gz::sim::systems::ForceTorque"/>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

{corpo_simulacao_xml}
  </world>
</sdf>
'''

    with open(caminho_world, 'w') as f:
        f.write(world)

    print(
        'Mundo diagnostico: '
        f'usar_cabo={usar_cabo}, spawn=({spawn_x:.2f}, {spawn_y:.2f}, {spawn_z:.2f}, yaw={spawn_yaw:.2f}), '
        f'conexao_cabo_drone={conexao_cabo_drone}, '
        f'prender_ancora={prender_ancora}, '
        f'tether_mode={tether_mode}, tether_attach_link={tether_attach_link}, '
        f'arquitetura_modelos={arquitetura_modelos}, '
        f'arquivo={caminho_world}'
    )
    gz_args = f'{caminho_world} -v4 -r'
    if _bool_launch(context, 'headless'):
        gz_args += ' -s'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': gz_args}.items()
    )
    return [gazebo]


def generate_launch_description():
    pkg_share = get_package_share_directory('pacote_do_drone')
    models_path = os.path.join(pkg_share, 'models')

    caminho_json = os.path.join(pkg_share, 'tether_parameters.json')
    
    try:
        with open(caminho_json, 'r') as f:
            params = json.load(f)
        # Multiplica quantidade de elos pelo tamanho de cada um
        tamanho_total_cabo = params["num_links"] * params["length"]
        # Usa 85% do tamanho do cabo para dar uma "barriga" (catenária) natural e evitar tensão infinita
        altura_x = str(tamanho_total_cabo * 0.85)
        print(f"Calculado altura_x automático: {altura_x}m")
    except FileNotFoundError:
        print("AVISO: tether_parameters.json não encontrado. Usando z=0.3 como segurança.")
        altura_x = '0.3'
    # ------------------------------------------------

    gazebo = OpaqueFunction(function=lambda context: _criar_mundo_diagnostico(context, pkg_share, params))


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cabo/tensao_drone@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/cabo/tensao_carretel@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/cabo/conexao_drone@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/meu_drone/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/meu_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/world/mundo_ic/model/sistema_cabo_drone/model/meu_drone/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/mundo_ic/model/sistema_cabo_drone/model/cabo_dinamico/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/mundo_ic/model/meu_drone/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/mundo_ic/model/cabo_dinamico/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/mundo_ic/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    sensores = Node(
        package='pacote_do_drone',
        executable='sensores',
        parameters=[{
            'janela_tangente_links': ParameterValue(LaunchConfiguration('janela_tangente_links'), value_type=int),
        }],
        output='screen'
    )

    controlador_trajetoria = Node(
        package='pacote_do_drone',
        executable='movimento_circular',
        condition=IfCondition(LaunchConfiguration('controlador_trajetoria')),
        parameters=[{
            'centro_x': ParameterValue(LaunchConfiguration('centro_x'), value_type=float),
            'centro_y': ParameterValue(LaunchConfiguration('centro_y'), value_type=float),
            'altura': ParameterValue(LaunchConfiguration('altura_trajetoria'), value_type=float),
            'waypoints_file': ParameterValue(LaunchConfiguration('waypoints_file'), value_type=str),
            'waypoints': ParameterValue(LaunchConfiguration('waypoints'), value_type=str),
            'tolerancia_posicao': ParameterValue(LaunchConfiguration('tolerancia_posicao'), value_type=float),
            'tolerancia_altura': ParameterValue(LaunchConfiguration('tolerancia_altura'), value_type=float),
            'histerese_chegada': ParameterValue(LaunchConfiguration('histerese_chegada'), value_type=float),
            'tempo_estabilizacao': ParameterValue(LaunchConfiguration('tempo_estabilizacao'), value_type=float),
            'tempo_hover': ParameterValue(LaunchConfiguration('tempo_hover'), value_type=float),
            'repetir': ParameterValue(LaunchConfiguration('repetir'), value_type=bool),
            'controlar_heading': ParameterValue(LaunchConfiguration('controlar_heading'), value_type=bool),
            'ganho_posicao_xy': ParameterValue(LaunchConfiguration('ganho_posicao_xy'), value_type=float),
            'ganho_altura': ParameterValue(LaunchConfiguration('ganho_altura'), value_type=float),
            'ganho_integral_xy': ParameterValue(LaunchConfiguration('ganho_integral_xy'), value_type=float),
            'ganho_integral_z': ParameterValue(LaunchConfiguration('ganho_integral_z'), value_type=float),
            'ganho_velocidade_xy': ParameterValue(LaunchConfiguration('ganho_velocidade_xy'), value_type=float),
            'ganho_velocidade_z': ParameterValue(LaunchConfiguration('ganho_velocidade_z'), value_type=float),
            'limite_vel_xy': ParameterValue(LaunchConfiguration('limite_vel_xy'), value_type=float),
            'limite_vel_z': ParameterValue(LaunchConfiguration('limite_vel_z'), value_type=float),
            'tolerancia_velocidade': ParameterValue(LaunchConfiguration('tolerancia_velocidade'), value_type=float),
            'heading_fixo': ParameterValue(LaunchConfiguration('heading_fixo'), value_type=float),
            'cmd_vel_frame': ParameterValue(LaunchConfiguration('cmd_vel_frame'), value_type=str),
            'odom_twist_frame': ParameterValue(LaunchConfiguration('odom_twist_frame'), value_type=str),
            'usar_velocidade_por_diferenca': ParameterValue(LaunchConfiguration('usar_velocidade_por_diferenca'), value_type=bool),
            'filtro_velocidade': ParameterValue(LaunchConfiguration('filtro_velocidade'), value_type=float),
            'log_periodo': ParameterValue(LaunchConfiguration('log_periodo'), value_type=float),
        }],
        output='screen',
    )

    velocity_test = Node(
        package='pacote_do_drone',
        executable='velocity_test',
        condition=IfCondition(LaunchConfiguration('velocity_test')),
        parameters=[{
            'vx_cmd': ParameterValue(LaunchConfiguration('vx_cmd'), value_type=float),
            'vy_cmd': ParameterValue(LaunchConfiguration('vy_cmd'), value_type=float),
            'vz_cmd': ParameterValue(LaunchConfiguration('vz_cmd'), value_type=float),
            'yaw_rate_cmd': ParameterValue(LaunchConfiguration('yaw_rate_cmd'), value_type=float),
            'duracao': ParameterValue(LaunchConfiguration('velocity_test_duracao'), value_type=float),
            'log_periodo': ParameterValue(LaunchConfiguration('log_periodo'), value_type=float),
        }],
        output='screen',
    )

    hover_metrics = Node(
        package='pacote_do_drone',
        executable='hover_metrics',
        condition=IfCondition(LaunchConfiguration('hover_metrics')),
        parameters=[{
            'target_x': ParameterValue(LaunchConfiguration('metricas_target_x'), value_type=float),
            'target_y': ParameterValue(LaunchConfiguration('metricas_target_y'), value_type=float),
            'target_z': ParameterValue(LaunchConfiguration('metricas_target_z'), value_type=float),
            'inicio_s': ParameterValue(LaunchConfiguration('metricas_inicio_s'), value_type=float),
            'duracao_s': ParameterValue(LaunchConfiguration('metricas_duracao_s'), value_type=float),
            'janela_final_s': ParameterValue(LaunchConfiguration('metricas_janela_final_s'), value_type=float),
            'log_periodo_s': ParameterValue(LaunchConfiguration('metricas_log_periodo_s'), value_type=float),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'controlador_trajetoria',
            default_value='false',
            description='Inicia o controlador de sequencia de waypoints para testes do cabo.',
        ),
        DeclareLaunchArgument(
            'usar_cabo',
            default_value='true',
            description='Inclui o cabo dinamico e as juntas de ancoragem. Use false para diagnostico do controlador sem tether.',
        ),
        DeclareLaunchArgument(
            'prender_ancora',
            default_value='true',
            description='Fixa a raiz do cabo na ancora. Use false para diagnostico com tether livre conectado ao drone.',
        ),
        DeclareLaunchArgument(
            'tether_mode',
            default_value='completo',
            description='Modo diagnostico do tether: completo, desacoplado, carga ou pendulo.',
        ),
        DeclareLaunchArgument(
            'tether_attach_link',
            default_value='cabo_sensor_link',
            description='Link do drone usado pela junta do tether: cabo_sensor_link ou base_link.',
        ),
        DeclareLaunchArgument(
            'arquitetura_modelos',
            default_value='nested',
            description='Arquitetura diagnostica: nested mantem sistema_cabo_drone; world coloca drone/cabo/ancora como modelos do mundo.',
        ),
        DeclareLaunchArgument(
            'massa_diagnostico',
            default_value='0.30',
            description='Massa total para modos diagnosticos carga/pendulo, em kg.',
        ),
        DeclareLaunchArgument(
            'comprimento_diagnostico',
            default_value='0.50',
            description='Comprimento total para o pendulo diagnostico, em metros.',
        ),
        DeclareLaunchArgument(
            'pendulo_links',
            default_value='1',
            description='Numero de links do pendulo diagnostico.',
        ),
        DeclareLaunchArgument(
            'velocity_test',
            default_value='false',
            description='Publica cmd_vel constante para diagnostico aberto do MulticopterVelocityControl.',
        ),
        DeclareLaunchArgument(
            'vx_cmd',
            default_value='0.0',
            description='Comando linear X usado por velocity_test.',
        ),
        DeclareLaunchArgument(
            'vy_cmd',
            default_value='0.0',
            description='Comando linear Y usado por velocity_test.',
        ),
        DeclareLaunchArgument(
            'vz_cmd',
            default_value='0.25',
            description='Comando linear Z usado por velocity_test.',
        ),
        DeclareLaunchArgument(
            'yaw_rate_cmd',
            default_value='0.0',
            description='Comando angular Z usado por velocity_test.',
        ),
        DeclareLaunchArgument(
            'velocity_test_duracao',
            default_value='8.0',
            description='Duracao em segundos do comando constante do velocity_test.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Executa apenas o servidor do Gazebo, sem GUI, para testes automatizados.',
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='',
            description='Override diagnostico da posicao inicial X do drone. Vazio usa o spawn calculado pelo cabo.',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='',
            description='Override diagnostico da posicao inicial Y do drone. Vazio usa o spawn calculado pelo cabo.',
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='',
            description='Override diagnostico da posicao inicial Z do drone. Vazio usa o spawn calculado pelo cabo.',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='',
            description='Override diagnostico do yaw inicial do drone. Vazio usa o yaw calculado pelo cabo.',
        ),
        DeclareLaunchArgument(
            'centro_x',
            default_value='0.0',
            description='Coordenada X do centro de referencia da trajetoria, em metros.',
        ),
        DeclareLaunchArgument(
            'centro_y',
            default_value='0.0',
            description='Coordenada Y do centro de referencia da trajetoria, em metros.',
        ),
        DeclareLaunchArgument(
            'altura_trajetoria',
            default_value='1.6',
            description='Altura padrao dos waypoints quando Z nao e informado.',
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='config/trajetoria_hover_ancora.json',
            description='Arquivo JSON com a sequencia de waypoints. Caminho relativo ao share do pacote ou absoluto.',
        ),
        DeclareLaunchArgument(
            'waypoints',
            default_value='',
            description='Lista JSON alternativa de waypoints, por exemplo [[0.8,0,1.6],[-0.8,0,1.6]].',
        ),
        DeclareLaunchArgument(
            'tolerancia_posicao',
            default_value='0.18',
            description='Erro XY maximo para considerar que o drone chegou ao waypoint.',
        ),
        DeclareLaunchArgument(
            'tolerancia_altura',
            default_value='0.15',
            description='Erro Z maximo para considerar que o drone chegou ao waypoint.',
        ),
        DeclareLaunchArgument(
            'histerese_chegada',
            default_value='1.6',
            description='Multiplicador da tolerancia para nao resetar hovering por pequeno overshoot.',
        ),
        DeclareLaunchArgument(
            'tempo_hover',
            default_value='10.0',
            description='Tempo em hovering no waypoint antes de avancar.',
        ),
        DeclareLaunchArgument(
            'tempo_estabilizacao',
            default_value='1.0',
            description='Tempo simulado continuo dentro das tolerancias antes de iniciar a contagem de hovering.',
        ),
        DeclareLaunchArgument(
            'repetir',
            default_value='false',
            description='Repete continuamente a sequencia de waypoints.',
        ),
        DeclareLaunchArgument(
            'controlar_heading',
            default_value='false',
            description='Ativa o controle de yaw para manter heading_fixo.',
        ),
        DeclareLaunchArgument(
            'ganho_posicao_xy',
            default_value='0.8',
            description='Ganho proporcional de posicao XY do controlador externo.',
        ),
        DeclareLaunchArgument(
            'ganho_altura',
            default_value='1.0',
            description='Ganho proporcional de altura do controlador externo.',
        ),
        DeclareLaunchArgument(
            'ganho_integral_xy',
            default_value='0.0',
            description='Ganho integral XY para compensar perturbacoes do cabo.',
        ),
        DeclareLaunchArgument(
            'ganho_integral_z',
            default_value='0.0',
            description='Ganho integral Z para compensar perturbacoes do cabo.',
        ),
        DeclareLaunchArgument(
            'ganho_velocidade_xy',
            default_value='0.8',
            description='Amortecimento por velocidade XY.',
        ),
        DeclareLaunchArgument(
            'ganho_velocidade_z',
            default_value='0.45',
            description='Amortecimento por velocidade Z.',
        ),
        DeclareLaunchArgument(
            'limite_vel_xy',
            default_value='0.6',
            description='Velocidade maxima comandada no plano XY.',
        ),
        DeclareLaunchArgument(
            'limite_vel_z',
            default_value='0.5',
            description='Velocidade maxima comandada em Z.',
        ),
        DeclareLaunchArgument(
            'tolerancia_velocidade',
            default_value='0.15',
            description='Velocidade maxima para contar tempo de hovering.',
        ),
        DeclareLaunchArgument(
            'heading_fixo',
            default_value='0.0',
            description='Yaw alvo fixo do drone, em radianos.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_frame',
            default_value='body',
            description='Frame usado pelo comando de velocidade: world ou body.',
        ),
        DeclareLaunchArgument(
            'odom_twist_frame',
            default_value='body',
            description='Frame da velocidade em /meu_drone/odom: body ou world.',
        ),
        DeclareLaunchArgument(
            'usar_velocidade_por_diferenca',
            default_value='true',
            description='Estima velocidade pela diferenca de posicao para amortecimento.',
        ),
        DeclareLaunchArgument(
            'filtro_velocidade',
            default_value='0.35',
            description='Peso da nova medida no filtro de velocidade por diferenca.',
        ),
        DeclareLaunchArgument(
            'log_periodo',
            default_value='1.0',
            description='Periodo dos logs compactos do controlador, em segundos.',
        ),
        DeclareLaunchArgument(
            'janela_tangente_links',
            default_value='6',
            description='Numero de links usados para estimar a tangente local do cabo no sensor.',
        ),
        DeclareLaunchArgument(
            'hover_metrics',
            default_value='false',
            description='Ativa coleta compacta de metricas de hover e angulos em tempo simulado.',
        ),
        DeclareLaunchArgument('metricas_target_x', default_value='2.0'),
        DeclareLaunchArgument('metricas_target_y', default_value='0.0'),
        DeclareLaunchArgument('metricas_target_z', default_value='1.0'),
        DeclareLaunchArgument('metricas_inicio_s', default_value='0.0'),
        DeclareLaunchArgument('metricas_duracao_s', default_value='10.0'),
        DeclareLaunchArgument('metricas_janela_final_s', default_value='5.0'),
        DeclareLaunchArgument('metricas_log_periodo_s', default_value='2.0'),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),
        gazebo,
        bridge,
        sensores,
        controlador_trajetoria,
        velocity_test,
        hover_metrics,
    ])
