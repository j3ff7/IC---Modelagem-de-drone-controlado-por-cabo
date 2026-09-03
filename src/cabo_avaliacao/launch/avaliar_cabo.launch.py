import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cabo_avaliacao.gerar_mundos import escrever_tabela_esperada, escrever_world


CASOS_DRONE = {
    'n': {
        'waypoints_file': 'config/trajetoria_sensor_n.json',
        'target': (0.0, 1.0, 2.0),
        'duracao': 34.0,
        'janela_final': 5.0,
    },
    's': {
        'waypoints_file': 'config/trajetoria_sensor_s.json',
        'target': (0.0, -1.0, 2.0),
        'duracao': 34.0,
        'janela_final': 5.0,
    },
    'e': {
        'waypoints_file': 'config/trajetoria_sensor_e.json',
        'target': (1.0, 0.0, 2.0),
        'duracao': 34.0,
        'janela_final': 5.0,
    },
    'w': {
        'waypoints_file': 'config/trajetoria_sensor_w.json',
        'target': (-1.0, 0.0, 2.0),
        'duracao': 34.0,
        'janela_final': 5.0,
    },
    'z060': {
        'waypoints_file': 'config/trajetoria_teste_z060.json',
        'target': (2.0, 0.0, 0.60),
        'duracao': 8.0,
        'janela_final': 2.0,
    },
    'z100': {
        'waypoints_file': 'config/trajetoria_teste_z100.json',
        'target': (2.0, 0.0, 1.00),
        'duracao': 12.0,
        'janela_final': 2.0,
    },
}


def _launch_setup(context, *args, **kwargs):
    caso = LaunchConfiguration('caso').perform(context).lower()
    tipo = LaunchConfiguration('tipo').perform(context).lower()
    if tipo == 'auto' and caso in CASOS_DRONE:
        return _launch_setup_drone(context, caso)
    return _launch_setup_postes(context, caso)


def _launch_setup_drone(context, caso):
    cfg = CASOS_DRONE[caso]
    pkg_drone = get_package_share_directory('pacote_do_drone')
    result_root = LaunchConfiguration('result_root').perform(context).strip() or 'results'
    run_id = LaunchConfiguration('run_id').perform(context).strip()
    duracao_arg = LaunchConfiguration('duracao_s').perform(context).strip()
    duracao_s = float(duracao_arg) if duracao_arg else cfg['duracao']
    if not run_id:
        run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    result_dir = os.path.abspath(os.path.join(result_root, caso, run_id))
    os.makedirs(result_dir, exist_ok=True)
    tx, ty, tz = cfg['target']
    print(f'Experimento drone caso={caso}')
    print(f'Waypoints: {cfg["waypoints_file"]}')
    print(f'Alvo final: ({tx:.2f}, {ty:.2f}, {tz:.2f})')
    print(f'Resultados: {result_dir}')

    start_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_drone, 'launch', 'start_sim.launch.py')
        ]),
        launch_arguments={
            'controlador_trajetoria': 'true',
            'usar_cabo': 'true',
            'prender_ancora': 'true',
            'headless': LaunchConfiguration('headless'),
            'waypoints_file': cfg['waypoints_file'],
            'cmd_vel_frame': 'body',
            'janela_tangente_metros': LaunchConfiguration('janela_tangente_metros'),
            'tempo_estabilizacao': LaunchConfiguration('tempo_estabilizacao'),
            'log_periodo': LaunchConfiguration('log_periodo'),
            'hover_metrics': 'false',
        }.items(),
    )

    tracking = Node(
        package='pacote_do_drone',
        executable='experimento_tracking',
        parameters=[{
            'caso': caso,
            'result_dir': result_dir,
            'duracao_s': duracao_s,
            'janela_final_s': cfg['janela_final'],
            'sample_period_s': ParameterValue(LaunchConfiguration('sample_period_s'), value_type=float),
        }],
        output='screen',
        on_exit=[Shutdown(reason=f'experimento {caso} concluido')],
    )

    return [start_sim, tracking]


def _launch_setup_postes(context, caso):
    modo_cabo = LaunchConfiguration('modo_cabo').perform(context).lower()
    config = LaunchConfiguration('config').perform(context) or None
    world_path = escrever_world(caso, modo_cabo=modo_cabo, config_path=config)
    tabela_path = escrever_tabela_esperada(config_path=config, modo_cabo=modo_cabo)
    print(f'Mundo de avaliacao: {world_path}')
    print(f'Tabela esperada: {tabela_path}')
    print(f'Modo do cabo: {modo_cabo}')
    print(f'Config dos postes: {config or "padrao"}')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'{world_path} -v4'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/cabo_avaliacao/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen',
    )

    avaliador = Node(
        package='cabo_avaliacao',
        executable='avaliador',
        parameters=[{
            'caso': ParameterValue(caso, value_type=str),
            'config': ParameterValue(config or '', value_type=str),
            'modo_cabo': ParameterValue(modo_cabo, value_type=str),
        }],
        output='screen',
    )

    return [gazebo, bridge, avaliador]


def generate_launch_description():
    models_path = os.path.join(get_package_share_directory('pacote_do_drone'), 'models')

    return LaunchDescription([
        DeclareLaunchArgument(
            'caso',
            default_value='e',
            description='Caso drone: n, s, e, w, z060, z100. Use tipo:=postes para casos estaticos.',
        ),
        DeclareLaunchArgument(
            'tipo',
            default_value='auto',
            description='auto usa testes do drone para n/s/e/w/z060/z100; postes usa avaliacao estatica antiga.',
        ),
        DeclareLaunchArgument(
            'modo_cabo',
            default_value='reto',
            description='Modo do cabo: reto, articulado ou catenaria',
        ),
        DeclareLaunchArgument(
            'config',
            default_value='',
            description='Arquivo JSON com altura, ancora e posicoes dos postes',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Executa Gazebo sem GUI nos testes do drone.',
        ),
        DeclareLaunchArgument(
            'janela_tangente_metros',
            default_value='0.15',
            description='Janela fisica para estimar tangente local do cabo no drone.',
        ),
        DeclareLaunchArgument(
            'tempo_estabilizacao',
            default_value='1.0',
            description='Tempo simulado continuo dentro das tolerancias antes do hover.',
        ),
        DeclareLaunchArgument(
            'log_periodo',
            default_value='2.0',
            description='Periodo dos logs do controlador nos testes do drone.',
        ),
        DeclareLaunchArgument(
            'sample_period_s',
            default_value='0.05',
            description='Periodo de amostragem do CSV/plots em tempo simulado.',
        ),
        DeclareLaunchArgument(
            'result_root',
            default_value='results',
            description='Diretorio raiz dos resultados dos testes do drone.',
        ),
        DeclareLaunchArgument(
            'run_id',
            default_value='',
            description='Identificador opcional da execucao; vazio usa timestamp.',
        ),
        DeclareLaunchArgument(
            'duracao_s',
            default_value='',
            description='Duracao simulada opcional para benchmarks; vazio usa o padrao do caso.',
        ),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),
        OpaqueFunction(function=_launch_setup),
    ])
