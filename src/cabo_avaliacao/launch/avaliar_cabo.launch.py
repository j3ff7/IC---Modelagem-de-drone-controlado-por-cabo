import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from cabo_avaliacao.gerar_mundos import escrever_tabela_esperada, escrever_world


def _launch_setup(context, *args, **kwargs):
    caso = LaunchConfiguration('caso').perform(context).lower()
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
            description='Caso: n, s, e, w, ne, nw, se, sw',
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
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),
        OpaqueFunction(function=_launch_setup),
    ])
