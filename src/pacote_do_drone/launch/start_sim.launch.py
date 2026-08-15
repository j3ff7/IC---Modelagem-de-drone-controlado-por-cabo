import os
import json # <-- Adicionado
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('pacote_do_drone')
    models_path = os.path.join(pkg_share, 'models')
    
    # Caminho para o seu novo arquivo de mundo
    world_path = os.path.join(pkg_share, 'worlds', 'my_world.sdf')

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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        # Passa o caminho do seu mundo aqui!
        launch_arguments={'gz_args': f'{world_path} -v4'}.items() 
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cabo/tensao_drone@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/cabo/tensao_carretel@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/meu_drone/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/meu_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/world/mundo_ic/model/sistema_cabo_drone/model/meu_drone/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/mundo_ic/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    sensores = Node(
        package='pacote_do_drone',
        executable='sensores',
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
            default_value='world',
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
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),
        gazebo,
        bridge,
        sensores,
        controlador_trajetoria,
    ])
