import os
import json # <-- Adicionado
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('pacote_do_drone')
    models_path = os.path.join(pkg_share, 'models')
    
    # Caminho para o seu novo arquivo de mundo
    world_path = os.path.join(pkg_share, 'worlds', 'my_world.sdf')

    caminho_json = os.path.join(pkg_share, 'parameters', 'tether_parameters.json')
    
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/meu_drone/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cabo/tensao_drone@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/cabo/tensao_carretel@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/tensao_cabo@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/angulos_cabo@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/meu_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/meu_drone/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators'
        ],
        output='screen'
    )

    return LaunchDescription([
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),
        gazebo,
        bridge
    ])
