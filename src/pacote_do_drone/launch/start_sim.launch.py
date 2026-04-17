import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('pacote_do_drone')
    models_path = os.path.join(pkg_share, 'models')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'meu_drone', '-file', os.path.join(models_path, 'meu_drone', 'meu_drone.sdf'), '-x', '0.0', '-y', '0.0', '-z', '1'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/tensao_cabo@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/angulos_cabo@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/meu_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        output='screen'
    )

    return LaunchDescription([
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),
        gazebo,
        spawn_drone,
        bridge
    ])