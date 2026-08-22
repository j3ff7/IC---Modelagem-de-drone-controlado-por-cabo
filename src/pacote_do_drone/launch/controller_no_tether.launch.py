import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pacote_do_drone')
    models_path = os.path.join(pkg_share, 'models')
    world_path = os.path.join(pkg_share, 'worlds', 'controller_no_tether.sdf')

    gz_args = [world_path, ' -v4']
    gz_args.append(' -s')
    gz_args.append(' -r')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ''.join(gz_args)}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/meu_drone/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/meu_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),
        gazebo,
        bridge,
    ])