from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Ponte da tensão
            '/cabo/tensao_drone@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            # Ponte dos ângulos do cabo (TÓPICO PADRÃO DO GAZEBO)
            '/world/mundo_ic/model/cabo_dinamico/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )

    test_moviment_node = Node(
        package='pacote_do_drone',
        executable='sensores',
        output='screen',
        
    )

    return LaunchDescription([
        bridge_node,
        test_moviment_node
    ])