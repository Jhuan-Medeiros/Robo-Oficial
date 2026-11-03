import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    yaml_config = "/home/vmx/projeto/src/kiwi_robot/config/params.yaml"

    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch'
    )

    studica_node = ExecuteProcess(
        cmd=[
            'sudo', '-E',
            '/home/vmx/studica_ws/install/studica_control/lib/studica_control/manual_composition',
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', yaml_config
        ],
        output='screen'
    )

    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[yaml_config]
    )

    # TF estatica do Laser: Z=10cm acima do base_link, de cabeca para baixo (roll=3.14159)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.10',
            '--roll', '3.14159',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame'
        ]
    )

    node_kiwi = Node(
        package='kiwi_robot',
        executable='kiwi_node',
        name='no_movimentacao',
        output='screen',
        parameters=[yaml_config]
    )

    # Inicializa o SLAM Toolbox explicitamente apontando para o params.yaml
    node_slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            yaml_config,
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        studica_node,
        node_ydlidar,
        static_tf_laser,
        node_kiwi,
        node_slam
    ])