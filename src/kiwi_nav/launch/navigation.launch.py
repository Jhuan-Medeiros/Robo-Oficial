import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_kiwi_robot = get_package_share_directory('kiwi_robot')
    pkg_kiwi_nav = get_package_share_directory('kiwi_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    robot_yaml = os.path.join(pkg_kiwi_robot, 'config', 'params.yaml')
    nav2_yaml = os.path.join(pkg_kiwi_nav, 'config', 'nav2_params.yaml')
    default_map_path = '/home/vmx/projeto/src/kiwi_robot/maps/meu_mapa.yaml'

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Caminho completo para o arquivo .yaml do mapa salvo'
    )

    studica_node = ExecuteProcess(
        cmd=[
            'sudo', '-E',
            '/home/vmx/studica_ws/install/studica_control/lib/studica_control/manual_composition',
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', robot_yaml
        ],
        output='screen'
    )

    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[robot_yaml]
    )

    # Mantem a mesma configuracao fisica do Lidar que funcionou no mapeamento
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.15',
            '--roll', '3.14159265',
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
        parameters=[robot_yaml]
    )

    # Inicia a stack completa de localizacao (AMCL) + navegacao (Nav2)
    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'params_file': nav2_yaml,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        map_arg,
        studica_node,
        node_ydlidar,
        static_tf_laser,
        node_kiwi,
        include_nav2
    ])
