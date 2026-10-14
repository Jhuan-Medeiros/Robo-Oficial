import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_movimentacao = get_package_share_directory('movimentacao')
    yaml_config = os.path.join(pkg_movimentacao, 'config', 'params.yaml')

    studica_launch_dir = os.path.join(
        get_package_share_directory('studica_control'),
        'launch'
    )

    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch'
    )

    include_studica = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(studica_launch_dir, 'studica_launch.py')
        ),
        launch_arguments={'setting_file': yaml_config}.items()
    )

    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': '/dev/ydlidar',
            'baudrate': 230400,
            'frame_id': 'laser_frame',
            'ignore_array': '',
            'range_min': 0.20,
            'range_max': 12.0,
            'frequency': 10.0,
            'is_single_channel': False,
            'intensity': False,
            'support_motor_dtr': True,
            'invalid_range_is_inf': True,
        }]
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['--x', '0.1', '--y', '0.0', '--z', '0.15', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']
    )

    node_movimentacao = Node(
        package='movimentacao',
        executable='movimentacao',
        name='no_movimentacao',
        output='screen',
        parameters=[yaml_config] 
    )

    include_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_launch_dir, 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': yaml_config
        }.items()
    )

    return LaunchDescription([
        include_studica,
        node_ydlidar,
        static_tf_laser,
        node_movimentacao,
        include_slam
    ])