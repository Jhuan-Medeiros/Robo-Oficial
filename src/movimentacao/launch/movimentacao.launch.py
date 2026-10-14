import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('movimentacao')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    studica_prefix = get_package_prefix('studica_control')
    studica_exec = os.path.join(studica_prefix, 'lib', 'studica_control', 'manual_composition')

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[params_file]
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.10', '0.0', '0.15', '0.0', '0.0', '0.0', 'base_link', 'laser_frame'],
        output='screen'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file]
    )

    # Inicia o SLAM 4 segundos depois para o YDLidar ja estar estabilizado em 10Hz
    delayed_slam = TimerAction(
        period=4.0,
        actions=[slam_node]
    )

    base_control_node = Node(
        package='movimentacao',
        executable='movimentacao',
        name='no_movimentacao',
        output='screen',
        parameters=[params_file]
    )

    studica_control_node = ExecuteProcess(
        cmd=[
            'sudo', '-E', studica_exec,
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', params_file,
        ],
        output='screen',
    )

    return LaunchDescription([
        ydlidar_node,
        static_tf_laser,
        base_control_node,
        studica_control_node,
        delayed_slam
    ])