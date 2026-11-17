import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    yaml_config = "/home/vmx/projeto/src/kiwi_robot/config/params.yaml"

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
        static_tf_laser,
        TimerAction(period=2.0, actions=[node_ydlidar]),
        TimerAction(period=4.0, actions=[node_kiwi]),
        #TimerAction(period=6.0, actions=[node_slam]),  ## Comenta esta linha quando fores usar o Nav2/AMCL!
    ])