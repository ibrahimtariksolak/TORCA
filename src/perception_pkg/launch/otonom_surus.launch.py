import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir    = os.path.expanduser('~/ros2_ws/src/perception_pkg')
    world_file = os.path.join(pkg_dir, 'worlds', 'otoban.world')
    urdf_file  = os.path.join(pkg_dir, 'urdf', 'otonom_arac.urdf.xacro')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gzserver', '--verbose',
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so',
                 '-s', 'libgazebo_ros_state.so',
                 world_file],
            output='screen'
        ),

        ExecuteProcess(cmd=['gzclient'], output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'odom'],
            output='screen'
        ),

        TimerAction(period=3.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', urdf_file,
                    '-entity', 'otonom_arac',
                    # Oval pistin alt duz bolumu, sag seridin ortasi.
                    # Bu degerleri worlds/generate_otoban.py ekrana yaziyor;
                    # pist parametreleri degisirse oradan guncelle.
                    '-x', '-18.0', '-y', '-26.75', '-z', '0.1', '-Y', '0.0',
                ],
                output='screen'
            ),
        ]),

        TimerAction(period=4.0, actions=[
            Node(
                package='perception_pkg',
                executable='sign_detector',
                name='sign_detector',
                output='screen'
            ),
        ]),

        TimerAction(period=5.0, actions=[
            Node(
                package='perception_pkg',
                executable='serit_takip',
                name='serit_takip',
                output='screen'
            ),
        ]),

        TimerAction(period=3.0, actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2', output='screen'),
        ]),
    ])
