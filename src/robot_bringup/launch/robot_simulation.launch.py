from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'urdf', 'robot.urdf'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'rviz', 'simulation.rviz'
    ])

    return LaunchDescription([
        Node(
            package='robot_simulator_cpp',
            executable='odometry_node',
            output='screen'
        ),
        Node(
            package='robot_simulator_py',
            executable='controller_node',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_file],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
