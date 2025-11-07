from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'urdf', 'robot.urdf'
    ])
    
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'), 'rviz', 'simulation.rviz'
    ])

    return LaunchDescription([
        # C++ Odometry Node
        Node(
            package='robot_simulator_cpp',
            executable='odometry_node',
            output='screen'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', urdf_file])
            }]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        
        # Joy Node - reads joystick input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }]
        ),
        
        # Teleop Twist Joy - converts joystick to cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'axis_linear.y': 1,      # Left stick vertical (forward/backward)
                'axis_linear.x': 0,      # Left stick horizontal (strafe)
                'axis_angular.yaw': 3,   # Left stick horizontal (rotation)
                'scale_linear.x': -1.0,   # Linear speed scaling
                'scale_linear.y': 1.0,   # Linear speed scaling
                'scale_angular.yaw': 1.0, # Angular speed scaling
                'enable_button': 3,      # L1/LB button to enable (button index 4)
                'require_enable_button': True
            }]
        ),
    ])
