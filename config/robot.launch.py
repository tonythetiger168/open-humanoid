from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='false'),
        DeclareLaunchArgument('mode', default_value='stand'),
        DeclareLaunchArgument('terrain', default_value='flat'),

        # Main control node
        Node(
            package='open_humanoid',
            executable='ros2_node.py',
            name='open_humanoid_control',
            parameters=[{
                'use_gui': LaunchConfiguration('use_gui'),
                'mode': LaunchConfiguration('mode'),
                'terrain_type': LaunchConfiguration('terrain'),
            }],
            output='screen',
            emulate_tty=True,
        ),

        # RViz2 visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'config/robot.rviz'],
            output='log',
        ),

        # Robot State Publisher (for TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open('design/open_humanoid_v1.urdf').read()}],
            output='log',
        ),

        # Joint State Publisher GUI (optional, for debugging)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=LaunchConfiguration('use_gui') == 'true',
        ),
    ])
