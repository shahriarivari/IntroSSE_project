"""Round 2: send Bug2's goal (or goal:=x,y[,yaw_deg]) to Nav2, after navigation.launch.py.

    ros2 launch group4_navigation send_goal.launch.py
    ros2 launch group4_navigation send_goal.launch.py goal:=3.0,0.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    goal = LaunchConfiguration('goal')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_4'),
        DeclareLaunchArgument('goal', default_value='',
                              description="'x,y' or 'x,y,yaw_deg' in map; empty = Bug2's goal"),
        Node(
            package='group4_navigation',
            executable='send_goal',
            name='send_goal',
            namespace=namespace,
            parameters=[{'goal': ParameterValue(goal, value_type=str)}],
            output='screen',
            emulate_tty=True,
        ),
    ])
