"""Round 1: Bug2 start -> goal -> start.

    ros2 launch bug2_create3 bug2.launch.py goal_forward:=3.0 goal_left:=0.0
    ros2 launch bug2_create3 bug2.launch.py pose_source:=odom     # without SLAM
    ros2 launch bug2_create3 bug2.launch.py use_lidar_tf:=false   # ignore the lidar mount TF
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    goal_forward = LaunchConfiguration('goal_forward')
    goal_left = LaunchConfiguration('goal_left')
    pose_source = LaunchConfiguration('pose_source')
    use_lidar_tf = LaunchConfiguration('use_lidar_tf')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_4'),
        DeclareLaunchArgument('goal_forward', default_value='3.0',
                              description='[m] goal ahead of the start pose'),
        DeclareLaunchArgument('goal_left', default_value='0.0',
                              description='[m] goal left of the start pose (+ = left)'),
        DeclareLaunchArgument('pose_source', default_value='slam',
                              choices=['slam', 'odom'],
                              description="'slam' = TF map->base_footprint, 'odom' = wheel odometry"),
        DeclareLaunchArgument('use_lidar_tf', default_value='true', choices=['true', 'false'],
                              description='measure from the robot centre using the TF '
                                          'base_footprint -> lidar (false = scan as it is)'),
        Node(
            package='bug2_create3',
            executable='bug2_maze',
            name='bug2_node',
            namespace=namespace,
            parameters=[{
                'goal_forward': ParameterValue(goal_forward, value_type=float),
                'goal_left': ParameterValue(goal_left, value_type=float),
                'pose_source': pose_source,
                'use_lidar_tf': ParameterValue(use_lidar_tf, value_type=bool),
            }],
            # tf2 always uses the absolute /tf: send it to the robot's (relative -> /robot_4/tf)
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
            emulate_tty=True,
        ),
    ])
