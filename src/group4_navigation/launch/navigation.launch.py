"""Round 2: Nav2 on the live SLAM map (slam_toolbox keeps running).

    ros2 launch group4_navigation navigation.launch.py

Everything runs in the robot namespace, like group4_slam and bug2_create3:
    map            /robot_4/map              (from slam_toolbox)
    goal action    /robot_4/navigate_to_pose
    cmd_vel        controller/behaviors -> /robot_4/cmd_vel_nav -> velocity_smoother
                   -> /robot_4/cmd_vel_smoothed -> collision_monitor -> /robot_4/cmd_vel (robot)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

# Nav2 servers started by this file, all managed by one lifecycle manager.
# (No map_server/amcl: slam_toolbox provides /robot_4/map and map->odom.)
# Extra remaps per node: controller and behaviors publish into the smoother (cmd_vel_nav), the
# smoother into the collision monitor (cmd_vel_smoothed, its default output; the monitor's
# topics are set in the yaml). Only the collision monitor drives the robot (as nav2_bringup).
# bt_navigator last: once it is active (what send_goal waits for), the rest is too.
NAV_NODES = [
    ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_planner', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_collision_monitor', 'collision_monitor', []),
    ('nav2_bt_navigator', 'bt_navigator', []),
]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    default_params = os.path.join(
        get_package_share_directory('group4_navigation'), 'config', 'nav2_params.yaml')

    # The yaml uses plain node names (controller_server: ...). root_key puts them under the
    # namespace, so they match /robot_4/controller_server etc. (same as nav2_bringup).
    params = ParameterFile(
        RewrittenYaml(source_file=params_file, root_key=namespace,
                      param_rewrites={}, convert_types=True),
        allow_substs=True)

    # tf2 always uses the absolute /tf: send it to the robot's (relative -> /robot_4/tf).
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    nodes = [
        Node(
            package=package,
            executable=name,
            name=name,
            namespace=namespace,
            parameters=[params],
            remappings=remappings + extra_remappings,
            output='screen',
        )
        for package, name, extra_remappings in NAV_NODES
    ]

    # Create 3 odom is BEST_EFFORT, Nav2 subscribes RELIABLE: relay it (odom_relay.py).
    nodes.append(Node(
        package='group4_navigation',
        executable='odom_relay',
        name='odom_relay',
        namespace=namespace,
        output='screen',
    ))

    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [name for _, name, _ in NAV_NODES],
        }],
    ))

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_4'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 parameters'),
        *nodes,
    ])
