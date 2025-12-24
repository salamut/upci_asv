from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    mission_master_arg = DeclareLaunchArgument(
        'mission_master',
        default_value='true',
        description='Enable mission master node'
    )

    mission_master_enable = LaunchConfiguration('mission_master')

    return LaunchDescription([
        mission_master_arg,

        Node(
            package='yolo_detector',
            executable='detector_node',
            name='yolo_detector'
        ),

        Node(
            package='obstacle_avoidance',
            executable='avoidance_node',
            name='avoidance'
        ),

        Node(
            package='mav_interface',
            executable='mav_monitor',
            name='mav_monitor'
        ),

        Node(
            package='mav_interface',
            executable='mission_master',
            name='mission_master',
            condition=IfCondition(mission_master_enable)
        ),
    ])
