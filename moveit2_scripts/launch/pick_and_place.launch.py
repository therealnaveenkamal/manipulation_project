import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition



def generate_launch_description():
    sim_moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()
    real_moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    use_sim_time = LaunchConfiguration('use_sim_time')


    return LaunchDescription([
    
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation '
        ),

        Node(
            name="pick_and_place_sim",
            package="moveit2_scripts",
            executable="pick_and_place_sim",
            output="screen",
            parameters=[
                sim_moveit_config.robot_description,
                sim_moveit_config.robot_description_semantic,
                sim_moveit_config.robot_description_kinematics,
                {'use_sim_time': True},
            ],
            condition=IfCondition(LaunchConfiguration('use_sim_time'))
        ),
        Node(
            name="pick_and_place_real",
            package="moveit2_scripts",
            executable="pick_and_place_real",
            output="screen",
            parameters=[
                real_moveit_config.robot_description,
                real_moveit_config.robot_description_semantic,
                real_moveit_config.robot_description_kinematics,
                {'use_sim_time': False},
            ],
            condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
        )
    ])