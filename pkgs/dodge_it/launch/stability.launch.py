import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arg = DeclareLaunchArgument(
        'sim',
        default_value='rviz_gui',
        choices=['rviz_gui', 'rviz_manual', 'gz']
    )

    def chooseLaunchFile(context, *args, **kwargs):
        sim_arg_val = LaunchConfiguration('sim').perform(context)
        if sim_arg_val == 'gz':
            launchFile = os.path.join(
                get_package_share_directory('ros_gz_h1_bringup'),
                'launch',
                'h1_gazebo_sim.launch.py')
        elif sim_arg_val == 'rviz_manual':
            launchFile = os.path.join(
                get_package_share_directory('dodge_it'),
                'launch',
                'h1_rviz_manual.launch.py')
        else:
            launchFile = os.path.join(
                get_package_share_directory('dodge_it'),
                'launch',
                'h1_rviz_gui.launch.py')
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launchFile)
        ),

    return LaunchDescription([
        arg,
        OpaqueFunction(function=chooseLaunchFile),
        Node(
            package='point_calculator',
            executable='center_of_mass',
            output='screen'
        ),
        Node(
            package='point_calculator',
            executable='support_polygon',
            output='screen'
        )
    ])