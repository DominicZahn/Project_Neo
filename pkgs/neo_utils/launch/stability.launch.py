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
        default_value='gz',
        choices=['gz', 'rviz']
    )

    def chooseLaunchFile(context, *args, **kwargs):
        bringupDir = get_package_share_directory('ros_gz_h1_bringup')
        gazeboLaunchFile = os.path.join(
            bringupDir,
            'launch',
            'h1_gazebo_sim.launch.py')
        rvizLaunchFile = os.path.join(
            bringupDir,
            'launch',
            'h1_rviz.launch.py')
        launchFile = ""
        sim_arg_val = LaunchConfiguration("sim").perform(context)
        if sim_arg_val == "gz":
            launchFile = gazeboLaunchFile
        else:
            launchFile = rvizLaunchFile

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