import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazeboLaunchFile = os.path.join(
        get_package_share_directory('ros_gz_h1_bringup'),
        'launch',
        'h1_gazebo_sim.launch.py'
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazeboLaunchFile)
        ),
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