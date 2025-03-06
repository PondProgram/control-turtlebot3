import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    lidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
        ),

        Node(
            package='ros2_final',
            executable='control_robot',
            name='control_robot',
            output='screen'
        ),
    	
        Node(
        package='ros2_final',  
        executable='stop_robot',
        name='stop_robot',
        output='screen',
        ),

        Node(
        package='ros2_final',  
        executable='obstacle_detection',
        name='obstacle_detection',
        output='screen',
        ),

    ])
