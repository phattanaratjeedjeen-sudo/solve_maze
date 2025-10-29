from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # SLAM Toolbox node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=["/home/phattanarat/onenight_ws/src/scantest/config/slam_toolbox_params.yaml"],  # Pass YAML directly
        ),
    ])
