import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # base_link → laser (static). Enable if no TF is being published by your robot.
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_base_laser',
        #     arguments=[
        #         '--x', '0.2', '--y', '0', '--z', '0.1',   # adjust if needed
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'base_link',
        #         '--child-frame-id', 'laser'
        #     ]
        # ),

        # SLAM Toolbox node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=["/home/phattanarat/onenight_ws/src/scantest/config/slam_toolbox_params.yaml"],  # Pass YAML directly
        ),
    ])
