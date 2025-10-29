from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_laser',
            arguments=[
                '--x', '0.2', '--y', '0', '--z', '0.1',  
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser'
            ]
        ),
    ])
