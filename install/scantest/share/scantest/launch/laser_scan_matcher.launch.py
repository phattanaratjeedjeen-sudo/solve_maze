import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Define the laser scan matcher node
    odometry_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='odometry_scan_publisher',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'odom_frame': 'odom_matcher',
            'laser_frame': 'laser',
            'publish_odom': '',
            'publish_tf': True,
        }],
    )

    # Create and return the launch description
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
        ),
        odometry_node,
    ])