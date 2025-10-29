import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('scantest')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    slam_params_file = os.path.join(pkg, 'config', 'handheld_slam_params.yaml')

    return LaunchDescription([

        # RPLIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'frame_id': frame_id,
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'channel_type': 'serial',
                'inverted': False,
                'angle_compensate': True
            }],
            output='screen'
        ),

        # Static transform: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_laser',
            arguments=['0.2', '0.0', '0.1', '0', '0', '0', '1', 'base_link', 'laser']
        ),

        # Fake odom publisher: odom -> base_link
        # Node(
        #     package='scantest',
        #     executable='fake_odom_publisher.py',
        #     name='fake_odom',
        #     output='screen'
        # ),

        # SLAM Toolbox (synchronous)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file]
        ),

        # RViz (optional, uncomment if needed)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(pkg, 'rviz', 'handheld_config.rviz')],
        #     output='screen'
        # ),
    ])
