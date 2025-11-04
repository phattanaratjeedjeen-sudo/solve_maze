
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    my_package_dir = get_package_share_directory('scantest')

    rps_path = os.path.join(
        my_package_dir,
        'launch',
        'rps.launch.py'
    )

    laser_active_path = os.path.join(
        my_package_dir,
        'launch',
        'laser_active.launch.py'
    )

    laser_scan_matcher_path = os.path.join(
        my_package_dir,
        'launch',
        'laser_scan_matcher.launch.py'
    )

    rviz_path = os.path.join(my_package_dir, 'rviz', 'config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen'
    )

    online_async_launch_path = os.path.join(
        my_package_dir,
        'launch',
        'online_async_launch.py'
    )
 

    rps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rps_path)
    )

    laser_active = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(laser_active_path)
    )

    laser_scan_matcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(laser_scan_matcher_path)
    )

    online_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(online_async_launch_path)
    )



    return LaunchDescription([
        Node(
            package='scantest',
            namespace='',
            executable='state_manager.py',
            name='state_manager',
        ),
        Node(
            package='scantest',
            namespace='',
            executable='location.py',
            name='location',  
        ),
        Node(
            package='scantest',
            namespace='',
            executable='prepare_mqtt.py',
            name='prepare_mqtt',  
        ),
        rps,
        laser_active,
        laser_scan_matcher,
        online_async_launch,
        rviz,
    ])