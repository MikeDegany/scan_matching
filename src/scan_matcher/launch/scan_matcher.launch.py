from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    scan1_topic_arg = DeclareLaunchArgument(
        'scan1_topic',
        default_value='/JK3/sensors/lidar2d_0/scan',
        description='Topic for first laser scan'
    )
    
    scan2_topic_arg = DeclareLaunchArgument(
        'scan2_topic',
        default_value='/JK5/sensors/lidar2d_0/scan',
        description='Topic for second laser scan'
    )
    
    fixed_frame_arg = DeclareLaunchArgument(
        'fixed_frame',
        default_value='map',
        description='Fixed frame for visualization'
    )

    # Scan matcher node
    scan_matcher_node = Node(
        package='scan_matcher',
        executable='scan_matcher',
        name='scan_matcher',
        output='screen',
        parameters=[{
            'scan1_topic': LaunchConfiguration('scan1_topic'),
            'scan2_topic': LaunchConfiguration('scan2_topic'),
            'fixed_frame': LaunchConfiguration('fixed_frame')
        }]
    )

    return LaunchDescription([
        scan1_topic_arg,
        scan2_topic_arg,
        fixed_frame_arg,
        scan_matcher_node
    ])
