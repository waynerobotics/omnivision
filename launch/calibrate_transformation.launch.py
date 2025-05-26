from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/unilidar/cloud',
        description='Point cloud topic to subscribe to'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='RGB image topic to subscribe to'
    )
    
    # Calibration node
    calibration_node = Node(
        package='omnivision',
        executable='transformation_calibrator',
        name='transformation_calibrator',
        parameters=[{
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'image_topic': LaunchConfiguration('image_topic'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        pointcloud_topic_arg,
        image_topic_arg,
        calibration_node
    ])