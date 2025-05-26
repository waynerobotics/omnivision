from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    yaw_angle_arg = DeclareLaunchArgument(
        'yaw_angle',
        default_value='0.0',
        description='Initial yaw angle in degrees for camera-LiDAR alignment'
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/unilidar/cloud',
        description='Point cloud topic to subscribe to'
    )
    
    mask_topic_arg = DeclareLaunchArgument(
        'mask_topic',
        default_value='/lane_detection/segmentation_mask',
        description='Binary mask topic to subscribe to'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='RGB image topic to subscribe to'
    )
    
    # Transformation reconfiguration server node
    reconfigure_node = Node(
        package='omnivision',
        executable='transformation_reconfigure',
        name='transformation_reconfiguration_server',
        parameters=[{
            'yaw_angle': LaunchConfiguration('yaw_angle'),
        }],
        output='screen'
    )
    
    # Mask to point cloud node with dynamic transformation updates
    mask_to_pointcloud_node = Node(
        package='omnivision',
        executable='mask_to_pointcloud',
        name='mask_to_pointcloud',
        parameters=[{
            'yaw_angle': LaunchConfiguration('yaw_angle'),
            'dynamic_transformation_updates': True,
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'mask_topic': LaunchConfiguration('mask_topic'),
            'rgb_image_topic': LaunchConfiguration('image_topic'),
        }],
        output='screen'
    )
    
    # QT GUI for transformation adjustment
    gui_node = Node(
        package='omnivision',
        executable='transformation_gui',
        name='transformation_gui',
        output='screen'
    )
    
    return LaunchDescription([
        yaw_angle_arg,
        pointcloud_topic_arg,
        mask_topic_arg,
        image_topic_arg,
        reconfigure_node,
        mask_to_pointcloud_node,
        gui_node
    ])