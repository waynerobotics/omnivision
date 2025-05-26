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
    
    # Reconfiguration server node
    reconfigure_node = Node(
        package='omnivision',
        executable='transformation_reconfigure',
        name='transformation_reconfiguration_server',
        parameters=[{
            'yaw_angle': LaunchConfiguration('yaw_angle'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        yaw_angle_arg,
        reconfigure_node
    ])