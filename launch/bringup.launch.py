##############################################################################
# Omnivision - 360 Degree Camera and LiDAR fusion
#
# Software License Agreement (GPL-3.0)
# Copyright (C) 2025 Blaine Oania
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
##############################################################################

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    fuser = Node(
            package='omnivision',
            executable='fusion',
            name='fuser',
            output='screen',
            parameters=[
                {
                    'transformation_matrix': [
                        1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, -0.203,
                        0, 0, 0, 1
                    ],
                    'pointcloud_topic': 'velodyne_points',
                    'image_topic': 'camera/image',
                    'depth_map': 'camera/image/depth',
                    'texturized_pointcloud': 'textured_pointcloud',
                    'texturized_depth_map': 'texturized_depth_map',
                    'image_overlay': 'image_overlay'
                }
            ]
    )
    
    

    return LaunchDescription([
        fuser
    ])

if __name__ == '__main__':
    generate_launch_description()