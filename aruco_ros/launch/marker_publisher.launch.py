# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.05',
        description='Marker size in m. '
    )

    side_arg = DeclareLaunchArgument(
        'side', default_value='left',
        description='Side. ',
        choices=['left', 'right'],
    )

    reference_frame = DeclareLaunchArgument(
        'reference_frame', default_value='base',
        description='Reference frame. Leave it empty and the pose will be published wrt param parent_name. '
    )

    aruco_marker_publisher_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'reference_frame': LaunchConfiguration(reference_frame),
        'camera_frame': LaunchConfiguration('side') + '_hand_camera',
    }

    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='single',
        parameters=[aruco_marker_publisher_params],
        remappings=[('/camera_info', '/cameras/' + LaunchConfiguration('side') + '_hand_camera/camera_info'),
                    ('/image', '/cameras/' + LaunchConfiguration('side') + '_hand_camera/image')],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(marker_size_arg)
    ld.add_action(side_arg)
    ld.add_action(reference_frame)

    ld.add_action(aruco_marker_publisher)

    return ld
