from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    eye = perform_substitutions(context, [LaunchConfiguration('eye')])

    aruco_double_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id1': LaunchConfiguration('marker_id1'),
        'marker_id2': LaunchConfiguration('marker_id2'),
        'normalizeImage': LaunchConfiguration('dct_normalization'),
        'dct_components_to_remove': LaunchConfiguration('dct_filter_size'),
        'parent_name': 'stereo_gazebo_' + eye + '_camera_optical_frame',
        'child_name1': LaunchConfiguration('marker1_frame'),
        'child_name2': LaunchConfiguration('marker2_frame'),
    }

    aruco_double = Node(
        package='aruco_ros',
        executable='double',
        parameters=[aruco_double_params],
        remappings=[('/camera_info', '/stereo/' + eye + '/camera_info'),
                    ('/image', '/stereo/' + eye + '/image_rect_color')],
    )

    return [aruco_double]


def generate_launch_description():

    marker_id1_arg = DeclareLaunchArgument(
        'marker_id1', default_value='582',
        description='Marker ID. '
    )

    marker_id2_arg = DeclareLaunchArgument(
        'marker_id2', default_value='26',
        description='Marker ID 2. '
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.04',
        description='Marker size in m. '
    )

    eye_arg = DeclareLaunchArgument(
        'eye', default_value='left',
        description='Eye. ',
        choices=['left', 'right'],
    )

    dct_normalization_arg = DeclareLaunchArgument(
        'dct_normalization', default_value='true',
        description='dct normalization. ',
        choices=['true', 'false'],
    )

    dct_filter_size_arg = DeclareLaunchArgument(
        'dct_filter_size', default_value='2',
        description='dct filter size. ',
    )

    marker1_frame_arg = DeclareLaunchArgument(
        'marker1_frame', default_value='marker_hand_frame',
        description='Frame in which the marker1 pose will be refered. '
    )

    marker2_frame_arg = DeclareLaunchArgument(
        'marker2_frame', default_value='marker_object_frame',
        description='Frame in which the marker2 pose will be refered. '
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(marker_id1_arg)
    ld.add_action(marker_id2_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(eye_arg)
    ld.add_action(dct_normalization_arg)
    ld.add_action(dct_filter_size_arg)
    ld.add_action(marker1_frame_arg)
    ld.add_action(marker2_frame_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
