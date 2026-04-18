import os

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    my_carto_pkg_share = FindPackageShare(package='my_carto_pkg').find('my_carto_pkg')
    uart_to_stm32_pkg_share = FindPackageShare(package='uart_to_stm32').find('uart_to_stm32')
    pid_control_pkg_share = FindPackageShare(package='pid_control_pkg').find('pid_control_pkg')
    activity_control_pkg_share = FindPackageShare(package='activity_control_pkg').find('activity_control_pkg')
    my_launch_pkg_share = FindPackageShare(package='my_launch').find('my_launch')

    workspace_root = os.path.normpath(os.path.join(my_launch_pkg_share, '..', '..', '..', '..'))
    photo_save_dir = os.path.join(workspace_root, 'src', 'photo')

    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_carto_pkg_share, 'launch', 'fly_carto.launch.py')
        )
    )

    uart_to_stm32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uart_to_stm32_pkg_share, 'launch', 'uart_to_stm32.launch.py')
        )
    )

    position_pid_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pid_control_pkg_share, 'launch', 'position_pid_controller.launch.py')
        )
    )

    route_target_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(activity_control_pkg_share, 'launch', 'route_target_publisher.launch.py')
        )
    )

    drone_camera_node = Node(
        package='drone_camera_pkg',
        executable='drone_camera_node',
        name='drone_camera_node',
        output='screen',
        parameters=[
            {
                'camera_device': '/dev/video0',
                'frame_width': 640,
                'frame_height': 480,
                'fps': 15.0,
                'window_name': 'drone_camera_preview',
                'photo_save_dir': photo_save_dir,
            }
        ]
    )

    return LaunchDescription([
        fly_carto_launch,
        uart_to_stm32_launch,
        position_pid_controller_launch,
        route_target_publisher_launch,
        drone_camera_node,
    ])
