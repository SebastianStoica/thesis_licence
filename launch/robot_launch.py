import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher#, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('licenta')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'khepera.urdf')).read_text()

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Khepera'},
        parameters=[{'robot_description': robot_description},
                     {'webots_ros2_driver': {'controller_name': 'Khepera'}}
                    ]
    )
    tf2_transform = Node(
        package='licenta',
        executable='tf2_transform',
        output='screen'
    )
    obstacle = Node(
        package='licenta',
        executable='obstacle',
        output='screen'
    )
    return LaunchDescription([
        my_robot_driver,
        tf2_transform,
        obstacle
    ])