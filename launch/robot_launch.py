import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix



def generate_launch_description():
    package_dir = get_package_share_directory('licenta')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'khepera.urdf')).read_text()

    webots = WebotsLauncher(  world=os.path.join(package_dir, 'worlds', 'world_new_obstcl.wbt')  )

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Khepera'},
        parameters=[{'robot_description': robot_description},
                     {'webots_ros2_driver': {'controller_name': 'Khepera'}}
                    ]
    )
    obstacle_avoider = Node(package='licenta',executable='obstacle_avoider',)
    return LaunchDescription([
        webots,
        my_robot_driver,
        obstacle_avoider ,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])