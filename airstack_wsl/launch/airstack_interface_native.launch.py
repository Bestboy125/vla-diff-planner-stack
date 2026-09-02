"""Put AirStack's interface launch under the robot namespace outside Docker."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    robot_name = os.environ.get("ROBOT_NAME", "robot_1").strip("/")
    interface_launch = os.path.join(
        get_package_share_directory("interface_bringup"),
        "launch",
        "interface.launch.xml",
    )
    return LaunchDescription(
        [
            GroupAction(
                [
                    PushRosNamespace(robot_name),
                    IncludeLaunchDescription(AnyLaunchDescriptionSource(interface_launch)),
                ]
            )
        ]
    )
