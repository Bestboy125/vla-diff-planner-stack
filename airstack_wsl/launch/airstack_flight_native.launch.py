"""AirStack local trajectory planning/control and behavior, native in WSL."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    robot_name = os.environ.get("ROBOT_NAME", "robot_1").strip("/")
    integration_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    local_launch = os.path.join(integration_root, "launch", "local_airstack_vla.launch.xml")
    behavior_launch = os.path.join(
        get_package_share_directory("behavior_bringup"), "launch", "behavior.launch.xml"
    )
    depth_adapter = os.path.join(integration_root, "scripts", "depth_to_disparity.py")

    return LaunchDescription(
        [
            # Pegasus creates the camera 0.30 m in front of the vehicle body.
            # Publish the mount transform expected by downstream image users.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="front_camera_static_tf",
                arguments=[
                    "--x", "0.30", "--y", "0.0", "--z", "0.0",
                    # ROS optical frame: +Z forward, +X right, +Y down.
                    "--roll", "-1.57079632679", "--pitch", "0.0", "--yaw", "-1.57079632679",
                    "--frame-id", "base_link", "--child-frame-id", "camera_front",
                ],
                output="screen",
            ),
            GroupAction(
                [
                    PushRosNamespace(robot_name),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(local_launch),
                        launch_arguments={
                            # The upstream node creates both subscriptions even
                            # when the sensors are disabled. They must not share
                            # a name because their ROS message types differ.
                            "local_disparity_in_topic": (
                                f"/{robot_name}/sensors/front_camera/disparity"
                            ),
                            "local_depth_in_topic": (
                                f"/{robot_name}/sensors/front_camera/image/depth"
                            ),
                            "local_camera_info_in_topic": (
                                f"/{robot_name}/sensors/front_camera/image/camera_info"
                            ),
                        }.items(),
                    ),
                    IncludeLaunchDescription(AnyLaunchDescriptionSource(behavior_launch)),
                ]
            ),
            ExecuteProcess(
                cmd=[
                    "python3", depth_adapter,
                    "--depth-topic", f"/{robot_name}/sensors/front_camera/image/depth",
                    "--camera-info-topic", f"/{robot_name}/sensors/front_camera/image/camera_info",
                    "--disparity-topic", f"/{robot_name}/sensors/front_camera/disparity",
                    "--baseline-m", "0.12",
                    "--sim-max-range-m", "8.0",
                ],
                output="screen",
            ),
        ]
    )
