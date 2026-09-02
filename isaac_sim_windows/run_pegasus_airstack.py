"""Isaac Sim + Pegasus entry point for the native AirStack WSL stack.

The Windows process owns physics/rendering and a TCP MAVLink simulator socket.
PX4 and ROS 2/AirStack run in WSL.  DDS traffic uses ROS_DOMAIN_ID=42 while
PX4 instance 0 maps to MAVLink system id 1; those identifiers are deliberately
independent.
"""

import os
import time
import numpy as np

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "headless": False,
        "width": 1280,
        "height": 720,
        "hide_ui": False,
        "display_options": 3286,
        "anti_aliasing": 2,
    },
    # The default standalone-Python experience is intentionally minimal and
    # produces only a bare grey Kit window.  Load the editor experience so the
    # desktop window contains the Stage tree, toolbar and active 3D viewport.
    experience=os.environ.get(
        "AIRSTACK_ISAAC_EXPERIENCE",
        os.path.join(os.environ["EXP_PATH"], "isaacsim.exp.full.kit"),
    ),
)

# With the full editor experience, several windows are docked asynchronously.
# Starting Pegasus from Kit's command line at the same time re-enters the
# asyncio loop and aborts those docking tasks, leaving a title bar over a grey
# client area.  Give the editor a short, clean startup window before loading
# the simulator extension from Python below.
for _ in range(90):
    simulation_app.update()
    time.sleep(0.01)

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.utils.extensions import enable_extension

# Register Isaac Sim's bundled ROS 2 Python libraries before importing rclpy.
# This bridge is safe to load after the editor has finished docking; Pegasus
# itself remains deferred until all normal imports below are complete.
enable_extension("isaacsim.ros2.bridge")
for _ in range(10):
    simulation_app.update()

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from isaacsim.core.api.world import World
from isaacsim.core.api.objects import FixedCuboid, GroundPlane
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.viewports import set_camera_view
from omni.kit.viewport.utility import get_viewport_from_window_name
from scipy.spatial.transform import Rotation
from pxr import UsdLux


enable_extension("pegasus.simulator")

# Let extension startup and ROS 2 bridge registration finish before imports.
for _ in range(20):
    omni.kit.app.get_app().update()

from pegasus.simulator.logic.backends.px4_mavlink_backend import (  # noqa: E402
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend  # noqa: E402
from pegasus.simulator.logic.graphs import ROS2CameraGraph  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import (  # noqa: E402
    Multirotor,
    MultirotorConfig,
)
from pegasus.simulator.params import ROBOTS  # noqa: E402


SCENE_USD = os.environ.get(
    "AIRSTACK_SCENE_USD",
    r"D:\AirStackWSL\scenes\RetroNeighborhood\RetroNeighborhood_Export.usd",
)
ROS_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "42"))
MAVLINK_BIND = os.environ.get("PEGASUS_MAVLINK_BIND", "0.0.0.0")
MAVLINK_PORT = int(os.environ.get("PEGASUS_MAVLINK_PORT", "4560"))
ADD_TEST_OBSTACLE = os.environ.get("AIRSTACK_ADD_TEST_OBSTACLE", "1").lower() not in {
    "0",
    "false",
    "no",
}
BENCHMARK_SCENE = os.environ.get("AIRSTACK_BENCHMARK_SCENE", "1").lower() not in {
    "0",
    "false",
    "no",
}
SPAWN_POSITION = np.array(
    [
        float(os.environ.get("AIRSTACK_SPAWN_X", "0.0")),
        float(os.environ.get("AIRSTACK_SPAWN_Y", "0.0")),
        float(os.environ.get("AIRSTACK_SPAWN_Z", "0.07")),
    ]
)
POLE_POSITION = np.array(
    [
        float(os.environ.get("AIRSTACK_POLE_X", "20.027")),
        float(os.environ.get("AIRSTACK_POLE_Y", "-1.963")),
        float(os.environ.get("AIRSTACK_POLE_CENTER_Z", "4.0")),
    ]
)
ADD_POLE_PROXY = os.environ.get("AIRSTACK_ADD_POLE_PROXY", "0").lower() in {
    "1",
    "true",
    "yes",
}
ANALYTIC_POLE_DEPTH = os.environ.get(
    "AIRSTACK_ANALYTIC_POLE_DEPTH", "1"
).lower() in {"1", "true", "yes"}


def require_file(path: str, label: str) -> None:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"{label} was not found: {path}")


class AirStackPegasusApp:
    def __init__(self) -> None:
        require_file(SCENE_USD, "RetroNeighborhood scene")
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(
            physics_dt=1.0 / 250.0,
            rendering_dt=1.0 / 30.0,
            stage_units_in_meters=1.0,
        )
        self.world = self.pg.world

        carb.log_info(f"[AIRSTACK] Loading scene: {SCENE_USD}")
        self.pg.load_environment(SCENE_USD)
        for _ in range(30):
            omni.kit.app.get_app().update()

        if BENCHMARK_SCENE:
            # Use the dedicated minimal benchmark USD selected by the launcher.
            # Deleting an environment prim after a render product exists can
            # silently stop Isaac camera annotators, so this mode only adds
            # deterministic ground, lighting and obstacles to the loaded USD.
            self.test_ground = GroundPlane(
                prim_path="/World/VLAAirStackGround",
                size=100.0,
                z_position=0.0,
                color=np.array([0.22, 0.25, 0.28]),
            )
            sun = UsdLux.DistantLight.Define(
                omni.usd.get_context().get_stage(), "/World/VLAAirStackSun"
            )
            sun.CreateIntensityAttr(3500.0)
            sun.CreateAngleAttr(0.6)
            carb.log_warn("[AIRSTACK_BENCHMARK_SCENE] controlled_ground=true")

        # A deterministic obstacle makes the VLA -> DROAN acceptance test
        # repeatable instead of depending on whichever building happens to be
        # in the camera frustum of a selected USD scene.
        if ADD_TEST_OBSTACLE:
            self.test_obstacle = FixedCuboid(
                name="vla_airstack_test_obstacle",
                prim_path="/World/VLAAirStackTestObstacle",
                position=np.array([2.0, 0.0, 0.85]),
                scale=np.array([0.45, 1.0, 1.7]),
                color=np.array([0.9, 0.15, 0.05]),
            )
            # DROAN deliberately rejects trajectories through unobserved
            # space. A finite far wall makes the open benchmark observable
            # without becoming part of the 3.5 m local planning corridor.
            self.benchmark_backstop = FixedCuboid(
                name="vla_airstack_benchmark_backstop",
                prim_path="/World/VLAAirStackBackstop",
                position=np.array([8.0, 0.0, 2.5]),
                scale=np.array([0.2, 20.0, 5.0]),
                color=np.array([0.08, 0.18, 0.32]),
            )
            carb.log_warn(
                "[AIRSTACK_TEST_OBSTACLE] center=(2.0,0.0,0.85) "
                "size=(0.45,1.0,1.7)"
            )

        # The RetroNeighborhood power-line meshes render correctly but do not
        # produce distance-to-image-plane samples in this Isaac 5.1 build.
        # Overlay a narrow cuboid collider at the selected real pole location.
        # FixedCuboid is used deliberately: unlike the imported meshes (and the
        # earlier FixedCylinder proxy), it is observed reliably by the RTX
        # distance-to-image-plane annotator in this Isaac 5.1 installation.
        # The brown envelope stays inside the visible pole silhouette while
        # giving both depth and physics a deterministic obstacle.
        if ADD_POLE_PROXY:
            self.power_pole_proxy = FixedCuboid(
                name="vla_power_pole_collision_proxy",
                prim_path="/World/VLAPowerPoleCollisionProxy",
                position=POLE_POSITION,
                scale=np.array([0.36, 0.36, 8.0]),
                color=np.array([0.20, 0.10, 0.04]),
            )
            carb.log_warn(
                "[AIRSTACK_POLE_PROXY] "
                f"center={tuple(float(v) for v in POLE_POSITION)} "
                "size=(0.36,0.36,8.0)"
            )

        mavlink = PX4MavlinkBackendConfig(
            {
                "vehicle_id": 0,
                "connection_type": "tcpin",
                "connection_ip": MAVLINK_BIND,
                "connection_baseport": MAVLINK_PORT,
                "px4_autolaunch": False,
                "px4_vehicle_model": "gazebo-classic_iris",
                "enable_lockstep": True,
                "update_rate": 250.0,
            }
        )

        config = MultirotorConfig()
        config.backends = [
            PX4MavlinkBackend(mavlink),
            ROS2Backend(
                vehicle_id=0,
                config={
                    "namespace": "/pegasus",
                    "pub_sensors": True,
                    "pub_graphical_sensors": False,
                    "pub_state": True,
                    "pub_tf": True,
                    "sub_control": False,
                    "use_sim_time": False,
                },
            ),
        ]
        self.camera_graph = ROS2CameraGraph(
                "body/Camera",
                config={
                    "resolution": [320, 240],
                    "qos_reliability": "reliable",
                    "qos_depth": 2,
                    # Isaac Sim 5.1 CameraInfoHelper works with this Pegasus
                    # graph, while its legacy viewport Image helpers can attach
                    # without ever producing frames. RGB/depth are therefore
                    # published from the Camera sensor directly below.
                    "types": ["camera_info"],
                    "namespace": "/robot_1/sensors/front_camera",
                    "topic": "/image",
                    "tf_frame_id": "camera_front",
                },
            )
        config.graphs = [self.camera_graph]

        self.vehicle = Multirotor(
            "/World/robot_1",
            ROBOTS["Iris"],
            0,
            SPAWN_POSITION.tolist(),
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config,
        )
        # Camera.set_local_pose() accepts a scalar-first quaternion expressed
        # in the selected camera axes and performs the USD basis conversion
        # internally.  Identity in ``world`` camera axes means body +X is
        # forward and body +Z is image-up.  Supplying an already converted USD
        # quaternion here applies the conversion twice and rolls RGB/depth.
        self.camera_graph.camera.set_local_pose(
            translation=np.array([0.30, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            camera_axes="world",
        )
        # Make the rendered RGB, CameraInfo and depth rays share one explicit
        # pinhole model.  The Pegasus camera default retained a 50 mm focal
        # length while the visible render product behaved as a wide-angle
        # camera, yielding fx~=763 for a 320 px image and invalid geometry.
        # focal/aperture=0.5 gives fx=fy=160 and a 90 degree horizontal FOV.
        self.camera_graph.camera.set_focal_length(0.018)
        # Set both apertures explicitly.  At 320x240, 36x27 mm with an
        # 18 mm focal length gives fx=fy=160; leaving vertical aperture at the
        # USD default produced fy=213.33 and distorted the ROS depth cloud.
        self.camera_graph.camera.set_horizontal_aperture(
            0.036, maintain_square_pixels=False
        )
        self.camera_graph.camera.set_vertical_aperture(
            0.027, maintain_square_pixels=False
        )
        self.camera_graph.camera.set_clipping_range(0.05, 100.0)
        carb.log_warn(
            "[AIRSTACK_CAMERA_EXTRINSICS] translation=(0.30,0.0,0.0) "
            "forward=body+x up=body+z camera_axes=world "
            "hfov=90deg fx=fy=160"
        )
        self.world.reset()
        # The ROS camera above is an onboard sensor and is not automatically
        # selected by Isaac Sim's desktop viewport.  Keep a separate observer
        # view aimed at the vehicle and deterministic benchmark obstacle so
        # the GUI never opens on an empty grey background.
        for _ in range(5):
            omni.kit.app.get_app().update()
        self.configure_observer_view("initial")
        for _ in range(5):
            simulation_app.update()
        self.camera_graph.camera.add_distance_to_image_plane_to_frame()
        # Isaac Sim 5.1 can return an all-infinite image-plane depth buffer for
        # an otherwise healthy RGB render product.  Attach radial distance as
        # a verified fallback and convert it to optical-axis depth below.
        self.camera_graph.camera.add_distance_to_camera_to_frame()
        if not rclpy.ok():
            rclpy.init(args=None)
        self.image_node = rclpy.create_node("isaac_direct_camera_publisher")
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.rgb_publisher = self.image_node.create_publisher(
            Image, "/robot_1/sensors/front_camera/image/rgb", image_qos
        )
        self.depth_publisher = self.image_node.create_publisher(
            Image, "/robot_1/sensors/front_camera/image/depth", image_qos
        )
        self._physics_frames = 0
        self._camera_frames = 0
        self._reported_radial_depth_fallback = False
        self._reported_analytic_projection = False
        carb.log_warn(
            "[AIRSTACK_SIM_READY] scene=RetroNeighborhood vehicle=Iris "
            f"spawn={tuple(float(v) for v in SPAWN_POSITION)} "
            f"mavlink=tcp://{MAVLINK_BIND}:{MAVLINK_PORT} "
            f"ros_domain={ROS_DOMAIN_ID} "
            "rgb=/robot_1/sensors/front_camera/image/rgb "
            "depth=/robot_1/sensors/front_camera/image/depth"
        )

    def configure_observer_view(self, phase: str) -> bool:
        """Bind the visible editor viewport, not a sensor render viewport."""
        viewport_api = get_viewport_from_window_name("Viewport")
        if viewport_api is None:
            carb.log_warn(
                f"[AIRSTACK_VIEWPORT_MISSING] phase={phase} window=Viewport"
            )
            return False

        viewport_api.camera_path = "/OmniverseKit_Persp"
        observer_target = np.array(
            [
                0.5 * (SPAWN_POSITION[0] + POLE_POSITION[0]),
                0.5 * (SPAWN_POSITION[1] + POLE_POSITION[1]),
                1.0,
            ]
        )
        observer_eye = observer_target + np.array([5.5, -8.0, 5.0])
        set_camera_view(
            eye=observer_eye,
            target=observer_target,
            camera_prim_path="/OmniverseKit_Persp",
            viewport_api=viewport_api,
        )
        carb.log_warn(
            f"[AIRSTACK_VIEWPORT_READY] phase={phase} "
            f"eye={tuple(float(v) for v in observer_eye)} "
            f"target={tuple(float(v) for v in observer_target)} "
            f"active={viewport_api.camera_path}"
        )
        return True

    def run(self) -> None:
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
            self._physics_frames += 1
            # Pegasus initializes the graphical camera during the first
            # simulation frames and may temporarily select it in the editor.
            # Re-apply the observer camera once that startup work is finished.
            if self._physics_frames == 60:
                self.configure_observer_view("frame_60")
            if self._physics_frames % 8 == 0:
                self.publish_camera_frame()
            rclpy.spin_once(self.image_node, timeout_sec=0.0)
        self.timeline.stop()
        self.image_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()

    def publish_camera_frame(self) -> None:
        rgba = self.camera_graph.camera.get_rgba()
        depth = self.camera_graph.camera.get_depth()
        if rgba is None or depth is None:
            return

        rgb_array = np.asarray(rgba)[..., :3]
        # Camera.get_rgba() returns normalized float pixels on Isaac Sim 5.1.
        # A direct uint8 cast would turn the whole observation almost black.
        if np.issubdtype(rgb_array.dtype, np.floating):
            rgb_array = np.clip(rgb_array * 255.0, 0.0, 255.0)
        rgb = np.ascontiguousarray(rgb_array, dtype=np.uint8)
        depth_array = np.asarray(depth, dtype=np.float32).squeeze()
        if not np.any(np.isfinite(depth_array) & (depth_array > 0.0)):
            radial = self.camera_graph.camera.get_current_frame().get(
                "distance_to_camera"
            )
            if radial is not None:
                radial = np.asarray(radial, dtype=np.float32).squeeze()
                if np.any(np.isfinite(radial) & (radial > 0.0)):
                    intrinsics = self.camera_graph.camera.get_intrinsics_matrix()
                    fx = float(intrinsics[0, 0])
                    # Isaac Sim 5.1 reports fy=fx*(width/height) here even
                    # though the render product uses square pixels.  Keep the
                    # radial-to-optical conversion identical to the corrected
                    # ROS CameraInfo consumed by AirStack.
                    fy = fx
                    cx, cy = float(intrinsics[0, 2]), float(intrinsics[1, 2])
                    pixel_x, pixel_y = np.meshgrid(
                        np.arange(radial.shape[1], dtype=np.float32),
                        np.arange(radial.shape[0], dtype=np.float32),
                    )
                    ray_scale = np.sqrt(
                        1.0
                        + ((pixel_x - cx) / fx) ** 2
                        + ((pixel_y - cy) / fy) ** 2
                    )
                    depth_array = radial / ray_scale
                    if not self._reported_radial_depth_fallback:
                        carb.log_warn(
                            "[AIRSTACK_CAMERA_DEPTH_FALLBACK] "
                            "radial distance converted to optical-axis depth"
                        )
                        self._reported_radial_depth_fallback = True
        if ADD_POLE_PROXY and ANALYTIC_POLE_DEPTH:
            depth_array = self.overlay_analytic_pole_depth(depth_array)
        depth_f32 = np.ascontiguousarray(depth_array, dtype=np.float32)
        stamp = self.image_node.get_clock().now().to_msg()

        rgb_message = Image()
        rgb_message.header.stamp = stamp
        rgb_message.header.frame_id = "camera_front"
        rgb_message.height, rgb_message.width = rgb.shape[:2]
        rgb_message.encoding = "rgb8"
        rgb_message.is_bigendian = False
        rgb_message.step = rgb_message.width * 3
        rgb_message.data = rgb.tobytes()
        self.rgb_publisher.publish(rgb_message)

        depth_message = Image()
        depth_message.header.stamp = stamp
        depth_message.header.frame_id = "camera_front"
        depth_message.height, depth_message.width = depth_f32.shape[:2]
        depth_message.encoding = "32FC1"
        depth_message.is_bigendian = False
        depth_message.step = depth_message.width * 4
        depth_message.data = depth_f32.tobytes()
        self.depth_publisher.publish(depth_message)
        self._camera_frames += 1
        if self._camera_frames == 1:
            carb.log_warn(
                f"[AIRSTACK_CAMERA_READY] rgb={rgb_message.width}x{rgb_message.height} "
                f"depth={depth_message.width}x{depth_message.height}"
            )

    def overlay_analytic_pole_depth(self, rendered_depth: np.ndarray) -> np.ndarray:
        """Fuse the known simulation pole collider into optical-axis depth.

        The RTX radial annotator in this Isaac 5.1 build observes the imported
        neighborhood but intermittently omits runtime-created fixed geometry.
        Ray/AABB intersection against the exact collider supplies only those
        missing pole pixels; all other rendered depth samples are preserved.
        """
        depth = np.asarray(rendered_depth, dtype=np.float32)
        height, width = depth.shape
        intrinsics = self.camera_graph.camera.get_intrinsics_matrix()
        fx = float(intrinsics[0, 0])
        # See publish_camera_frame(): use the calibrated square-pixel model
        # for analytic obstacle rays as well as the published CameraInfo.
        fy = fx
        cx, cy = float(intrinsics[0, 2]), float(intrinsics[1, 2])
        pixel_x, pixel_y = np.meshgrid(
            np.arange(width, dtype=np.float64),
            np.arange(height, dtype=np.float64),
        )
        # Camera ``world`` axes are +X forward, +Y left, +Z up.  Pixel x
        # increases toward camera-right and pixel y toward camera-down.
        rays_camera = np.stack(
            (
                np.ones_like(pixel_x),
                -(pixel_x - cx) / fx,
                -(pixel_y - cy) / fy,
            ),
            axis=-1,
        )
        camera_position, camera_orientation = (
            self.camera_graph.camera.get_world_pose(camera_axes="world")
        )
        quaternion_wxyz = np.asarray(camera_orientation, dtype=np.float64)
        camera_rotation = Rotation.from_quat(
            [
                quaternion_wxyz[1],
                quaternion_wxyz[2],
                quaternion_wxyz[3],
                quaternion_wxyz[0],
            ]
        ).as_matrix()
        rays_world = rays_camera @ camera_rotation.T

        pole_in_camera = camera_rotation.T @ (
            POLE_POSITION - np.asarray(camera_position, dtype=np.float64)
        )
        if not self._reported_analytic_projection:
            projected_u = cx - fx * pole_in_camera[1] / pole_in_camera[0]
            projected_v = cy - fy * pole_in_camera[2] / pole_in_camera[0]
            carb.log_warn(
                "[AIRSTACK_ANALYTIC_POLE_PROJECTION] "
                f"camera_world={tuple(float(v) for v in camera_position)} "
                f"pole_camera={tuple(float(v) for v in pole_in_camera)} "
                f"pixel=({projected_u:.2f},{projected_v:.2f})"
            )
            self._reported_analytic_projection = True

        box_half_extent = np.array([0.18, 0.18, 4.0], dtype=np.float64)
        box_min = POLE_POSITION - box_half_extent
        box_max = POLE_POSITION + box_half_extent
        origin = np.asarray(camera_position, dtype=np.float64)
        safe_rays = np.where(
            np.abs(rays_world) < 1e-9,
            np.copysign(1e-9, rays_world + 1e-12),
            rays_world,
        )
        inverse_rays = 1.0 / safe_rays
        first = (box_min - origin) * inverse_rays
        second = (box_max - origin) * inverse_rays
        near = np.max(np.minimum(first, second), axis=-1)
        far = np.min(np.maximum(first, second), axis=-1)
        hit = (far >= np.maximum(near, 0.0)) & (near > 0.05)
        if not np.any(hit):
            return depth
        pole_depth = np.full(depth.shape, np.inf, dtype=np.float32)
        pole_depth[hit] = near[hit].astype(np.float32)
        fused = np.minimum(depth, pole_depth)
        if self._camera_frames == 0:
            carb.log_warn(
                "[AIRSTACK_ANALYTIC_POLE_DEPTH] "
                f"pixels={int(np.count_nonzero(hit))} "
                f"minimum={float(np.min(pole_depth[hit])):.3f}m"
            )
        return fused


def main() -> None:
    try:
        AirStackPegasusApp().run()
    except Exception as exc:
        carb.log_error(f"[AIRSTACK_SIM_FATAL] {type(exc).__name__}: {exc}")
        simulation_app.close()
        raise


if __name__ == "__main__":
    main()
