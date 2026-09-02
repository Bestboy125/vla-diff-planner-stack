#!/usr/bin/env python3
import asyncio
import hmac
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import actionlib
import cv2
import numpy as np
import rospy
import websockets
from actionlib_msgs.msg import GoalStatus
from atomic_skill_executor.msg import ExecuteAtomicSkillAction, ExecuteAtomicSkillGoal
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from command_protocol import ProtocolError, validate_command


class MobileGateway:
    def __init__(self):
        gp = rospy.get_param
        self.ws_host = gp("~websocket_host", "0.0.0.0")
        self.ws_port = int(gp("~websocket_port", 8765))
        self.http_host = gp("~http_host", "0.0.0.0")
        self.http_port = int(gp("~http_port", 8080))
        self.advertise_host = gp("~advertise_host", "127.0.0.1")
        self.video_auth_required = bool(gp("~video_auth_required", True))
        self.command_forwarding_enabled = bool(gp("~command_forwarding_enabled", False))
        self.mission_forwarding_enabled = bool(gp("~mission_forwarding_enabled", False))
        self.atomic_action_name = gp("~atomic_action_name", "/atomic_skill_executor/execute")
        self.odom_timeout = float(gp("~odom_timeout", 1.0))
        self.primary_odom_topic = gp("~primary_odom_topic", "/ekf/ekf_odom")
        self.fallback_odom_topic = gp("~fallback_odom_topic", "/mavros/local_position/odom")
        self.camera_mode = gp("~camera_mode", "ros_compressed")
        self.stream_max_fps = float(gp("~stream_max_fps", 15.0))
        self.jpeg_quality = int(gp("~jpeg_quality", 75))

        self.limits = {
            "command_ttl_sec": float(gp("~command_ttl_sec", 10.0)),
            "max_clock_skew_sec": float(gp("~max_clock_skew_sec", 5.0)),
            "max_move_distance": float(gp("~max_move_distance", 3.0)),
            "max_rotate_angle_deg": float(gp("~max_rotate_angle_deg", 180.0)),
            "max_orbit_radius": float(gp("~max_orbit_radius", 5.0)),
            "max_orbit_angle_deg": float(gp("~max_orbit_angle_deg", 360.0)),
        }
        self.defaults = {
            "move_distance": float(gp("~default_move_distance", 0.35)),
            "rotate_angle_deg": float(gp("~default_rotate_angle_deg", 20.0)),
            "orbit_radius": float(gp("~default_orbit_radius", 1.0)),
            "orbit_angle_deg": float(gp("~default_orbit_angle_deg", 45.0)),
            "timeout": float(gp("~default_timeout", 30.0)),
        }
        self.auth_token = self._load_token(
            gp("~auth_token", ""), gp("~auth_token_file", ""))

        self.state_lock = threading.Lock()
        self.flight_state = None
        self.battery = None
        self.odom = {}
        self.active_request_id = None
        self.active_skill = "NONE"
        self.seen_requests = {}

        self.frame_condition = threading.Condition()
        self.frame_sequence = 0
        self.latest_jpeg = self._placeholder_frame("WAITING FOR CAMERA")
        self.image_source = "placeholder"
        self.capture_stop = threading.Event()
        self.capture_thread = None

        self.async_loop = None
        self.async_stop = None
        self.websocket_clients = set()
        self.session_clocks = {}
        self.websocket_thread = None
        self.http_server = None
        self.http_thread = None

        rospy.Subscriber(gp("~flight_state_topic", "/mavros/state"), State,
                         self._on_flight_state, queue_size=1)
        rospy.Subscriber(gp("~battery_topic", "/mavros/battery"), BatteryState,
                         self._on_battery, queue_size=1)
        rospy.Subscriber(self.primary_odom_topic, Odometry,
                         lambda msg: self._on_odom("primary", msg), queue_size=1)
        if self.fallback_odom_topic != self.primary_odom_topic:
            rospy.Subscriber(self.fallback_odom_topic, Odometry,
                             lambda msg: self._on_odom("fallback", msg), queue_size=1)

        if self.camera_mode == "ros_compressed":
            rospy.Subscriber(gp("~compressed_image_topic", "/camera/color/image_raw/compressed"),
                             CompressedImage, self._on_compressed_image, queue_size=1,
                             buff_size=4 * 1024 * 1024)
        elif self.camera_mode == "gstreamer_csi":
            self._start_csi_capture()
        elif self.camera_mode != "disabled":
            raise RuntimeError("camera_mode must be ros_compressed, gstreamer_csi or disabled")

        self.mission_publisher = rospy.Publisher(
            gp("~mission_request_topic", "/mobile_gateway/mission_request"),
            String, queue_size=10)
        self.action_client = actionlib.SimpleActionClient(
            self.atomic_action_name, ExecuteAtomicSkillAction)

        self._start_http_server()
        self._start_websocket_server()
        rospy.Timer(rospy.Duration(1.0 / max(0.2, float(gp("~status_rate", 2.0)))),
                    self._status_timer)
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn("mobile gateway ready; forwarding=%s mission_forwarding=%s camera=%s",
                      self.command_forwarding_enabled, self.mission_forwarding_enabled,
                      self.camera_mode)

    @staticmethod
    def _load_token(configured, path):
        token = str(configured).strip()
        if not token and path:
            with open(os.path.expanduser(path), "r", encoding="utf-8") as handle:
                token = handle.read().strip()
        if len(token) < 16:
            raise RuntimeError("mobile gateway auth token must contain at least 16 characters")
        return token

    def _on_flight_state(self, message):
        with self.state_lock:
            self.flight_state = message

    def _on_battery(self, message):
        with self.state_lock:
            self.battery = message

    def _on_odom(self, source, message):
        with self.state_lock:
            self.odom[source] = (time.monotonic(), message)

    def _on_compressed_image(self, message):
        data = bytes(message.data)
        if len(data) < 4 or data[:2] != b"\xff\xd8":
            return
        header = getattr(message, "_connection_header", {}) or {}
        self._set_frame(data, "ros:" + header.get("topic", "compressed"))

    def _set_frame(self, jpeg, source):
        with self.frame_condition:
            self.latest_jpeg = jpeg
            self.image_source = source
            self.frame_sequence += 1
            self.frame_condition.notify_all()

    def _get_frame(self, previous, timeout=2.0):
        with self.frame_condition:
            if self.frame_sequence == previous:
                self.frame_condition.wait(timeout)
            return self.frame_sequence, self.latest_jpeg, self.image_source

    def _placeholder_frame(self, text):
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        image[:] = (20, 36, 58)
        cv2.putText(image, text, (95, 180), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (230, 215, 50), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", image,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        return encoded.tobytes() if ok else b""

    def _start_csi_capture(self):
        gp = rospy.get_param
        sensor_id = int(gp("~camera_sensor_id", 0))
        width = int(gp("~camera_width", 1280))
        height = int(gp("~camera_height", 720))
        fps = int(gp("~camera_fps", 20))
        pipeline = (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM),width=%d,height=%d,format=NV12,framerate=%d/1 ! "
            "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
        ) % (sensor_id, width, height, fps)

        def capture():
            camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not camera.isOpened():
                rospy.logerr("failed to open CSI camera; MJPEG remains on placeholder")
                return
            try:
                while not rospy.is_shutdown() and not self.capture_stop.is_set():
                    ok, image = camera.read()
                    if not ok:
                        time.sleep(0.05)
                        continue
                    encoded_ok, encoded = cv2.imencode(
                        ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                    if encoded_ok:
                        self._set_frame(encoded.tobytes(), "gstreamer_csi:%d" % sensor_id)
            finally:
                camera.release()

        self.capture_thread = threading.Thread(target=capture, name="csi-camera", daemon=True)
        self.capture_thread.start()

    def _authorized_http(self, handler):
        if not self.video_auth_required:
            return True
        header = handler.headers.get("Authorization", "")
        query = parse_qs(urlparse(handler.path).query)
        supplied = header[7:] if header.startswith("Bearer ") else query.get("token", [""])[0]
        return hmac.compare_digest(supplied, self.auth_token)

    def _start_http_server(self):
        gateway = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                rospy.logdebug("video http: " + fmt, *args)

            def do_GET(self):
                path = urlparse(self.path).path
                if path == "/health":
                    payload = json.dumps({"ok": True, "camera_source": gateway.image_source}).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                if path not in ("/snapshot.jpg", "/stream.mjpg"):
                    self.send_error(404)
                    return
                if not gateway._authorized_http(self):
                    self.send_error(401, "Bearer token required")
                    return
                if path == "/snapshot.jpg":
                    _, frame, _ = gateway._get_frame(-1, 0)
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    return
                self.send_response(200)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                sequence = -1
                period = 1.0 / max(1.0, gateway.stream_max_fps)
                try:
                    while not rospy.is_shutdown():
                        sequence, frame, _ = gateway._get_frame(sequence, 2.0)
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(("Content-Length: %d\r\n\r\n" % len(frame)).encode())
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        time.sleep(period)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.http_server = ThreadingHTTPServer((self.http_host, self.http_port), Handler)
        self.http_thread = threading.Thread(target=self.http_server.serve_forever,
                                            name="mjpeg-http", daemon=True)
        self.http_thread.start()

    def _start_websocket_server(self):
        ready = threading.Event()

        def run():
            self.async_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.async_loop)
            self.async_stop = asyncio.Event()

            async def serve():
                server = await websockets.serve(self._websocket_handler,
                                                self.ws_host, self.ws_port,
                                                ping_interval=10, ping_timeout=10,
                                                max_size=65536)
                ready.set()
                await self.async_stop.wait()
                server.close()
                await server.wait_closed()

            self.async_loop.run_until_complete(serve())
            self.async_loop.close()

        self.websocket_thread = threading.Thread(target=run, name="websocket", daemon=True)
        self.websocket_thread.start()
        if not ready.wait(5.0):
            raise RuntimeError("websocket server did not start")

    async def _websocket_handler(self, websocket, path):
        if path != "/ws/control":
            await websocket.close(code=1008, reason="unsupported path")
            return
        try:
            raw = await asyncio.wait_for(websocket.recv(), timeout=5.0)
            hello = json.loads(raw)
            supplied = str(hello.get("auth_token", ""))
            if hello.get("type") != "hello" or hello.get("protocol_version") != "1.0":
                await websocket.close(code=1008, reason="hello required")
                return
            if not hmac.compare_digest(supplied, self.auth_token):
                await websocket.close(code=1008, reason="authentication failed")
                return
            client_time_ms = hello.get("client_time_ms")
            wall_clock_valid = time.time() >= 1577836800
            if not wall_clock_valid and not isinstance(client_time_ms, int):
                await websocket.close(code=1008, reason="client_time_ms required while onboard clock is unsynchronized")
                return
            if isinstance(client_time_ms, int):
                self.session_clocks[websocket] = (time.monotonic(), client_time_ms)
            self.websocket_clients.add(websocket)
            await websocket.send(json.dumps({
                "type": "hello_ack", "protocol_version": "1.0",
                "message": "authenticated",
                "command_forwarding_enabled": self.command_forwarding_enabled,
                "mission_forwarding_enabled": self.mission_forwarding_enabled,
            }))
            await websocket.send(json.dumps(self._status_payload()))
            async for message in websocket:
                await self._handle_websocket_message(websocket, message)
        except (asyncio.TimeoutError, json.JSONDecodeError):
            await websocket.close(code=1008, reason="invalid handshake")
        except websockets.ConnectionClosed:
            pass
        finally:
            self.websocket_clients.discard(websocket)
            self.session_clocks.pop(websocket, None)

    async def _handle_websocket_message(self, websocket, raw):
        try:
            payload = json.loads(raw)
            command = validate_command(payload, self.limits, now_ms=self._session_now_ms(websocket))
            request_id = command["request_id"]
            self._expire_replay_cache()
            if request_id in self.seen_requests:
                raise ProtocolError("REPLAY", "request_id was already handled")
            self.seen_requests[request_id] = time.monotonic()
            await self._dispatch_command(websocket, command)
        except ProtocolError as error:
            request_id = "--"
            try:
                request_id = json.loads(raw).get("request_id", "--")
            except Exception:
                pass
            await self._send_ack(websocket, request_id, False, "REJECTED_" + error.code,
                                 error.message)
        except (json.JSONDecodeError, TypeError) as error:
            await self._send_ack(websocket, "--", False, "REJECTED_INVALID_JSON", str(error))

    def _expire_replay_cache(self):
        cutoff = time.monotonic() - 300.0
        self.seen_requests = {key: stamp for key, stamp in self.seen_requests.items()
                              if stamp >= cutoff}

    def _session_now_ms(self, websocket):
        base = self.session_clocks.get(websocket)
        if base:
            return int(base[1] + (time.monotonic() - base[0]) * 1000.0)
        return int(time.time() * 1000)

    async def _dispatch_command(self, websocket, command):
        request_id = command["request_id"]
        action = command["action"]
        if not self.command_forwarding_enabled:
            await self._send_ack(websocket, request_id, False, "REJECTED_SAFETY_GATE",
                                 "command forwarding is disabled on the onboard gateway")
            return

        if action in ("START_TAKEOFF_LOCALIZE_ORBIT", "LAND"):
            if not self.mission_forwarding_enabled or self.mission_publisher.get_num_connections() == 0:
                await self._send_ack(websocket, request_id, False, "REJECTED_UNAVAILABLE",
                                     "no authenticated onboard mission orchestrator is available")
                return
            self.mission_publisher.publish(String(data=json.dumps(command, ensure_ascii=False)))
            await self._send_ack(websocket, request_id, True, "QUEUED",
                                 "forwarded to onboard mission orchestrator")
            return

        if action == "EMERGENCY_STOP":
            self.action_client.cancel_all_goals()
            with self.state_lock:
                self.active_request_id = None
                self.active_skill = "NONE"
            await self._send_ack(websocket, request_id, True, "CANCEL_REQUESTED",
                                 "active atomic skill cancellation requested; flight safety remains onboard")
            return

        with self.state_lock:
            if self.active_request_id is not None:
                active = self.active_request_id
            else:
                active = None
        if active:
            await self._send_ack(websocket, request_id, False, "REJECTED_BUSY",
                                 "another atomic skill is active: " + active)
            return
        if not self.action_client.wait_for_server(rospy.Duration(0.2)):
            await self._send_ack(websocket, request_id, False, "REJECTED_UNAVAILABLE",
                                 "atomic skill action server is unavailable")
            return

        goal = self._build_atomic_goal(command)
        with self.state_lock:
            self.active_request_id = request_id
            self.active_skill = action
        self.action_client.send_goal(
            goal,
            done_cb=lambda state, result: self._action_done(websocket, command, state, result),
            feedback_cb=lambda feedback: self._action_feedback(websocket, command, feedback),
        )
        await self._send_ack(websocket, request_id, True, "QUEUED",
                             "accepted by onboard atomic skill action client")

    def _build_atomic_goal(self, command):
        action = command["action"]
        args = command["arguments"]
        goal = ExecuteAtomicSkillGoal()
        goal.skill = action
        goal.timeout = self.defaults["timeout"]
        if action == "MOVE":
            directions = {
                "FORWARD": "forward", "BACKWARD": "back", "LEFT": "left",
                "RIGHT": "right", "UP": "up", "DOWN": "down",
            }
            goal.direction = directions[args["direction"]]
            goal.distance = float(args.get("distance_m", self.defaults["move_distance"]))
        elif action == "ROTATE":
            goal.direction = "left" if args["direction"] == "COUNTERCLOCKWISE" else "right"
            degrees = float(args.get("angle_deg", self.defaults["rotate_angle_deg"]))
            goal.angle = math.radians(degrees)
        elif action == "ORBIT":
            direction = args.get("direction", "COUNTERCLOCKWISE")
            goal.direction = "ccw" if direction == "COUNTERCLOCKWISE" else "cw"
            goal.radius = float(args.get("radius_m", self.defaults["orbit_radius"]))
            goal.orbit_angle = math.radians(float(
                args.get("orbit_angle_deg", self.defaults["orbit_angle_deg"])))
            goal.center_frame = "relative_body"
            goal.center.y = goal.radius
            goal.yaw_mode = "face_center"
        return goal

    def _action_feedback(self, websocket, command, feedback):
        self._send_from_ros_thread(websocket, {
            "type": "command_ack", "request_id": command["request_id"],
            "accepted": True, "state": feedback.status,
            "message": "atomic skill progress %.1f%%" % (feedback.progress * 100.0),
            "progress": feedback.progress,
        })

    def _action_done(self, websocket, command, state, result):
        with self.state_lock:
            self.active_request_id = None
            self.active_skill = "NONE"
        success = bool(result and result.success and state == GoalStatus.SUCCEEDED)
        result_state = result.status if result else "FAILED_NO_RESULT"
        message = result.message if result else "atomic skill returned no result"
        self._send_from_ros_thread(websocket, {
            "type": "command_ack", "request_id": command["request_id"],
            "accepted": success, "state": result_state, "message": message,
        })

    def _send_from_ros_thread(self, websocket, payload):
        if self.async_loop and self.async_loop.is_running():
            asyncio.run_coroutine_threadsafe(self._safe_send(websocket, payload), self.async_loop)

    async def _safe_send(self, websocket, payload):
        if not websocket.closed:
            await websocket.send(json.dumps(payload, ensure_ascii=False))

    async def _send_ack(self, websocket, request_id, accepted, state, message):
        await self._safe_send(websocket, {
            "type": "command_ack", "request_id": request_id,
            "accepted": accepted, "state": state, "message": message,
        })

    def _select_odom(self):
        now = time.monotonic()
        primary = self.odom.get("primary")
        fallback = self.odom.get("fallback")
        if primary and now - primary[0] <= self.odom_timeout:
            return "FAST_LIO_EKF", primary[1], now - primary[0]
        if fallback and now - fallback[0] <= self.odom_timeout:
            return "MAVROS_LOCAL", fallback[1], now - fallback[0]
        candidate = primary or fallback
        return ("STALE", candidate[1] if candidate else None,
                now - candidate[0] if candidate else None)

    def _status_payload(self):
        with self.state_lock:
            state = self.flight_state
            battery = self.battery
            odom_source, odom, age = self._select_odom()
            active_skill = self.active_skill
            active_request = self.active_request_id
        percentage = None
        if battery is not None and math.isfinite(battery.percentage) and battery.percentage >= 0:
            percentage = round(battery.percentage * 100.0 if battery.percentage <= 1.0
                               else battery.percentage, 1)
        position = None
        if odom is not None:
            point = odom.pose.pose.position
            position = {"x": round(point.x, 3), "y": round(point.y, 3), "z": round(point.z, 3)}
        return {
            "type": "status",
            "status": {
                "flight_mode": state.mode if state else "UNKNOWN",
                "armed": bool(state.armed) if state else False,
                "flight_controller_connected": bool(state.connected) if state else False,
                "battery_percent": percentage,
                "localization_state": "TRACKING" if odom_source != "STALE" else "STALE",
                "localization_source": odom_source,
                "lidar_localization_active": odom_source == "FAST_LIO_EKF",
                "position": position,
                "active_skill": active_skill,
                "active_request_id": active_request,
                "camera_source": self.image_source,
                "video_url": "http://%s:%d/stream.mjpg" % (self.advertise_host, self.http_port),
                "command_forwarding_enabled": self.command_forwarding_enabled,
                "clock_synchronized": time.time() >= 1577836800,
                "timestamp_ms": int(time.time() * 1000) if time.time() >= 1577836800 else None,
            },
        }

    def _status_timer(self, _event):
        if not self.async_loop or not self.async_loop.is_running():
            return
        payload = self._status_payload()
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self.async_loop)

    async def _broadcast(self, payload):
        if not self.websocket_clients:
            return
        message = json.dumps(payload, ensure_ascii=False)
        await asyncio.gather(
            *[client.send(message) for client in list(self.websocket_clients) if not client.closed],
            return_exceptions=True)

    def shutdown(self):
        self.capture_stop.set()
        if self.http_server:
            self.http_server.shutdown()
            self.http_server.server_close()
        if self.async_loop and self.async_stop:
            self.async_loop.call_soon_threadsafe(self.async_stop.set)


def main():
    rospy.init_node("mobile_gateway")
    MobileGateway()
    rospy.spin()


if __name__ == "__main__":
    main()
