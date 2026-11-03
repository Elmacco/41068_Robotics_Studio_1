#!/usr/bin/env python3
"""HTTP bridge between the ROS graph and the dashboard page in GUI/.

Top-down map (the "Top Down Map" panel):
  GET  /api/map.png     terrain/overview, the terrain mapper's picture of the
                        map (+y up)
  GET  /api/map/state   where that picture sits in the world, the trails, the
                        points of interest and the last ranger route
  GET  /api/map/graph   the route roadmap's nodes and edges
  POST /api/route       {"x": .., "y": ..}: plan a ranger route from the
                        station to that point (plan_ranger_route service)

Points of interest come from points_of_interest (std_msgs/String, JSON):
  {"points": [{"id": "P1", "x": -15.0, "y": 12.0, "type": "red"}, ...]}
with type "red", "green" or "person". Nothing publishes it yet; it is the
slot for the people found by the detection part. To try it by hand:
  ros2 topic pub --once /parrot1/points_of_interest std_msgs/msg/String \
    "{data: '{\"points\": [{\"id\": \"P1\", \"x\": -15, \"y\": 12, \"type\": \"red\"}]}'}"
"""

import base64
import json
import math
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from socketserver import ThreadingMixIn
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

POI_TYPES = ('red', 'green', 'person')

# Telemetry shows MANUAL for this long (s) after the last manual command;
# the page resends held buttons every 0.3 s.
MANUAL_HOLD = 1.0

# Sent on a camera stream until the first frame arrives.
_, BLANK_JPEG = cv2.imencode('.jpg', np.zeros((240, 320, 3), dtype=np.uint8))
BLANK_JPEG = BLANK_JPEG.tobytes()


class DashboardState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latitude = -33.8688
        self.longitude = 151.2093
        # Map position, None until the first odometry (the latitude/longitude
        # defaults above are display placeholders, not map coordinates)
        self.x = None
        self.y = None
        self.altitude = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.battery = 98.0
        # Mission mode from mission_state; MANUAL overrides it while manual
        # commands keep arriving (see telemetry_snapshot).
        self.mode = 'STANDBY'
        self.last_manual = None          # time.monotonic() of the last one
        # Latest JPEG per camera, and a count that goes up with each new one,
        # so a stream can wait for the next frame instead of resending.
        self.frames = {'rgb': None, 'thermal': None}
        self.frame_seq = {'rgb': 0, 'thermal': 0}
        self.new_frame = threading.Condition(self.lock)
        # Top-down map
        self.map_png = None
        self.map_info = None
        self.trails = []
        self.points_of_interest = []
        self.route = None
        self.graph = None

    def telemetry_snapshot(self):
        with self.lock:
            manual = (self.last_manual is not None
                      and time.monotonic() - self.last_manual < MANUAL_HOLD)
            return {
                # latitude/longitude carry the map x/y once odometry arrives
                # (see odom_callback); x and y name them plainly for the map
                # panel, and stay None until then.
                'x': self.x,
                'y': self.y,
                'latitude': self.latitude,
                'longitude': self.longitude,
                'altitude': self.altitude,
                'heading': self.heading,
                'speed': self.speed,
                'battery': self.battery,
                'mode': 'MANUAL' if manual else self.mode,
            }

    def set_odom(self, latitude, longitude, altitude, heading, speed):
        with self.lock:
            self.latitude = latitude
            self.longitude = longitude
            self.x = latitude
            self.y = longitude
            self.altitude = altitude
            self.heading = heading
            self.speed = speed

    def set_mode(self, mode):
        with self.lock:
            self.mode = mode

    def manual_command_sent(self):
        with self.lock:
            self.last_manual = time.monotonic()

    def set_frame(self, which, frame_bytes):
        with self.lock:
            self.frames[which] = frame_bytes
            self.frame_seq[which] += 1
            self.new_frame.notify_all()

    def get_frame(self, which):
        with self.lock:
            return self.frames[which]

    def wait_frame(self, which, last_seq, timeout):
        """Wait up to timeout (s) for a frame newer than last_seq.

        Returns (seq, frame); seq == last_seq if none arrived in time.
        """
        with self.new_frame:
            self.new_frame.wait_for(lambda: self.frame_seq[which] != last_seq, timeout)
            return self.frame_seq[which], self.frames[which]

    def set_map(self, **values):
        with self.lock:
            for key, value in values.items():
                setattr(self, key, value)

    def map_snapshot(self):
        with self.lock:
            return {
                'info': self.map_info,
                'trails': self.trails,
                'points_of_interest': self.points_of_interest,
                'route': self.route,
            }

    def get_map(self, key):
        with self.lock:
            return getattr(self, key)


class BridgeNode(Node):
    def __init__(self, state: DashboardState):
        super().__init__('web_dashboard_bridge')
        self.state = state
        self.command_pub = self.create_publisher(String, '/parrot1/mission_command', 10)
        self.manual_pub = self.create_publisher(Twist, '/parrot1/cmd_vel', 10)
        self.target_altitude_pub = self.create_publisher(Float64, '/parrot1/target_altitude', 10)
        self.create_subscription(Odometry, '/parrot1/odometry', self.odom_callback, 10)
        self.create_subscription(Image, '/parrot1/camera/image', self.rgb_callback, 10)
        self.create_subscription(Image, '/parrot1/thermal_camera/image', self.thermal_callback, 10)

        # Top-down map and mission state. These topics are latched, so
        # subscribe the same way to get the latest message even if it was
        # sent before we started.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/parrot1/mission_state', self.mission_state_callback, latched)
        self.create_subscription(Image, '/parrot1/terrain/overview', self.overview_callback, latched)
        self.create_subscription(String, '/parrot1/terrain/overview_info', self.overview_info_callback, latched)
        self.create_subscription(String, '/parrot1/terrain/route_graph', self.graph_callback, latched)
        self.create_subscription(String, '/parrot1/trails', self.trails_callback, latched)
        self.create_subscription(String, '/parrot1/points_of_interest', self.poi_callback, latched)
        self.create_subscription(Path, '/parrot1/ranger_route', self.route_callback, latched)
        self.create_subscription(String, '/parrot1/ranger_route/info', self.route_info_callback, latched)
        self.route_client = self.create_client(GetPlan, '/parrot1/plan_ranger_route')
        self.route_points = []
        self.route_info = None

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear = msg.twist.twist.linear

        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        heading_deg = (math.degrees(yaw) + 360.0) % 360.0
        speed = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)

        self.state.set_odom(position.x, position.y, position.z, heading_deg, speed)

    def mission_state_callback(self, msg: String):
        # "MAPPING: 42% observed, flying to F7" -> "MAPPING"
        self.state.set_mode(msg.data.split(':', 1)[0].strip())

    def rgb_callback(self, msg: Image):
        frame = self._image_to_jpeg(msg, thermal=False)
        if frame is not None:
            self.state.set_frame('rgb', frame)

    def thermal_callback(self, msg: Image):
        frame = self._image_to_jpeg(msg, thermal=True)
        if frame is not None:
            self.state.set_frame('thermal', frame)

    def overview_callback(self, msg: Image):
        if msg.encoding != 'rgb8' or msg.height == 0 or msg.width == 0:
            return
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        ok, encoded = cv2.imencode('.png', cv2.cvtColor(array, cv2.COLOR_RGB2BGR))
        if ok:
            self.state.set_map(map_png=encoded.tobytes())

    def overview_info_callback(self, msg: String):
        info = self._parse_json(msg.data)
        if info is not None:
            self.state.set_map(map_info=info)

    def graph_callback(self, msg: String):
        graph = self._parse_json(msg.data)
        if graph is not None:
            self.state.set_map(graph=graph)

    def trails_callback(self, msg: String):
        data = self._parse_json(msg.data)
        if data is not None:
            self.state.set_map(trails=[
                {'name': t.get('name', ''), 'network': t.get('network', ''),
                 'length': t.get('length', 0.0), 'points': t.get('points', [])}
                for t in data.get('trails', [])
            ])

    def poi_callback(self, msg: String):
        data = self._parse_json(msg.data)
        if data is None:
            return
        points = []
        for i, point in enumerate(data.get('points', [])):
            try:
                x, y = float(point['x']), float(point['y'])
            except (KeyError, TypeError, ValueError):
                continue
            kind = str(point.get('type', 'person')).lower()
            points.append({
                'id': str(point.get('id', f'P{i + 1}')),
                'x': x,
                'y': y,
                'type': kind if kind in POI_TYPES else 'person',
            })
        self.state.set_map(points_of_interest=points)

    def route_callback(self, msg: Path):
        self.route_points = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self._update_route()

    def route_info_callback(self, msg: String):
        self.route_info = self._parse_json(msg.data)
        self._update_route()

    def _update_route(self):
        self.state.set_map(route={'points': self.route_points, 'info': self.route_info})

    def request_route(self, x: float, y: float, timeout: float = 10.0):
        """Ask the terrain mapper for a route from the ranger station to (x, y).

        Called from an HTTP thread while rclpy spins in the main thread, which
        delivers the response. Returns (number of waypoints, error message).
        """
        if not self.route_client.wait_for_service(timeout_sec=1.0):
            return None, 'route planner (plan_ranger_route) is not running'
        request = GetPlan.Request()
        # An empty start frame means "from the ranger station"
        info = self.state.get_map('map_info') or {}
        request.goal.header.frame_id = info.get('frame_id', 'parrot1_map')
        request.goal.pose.position.x = x
        request.goal.pose.position.y = y
        request.goal.pose.orientation.w = 1.0

        done = threading.Event()
        future = self.route_client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return None, 'route planner timed out'
        poses = future.result().plan.poses
        if not poses:
            return 0, 'no walking route to that point'
        return len(poses), None

    @staticmethod
    def _parse_json(text: str):
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            return None
        return data if isinstance(data, dict) else None

    def publish_command(self, command: str):
        # The mode shown comes back on mission_state once mission_manager
        # has switched, so a command it rejects doesn't change it.
        message = String()
        message.data = command
        self.command_pub.publish(message)

    def publish_manual(self, forward: float, lateral: float, yaw: float = 0.0, altitude: Optional[float] = None):
        message = Twist()
        message.linear.x = forward
        message.linear.y = lateral
        message.linear.z = 0.0
        message.angular.z = yaw
        self.manual_pub.publish(message)

        if altitude is not None and math.isfinite(altitude):
            target_msg = Float64()
            target_msg.data = max(0.0, altitude)
            self.target_altitude_pub.publish(target_msg)

        self.state.manual_command_sent()

    @staticmethod
    def _image_to_jpeg(msg: Image, thermal: bool) -> Optional[bytes]:
        if msg.height == 0 or msg.width == 0:
            return None

        encoding = (msg.encoding or '').lower()
        if encoding in ('rgb8', 'bgr8'):
            array = np.frombuffer(msg.data, dtype=np.uint8)
            array = array.reshape((msg.height, msg.width, 3))
            if encoding == 'rgb8':
                array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        elif encoding in ('mono8', '8uc1'):
            array = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            array = cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
        elif encoding in ('16uc1', 'mono16'):
            array16 = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            if thermal:
                # Use a fixed thermal display window so hotter objects remain
                # visually brighter across frames instead of being flattened by
                # per-frame auto-normalization.
                resolution_kelvin = 0.01
                display_min_kelvin = 285.0
                display_max_kelvin = 325.0

                kelvin = array16.astype(np.float32) * resolution_kelvin
                scale = 255.0 / max(display_max_kelvin - display_min_kelvin, 1e-6)
                array8 = np.clip((kelvin - display_min_kelvin) * scale, 0.0, 255.0).astype(np.uint8)
                array = cv2.applyColorMap(array8, cv2.COLORMAP_INFERNO)
            else:
                array8 = cv2.convertScaleAbs(array16, alpha=255.0 / 65535.0)
                array = cv2.applyColorMap(array8, cv2.COLORMAP_JET)
        else:
            # Fall back to a grayscale interpretation for uncommon encodings.
            array = np.frombuffer(msg.data, dtype=np.uint8)
            if array.size < msg.height * msg.width:
                return None
            array = array[: msg.height * msg.width].reshape((msg.height, msg.width))
            array = cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)

        ok, encoded = cv2.imencode('.jpg', array, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return None
        return encoded.tobytes()


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server_version = '41068DashboardBridge/1.0'

    def _set_common_headers(self, content_type: str):
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def do_OPTIONS(self):
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        if self.path.startswith('/api/telemetry'):
            payload = self.server.state.telemetry_snapshot()
            body = json.dumps(payload).encode('utf-8')
            self._set_common_headers('application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith('/api/frame/rgb.jpg'):
            self._serve_single_frame('rgb')
            return

        if self.path.startswith('/api/frame/thermal.jpg'):
            self._serve_single_frame('thermal')
            return

        if self.path.startswith('/stream/rgb.mjpg'):
            self._stream_frames('rgb')
            return

        if self.path.startswith('/stream/thermal.mjpg'):
            self._stream_frames('thermal')
            return

        if self.path.startswith('/api/map.png'):
            image = self.server.state.get_map('map_png')
            if image is None:
                self.send_response(HTTPStatus.NO_CONTENT)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                return
            self._set_common_headers('image/png')
            self.send_header('Content-Length', str(len(image)))
            self.end_headers()
            self.wfile.write(image)
            return

        if self.path.startswith('/api/map/state'):
            self._send_json(self.server.state.map_snapshot())
            return

        if self.path.startswith('/api/map/graph'):
            self._send_json(self.server.state.get_map('graph') or {'nodes': [], 'edges': []})
            return

        self.send_response(HTTPStatus.NOT_FOUND)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def _send_json(self, payload, status=HTTPStatus.OK):
        body = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path.startswith('/api/route'):
            length = int(self.headers.get('Content-Length', '0'))
            raw = self.rfile.read(length) if length else b'{}'
            try:
                payload = json.loads(raw.decode('utf-8'))
                x, y = float(payload['x']), float(payload['y'])
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                self._send_json({'ok': False, 'error': 'expected {"x": .., "y": ..}'},
                                HTTPStatus.BAD_REQUEST)
                return
            waypoints, error = self.server.node.request_route(x, y)
            self._send_json({'ok': error is None, 'waypoints': waypoints or 0, 'error': error})
            return

        if self.path.startswith('/api/manual'):
            length = int(self.headers.get('Content-Length', '0'))
            raw = self.rfile.read(length) if length else b'{}'
            try:
                payload = json.loads(raw.decode('utf-8'))
            except json.JSONDecodeError:
                self.send_response(HTTPStatus.BAD_REQUEST)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                return

            try:
                forward = float(payload.get('forward', 0.0))
                lateral = float(payload.get('lateral', 0.0))
                yaw = float(payload.get('yaw', 0.0))
                altitude_raw = payload.get('altitude', None)
                altitude = float(altitude_raw) if altitude_raw is not None else None
            except (TypeError, ValueError):
                self.send_response(HTTPStatus.BAD_REQUEST)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                return

            self.server.node.publish_manual(forward, lateral, yaw, altitude)
            body = json.dumps({
                'ok': True,
                'forward': forward,
                'lateral': lateral,
                'yaw': yaw,
                'altitude': altitude,
            }).encode('utf-8')
            self._set_common_headers('application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if not self.path.startswith('/api/command'):
            self.send_response(HTTPStatus.NOT_FOUND)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            return

        length = int(self.headers.get('Content-Length', '0'))
        raw = self.rfile.read(length) if length else b'{}'
        try:
            payload = json.loads(raw.decode('utf-8'))
        except json.JSONDecodeError:
            self.send_response(HTTPStatus.BAD_REQUEST)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            return

        command = str(payload.get('command', '')).strip()
        if not command:
            self.send_response(HTTPStatus.BAD_REQUEST)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            return

        self.server.node.publish_command(command)
        body = json.dumps({'ok': True, 'command': command}).encode('utf-8')
        self._set_common_headers('application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream_frames(self, which: str):
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        # Send each frame once, as it arrives. With no new frame for a second
        # the last one is sent again, so the connection never goes quiet.
        seq = -1
        while True:
            seq, frame = self.server.state.wait_frame(which, seq, timeout=1.0)
            if frame is None:
                frame = BLANK_JPEG
            try:
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode('utf-8'))
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                break

    def _serve_single_frame(self, which: str):
        frame = self.server.state.get_frame(which) or BLANK_JPEG

        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def log_message(self, format, *args):
        return


class DashboardHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, server_address, RequestHandlerClass, state, node):
        super().__init__(server_address, RequestHandlerClass)
        self.state = state
        self.node = node


def main():
    rclpy.init()
    state = DashboardState()
    node = BridgeNode(state)

    server = DashboardHTTPServer(('0.0.0.0', 8081), DashboardRequestHandler, state, node)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    node.get_logger().info('Dashboard bridge running on http://localhost:8081')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
