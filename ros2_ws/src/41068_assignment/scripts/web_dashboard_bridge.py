#!/usr/bin/env python3

import base64
import json
import math
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from socketserver import ThreadingMixIn
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DashboardState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latitude = -33.8688
        self.longitude = 151.2093
        self.altitude = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.battery = 98.0
        self.mode = 'STANDBY'
        self.rgb_frame = None
        self.thermal_frame = None

    def telemetry_snapshot(self):
        with self.lock:
            return {
                'latitude': self.latitude,
                'longitude': self.longitude,
                'altitude': self.altitude,
                'heading': self.heading,
                'speed': self.speed,
                'battery': self.battery,
                'mode': self.mode,
            }

    def set_odom(self, latitude, longitude, altitude, heading, speed):
        with self.lock:
            self.latitude = latitude
            self.longitude = longitude
            self.altitude = altitude
            self.heading = heading
            self.speed = speed

    def set_mode(self, mode):
        with self.lock:
            self.mode = mode

    def set_frame(self, which, frame_bytes):
        with self.lock:
            if which == 'rgb':
                self.rgb_frame = frame_bytes
            else:
                self.thermal_frame = frame_bytes

    def get_frame(self, which):
        with self.lock:
            if which == 'rgb':
                return self.rgb_frame
            return self.thermal_frame


class BridgeNode(Node):
    def __init__(self, state: DashboardState):
        super().__init__('web_dashboard_bridge')
        self.state = state
        self.command_pub = self.create_publisher(String, '/parrot1/mission_command', 10)
        self.manual_pub = self.create_publisher(Twist, '/parrot1/cmd_vel', 10)
        self.create_subscription(Odometry, '/parrot1/odometry', self.odom_callback, 10)
        self.create_subscription(Image, '/parrot1/camera/image', self.rgb_callback, 10)
        self.create_subscription(Image, '/parrot1/thermal_camera/image', self.thermal_callback, 10)

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

    def rgb_callback(self, msg: Image):
        frame = self._image_to_jpeg(msg, thermal=False)
        if frame is not None:
            self.state.set_frame('rgb', frame)

    def thermal_callback(self, msg: Image):
        frame = self._image_to_jpeg(msg, thermal=True)
        if frame is not None:
            self.state.set_frame('thermal', frame)

    def publish_command(self, command: str):
        message = String()
        message.data = command
        self.command_pub.publish(message)
        if command == 'return_home':
            self.state.set_mode('RETURN HOME')
        elif command == 'takeoff':
            self.state.set_mode('TAKE OFF')
        else:
            self.state.set_mode('TAKE OVER')

    def publish_manual(self, forward: float, lateral: float):
        message = Twist()
        message.linear.x = forward
        message.linear.y = lateral
        message.linear.z = 0.0
        self.manual_pub.publish(message)
        self.state.set_mode('MANUAL')

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
            array = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            array = cv2.convertScaleAbs(array, alpha=255.0 / 65535.0)
            array = cv2.applyColorMap(array, cv2.COLORMAP_INFERNO if thermal else cv2.COLORMAP_JET)
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

        self.send_response(HTTPStatus.NOT_FOUND)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def do_POST(self):
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
            except (TypeError, ValueError):
                self.send_response(HTTPStatus.BAD_REQUEST)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                return

            self.server.node.publish_manual(forward, lateral)
            body = json.dumps({'ok': True, 'forward': forward, 'lateral': lateral}).encode('utf-8')
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

        while True:
            frame = self.server.state.get_frame(which)
            if frame is None:
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                blank = np.zeros((240, 320, 3), dtype=np.uint8)
                ok, encoded = cv2.imencode('.jpg', blank)
                if not ok:
                    break
                frame = encoded.tobytes()
            try:
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode('utf-8'))
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                break

    def _serve_single_frame(self, which: str):
        frame = self.server.state.get_frame(which)
        if frame is None:
            blank = np.zeros((240, 320, 3), dtype=np.uint8)
            ok, encoded = cv2.imencode('.jpg', blank)
            if not ok:
                self.send_response(HTTPStatus.INTERNAL_SERVER_ERROR)
                self.end_headers()
                return
            frame = encoded.tobytes()

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
