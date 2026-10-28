#!/usr/bin/env python3
"""Turn 2D target detections + the depth camera into a map-frame 3D pose.

Pipeline: target_detector.py finds the target in the RGB image (pixel
coordinates) -> this node looks up the depth at that pixel + the camera's
intrinsics (from camera_info) to back-project to a 3D point in the camera's
optical frame -> transforms that point into the map frame via TF -> publishes
it as a pose other nodes (e.g. a mission-interrupt node) can send to Nav2 as
a goal.

Kept as a separate node from target_detector.py so the detector stays a pure
"what's in this image" node — swappable/testable on its own — while all the
depth/TF/geometry logic lives here.

Subscribes (time-synchronised):
  target_detections (vision_msgs/Detection2DArray)
  camera/depth/image (sensor_msgs/Image)
  camera/depth/camera_info (sensor_msgs/CameraInfo)  - latched, read once

Publishes:
  target_pose (geometry_msgs/PoseStamped) - highest-confidence detection's
    3D position, in the map frame
  target_marker (visualization_msgs/Marker) - same, for RViz
"""

import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform support)
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker


class TargetProjector(Node):

    def __init__(self):
        super().__init__('target_projector')

        self.declare_parameter('robot_name', 'parrot1')
        self.declare_parameter('map_frame', '')  # defaults to '<robot_name>_map' if empty
        self.declare_parameter('min_score', 0.3)
        self.declare_parameter('max_valid_depth', 15.0)

        robot_name = self.get_parameter('robot_name').value
        configured_map_frame = self.get_parameter('map_frame').value
        self.map_frame = configured_map_frame or f'{robot_name}_map'

        self.bridge = CvBridge()
        self.camera_info = None  # type: CameraInfo | None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.info_sub = self.create_subscription(
            CameraInfo, 'camera/depth/camera_info', self._on_camera_info, 1,
        )

        det_sub = message_filters.Subscriber(self, Detection2DArray, 'target_detections')
        depth_sub = message_filters.Subscriber(self, Image, 'camera/depth/image')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [det_sub, depth_sub], queue_size=10, slop=0.1,
        )
        self.sync.registerCallback(self._on_synced)

        self.pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)

        self.get_logger().info(
            f'target_projector started, publishing target_pose in frame "{self.map_frame}"',
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _on_synced(self, detections: Detection2DArray, depth_msg: Image) -> None:
        if self.camera_info is None:
            self.get_logger().warn(
                'No camera_info received yet on camera/depth/camera_info - '
                'is the camera_info bridge running?', throttle_duration_sec=5.0,
            )
            return

        if not detections.detections:
            return

        best = max(
            detections.detections,
            key=lambda d: d.results[0].hypothesis.score if d.results else 0.0,
        )
        score = best.results[0].hypothesis.score if best.results else 0.0
        if score < float(self.get_parameter('min_score').value):
            return

        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        px = int(round(best.bbox.center.position.x))
        py = int(round(best.bbox.center.position.y))
        depth = self._sample_depth(depth_frame, px, py)

        if depth is None or not np.isfinite(depth) or depth <= 0.0:
            self.get_logger().info('Target detected but depth at that pixel is invalid; skipping.')
            return
        if depth > float(self.get_parameter('max_valid_depth').value):
            return

        camera_point = self._backproject(px, py, depth)

        pose_camera = PoseStamped()
        pose_camera.header = depth_msg.header  # camera optical frame
        pose_camera.pose.position.x = camera_point[0]
        pose_camera.pose.position.y = camera_point[1]
        pose_camera.pose.position.z = camera_point[2]
        pose_camera.pose.orientation.w = 1.0

        try:
            pose_map = self.tf_buffer.transform(
                pose_camera, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(f'TF transform to {self.map_frame} failed: {exc}',
                                    throttle_duration_sec=5.0)
            return

        self.pose_pub.publish(pose_map)
        self._publish_marker(pose_map)

    def _sample_depth(self, depth_frame: np.ndarray, px: int, py: int, patch: int = 2):
        """Median depth over a small patch around (px, py) - more robust than
        a single pixel, which is easy to land on a NaN/inf edge/no-return."""
        h, w = depth_frame.shape[:2]
        x0, x1 = max(0, px - patch), min(w, px + patch + 1)
        y0, y1 = max(0, py - patch), min(h, py + patch + 1)
        region = depth_frame[y0:y1, x0:x1]
        valid = region[np.isfinite(region) & (region > 0.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _backproject(self, px: int, py: int, depth: float):
        """Pinhole back-projection: pixel + depth -> point in optical frame
        (x right, y down, z forward — matches sensor_msgs/CameraInfo / REP 103
        optical frame convention)."""
        k = self.camera_info.k  # row-major 3x3 intrinsic matrix
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth
        return (x, y, z)

    def _publish_marker(self, pose_map: PoseStamped) -> None:
        marker = Marker()
        marker.header = pose_map.header
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_map.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5
        marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9)
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TargetProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
