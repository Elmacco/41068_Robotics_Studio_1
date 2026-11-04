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

Also keeps a persistent list of every person seen (person_registry.py: repeat
sightings are merged, one-frame false positives dropped) and publishes it:
  points_of_interest (std_msgs/String, latched) - JSON the dashboard reads:
    {"points": [{"id": "P1", "x": .., "y": .., "type": "red"|"person",
    "first_seen": <ROS/sim time, s>, "detected_at": "<wall clock ISO time>",
    "last_seen": <ROS/sim time, s>, "last_seen_at": "<wall clock ISO time>"}, ...]}
    last_seen / last_seen_at refresh at most once a second while in view.
  people/markers (visualization_msgs/MarkerArray) - sphere + label per person,
    red for a red-shirted person, for RViz
  people/waypoints (geometry_msgs/PoseArray, map frame, same order as the
    people) - a Nav2-ready pose per person: standoff metres from them on the
    side the drone saw them from, facing them. Pose i is person i of
    points_of_interest. Send one to navigate_to_pose, or the whole array to
    nav2's follow_waypoints.
"""

import json
import math
from datetime import datetime

import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform support)
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from person_registry import PersonRegistry, RegistryConfig


class TargetProjector(Node):

    def __init__(self):
        super().__init__('target_projector')

        self.declare_parameter('robot_name', 'parrot1')
        self.declare_parameter('map_frame', '')  # defaults to '<robot_name>_map' if empty
        self.declare_parameter('min_score', 0.3)
        self.declare_parameter('max_valid_depth', 15.0)
        # People registry (see person_registry.py)
        self.declare_parameter('merge_radius', 1.5)
        self.declare_parameter('min_observations', 3)
        self.declare_parameter('waypoint_standoff', 1.5)
        self.declare_parameter('base_frame', '')  # defaults to '<robot_name>_base_link'

        robot_name = self.get_parameter('robot_name').value
        configured_map_frame = self.get_parameter('map_frame').value
        self.map_frame = configured_map_frame or f'{robot_name}_map'
        self.base_frame = self.get_parameter('base_frame').value or f'{robot_name}_base_link'
        self.registry = PersonRegistry(RegistryConfig(
            merge_radius=float(self.get_parameter('merge_radius').value),
            min_observations=int(self.get_parameter('min_observations').value),
            standoff=float(self.get_parameter('waypoint_standoff').value),
            max_range=float(self.get_parameter('max_valid_depth').value),
        ))

        self.bridge = CvBridge()
        self.camera_info = None  # type: CameraInfo | None

        self.tf_buffer = tf2_ros.Buffer()
        # Own thread, so TF keeps arriving while _on_synced waits on the
        # lookup timeout (the node's executor is blocked in that callback).
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

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
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.poi_pub = self.create_publisher(String, 'points_of_interest', latched)
        self.people_marker_pub = self.create_publisher(MarkerArray, 'people/markers', latched)
        self.waypoint_pub = self.create_publisher(PoseArray, 'people/waypoints', latched)
        self.last_published = None

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

        min_score = float(self.get_parameter('min_score').value)
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        observer = self._drone_xy()
        now = self.get_clock().now().nanoseconds * 1e-9

        best_pose, best_score = None, -1.0
        for det in detections.detections:
            score = det.results[0].hypothesis.score if det.results else 0.0
            if score < min_score:
                continue
            pose_map = self._project(det, depth_frame, depth_msg)
            if pose_map is None:
                continue
            if score > best_score:
                best_pose, best_score = pose_map, score
            # Detection2D.id carries the colour flag from target_detector.
            if observer is not None:
                self.registry.update(pose_map.pose.position.x, pose_map.pose.position.y,
                                     det.id, score, observer, now)

        if best_pose is not None:
            self.pose_pub.publish(best_pose)
            self._publish_marker(best_pose)
        self._publish_people()

    def _drone_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def _project(self, det, depth_frame, depth_msg):
        """One detection -> PoseStamped in the map frame, or None."""
        px = int(round(det.bbox.center.position.x))
        py = int(round(det.bbox.center.position.y))
        depth = self._sample_depth(depth_frame, px, py)

        if depth is None or not np.isfinite(depth) or depth <= 0.0:
            self.get_logger().info('Target detected but depth at that pixel is invalid; skipping.')
            return None
        if depth > float(self.get_parameter('max_valid_depth').value):
            return None

        camera_point = self._backproject(px, py, depth)

        # Gazebo stamps the depth image with camera_link, which uses body
        # axes (x forward, y left, z up), not the optical axes the point is
        # in, so rotate it: forward = z, left = -x, up = -y.
        pose_camera = PoseStamped()
        pose_camera.header = depth_msg.header  # camera_link
        pose_camera.pose.position.x = camera_point[2]
        pose_camera.pose.position.y = -camera_point[0]
        pose_camera.pose.position.z = -camera_point[1]
        pose_camera.pose.orientation.w = 1.0

        try:
            try:
                return self.tf_buffer.transform(
                    pose_camera, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.2),
                )
            except tf2_ros.ExtrapolationException:
                # TF lags the image by a step or so, more when the sim runs
                # slower than real time. Use the latest transform instead:
                # the drone moves at most a few cm in that time.
                pose_camera.header.stamp = rclpy.time.Time().to_msg()
                return self.tf_buffer.transform(pose_camera, self.map_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(f'TF transform to {self.map_frame} failed: {exc}',
                                    throttle_duration_sec=5.0)
            return None

    def _publish_people(self) -> None:
        """Publish the registry; only when a person is new, changes colour, or moves by about 10 cm."""
        people = self.registry.confirmed()
        # int(wall_last_seen) makes a person still in view republish about
        # once a second, so last_seen stays current without publishing every frame.
        snapshot = [(p.id, round(p.x, 1), round(p.y, 1), p.colour, int(p.wall_last_seen))
                    for p in people]
        if snapshot == self.last_published:
            return
        self.last_published = snapshot
        stamp = self.get_clock().now().to_msg()

        points = [{'id': p.id, 'x': round(p.x, 2), 'y': round(p.y, 2),
                   'type': 'red' if p.colour == 'red' else 'person',
                   'first_seen': round(p.first_seen, 2),
                   'detected_at': datetime.fromtimestamp(p.wall_first_seen).isoformat(
                       timespec='seconds'),
                   'last_seen': round(p.last_seen, 2),
                   'last_seen_at': datetime.fromtimestamp(p.wall_last_seen).isoformat(
                       timespec='seconds')} for p in people]
        self.poi_pub.publish(String(data=json.dumps({'points': points})))

        waypoints = PoseArray()
        waypoints.header.frame_id = self.map_frame
        waypoints.header.stamp = stamp
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for i, p in enumerate(people):
            wx, wy, yaw = self.registry.waypoint(p)
            pose = PoseStamped().pose
            pose.position.x, pose.position.y = wx, wy
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            waypoints.poses.append(pose)

            red = p.colour == 'red'
            colour = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9) if red else ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)
            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = stamp
            sphere.ns, sphere.id = 'people', i
            sphere.type, sphere.action = Marker.SPHERE, Marker.ADD
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = p.x, p.y, 1.0
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.6
            sphere.color = colour
            markers.markers.append(sphere)

            label = Marker()
            label.header = sphere.header
            label.ns, label.id = 'people_labels', i
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = p.x, p.y, 1.8
            label.pose.orientation.w = 1.0
            label.scale.z = 0.5
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = (f'{p.id}{" (red)" if red else ""}\n'
                          f'first {datetime.fromtimestamp(p.wall_first_seen):%H:%M:%S}\n'
                          f'last {datetime.fromtimestamp(p.wall_last_seen):%H:%M:%S}')
            markers.markers.append(label)

        self.waypoint_pub.publish(waypoints)
        self.people_marker_pub.publish(markers)
        self.get_logger().info(
            'People: ' + (', '.join(f'{p.id} ({p.x:.1f}, {p.y:.1f}) {p.colour}' for p in people) or 'none'))

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