#!/usr/bin/env python3
"""Detect a person in the RGB camera stream and publish 2D detections.

Uses Ultralytics YOLO26 (COCO-pretrained) filtered to the "person" class
(COCO class id 0). The model loads once at startup; weights auto-download
from Ultralytics' release assets on first use if not already cached, so the
first run needs internet access.

Swapping in a different model/checkpoint later only means changing
`_detect()` and the `model`/`person_class_id` params — the rest of the node
(image handling, publishing vision_msgs/Detection2DArray) stays the same,
which is why detection logic is isolated in one method.

Subscribes:
  camera/image (sensor_msgs/Image)      - RGB frames

Publishes:
  target_detections (vision_msgs/Detection2DArray) - one entry per person
    detected, bbox in pixel coordinates, one frame's detections per message.
  target_detections/debug_image (sensor_msgs/Image) - annotated frame,
    only published if debug:=true (handy for rqt_image_view while tuning).
"""

from typing import List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


class TargetDetector(Node):

    def __init__(self):
        super().__init__('target_detector')

        # Model file name/path. Any YOLO26 variant works (yolo26n/s/m/l/x.pt);
        # start with nano for CPU-friendly real-time inference, size up only
        # if detection quality needs it.
        self.declare_parameter('model', 'yolo26n.pt')
        # COCO class id for "person". Leave as-is unless using a custom-
        # trained checkpoint with a different class mapping.
        self.declare_parameter('person_class_id', 0)
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('imgsz', 640)
        # 'cpu', 'cuda', or 'cuda:0' etc. Falls back to CPU everywhere unless
        # you know a GPU is passed through to this environment.
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('debug', False)

        model_name = str(self.get_parameter('model').value)
        self.get_logger().info(f'Loading {model_name} (downloads on first use if not cached)...')
        self.model = YOLO(model_name)
        self.person_class_id = int(self.get_parameter('person_class_id').value)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 'camera/image', self._on_image, 10)
        self.detections_pub = self.create_publisher(Detection2DArray, 'target_detections', 10)
        self.debug_pub = self.create_publisher(Image, 'target_detections/debug_image', 1)

        self.get_logger().info('target_detector started, subscribed to camera/image')

    def _on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        boxes = self._detect(frame)

        detections = Detection2DArray()
        detections.header = msg.header
        for (cx, cy, w, h, score) in boxes:
            det = Detection2D()
            det.header = msg.header
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'person'
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            detections.detections.append(det)

        self.detections_pub.publish(detections)

        if self.get_parameter('debug').value:
            self._publish_debug(frame, boxes, msg.header)

    def _detect(self, frame: np.ndarray) -> List[Tuple[float, float, float, float, float]]:
        """Return a list of (center_x, center_y, width, height, score) in pixels.

        Runs YOLO26 on the frame and keeps only detections for
        `person_class_id`. Replace this method (only this method) to swap in
        a different model later; keep the same return type and the rest of
        the node needs no changes.
        """
        confidence = float(self.get_parameter('confidence').value)
        imgsz = int(self.get_parameter('imgsz').value)
        device = str(self.get_parameter('device').value)

        results = self.model.predict(
            frame,
            classes=[self.person_class_id],
            conf=confidence,
            imgsz=imgsz,
            device=device,
            verbose=False,
        )[0]

        boxes = []
        for box in results.boxes:
            cx, cy, w, h = box.xywh[0].tolist()
            score = float(box.conf[0])
            boxes.append((cx, cy, w, h, score))

        return boxes

    def _publish_debug(self, frame, boxes, header) -> None:
        annotated = frame.copy()
        for (cx, cy, w, h, score) in boxes:
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f'person {score:.2f}', (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        debug_msg.header = header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
