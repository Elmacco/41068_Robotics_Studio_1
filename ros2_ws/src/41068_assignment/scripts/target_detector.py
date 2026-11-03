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
    only published if debug:=true and something subscribes (handy for
    rqt_image_view while tuning).
  target_colour (std_msgs/String) - "red" or "none", every frame a person is
    seen. With several people visible, "red" if any of them is red.

Red flagging (see colour_classifier.py): each person's box is cropped to the
torso region and checked for red by HSV colour. The per-person flag is also
stored in Detection2D.id. Only red is flagged; green and other colours read
as "none". Turn it off with `-p colour_enabled:=false`.
"""

import os
import sys
from typing import List, Tuple

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import colour_classifier as cc  # noqa: E402

# Debug-image box colours (BGR) for each flag.
FLAG_BGR = {cc.RED: (0, 0, 255), cc.NONE: (200, 200, 200)}


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
        # 'auto' (CUDA if PyTorch can see a GPU, else CPU), 'cpu', 'cuda',
        # 'cuda:0' etc. WSL2 passes NVIDIA GPUs through to CUDA.
        self.declare_parameter('device', 'auto')
        # CPU threads for inference on the CPU. PyTorch otherwise takes every
        # core, and competes with Gazebo, which renders the cameras on the CPU.
        self.declare_parameter('threads', 4)
        self.declare_parameter('debug', True)

        # Red flagging. The torso_* values pick the slice of each person box
        # that gets sampled: heights are fractions of box height from the top,
        # half-width is a fraction of box width either side of the centre.
        # colour_red_hue_max is kept low so orange doesn't count as red; raise
        # colour_min_fraction if the normal person is being flagged.
        self.declare_parameter('colour_enabled', True)
        self.declare_parameter('colour_min_fraction', 0.25)
        self.declare_parameter('colour_min_sat', 120)
        self.declare_parameter('colour_min_val', 50)
        self.declare_parameter('colour_red_hue_max', 10)
        self.declare_parameter('colour_red_hue_min', 170)
        self.declare_parameter('torso_top', 0.18)
        self.declare_parameter('torso_bottom', 0.55)
        self.declare_parameter('torso_half_width', 0.25)
        self.colour_params = cc.ColourParams(
            min_fraction=float(self.get_parameter('colour_min_fraction').value),
            min_sat=int(self.get_parameter('colour_min_sat').value),
            min_val=int(self.get_parameter('colour_min_val').value),
            red_low_hue_max=int(self.get_parameter('colour_red_hue_max').value),
            red_high_hue_min=int(self.get_parameter('colour_red_hue_min').value),
            torso_top=float(self.get_parameter('torso_top').value),
            torso_bottom=float(self.get_parameter('torso_bottom').value),
            torso_half_width=float(self.get_parameter('torso_half_width').value),
        )
        self.colour_enabled = bool(self.get_parameter('colour_enabled').value)
        self.last_flag = None

        torch.set_num_threads(int(self.get_parameter('threads').value))
        self.device = str(self.get_parameter('device').value)
        if self.device == 'auto':
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        model_name = str(self.get_parameter('model').value)
        self.get_logger().info(f'Loading {model_name} (downloads on first use if not cached)...')
        self.model = YOLO(model_name)
        self.person_class_id = int(self.get_parameter('person_class_id').value)

        self.bridge = CvBridge()
        # Depth 1: if inference falls behind, skip to the newest frame rather
        # than working through a backlog of old ones.
        self.image_sub = self.create_subscription(Image, 'camera/image', self._on_image, 1)
        self.detections_pub = self.create_publisher(Detection2DArray, 'target_detections', 10)
        self.debug_pub = self.create_publisher(Image, 'target_detections/debug_image', 1)
        self.colour_pub = self.create_publisher(String, 'target_colour', 10)

        self.get_logger().info(
            f'target_detector started on {self.device}, subscribed to camera/image '
            f'(colour flagging {"on" if self.colour_enabled else "off"})'
        )

    def _on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        boxes = self._detect(frame)

        # Per-person colour flag, sampled from the torso region of each box.
        colour_info = []
        if self.colour_enabled:
            for (cx, cy, w, h, _score) in boxes:
                colour_info.append(cc.classify_person(frame, (cx, cy, w, h), self.colour_params))

        detections = Detection2DArray()
        detections.header = msg.header
        for i, (cx, cy, w, h, score) in enumerate(boxes):
            det = Detection2D()
            det.header = msg.header
            if colour_info:
                det.id = colour_info[i][0]
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

        if colour_info:
            self._publish_colour(colour_info)

        if self.get_parameter('debug').value and self.debug_pub.get_subscription_count() > 0:
            self._publish_debug(frame, boxes, msg.header, colour_info)

    def _publish_colour(self, colour_info) -> None:
        """Publish the frame's flag ("red" if any person is red); log changes."""
        flag = cc.overall_flag(info[0] for info in colour_info)
        self.colour_pub.publish(String(data=flag))
        if flag != self.last_flag:
            self.last_flag = flag
            if flag == cc.RED:
                self.get_logger().warn('Colour flag: RED person detected.')
            else:
                self.get_logger().info('Colour flag: none (person seen, not red).')

    def _detect(self, frame: np.ndarray) -> List[Tuple[float, float, float, float, float]]:
        """Return a list of (center_x, center_y, width, height, score) in pixels.

        Runs YOLO26 on the frame and keeps only detections for
        `person_class_id`. Replace this method (only this method) to swap in
        a different model later; keep the same return type and the rest of
        the node needs no changes.
        """
        confidence = float(self.get_parameter('confidence').value)
        imgsz = int(self.get_parameter('imgsz').value)

        results = self.model.predict(
            frame,
            classes=[self.person_class_id],
            conf=confidence,
            imgsz=imgsz,
            device=self.device,
            verbose=False,
        )[0]

        boxes = []
        for box in results.boxes:
            cx, cy, w, h = box.xywh[0].tolist()
            score = float(box.conf[0])
            boxes.append((cx, cy, w, h, score))

        return boxes

    def _publish_debug(self, frame, boxes, header, colour_info=()) -> None:
        annotated = frame.copy()
        for i, (cx, cy, w, h, score) in enumerate(boxes):
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)
            if colour_info:
                flag, red_frac, torso = colour_info[i]
                bgr = FLAG_BGR[flag]
                label = f'person {score:.2f} [{flag.upper()}] red={red_frac:.0%}'
                # Thin rectangle = the torso region that was sampled.
                cv2.rectangle(annotated, (torso[0], torso[1]), (torso[2], torso[3]), (255, 255, 255), 1)
            else:
                bgr = (0, 255, 0)
                label = f'person {score:.2f}'
            cv2.rectangle(annotated, (x1, y1), (x2, y2), bgr, 2)
            cv2.putText(annotated, label, (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 1)
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
