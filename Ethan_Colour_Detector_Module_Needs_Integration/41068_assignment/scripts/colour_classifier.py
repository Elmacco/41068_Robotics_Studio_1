"""Red shirt-colour flagging for detected people.

Not an executable node. target_detector.py imports this, and
test_colour_classifier.py exercises it without ROS. Only needs OpenCV + numpy.

How it works: for each person box from YOLO, crop to the torso region (a
parameterised slice of the box: by default the central 50% of the width, from
18% to 55% of the height down from the top), convert that crop to HSV, and
count the share of pixels that are strongly red. If that share reaches
`min_fraction` the person is flagged `red`; otherwise `none`. OpenCV hue runs
0-179 and red wraps around 0, so both ends of the hue range are checked.
Green (and any other colour) is deliberately not flagged.
"""

from dataclasses import dataclass
from typing import Iterable, Tuple

import cv2
import numpy as np

RED = 'red'
NONE = 'none'


@dataclass(frozen=True)
class ColourParams:
    # Share of torso pixels that must be red for the flag to fire.
    min_fraction: float = 0.25
    # Pixels dimmer or less saturated than this are ignored (greys, shadows,
    # pale skin tones).
    min_sat: int = 120
    min_val: int = 50
    # Red wraps around hue 0: low end 0..red_low_hue_max, high end
    # red_high_hue_min..179. Keep red_low_hue_max small so orange (hue ~10-20)
    # is not counted as red.
    red_low_hue_max: int = 10
    red_high_hue_min: int = 170
    # Torso slice of the YOLO box. Heights are fractions of box height measured
    # down from the top; half-width is a fraction of box width either side of
    # the box centre.
    torso_top: float = 0.18
    torso_bottom: float = 0.55
    torso_half_width: float = 0.25


def torso_region(box, img_w: int, img_h: int, p: ColourParams) -> Tuple[int, int, int, int]:
    """Torso rectangle (x1, y1, x2, y2) in pixels, clipped to the image.

    `box` is (center_x, center_y, width, height) in pixels, as YOLO gives it.
    """
    cx, cy, w, h = box[:4]
    top = cy - h / 2.0
    x1 = int(round(cx - p.torso_half_width * w))
    x2 = int(round(cx + p.torso_half_width * w))
    y1 = int(round(top + p.torso_top * h))
    y2 = int(round(top + p.torso_bottom * h))
    x1, x2 = max(0, x1), min(img_w, x2)
    y1, y2 = max(0, y1), min(img_h, y2)
    return x1, y1, x2, y2


def classify_crop(bgr_crop: np.ndarray, p: ColourParams) -> Tuple[str, float]:
    """Return (flag, red_fraction) for a BGR crop."""
    if bgr_crop is None or bgr_crop.size == 0:
        return NONE, 0.0
    hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]

    red = (
        (val >= p.min_val) & (sat >= p.min_sat)
        & ((hue <= p.red_low_hue_max) | (hue >= p.red_high_hue_min))
    )
    red_frac = float(red.mean())
    return (RED if red_frac >= p.min_fraction else NONE), red_frac


def classify_person(frame_bgr: np.ndarray, box, p: ColourParams):
    """Classify one detected person.

    Returns (flag, red_fraction, (x1, y1, x2, y2)) where the rectangle is the
    torso region that was sampled.
    """
    img_h, img_w = frame_bgr.shape[:2]
    x1, y1, x2, y2 = torso_region(box, img_w, img_h, p)
    # Need at least a few pixels each way to say anything meaningful.
    if x2 - x1 < 4 or y2 - y1 < 4:
        return NONE, 0.0, (x1, y1, x2, y2)
    flag, red_frac = classify_crop(frame_bgr[y1:y2, x1:x2], p)
    return flag, red_frac, (x1, y1, x2, y2)


def overall_flag(flags: Iterable[str]) -> str:
    """One flag for a frame with several people: red if any of them is."""
    return RED if RED in list(flags) else NONE
