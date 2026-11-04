"""Body-heat check for a YOLO person box, on the thermal camera's image.

Not an executable node: target_projector.py uses it to accept a sighting
only when the person's box is hot. No ROS imports, so the tests can use it.

The thermal camera has the same resolution and FOV as the RGB camera and
sits 7 cm beside it (parrot_thermal.urdf.xacro), so a YOLO box in RGB
pixels covers the same person in the thermal image (~3 px off at 3 m).
"""

import numpy as np

THERMAL_PERSON_THRESHOLD_K = 303.15   # 30 C; trees and grass stay below it
MIN_HOT_PIXELS = 20


def decode_thermal(msg) -> np.ndarray:
    """sensor_msgs/Image (16UC1 / mono16, 0.01 K per unit) -> kelvin array.

    Raises ValueError on an unexpected encoding or a truncated image.
    """
    if msg.encoding not in ('16UC1', 'mono16'):
        raise ValueError(f'unexpected thermal image encoding: {msg.encoding}')
    pixels = np.frombuffer(msg.data, dtype=np.uint16)
    # msg.step is bytes per row; uint16 uses two bytes per pixel.
    row_pixels = msg.step // 2
    if row_pixels < msg.width:
        raise ValueError(f'thermal image step ({msg.step}) is smaller than its width ({msg.width})')
    if pixels.size < msg.height * row_pixels:
        raise ValueError(f'thermal image data is truncated: expected '
                         f'{msg.height * row_pixels} pixels, got {pixels.size}')
    pixels = pixels.reshape((msg.height, row_pixels))[:, :msg.width]
    if msg.is_bigendian:
        pixels = pixels.byteswap()
    # The URDF specifies 0.01 Kelvin per pixel unit.
    return pixels.astype(np.float32) * 0.01


def hot_pixels(thermal: np.ndarray, box=None,
               threshold_k: float = THERMAL_PERSON_THRESHOLD_K) -> int:
    """Pixels at or above threshold_k, inside box (center_x, center_y, w, h) if given."""
    temperatures = np.asarray(thermal)
    if box is not None:
        height, width = temperatures.shape[:2]
        center_x, center_y, box_width, box_height = box[:4]
        x1 = max(0, int(round(center_x - box_width / 2.0)))
        x2 = min(width, int(round(center_x + box_width / 2.0)))
        y1 = max(0, int(round(center_y - box_height / 2.0)))
        y2 = min(height, int(round(center_y + box_height / 2.0)))
        if x1 >= x2 or y1 >= y2:
            return 0
        temperatures = temperatures[y1:y2, x1:x2]
    return int(np.count_nonzero(np.isfinite(temperatures) & (temperatures >= threshold_k)))


def thermal_confirms_person(thermal: np.ndarray, box=None,
                            threshold_k: float = THERMAL_PERSON_THRESHOLD_K,
                            min_hot_pixels: int = MIN_HOT_PIXELS) -> bool:
    """Whether a frame, or the person box in it, has enough hot pixels."""
    return hot_pixels(thermal, box, threshold_k) >= min_hot_pixels
