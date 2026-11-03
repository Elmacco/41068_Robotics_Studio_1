import numpy as np
from geometry_msgs.msg import Pose2D, Twist


THERMAL_PERSON_THRESHOLD_K = 303.15
MIN_HOT_PIXELS = 20
TARGET_COLOUR = 'red'


def thermal_confirms_person(
    thermal: np.ndarray,
    threshold_k: float = THERMAL_PERSON_THRESHOLD_K,
    min_hot_pixels: int = MIN_HOT_PIXELS,
    box=None,
) -> bool:
    """Return whether a frame or person box contains enough hot pixels."""
    temperatures = np.asarray(thermal)
    if temperatures.size == 0:
        return False
    if box is not None:
        height, width = temperatures.shape[:2]
        center_x, center_y, box_width, box_height = box[:4]
        x1 = max(0, int(round(center_x - box_width / 2.0)))
        x2 = min(width, int(round(center_x + box_width / 2.0)))
        y1 = max(0, int(round(center_y - box_height / 2.0)))
        y2 = min(height, int(round(center_y + box_height / 2.0)))
        if x1 >= x2 or y1 >= y2:
            return False
        temperatures = temperatures[y1:y2, x1:x2]
    hot_pixels = np.isfinite(temperatures) & (temperatures >= threshold_k)
    return int(np.count_nonzero(hot_pixels)) >= min_hot_pixels


class ActiveMode:
    name = 'ACTIVE'

    def __init__(self, manager):
        self.manager = manager            # mission_manager.MissionManager
        self.finished = False

        self.altitude = manager.get_parameter('active_altitude').value
        self.last_person_confirmed = False
        self.person_pose = None

    def start(self):
        self.manager.publish_cmd(Twist())
        self.manager.set_altitude(self.altitude)

    def stop(self):
        self.manager.publish_cmd(Twist())

    def step(self) -> str:
        if not self.manager.at_altitude(self.altitude):
            return f'moving to {self.altitude:.1f} m'
        if self.last_person_confirmed and self.person_pose is not None:
            return (
                f'person confirmed at '
                f'({self.person_pose.x:.2f}, {self.person_pose.y:.2f}); holding position\n\n\n\n\n\n'
            )
        return 'hovering and checking YOLO colour targets, then thermal presence'

    def person_detected(self, pose: Pose2D, colour: str | None = None) -> bool:
        """Confirm a YOLO person with the requested colour and thermal presence."""
        if colour != TARGET_COLOUR:
            self.manager.get_logger().info(
                f'Ignoring person detection with colour "{colour or "unknown"}"'
            )
            self.last_person_confirmed = False
            self.person_pose = None
            return False

        thermal = self.manager.latest_thermal
        person_box = self.manager.latest_person_bbox

        if thermal is None or not self.manager.thermal_is_current():
            self.manager.get_logger().warning(
                'Coloured person detected, but no current thermal image is available'
            )
            self.last_person_confirmed = False
            self.person_pose = None
            return False

        if person_box is None:
            self.manager.get_logger().warning(
                'Coloured person detected, but no YOLO bounding box is available'
            )
            self.last_person_confirmed = False
            self.person_pose = None
            return False

        if not thermal_confirms_person(thermal, box=person_box):
            self.manager.get_logger().info(
                'Coloured person detection has no sufficient thermal presence'
            )
            self.last_person_confirmed = False
            self.person_pose = None
            return False

        center_x, center_y, box_width, box_height = person_box[:4]
        height, width = thermal.shape[:2]
        x1 = max(0, int(round(center_x - box_width / 2.0)))
        x2 = min(width, int(round(center_x + box_width / 2.0)))
        y1 = max(0, int(round(center_y - box_height / 2.0)))
        y2 = min(height, int(round(center_y + box_height / 2.0)))
        thermal_person = thermal[y1:y2, x1:x2]
        hot_pixels = np.isfinite(thermal_person) & (
            thermal_person >= THERMAL_PERSON_THRESHOLD_K
        )
        hot_pixel_count = int(np.count_nonzero(hot_pixels))
        hottest_kelvin = float(np.nanmax(thermal_person))
        hottest_celsius = hottest_kelvin - 273.15

        self.manager.get_logger().info(
            f'YOLO person confirmed as {colour}; thermal presence: '
            f'{hot_pixel_count} hot pixels, '
            f'hottest temperature {hottest_celsius:.1f} C'
        )

        self.last_person_confirmed = True
        self.person_pose = Pose2D(x=pose.x, y=pose.y, theta=pose.theta)
        self.manager.get_logger().info(
            f'Confirmed person position: ({pose.x:.2f}, {pose.y:.2f})'
        )
        return True
