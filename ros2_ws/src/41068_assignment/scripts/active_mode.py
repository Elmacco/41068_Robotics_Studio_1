import numpy as np
from geometry_msgs.msg import Pose2D, Twist


THERMAL_PERSON_THRESHOLD_K = 303.15
MIN_HOT_PIXELS = 20


def thermal_confirms_person(
    thermal: np.ndarray,
    threshold_k: float = THERMAL_PERSON_THRESHOLD_K,
    min_hot_pixels: int = MIN_HOT_PIXELS,
) -> bool:
    """Return whether a thermal frame contains enough hot pixels."""
    temperatures = np.asarray(thermal)
    if temperatures.size == 0:
        return False
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
                f'({self.person_pose.x:.2f}, {self.person_pose.y:.2f}); holding position'
            )
        return 'hovering and checking thermal detections'

    def person_detected(self, pose: Pose2D) -> bool:
        """Confirm a map-frame person detection against the latest thermal frame."""
        thermal = self.manager.latest_thermal

        if thermal is None or not self.manager.thermal_is_current():
            self.manager.get_logger().warning('Person detected, but no thermal image is available yet')
            self.last_person_confirmed = False
            self.person_pose = None
            return False

        hot_pixels = np.isfinite(thermal) & (thermal >= THERMAL_PERSON_THRESHOLD_K)
        hot_pixel_count = int(np.count_nonzero(hot_pixels))

        if not thermal_confirms_person(thermal):
            self.manager.get_logger().info('Person detection has no sufficient thermal confirmation')
            self.last_person_confirmed = False
            self.person_pose = None
            return False

        hottest_kelvin = float(np.nanmax(thermal))
        hottest_celsius = hottest_kelvin - 273.15

        self.manager.get_logger().info(
            f'Thermal person confirmation: '
            f'{hot_pixel_count} hot pixels, '
            f'hottest temperature {hottest_celsius:.1f} C'
        )

        self.last_person_confirmed = True
        self.person_pose = Pose2D(x=pose.x, y=pose.y, theta=pose.theta)
        self.manager.get_logger().info(
            f'Confirmed person position: ({pose.x:.2f}, {pose.y:.2f})'
        )
        return True
