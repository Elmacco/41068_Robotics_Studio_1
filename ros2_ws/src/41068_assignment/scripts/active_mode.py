"""Active mode: hover over a red-shirted person the detection stack has found.

Not an executable node: mission_manager.py runs it. The person comes from
target_projector's people list (points_of_interest), where YOLO, the depth
camera and the thermal camera have already agreed on them in several frames,
so this mode does no checking of its own.

Started by the `active` command it hovers until such a person appears.
Started by Mapping, which found one, it already has them. Either way it holds
for `active_dwell` seconds, then finishes: mission_manager resumes Mapping
(resume_mapping) or goes Idle. The person's id is then handled and does not
start Active mode again.

It never climbs. Mapping flies at 0.8 m, under the oak canopy (from 1.04 m),
and a climb straight up to active_altitude from there would fly into the
branches and never get there. From cruise altitude it still drops to
active_altitude for a closer look.
"""

from geometry_msgs.msg import Twist


class ActiveMode:
    name = 'ACTIVE'

    def __init__(self, manager, person: dict = None, resume_mapping: bool = False):
        self.manager = manager            # mission_manager.MissionManager
        self.finished = False
        self.resume_mapping = resume_mapping

        self.altitude = manager.get_parameter('active_altitude').value
        self.dwell = manager.get_parameter('active_dwell').value
        self.person = None                # points_of_interest entry: id, x, y, type, ...
        self.held_since = None            # when it first hovered at altitude with a person, s
        if person is not None:
            self.take(person)

    def start(self):
        self.manager.publish_cmd(Twist())
        if self.manager.altitude is not None:
            self.altitude = min(self.altitude, self.manager.altitude)
        self.manager.set_altitude(self.altitude)

    def stop(self):
        self.manager.publish_cmd(Twist())

    def wants_person(self) -> bool:
        return self.person is None

    def take(self, person: dict):
        self.person = person
        self.manager.get_logger().info(
            f'Active: {person["id"]} confirmed (red, body heat) at '
            f'({person["x"]:.2f}, {person["y"]:.2f})')

    def step(self) -> str:
        if not self.manager.at_altitude(self.altitude):
            return f'moving to {self.altitude:.1f} m'
        if self.person is None:
            return 'hovering, watching for a red-shirted person'
        if self.held_since is None:
            self.held_since = self.manager.now()
        p = self.person
        where = f'{p["id"]} confirmed at ({p["x"]:.2f}, {p["y"]:.2f})'
        left = self.dwell - (self.manager.now() - self.held_since)
        if left <= 0.0:
            self.finished = True
            return where
        then = 'mapping' if self.resume_mapping else 'idle'
        return f'{where}; holding {left:.0f} s, then {then}'
