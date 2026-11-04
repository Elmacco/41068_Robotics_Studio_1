"""Persistent list of detected people, with Nav2-ready waypoints.

Not an executable node: target_projector.py feeds it map-frame positions and
publishes what it holds. No ROS imports, so it can be tested on its own
(test_person_registry.py).

Why it exists: target_projector sees the same person in many frames, each
with a slightly different depth reading. Publishing every reading as its own
point would scatter markers around one person. The registry merges sightings
that land within merge_radius of a known person into one entry with a stable
id (P1, P2, ...) and a running-average position, so a marker does not jitter
and a goal does not change under Nav2 while it is driving there.

A person is only reported once seen in min_observations frames, which drops
one-frame YOLO false positives and bad depth readings.

Waypoints: the person is an obstacle in Nav2's costmap, so a goal on the
person's own cell is rejected or aborts. waypoint() instead returns a pose
standoff metres from the person, on the side the drone first saw them from
(known to be reachable and in view), facing the person.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

RED = 'red'
NONE = 'none'


@dataclass
class RegistryConfig:
    merge_radius: float = 1.5      # a sighting this close to a known person is the same person (m)
    min_observations: int = 3      # sightings before a person is reported
    max_average_weight: int = 20   # position average becomes an EMA after this many sightings
    red_fraction: float = 0.5      # share of sightings flagged red for the person to count as red
    standoff: float = 1.5          # waypoint distance from the person (m)
    max_range: float = 15.0        # ignore sightings further from the drone than this (m)


@dataclass
class Person:
    id: str
    x: float
    y: float
    observations: int = 0
    red_observations: int = 0
    score: float = 0.0
    observer: Tuple[float, float] = (0.0, 0.0)   # drone x, y at the first sighting
    first_seen: float = 0.0                      # ROS time (sim time in the demo), s
    wall_first_seen: float = 0.0                 # unix time of the first sighting, s
    last_seen: float = 0.0                       # ROS time of the latest sighting, s
    wall_last_seen: float = 0.0                  # unix time of the latest sighting, s
    history: List[Tuple[float, float]] = field(default_factory=list, repr=False)

    @property
    def colour(self) -> str:
        return RED if self.observations and self.red_observations / self.observations >= 0.5 else NONE


class PersonRegistry:

    def __init__(self, config: Optional[RegistryConfig] = None):
        self.config = config or RegistryConfig()
        self.people: Dict[str, Person] = {}
        self._next = 1

    def update(self, x: float, y: float, colour: str, score: float,
               observer: Tuple[float, float], now: float) -> Person:
        """Add one sighting; returns the person it was merged into (or created)."""
        cfg = self.config
        person = self._nearest(x, y)
        if person is None:
            person = Person(id=f'P{self._next}', x=x, y=y, observer=observer, first_seen=now,
                            wall_first_seen=time.time())
            self._next += 1
            self.people[person.id] = person
        else:
            n = min(person.observations, cfg.max_average_weight)
            person.x = (person.x * n + x) / (n + 1)
            person.y = (person.y * n + y) / (n + 1)
        person.observations += 1
        person.red_observations += 1 if colour == RED else 0
        person.score = max(person.score, score)
        person.last_seen = now
        person.wall_last_seen = time.time()
        return person

    def confirmed(self) -> List[Person]:
        """People seen often enough to report, in id order."""
        return [p for p in self.people.values()
                if p.observations >= self.config.min_observations]

    def get(self, person_id: str) -> Optional[Person]:
        person = self.people.get(person_id.upper())
        if person is not None and person.observations >= self.config.min_observations:
            return person
        return None

    def waypoint(self, person: Person, approach_from: Optional[Tuple[float, float]] = None
                 ) -> Tuple[float, float, float]:
        """(x, y, yaw) standoff metres from the person, facing them.

        approach_from: where the drone is now; the waypoint lies on that side
        of the person. Defaults to where the drone first saw them.
        """
        ox, oy = approach_from or person.observer
        dx, dy = ox - person.x, oy - person.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            dx, dy, dist = 1.0, 0.0, 1.0
        ux, uy = dx / dist, dy / dist
        # Already closer than the standoff: stay where we are, still facing them.
        d = min(self.config.standoff, dist)
        wx, wy = person.x + ux * d, person.y + uy * d
        yaw = math.atan2(person.y - wy, person.x - wx)
        return wx, wy, yaw

    def _nearest(self, x: float, y: float) -> Optional[Person]:
        best, best_d = None, self.config.merge_radius
        for person in self.people.values():
            d = math.hypot(person.x - x, person.y - y)
            if d <= best_d:
                best, best_d = person, d
        return best