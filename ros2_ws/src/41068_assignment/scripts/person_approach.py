"""Active mode's approach to a found person: where to hover, and when it worked.

Not an executable node: active_mode.py drives it. This is the close-range
inspection from 49274 AT3 (Planning 2 and 3, inspection_planner.py), cut down
to one target, since target_projector's people list already does the target
selection AT3 needed (repeat sightings merged into one id, a minimum number
of sightings, body heat in the person's own box) and mission_manager only
hands over each id once (AT3's visited list).

Planning 2, the standoff goal (standoff_goal): candidates on rings around the
person, each kept only if it is known free ground with room for the drone,
reachable (Mapping's wavefront, as AT3 used Planning 1's), and in line of sight
of the person so the camera sees them on arrival. All rings are compared
together on travel plus a penalty per metre further out, and the goal faces
the person. A goal beyond Nav2's rolling costmap is reached in hops
(hop_towards), as Mapping does for far frontiers.

Planning 3, the state machine (ApproachPlanner): PLANNING -> APPROACHING ->
VERIFYING -> DONE. Arriving is not enough: success needs the person seen again
from the standoff. A failure (Nav2 abort, timeout, not seen) retries once from
a different standoff, then abandons. Either way Active mode then finishes and
Mapping resumes.

No ROS node dependencies, so the tests run it on synthetic grids.
"""

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Sequence, Tuple

import numpy as np
from geometry_msgs.msg import Pose2D

from coverage_explorer import FREE_MAX, OCCUPIED_MIN, UNKNOWN_MAX, MapGrid
from grid_tools import distance_to_outside, is_occluded, nearest_cell


@dataclass
class ApproachConfig:
    # Standoff rings around the person (m), as AT3. The registry's estimate is
    # the torso surface the depth camera hit, so it sits slightly towards the
    # drone; 1.5 m still keeps the drone outside the person's costmap
    # inflation (0.3 m inscribed radius + 0.5 m inflation).
    standoff_distances: Tuple[float, ...] = (1.5, 2.0, 2.5, 3.0)
    ring_samples: int = 24
    # Extra travel (m) accepted per metre closer (AT3). Rings are compared
    # together, so the drone does not fly round the person to stand 0.5 m closer.
    standoff_weight: float = 2.0
    # Clearance from obstacles and unknown ground, matched to coverage_explorer
    # (the Parrot's footprint and inflation), with a fallback for tight spots.
    preferred_clearance: float = 1.0
    min_clearance: float = 0.5
    # The person is mapped as an obstacle, so the last stretch of the line of
    # sight is expected to be blocked and is not checked (AT3's target radius).
    target_radius: float = 1.0
    # Nav2's global costmap is a 20 x 20 m window on the drone, so a goal
    # further than this is flown to in hops (coverage_explorer's value).
    max_goal_distance: float = 9.0
    # 15 m (the projector's depth limit) at Nav2's 1 m/s, plus turning.
    approach_timeout: float = 40.0
    # Time at the standoff before a sighting counts, then how long to wait
    # for one. The people list refreshes a person about once a second.
    min_verify: float = 1.5
    verify_window: float = 5.0
    # A sighting this close to the estimate is the same person (AT3).
    verify_radius: float = 3.0
    retry_limit: int = 1
    # On retry, skip standoff samples this close to the one that failed (AT3).
    retry_avoid_radius: float = 1.0
    # Resend the goal if the person's estimate moves this far mid-approach (AT3).
    replan_shift: float = 0.75
    # With no valid standoff (e.g. ground around the person not mapped yet),
    # keep trying on each new map for this long before giving up (AT3's defer).
    defer: float = 5.0


class Phase(Enum):
    PLANNING = auto()       # needs a goal
    APPROACHING = auto()    # Nav2 goal running
    VERIFYING = auto()      # at the standoff, waiting to see the person
    DONE = auto()           # visited or abandoned


def clearance_map(grid: MapGrid) -> np.ndarray:
    """Distance (m) from each cell to the nearest obstacle or unknown cell (AT3).

    Unknown ground counts: that is where unmapped obstacles are.
    """
    hazard = (grid.data >= OCCUPIED_MIN) | (grid.data <= UNKNOWN_MAX)
    return distance_to_outside(~hazard) * grid.resolution


def standoff_goal(person_xy: Tuple[float, float], grid: MapGrid, dist_field: np.ndarray,
                  clearance: np.ndarray, config: ApproachConfig,
                  avoid: Sequence[Tuple[float, float]] = ()) -> Optional[Pose2D]:
    """A pose on a ring around the person, facing them, or None (AT3 Planning 2)."""
    px, py = person_xy
    free = (grid.data >= 0) & (grid.data <= FREE_MAX)
    blocked = grid.data >= OCCUPIED_MIN
    nearest_ring = min(config.standoff_distances)

    for min_clear in (config.preferred_clearance, config.min_clearance):
        best = None
        for radius in config.standoff_distances:
            for k in range(config.ring_samples):
                angle = 2.0 * math.pi * k / config.ring_samples
                x = px + radius * math.cos(angle)
                y = py + radius * math.sin(angle)
                row, col = grid.world_to_grid(x, y)
                if not grid.in_bounds(row, col):
                    continue
                if not free[row, col] or clearance[row, col] < min_clear:
                    continue
                travel = float(dist_field[row, col])
                if not math.isfinite(travel):
                    continue
                if any(math.hypot(x - ax, y - ay) <= config.retry_avoid_radius
                       for ax, ay in avoid):
                    continue
                if not _line_of_sight(grid, blocked, (x, y), person_xy, config.target_radius):
                    continue
                cost = travel + config.standoff_weight * (radius - nearest_ring)
                if best is None or cost < best[0]:
                    best = (cost, x, y)
        if best is not None:
            _, x, y = best
            return Pose2D(x=float(x), y=float(y), theta=math.atan2(py - y, px - x))
    return None


def _line_of_sight(grid: MapGrid, blocked: np.ndarray, start: Tuple[float, float],
                   target: Tuple[float, float], target_radius: float) -> bool:
    """No mapped obstacle between start and target, short of target_radius from it.

    A2's is_occluded, on the ray shortened by the person's own footprint.
    """
    (x0, y0), (x1, y1) = start, target
    length = math.hypot(x1 - x0, y1 - y0)
    if length <= target_radius:
        return True
    scale = (length - target_radius) / length
    end = (x0 + (x1 - x0) * scale, y0 + (y1 - y0) * scale)
    return not is_occluded(blocked, grid.world_to_grid(x0, y0), grid.world_to_grid(*end))


def hop_towards(goal: Pose2D, robot: Pose2D, grid: MapGrid, dist_field: np.ndarray,
                clearance: np.ndarray, config: ApproachConfig) -> Pose2D:
    """An intermediate goal max_goal_distance towards a goal beyond Nav2's costmap.

    The point that far along the straight line, moved to the nearest reachable
    cell with min_clearance; Nav2 plans the way there round the trees. Faces
    the goal.
    """
    heading = math.atan2(goal.y - robot.y, goal.x - robot.x)
    x = robot.x + config.max_goal_distance * math.cos(heading)
    y = robot.y + config.max_goal_distance * math.sin(heading)
    allowed = np.isfinite(dist_field) & (clearance >= config.min_clearance)
    row, col = grid.world_to_grid(x, y)
    cell = nearest_cell(allowed, row, col, grid.metres_to_cells(2.0))
    if cell is not None:
        x, y = grid.grid_to_world(*cell)
    return Pose2D(x=float(x), y=float(y), theta=heading)


class ApproachPlanner:
    """Approach one person and confirm them close up (AT3 Planning 3).

    The caller (ActiveMode) owns Nav2 and the map; per step it calls plan()
    while phase is PLANNING, on_goal_result() when Nav2 finishes, and tick().
    """

    def __init__(self, config: ApproachConfig, person: dict, now: float):
        self.config = config
        self.person = person                  # points_of_interest entry
        self.phase = Phase.PLANNING
        self.phase_start = now
        self.attempts = 1
        self.attempt_start = now              # approach_timeout covers hops and replans
        self.goal: Optional[Pose2D] = None    # sent to Nav2 (may be a hop)
        self.standoff: Optional[Pose2D] = None
        self.is_hop = False
        self.goal_target: Optional[Tuple[float, float]] = None
        self.failed: List[Tuple[float, float]] = []
        self.visited = False
        self.reason = ''

    @property
    def person_xy(self) -> Tuple[float, float]:
        return float(self.person['x']), float(self.person['y'])

    def update_person(self, people: List[dict]):
        """Follow the registry's estimate of the target as it is refined."""
        for p in people:
            if p.get('id') == self.person['id']:
                self.person = p
                return

    def plan(self, grid: MapGrid, dist_field: np.ndarray, robot: Pose2D,
             now: float) -> Optional[Pose2D]:
        """The next goal to send, or None if no standoff is valid on this map."""
        clearance = clearance_map(grid)
        standoff = standoff_goal(self.person_xy, grid, dist_field, clearance,
                                 self.config, self.failed)
        if standoff is None:
            return None
        self.standoff = standoff
        self.is_hop = (math.hypot(standoff.x - robot.x, standoff.y - robot.y)
                       > self.config.max_goal_distance)
        self.goal = (hop_towards(standoff, robot, grid, dist_field, clearance, self.config)
                     if self.is_hop else standoff)
        self.goal_target = self.person_xy
        self._enter(Phase.APPROACHING, now)
        return self.goal

    def on_goal_result(self, succeeded: bool, now: float):
        if self.phase != Phase.APPROACHING:
            return
        if not succeeded:
            self._fail('navigation failed', now)
        elif self.is_hop:
            self._enter(Phase.PLANNING, now)      # next hop, or the standoff
        else:
            self._enter(Phase.VERIFYING, now)

    def tick(self, people: List[dict], now: float):
        """Timeouts, replanning and verification; call every step."""
        elapsed = now - self.phase_start
        if self.phase == Phase.PLANNING:
            if elapsed > self.config.defer:
                self._finish(False, 'no reachable standoff with a view of the person')
        elif self.phase == Phase.APPROACHING:
            gx, gy = self.goal_target
            px, py = self.person_xy
            if now - self.attempt_start > self.config.approach_timeout:
                self._fail('approach timed out', now)
            elif math.hypot(px - gx, py - gy) > self.config.replan_shift:
                self._enter(Phase.PLANNING, now)
        elif self.phase == Phase.VERIFYING:
            # Seen from the standoff: the target, or anyone within
            # verify_radius of it (close up, the registry can file a sighting
            # as a neighbouring id, as AT3's localiser did).
            px, py = self.person_xy
            seen = [p for p in people
                    if math.hypot(float(p['x']) - px, float(p['y']) - py)
                    <= self.config.verify_radius
                    and p.get('last_seen', -math.inf) >= self.phase_start - 0.5]
            if seen and elapsed >= self.config.min_verify:
                self._finish(True, 'seen from the standoff')
            elif elapsed > self.config.verify_window:
                self._fail('person not seen from the standoff', now)

    def _fail(self, reason: str, now: float):
        """One retry from a different standoff, then abandon (AT3)."""
        self.reason = reason
        if self.standoff is not None:
            self.failed.append((self.standoff.x, self.standoff.y))
        if self.attempts <= self.config.retry_limit:
            self.attempts += 1
            self.attempt_start = now
            self._enter(Phase.PLANNING, now)
        else:
            self._finish(False, reason)

    def _finish(self, visited: bool, reason: str):
        self.visited = visited
        self.reason = reason
        self.phase = Phase.DONE

    def _enter(self, phase: Phase, now: float):
        self.phase = phase
        self.phase_start = now
