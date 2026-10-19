"""detect_trails on synthetic trail masks."""
import cv2
import numpy as np

from trail_detector import TrailConfig, _crossing_number, detect_trails, nearest_end_first

RES = 0.1


def trail_grid(lines, size=100, width_cells=5):
    """ground_hits / trail_hits for trails drawn as thick lines (cells)."""
    mask = np.zeros((size, size), np.uint8)
    for (c0, r0), (c1, r1) in lines:
        cv2.line(mask, (c0, r0), (c1, r1), 1, width_cells)
    ground = np.full((size, size), 10, np.int32)
    return ground, mask.astype(np.int32) * 10


def test_straight_trail_is_one_trail_of_the_right_length():
    ground, trail = trail_grid([((10, 50), (90, 50))])
    result = detect_trails(ground, trail, RES, 0.0, 0.0, TrailConfig())
    assert len(result.trails) == 1
    t = result.trails[0]
    assert t.name == 'T1'
    assert 7.0 < t.length < 8.5              # drawn 8 m long, thinning trims the ends
    drawn_width = (trail[:, 50] > 0).sum() * RES     # cv2.line's thickness 5 covers 7 cells
    assert abs(t.width - drawn_width) < 0.15
    ys = [y for _, y in t.points]
    assert max(ys) - min(ys) < 0.2           # stays on the centreline


def test_t_junction_splits_into_three_trails():
    ground, trail = trail_grid([((10, 50), (90, 50)), ((50, 50), (50, 95))])
    result = detect_trails(ground, trail, RES, 0.0, 0.0, TrailConfig())
    assert result.junctions.sum() >= 1
    assert len(result.trails) == 3
    assert result.trails[0].length >= result.trails[-1].length   # longest first


def test_specks_and_thin_lines_are_not_trails():
    ground, trail = trail_grid([((10, 20), (90, 20))], width_cells=1)  # 0.1 m wide
    trail[70:72, 70:72] = 10                                        # a speck
    result = detect_trails(ground, trail, RES, 0.0, 0.0, TrailConfig())
    assert result.trails == []


def test_crossing_number_tells_ends_lines_and_junctions_apart():
    skeleton = np.zeros((7, 7), bool)
    skeleton[3, 1:6] = True          # horizontal line
    skeleton[4:6, 3] = True          # branch down from the middle
    cn = _crossing_number(skeleton)
    assert cn[3, 1] == 1             # end
    assert cn[3, 2] == 2             # along the line
    assert cn[3, 3] >= 3             # junction


def test_nearest_end_first_orients_the_trail():
    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert nearest_end_first(points, 2.5, 0.0)[0] == (2.0, 0.0)
    assert nearest_end_first(points, -1.0, 0.0)[0] == (0.0, 0.0)
