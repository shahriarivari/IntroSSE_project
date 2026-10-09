import math

import pytest

from bug2_create3 import lidar

# Real values from: ros2 topic echo --once --no-arr /robot_4/scan
ANGLE_MIN = -3.124139
INC = 0.00871450919657
RANGE_MIN = 0.15
RANGE_MAX = 12.0
N = 720

FRONT = 0.0                         # [rad] centre of the front sector
FRONT_HALF_WIDTH = math.radians(40)  # [rad]


def empty_scan():
    """A scan where nothing is seen in any direction."""
    return [float('inf')] * N


def front_min(ranges):
    """Shortcut: smallest valid distance in the front sector."""
    return lidar.sector_min(ranges, FRONT, FRONT_HALF_WIDTH,
                            ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX)


# ---------- angle_to_index ----------

def test_index_front():
    assert lidar.angle_to_index(0.0, ANGLE_MIN, INC) == 358


def test_index_left():
    assert lidar.angle_to_index(math.pi / 2, ANGLE_MIN, INC) == 539


def test_index_right():
    assert lidar.angle_to_index(-math.pi / 2, ANGLE_MIN, INC) == 178


# ---------- sector_min ----------

def test_empty_scan_gives_inf():
    assert front_min(empty_scan()) == float('inf')


def test_obstacle_front():
    ranges = empty_scan()
    ranges[358] = 0.5
    assert front_min(ranges) == pytest.approx(0.5)


def test_obstacle_left_not_in_front_sector():
    ranges = empty_scan()
    ranges[539] = 0.3                  # 90 deg left, outside +-40 deg
    assert front_min(ranges) == float('inf')


def test_too_close_is_ignored():
    ranges = empty_scan()
    ranges[358] = 0.1                  # below RANGE_MIN: not a real measurement
    assert front_min(ranges) == float('inf')


def test_nan_is_ignored():
    ranges = empty_scan()
    ranges[358] = float('nan')
    assert front_min(ranges) == float('inf')


def test_smallest_wins():
    ranges = empty_scan()
    ranges[350] = 0.8
    ranges[365] = 0.4
    assert front_min(ranges) == pytest.approx(0.4)


def test_edge_of_sector_is_included():
    ranges = empty_scan()
    edge = lidar.angle_to_index(FRONT_HALF_WIDTH, ANGLE_MIN, INC)   # exactly +40 deg
    ranges[edge] = 0.6
    assert front_min(ranges) == pytest.approx(0.6)


def test_just_outside_sector_is_excluded():
    ranges = empty_scan()
    edge = lidar.angle_to_index(FRONT_HALF_WIDTH, ANGLE_MIN, INC)
    ranges[edge + 1] = 0.6
    assert front_min(ranges) == float('inf')


# ---------- sector_closest ----------

def test_closest_empty_scan():
    d, a = lidar.sector_closest(empty_scan(), math.radians(-90), math.radians(60),
                                ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX)
    assert d == float('inf')
    assert a is None


def test_closest_gives_distance_and_angle():
    ranges = empty_scan()
    i = lidar.angle_to_index(math.radians(-60), ANGLE_MIN, INC)
    ranges[i] = 0.40
    ranges[178] = 0.50                     # -90 deg, farther away
    d, a = lidar.sector_closest(ranges, math.radians(-90), math.radians(60),
                                ANGLE_MIN, INC, RANGE_MIN, RANGE_MAX)
    assert d == pytest.approx(0.40)
    assert a == pytest.approx(math.radians(-60), abs=INC)


# ---- corridor_clearance: straight corridor instead of a cone ----
import math as _math
from bug2_create3.lidar import corridor_clearance

_N = 720
_MIN = -_math.pi
_INC = 2 * _math.pi / _N


def _scan_with_parallel_wall(side_distance):
    """Long wall parallel to the x-axis at y = -side_distance (on the right)."""
    ranges = []
    for i in range(_N):
        a = _MIN + i * _INC
        s = _math.sin(a)
        ranges.append(side_distance / -s if s < -1e-6 else float('inf'))
    return ranges


def test_parallel_wall_beside_robot_is_not_in_the_way():
    ranges = _scan_with_parallel_wall(0.22)          # wall followed at 22 cm
    assert corridor_clearance(ranges, 0.0, 0.19, _MIN, _INC, 0.05, 12.0) == float('inf')


def test_wall_straight_ahead_is_in_the_way():
    ranges = [float('inf')] * _N
    for i in range(_N):
        a = _MIN + i * _INC
        if abs(a) < _math.radians(60):
            ranges[i] = 0.5 / _math.cos(a)             # wall across the path at x = 0.5
    assert corridor_clearance(ranges, 0.0, 0.19, _MIN, _INC, 0.05, 12.0) == pytest.approx(0.5, abs=0.01)


def _point_scan(angle, distance):
    """Scan with one single point at (angle, distance)."""
    ranges = [float('inf')] * _N
    ranges[round((angle - _MIN) / _INC)] = distance
    return ranges


def test_point_beside_robot_in_margin_is_not_in_the_way():
    # start of run 2026-10-07: wall 17.7 cm to the left, just inside the 19 cm margin
    ranges = _point_scan(_math.radians(89), 0.177)
    assert corridor_clearance(ranges, 0.0, 0.19, _MIN, _INC, 0.05, 12.0,
                              body_radius=0.17) == float('inf')


def test_point_ahead_in_margin_is_in_the_way():
    # 18 cm to the side but 30 cm ahead: the margin should still catch it
    ranges = _point_scan(_math.atan2(0.18, 0.30), _math.hypot(0.30, 0.18))
    assert corridor_clearance(ranges, 0.0, 0.19, _MIN, _INC, 0.05, 12.0,
                              body_radius=0.17) == pytest.approx(0.30, abs=0.01)


def test_point_in_body_path_is_in_the_way():
    ranges = _point_scan(_math.atan2(0.10, 0.25), _math.hypot(0.25, 0.10))
    assert corridor_clearance(ranges, 0.0, 0.19, _MIN, _INC, 0.05, 12.0,
                              body_radius=0.17) == pytest.approx(0.25, abs=0.01)
