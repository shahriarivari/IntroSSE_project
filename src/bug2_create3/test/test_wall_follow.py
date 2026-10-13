import math

import pytest

from bug2_create3.wall_follow import (compute_wall_follow, MAX_V, MAX_W, TURN_W, ARC_V,
                                      K_WALL, WALL_DISTANCE, WALL_IN_FRONT, TURN_RELEASE,
                                      MIN_V_FACTOR, FRONT_CRITICAL, ROUNDING_W,
                                      ROUNDING_EXIT)

FAR = 2.0                    # nothing in front
RIGHT = math.radians(-90)


def test_inner_corner_turns_left_in_place():
    v, w, rule = compute_wall_follow(front=0.2, closest=0.30, closest_angle=RIGHT)
    assert rule == 'inner'
    assert v == 0.0
    assert w == TURN_W


def test_hysteresis_keeps_turning_until_front_is_really_clear():
    # front just above WALL_IN_FRONT, but below TURN_RELEASE: keep turning if we were turning
    # (wall on the right at exactly WALL_DISTANCE: the follow rule itself goes straight)
    front = (WALL_IN_FRONT + TURN_RELEASE) / 2
    _, _, rule = compute_wall_follow(front, WALL_DISTANCE, RIGHT, previous='inner')
    assert rule == 'inner'
    # ... but do NOT start turning at that distance if we were not turning
    _, _, rule = compute_wall_follow(front, WALL_DISTANCE, RIGHT, previous='follow')
    assert rule == 'follow'


def test_stops_turning_when_front_is_clear():
    _, _, rule = compute_wall_follow(FAR, WALL_DISTANCE, RIGHT, previous='inner')
    assert rule == 'follow'


# ---- going around a corner to the right ('round') ----
# Wall far away on the right and falling behind: the follow rule turns right hard.
ROUNDING_CASE = dict(closest=WALL_DISTANCE + 0.05, closest_angle=math.radians(-110))


def test_starts_rounding_when_follow_turns_right_hard():
    _, w, rule = compute_wall_follow(FAR, **ROUNDING_CASE)
    assert w < ROUNDING_W
    assert rule == 'round'


def test_while_rounding_a_wall_ahead_only_counts_when_critical():
    # Between FRONT_CRITICAL and WALL_IN_FRONT: the far wall of the corridor we turn into.
    front = (FRONT_CRITICAL + WALL_IN_FRONT) / 2
    _, w, rule = compute_wall_follow(front, **ROUNDING_CASE)
    assert rule == 'round' and w < 0          # keep turning right, not left
    # Really close: turn left anyway (safety).
    _, _, rule = compute_wall_follow(FRONT_CRITICAL - 0.01, **ROUNDING_CASE)
    assert rule == 'inner'


def test_rounding_is_sticky():
    # Follow rule turns right a little (between ROUNDING_W and ROUNDING_EXIT), e.g. one noisy tick.
    closest = WALL_DISTANCE + 0.02           # w = K_WALL * -0.02 = -0.16
    _, w, _ = compute_wall_follow(FAR, closest, RIGHT)
    assert ROUNDING_W < w < ROUNDING_EXIT
    front = (FRONT_CRITICAL + WALL_IN_FRONT) / 2
    # Already rounding: stay rounding, so the wall ahead is still ignored.
    _, _, rule = compute_wall_follow(front, closest, RIGHT, previous='round')
    assert rule == 'round'
    # Not rounding: do not start, and the wall ahead is an inner corner.
    _, _, rule = compute_wall_follow(front, closest, RIGHT, previous='follow')
    assert rule == 'inner'


def test_rounding_ends_when_nearly_straight():
    _, _, rule = compute_wall_follow(FAR, WALL_DISTANCE, RIGHT, previous='round')
    assert rule == 'follow'


def test_no_wall_on_the_right_arcs_right():
    v, w, rule = compute_wall_follow(front=FAR, closest=float('inf'), closest_angle=None)
    assert rule == 'outer'
    assert v == ARC_V
    assert w < 0


def test_parallel_at_right_distance_goes_straight():
    v, w, rule = compute_wall_follow(front=FAR, closest=WALL_DISTANCE, closest_angle=RIGHT)
    assert rule == 'follow'
    assert v == pytest.approx(MAX_V)
    assert w == pytest.approx(0.0)


def test_too_close_turns_left():
    v, w, _ = compute_wall_follow(front=FAR, closest=WALL_DISTANCE - 0.05, closest_angle=RIGHT)
    assert w == pytest.approx(min(K_WALL * 0.05, MAX_W))


def test_too_far_turns_right():
    v, w, _ = compute_wall_follow(front=FAR, closest=WALL_DISTANCE + 0.10, closest_angle=RIGHT)
    assert w < 0


def test_wall_ahead_right_turns_left():
    v, w, _ = compute_wall_follow(front=FAR, closest=WALL_DISTANCE,
                                     closest_angle=math.radians(-60))
    assert w > 0


def test_wall_behind_right_turns_right():
    v, w, _ = compute_wall_follow(front=FAR, closest=WALL_DISTANCE,
                                     closest_angle=math.radians(-120))
    assert w < 0


def test_slows_down_when_not_parallel():
    v, w, _ = compute_wall_follow(front=FAR, closest=WALL_DISTANCE,
                                     closest_angle=math.radians(-30))   # 60 deg off
    assert v <= MAX_V * 0.5 + 1e-9          # cos(60 deg) = 0.5, and turning slows it down more


def test_never_slower_than_minimum():
    v, w, _ = compute_wall_follow(front=FAR, closest=WALL_DISTANCE,
                                     closest_angle=math.radians(0))      # 90 deg off
    assert v == pytest.approx(MAX_V * MIN_V_FACTOR)


def test_turn_rate_is_clamped():
    v, w, _ = compute_wall_follow(front=FAR, closest=0.05, closest_angle=math.radians(-30))
    assert w == MAX_W


def test_slows_down_when_turning_hard():
    # parallel (angle error 0) but too far -> turns right; the harder it turns, the slower
    v_small, w_small, _ = compute_wall_follow(FAR, WALL_DISTANCE + 0.02, RIGHT)
    # (+0.06, not more: from WALL_LOST on the follower switches to 'outer' arcing)
    v_big, w_big, _ = compute_wall_follow(FAR, WALL_DISTANCE + 0.06, RIGHT)
    assert abs(w_big) > abs(w_small)
    assert v_big < v_small
    assert v_big == pytest.approx(MAX_V * max(MIN_V_FACTOR, 1 - abs(w_big) / MAX_W))