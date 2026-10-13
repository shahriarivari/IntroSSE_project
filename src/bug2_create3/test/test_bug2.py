from bug2_create3 import bug2
from bug2_create3.bug2 import should_leave

HIT = 1.47      # distance to goal at the hit point (single-box simulator)


def test_leave_at_far_side_crossing():
    assert should_leave(0.0, 0.73, HIT) is True


def test_no_leave_at_hit_point():
    assert should_leave(0.0, HIT, HIT) is False


def test_no_leave_when_barely_closer():
    assert should_leave(0.0, HIT - 0.02, HIT) is False


def test_no_leave_when_off_m_line():
    assert should_leave(0.20, 0.73, HIT) is False


def test_leave_even_if_next_obstacle_is_close():
    # plain Bug2: leave, the next obstacle then becomes a new (closer) hit point
    assert should_leave(0.0, 0.73, HIT) is True

# ---- on_m_line: band or crossing between two ticks ----
from bug2_create3.bug2 import on_m_line


def test_on_m_line_inside_band():
    assert on_m_line(0.01, 0.20) is True


def test_on_m_line_crossing_between_ticks():
    # 4 cm left -> 4 cm right: never inside the +-3 cm band, but it crossed
    assert on_m_line(-0.04, 0.04) is True


def test_on_m_line_near_miss_same_side():
    # t = 104.6 in the 2026-10-07 log: 5.5 cm away, never crossed
    assert on_m_line(0.055, 0.057) is False


def test_on_m_line_first_tick():
    assert on_m_line(0.05, None) is False


# ---- goal_reachable: obstacle before or behind the goal ----

def test_far_goal_with_free_corridor_is_reachable():
    assert bug2.goal_reachable(float('inf'), 3.0, 0.15, 0.19)


def test_obstacle_before_the_goal_blocks():
    assert not bug2.goal_reachable(0.5, 2.0, 0.15, 0.19)


def test_wall_just_behind_a_close_goal_does_not_block():
    # run 2026-10-13: goal 0.18 m away, wall 0.23 m ahead (5 cm behind the goal)
    assert bug2.goal_reachable(0.23, 0.18, 0.15, 0.19)


def test_wall_right_at_the_goal_blocks():
    # stopping 0.15 before the goal would leave only 0.15 m: the body (0.19) would touch
    assert not bug2.goal_reachable(0.30, 0.30, 0.15, 0.19)


# ---- goal_close_enough: goal next to a wall ----

def test_goal_against_a_wall_counts_as_reached_when_closest():
    # goal 5 cm in front of a wall, robot 0.26 m from the goal, wall 0.29 m ahead
    assert bug2.goal_close_enough(0.26, 0.29, 0.15, 0.19)


def test_goal_behind_a_wall_is_not_reached():
    # 0.25 m from the goal but a wall 0.10 m ahead: the goal is on the other side
    assert not bug2.goal_close_enough(0.25, 0.10, 0.15, 0.19)


def test_reachable_goal_is_not_cut_short():
    # free corridor: just drive on to within goal_tolerance
    assert not bug2.goal_close_enough(0.25, float('inf'), 0.15, 0.19)


def test_far_from_a_blocked_goal_is_not_reached():
    assert not bug2.goal_close_enough(0.50, 0.55, 0.15, 0.19)
