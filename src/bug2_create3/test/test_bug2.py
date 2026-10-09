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
