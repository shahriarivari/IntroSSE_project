"""Bug2 decisions (ROS-free)."""

M_LINE_TOLERANCE = 0.03   # [m] counts as "on the m-line" (robot moves 1.5 cm per tick + noise)
LEAVE_MARGIN = 0.10       # [m] must be this much closer to the goal than at the hit point,
                          #     so noise/drift right after the hit doesn't count as "closer".
                          #     Was 0.05: the robot coasts 5-6 cm after the stop command at a
                          #     hit, which already looked "closer" -> leave -> hit flapping.


def on_m_line(side, previous_side):
    """True if the robot is on the m-line this tick.

    side:           signed distance to the m-line now (geometry.signed_distance_to_line) [m]
    previous_side:  the same value one tick earlier (None on the first tick)              [m]

    On the line = inside the +-M_LINE_TOLERANCE band, OR the sign flipped since the
    previous tick. The robot moves ~2.5 cm per tick, so at a steep crossing it can jump
    right over the 6 cm wide band without any tick landing inside it.
    """
    inside_band = abs(side) < M_LINE_TOLERANCE
    crossed = previous_side is not None and side * previous_side < 0.0
    return inside_band or crossed


def should_leave(line_distance, goal_distance, hit_goal_distance):
    """Decide whether to stop wall-following and drive to the goal again (plain Bug2).

    line_distance:      distance from the robot to the m-line (start -> goal)        [m]
    goal_distance:      distance from the robot to the goal now                      [m]
    hit_goal_distance:  distance from the robot to the goal at the hit point         [m]

    Returns True if both hold:
      - on the m-line:          line_distance < M_LINE_TOLERANCE
      - closer than at the hit: hit_goal_distance - goal_distance > LEAVE_MARGIN

    There is deliberately NO "path toward the goal is free" check. If the next obstacle
    is right in front, the robot simply hits it: a new hit point that is closer to the
    goal, and Bug2 goes around that obstacle. With a free-path check the robot kept
    following the old obstacle back to its hit point and reported UNREACHABLE.
    LEAVE_MARGIN prevents leave -> hit -> leave at the same spot.
    """
    on_m_line = line_distance < M_LINE_TOLERANCE
    closer = hit_goal_distance - goal_distance > LEAVE_MARGIN
    return on_m_line and closer
