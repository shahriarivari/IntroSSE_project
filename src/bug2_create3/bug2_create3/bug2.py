"""Bug2 decisions (ROS-free)."""

M_LINE_TOLERANCE = 0.03   # [m] counts as "on the m-line" (robot moves 1.5 cm per tick + noise)
GOAL_NEAR = 0.30          # [m] goal next to an obstacle: "reached" within this distance if the
                          #     robot cannot get closer (see goal_close_enough)
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


def goal_reachable(goal_clearance, goal_distance, goal_tolerance, body_radius):
    """True if the robot can drive straight on to the goal without touching anything.

    goal_clearance:  free distance in the corridor toward the goal (lidar.corridor_clearance) [m]
    goal_distance:   distance from the robot to the goal                                   [m]
    goal_tolerance:  the robot stops this far before the goal (goal reached)               [m]
    body_radius:     robot radius + safety margin                                          [m]

    The robot only drives max(0, goal_distance - goal_tolerance) more. Whatever the corridor
    sees must then still be more than body_radius ahead. So an obstacle just BEHIND the goal
    is not in the way. (Run 2026-10-13: the goal was ~5 cm from a wall. At 18 cm from the
    goal that wall, 23 cm ahead, counted as a hit every time: the robot never reached the
    goal, never started leg 2 and ended UNREACHABLE.)
    """
    return goal_clearance - max(0.0, goal_distance - goal_tolerance) > body_radius


def goal_close_enough(goal_distance, goal_clearance, goal_tolerance, body_radius):
    """Goal next to (or in) an obstacle: True when the robot is as close as it can get.

    A goal a few cm from a wall cannot be reached within goal_tolerance: the robot body
    hits the wall first (goal 5 cm from a wall: closest ~0.20 m). Count it as reached when
      - the robot is within GOAL_NEAR of the goal,
      - nothing is BETWEEN robot and goal (goal_clearance >= goal_distance: the obstacle
        is at or behind the goal, so the goal is not on the other side of a wall), and
      - it cannot drive closer (not goal_reachable).
    """
    return (goal_distance < GOAL_NEAR and goal_clearance >= goal_distance
            and not goal_reachable(goal_clearance, goal_distance, goal_tolerance, body_radius))
