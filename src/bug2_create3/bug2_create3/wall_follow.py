"""Right-hand wall follower (ROS-free).

Idea: look at the CLOSEST point of the obstacle on the right-hand side of the robot.
Keep that point
  - at WALL_DISTANCE from the robot, and
  - exactly at -90 deg (straight to the right).
On a straight wall the closest point is the perpendicular one, so this keeps the robot
parallel. Around a corner or the end of a wall the closest point is the corner itself,
so the robot automatically drives a circle around it.
"""
import math

# ---- distances [m], measured from the lidar (robot radius is ~0.17 m) ----
WALL_DISTANCE = 0.22        # desired distance to the closest point on the right (~11 cm gap)
WALL_IN_FRONT = 0.34        # wall in front closer than this -> start turning left in place
TURN_RELEASE = 0.40         # ... and keep turning until the front is clear beyond this
FRONT_CRITICAL = 0.27       # while going around a corner, only turn left if this close
                            # (was 0.29: in a 50 cm gap the opposite obstacle is ~0.26 away
                            #  while rounding an end, so 0.29 made the robot turn onto it)
ROUNDING_W = -0.2           # [rad/s] follow rule turns right harder than this -> start rounding a corner
ROUNDING_EXIT = -0.05       # [rad/s] ... and stay rounding until it turns right less than this
WALL_LOST = 0.30            # nothing on the right closer than this -> arc right to find it

# ---- speeds and gains ----
MAX_V = 0.20                # [m/s] (was 0.25: less overshoot around corners, see DECISIONS)
MAX_W = 1.5                 # [rad/s] limit while following
TURN_W = 1.2                # [rad/s] turn rate in place at an inner corner (was 1.5; ~0.3 s delay
                            #         between scan and wheels -> the faster, the more overshoot)
MIN_V_FACTOR = 0.2          # never slower than 20 % of MAX_V while following
K_WALL = 8.0                # [1/s]  distance error [m]  -> turn rate (higher = tighter around ends)
K_ANGLE = 2.0               # [1/s]  angle error [rad]   -> turn rate
ARC_V = 0.66 * MAX_V               # [m/s]  speed while searching for the wall
ARC_W = ARC_V / WALL_DISTANCE   # [rad/s] arc with radius = WALL_DISTANCE

SIDE = math.radians(-90)    # [rad] where we want the closest point: straight to the right


def clamp(value, limit):
    """Keep value between -limit and +limit."""
    return max(-limit, min(value, limit))


def compute_wall_follow(front, closest, closest_angle, previous='follow'):
    """Compute (v, w, rule) for right-hand wall following.

    front:          smallest distance straight ahead [m]
    closest:        distance to the closest point on the right side [m] (inf if none)
    closest_angle:  angle of that point in the lidar frame [rad] (None if none)
    previous:       the rule returned in the previous tick (the caller stores it)
    rule:           'inner'  turning left in place at an inner corner
                    'round'  going around a corner to the right
                    'follow' following a (roughly straight) wall
                    'outer'  nothing on the right, arc right to find the wall

    The rule doubles as the follower's memory (state). Two places use it:
      - 'inner' hysteresis: start turning when front < WALL_IN_FRONT, but only stop
        when the front is clear beyond TURN_RELEASE.
      - 'round' hysteresis: start rounding when the follow rule turns right harder than
        ROUNDING_W, only stop when it is back below ROUNDING_EXIT (nearly straight).
        While rounding, a wall ahead only counts if it is closer than FRONT_CRITICAL.
    Without the second one, a single noisy tick during a right turn in a narrow
    corridor makes the far wall look like an inner corner, and the robot turns back.

    Note: the side sector the caller uses for 'closest' must NOT reach far behind
    the robot (see SIDE_CENTER / SIDE_HALF_WIDTH in the node: -120 .. -30 deg).
    """
    # 1a. Busy turning at an inner corner: keep going until the front is really clear.
    if previous == 'inner' and front < TURN_RELEASE:
        return 0.0, TURN_W, 'inner'

    # What would the follow rule do?
    follow = None
    if closest <= WALL_LOST and closest_angle is not None:
        follow = _follow(closest, closest_angle)

    # Going around a corner to the right? (sticky: see docstring)
    rounding = False
    if follow is not None:
        w_follow = follow[1]
        rounding = w_follow < ROUNDING_W or (previous == 'round' and w_follow < ROUNDING_EXIT)

    # 1b. Wall in front: start turning left in place.
    front_limit = FRONT_CRITICAL if rounding else WALL_IN_FRONT
    if front < front_limit:
        return 0.0, TURN_W, 'inner'

    # 2. Nothing on the right: arc to the right until we find the wall again.
    if follow is None:
        return ARC_V, -ARC_W, 'outer'

    # 3. Follow the wall (or go around its corner).
    v, w = follow
    return v, w, ('round' if rounding else 'follow')


def _follow(closest, closest_angle):
    """Follow rule: keep the closest point at WALL_DISTANCE and at -90 deg. Returns (v, w).

    angle_error > 0: closest point is ahead of -90 -> wall ahead-right  -> turn left
    angle_error < 0: closest point is behind -90  -> wall falls behind -> turn right
    """
    angle_error = closest_angle - SIDE
    w = clamp(K_WALL * (WALL_DISTANCE - closest) + K_ANGLE * angle_error, MAX_W)

    # Slow down when
    #   - the robot is far from parallel (big angle error), or
    #   - it has to turn hard (e.g. going around the end of a wall).
    parallel_factor = math.cos(angle_error)
    turn_factor = 1.0 - abs(w) / MAX_W
    v = MAX_V * max(MIN_V_FACTOR, min(parallel_factor, turn_factor))
    return v, w