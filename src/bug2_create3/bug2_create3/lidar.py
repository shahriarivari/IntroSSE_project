import math

def angle_to_index(angle, angle_min, angle_increment):
    i = round((angle - angle_min) / angle_increment)
    return i

def sector_min(ranges, center, half_width, angle_min, angle_increment, range_min, range_max):
    start = angle_to_index(center - half_width, angle_min, angle_increment)
    end = angle_to_index(center + half_width, angle_min, angle_increment)
    smallest_value = float('inf')
    for value in ranges[start:end + 1]:
        if not math.isfinite(value):
            continue
        if value < range_min or value > range_max:
            continue
        if value < smallest_value:
            smallest_value = value
    return smallest_value


def sector_closest(ranges, center, half_width, angle_min, angle_increment, range_min, range_max,
                   tolerance=0.02):
    """Closest valid point in a sector.

    Returns (distance, angle): distance [m] and its angle [rad] in the lidar frame.
    Returns (inf, None) if the sector has no valid measurement.

    On a flat wall many points are almost equally close, and lidar noise makes the
    single closest one jump around by several degrees. So the angle is the AVERAGE
    angle of all points within 'tolerance' [m] of the closest distance. That gives a
    much steadier angle.
    """
    start = angle_to_index(center - half_width, angle_min, angle_increment)
    end = angle_to_index(center + half_width, angle_min, angle_increment)

    valid = []                                 # (distance, index) of all valid points
    for index in range(start, end + 1):
        value = ranges[index]
        if not math.isfinite(value):
            continue
        if value < range_min or value > range_max:
            continue
        valid.append((value, index))

    if not valid:
        return float('inf'), None

    best_distance = min(d for d, _ in valid)
    near = [i for d, i in valid if d <= best_distance + tolerance]
    mean_index = sum(near) / len(near)
    return best_distance, angle_min + mean_index * angle_increment


def corridor_clearance(ranges, direction, half_width, angle_min, angle_increment,
                       range_min, range_max, body_radius=None):
    """Free distance in the area the robot will drive through (not a cone).

    direction:   direction of the corridor in the lidar frame [rad] (e.g. toward the goal)
    half_width:  half the corridor width [m] = robot radius + a small safety margin
    body_radius: real robot radius [m]. None = same as half_width (no margin).

    Every valid point is split into 'ahead' (along direction) and 'sideways'
    (perpendicular to it). A point is in the way if the robot, driving forward from
    where it is NOW, would sweep over it:
      - |sideways| < body_radius and ahead > 0: straight in the robot's path, or
      - |sideways| < half_width (safety margin) and the point lies AHEAD of the
        robot's current (margin-inflated) circle: ahead > sqrt(half_width^2 - sideways^2).
    A point in the margin right BESIDE the robot (ahead ~ 0) is not counted: driving
    forward moves away from it. (Run 2026-10-07: wall 17.7 cm to the left at the start
    gave ahead = 3 mm -> false hit before the robot even moved.)
    Returns the smallest 'ahead' of the points in the way [m], inf if there are none.

    Why not a cone (sector_min)? A cone gets wider with distance, so a wall running
    parallel next to the robot falls inside it and looks like an obstacle in front.
    """
    if body_radius is None:
        body_radius = half_width
    best = float('inf')
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min or value > range_max:
            continue
        relative = angle_min + index * angle_increment - direction
        ahead = value * math.cos(relative)
        sideways = abs(value * math.sin(relative))
        if ahead <= 0.0 or sideways >= half_width:
            continue
        in_body_path = sideways < body_radius
        ahead_of_robot = ahead > math.sqrt(half_width ** 2 - sideways ** 2)
        if (in_body_path or ahead_of_robot) and ahead < best:
            best = ahead
    return best
