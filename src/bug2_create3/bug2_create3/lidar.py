import math

def angle_to_index(angle, angle_min, angle_increment):
    i = round((angle - angle_min) / angle_increment)
    return i


def sector_indices(n, center, half_width, angle_min, angle_increment):
    """Indices of the rays in the sector center +- half_width.

    A full 360 deg scan wraps around: a sector around the back (e.g. 170 .. 190 deg) takes
    the last rays and then the first ones. A partial scan is cut off at its ends instead.
    """
    start = angle_to_index(center - half_width, angle_min, angle_increment)
    end = angle_to_index(center + half_width, angle_min, angle_increment)
    full_circle = n * angle_increment >= 2 * math.pi - 1.5 * angle_increment
    if full_circle:
        end = min(end, start + n - 1)            # never the same ray twice
        return [index % n for index in range(start, end + 1)]
    return list(range(max(start, 0), min(end, n - 1) + 1))


def sector_min(ranges, center, half_width, angle_min, angle_increment, range_min, range_max):
    smallest_value = float('inf')
    for index in sector_indices(len(ranges), center, half_width, angle_min, angle_increment):
        value = ranges[index]
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

    Returns (distance, angle): distance [m] and its angle [rad] in the scan frame.
    Returns (inf, None) if the sector has no valid measurement.

    On a flat wall many points are almost equally close, and lidar noise makes the
    single closest one jump around by several degrees. So the angle is the AVERAGE
    angle of all points within 'tolerance' [m] of the closest distance. That gives a
    much steadier angle.
    """
    valid = []                                 # (distance, angle) of all valid points
    for index in sector_indices(len(ranges), center, half_width, angle_min, angle_increment):
        value = ranges[index]
        if not math.isfinite(value):
            continue
        if value < range_min or value > range_max:
            continue
        # angle relative to the sector centre, so a sector across +-180 deg averages right
        angle = angle_min + index * angle_increment
        valid.append((value, math.atan2(math.sin(angle - center), math.cos(angle - center))))

    if not valid:
        return float('inf'), None

    best_distance = min(d for d, _ in valid)
    near = [a for d, a in valid if d <= best_distance + tolerance]
    mean_angle = center + sum(near) / len(near)
    return best_distance, math.atan2(math.sin(mean_angle), math.cos(mean_angle))


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


def mount_from_transform(tx, ty, qx, qy, qz, qw):
    """Lidar mount on the robot, from the TF base_footprint -> lidar frame.

    tx, ty:          lidar position in the robot frame [m] (z is not needed)
    qx, qy, qz, qw:  lidar orientation (quaternion). Full rotation, so a lidar that is
                     mounted backwards (yaw 180 deg) or upside down (roll 180 deg) works too.
    Returns (tx, ty, a, b, c, d): the lidar's x axis is (a, c) and its y axis (b, d) in the
    robot frame, or None if the lidar sits in the robot centre and looks forward (then the
    scan can be used as it is).
    """
    a = 1 - 2 * (qy * qy + qz * qz)
    b = 2 * (qx * qy - qz * qw)
    c = 2 * (qx * qy + qz * qw)
    d = 1 - 2 * (qx * qx + qz * qz)
    if math.hypot(tx, ty) < 0.001 and abs(a - 1) < 1e-4 and abs(d - 1) < 1e-4:
        return None
    return tx, ty, a, b, c, d


def to_base_frame(ranges, angle_min, angle_increment, range_min, range_max, mount):
    """The scan as if it were measured from the robot centre (base_footprint), 0 = front.

    mount: from mount_from_transform, or None (= lidar already in the centre, looking forward).
    Returns (ranges, (angle_min, angle_increment, range_min, range_max)), ready for
    sector_min / sector_closest / corridor_clearance. Every valid point is moved into the
    robot frame and put in the ray of its new angle (a full circle of len(ranges) rays,
    starting at -180 deg). Two points in one ray: the closest one is kept.
    Invalid rays (inf, nan, outside range_min..range_max) are dropped.
    """
    if mount is None:
        return ranges, (angle_min, angle_increment, range_min, range_max)
    tx, ty, a, b, c, d = mount
    n = len(ranges)
    out_increment = 2 * math.pi / n
    out = [float('inf')] * n
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min or value > range_max:
            continue
        angle = angle_min + index * angle_increment
        lx, ly = value * math.cos(angle), value * math.sin(angle)   # point in the lidar frame
        x, y = tx + a * lx + b * ly, ty + c * lx + d * ly           # point in the robot frame
        out_index = round((math.atan2(y, x) + math.pi) / out_increment) % n
        out[out_index] = min(out[out_index], math.hypot(x, y))
    # A point just in front of the lidar can be closer to the robot centre than range_min.
    return out, (-math.pi, out_increment, 0.0, range_max + math.hypot(tx, ty))
