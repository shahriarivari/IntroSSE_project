import math

def quaternion_to_yaw(z, w):
    '''
        returns radians
    '''
    yaw = math.atan2(z, w) * 2
    return yaw

def wrap_angle(angle):
    '''
        returns correct angle between -Pi and Pi
    '''
    x = math.cos(angle)
    y = math.sin(angle)
    return math.atan2(y, x)

def distance_to_goal(x, y, goal_x, goal_y):
    dx = goal_x - x
    dy = goal_y - y
    return math.hypot(dx, dy) 

def angle_to_goal(x, y, goal_x, goal_y):
    dx = goal_x - x
    dy = goal_y - y
    return math.atan2(dy, dx)

def distance_to_line(sx, sy, gx, gy, px, py):
    """Perpendicular distance [m] from P to the infinite line through S and G.

    Start:   sx, sy
    Goal:    gx, gy
    Robot:   px, py
    F (not computed) is the point on the line straight below P. The answer is |FP|.
    """
    m_x, m_y = gx - sx, gy - sy                   # direction of the m-line
    length_m = math.hypot(m_x, m_y)
    if length_m == 0:
        raise ValueError('start and goal are the same point: no m-line')

    relative_x, relative_y = px - sx, py - sy     # robot relative to start
    hypotenuse_sq = relative_x**2 + relative_y**2 # |SP|^2, no sqrt needed
    t = (m_x * relative_x + m_y * relative_y) / length_m   # distance along the line, |SF|
    return math.sqrt(max(0.0, hypotenuse_sq - t**2))       # |FP|, never sqrt of < 0

def signed_distance_to_line(sx, sy, gx, gy, px, py):
    """Signed distance [m] from P to the line S -> G: > 0 = left of S -> G, < 0 = right.

    Same size as distance_to_line, but the sign tells on which side the robot is.
    A sign change between two ticks means the robot crossed the line.
    """
    m_x, m_y = gx - sx, gy - sy                   # direction of the m-line
    length_m = math.hypot(m_x, m_y)
    if length_m == 0:
        raise ValueError('start and goal are the same point: no m-line')

    relative_x, relative_y = px - sx, py - sy     # robot relative to start
    return (m_x * relative_y - m_y * relative_x) / length_m   # 2D cross product / |SG|

if __name__ == "__main__":
    pass