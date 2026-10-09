from bug2_create3 import geometry
import math

MAX_V=0.25                              #[m/s]
MAX_W=1.0                               #[rad/s]
K_ANG=1.5                               #[1/s]
K_LIN=0.5                               #[1/s]
TURN_IN_PLACE_ANGLE=math.radians(30)    #[radians]
GOAL_TOLERANCE=0.15                     #[m] was 0.10: robot passed the goal at 12 cm (odom noise + latency)

def compute_command(x, y, yaw, goal_x, goal_y):
    distance = geometry.distance_to_goal(x, y, goal_x, goal_y)  
    if distance < GOAL_TOLERANCE:
        return 0.0, 0.0

    heading_error = geometry.wrap_angle((geometry.angle_to_goal(x, y, goal_x, goal_y) - yaw))

    w = max(-MAX_W, min(K_ANG * heading_error, MAX_W))

    if abs(heading_error) > TURN_IN_PLACE_ANGLE:
        v = 0.0
    else:
        v = min(K_LIN * distance, MAX_V)
    return v, w