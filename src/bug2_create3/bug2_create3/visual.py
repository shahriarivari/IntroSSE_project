"""Bug2's internal state as ROS messages, for Foxglove (3D panel). No decisions in here.

Everything is drawn in the pose frame (map for pose_source 'slam', odom for 'odom'):
  m-line (current leg), goal + GOAL_TOLERANCE circle, home, hit point with the
  HIT_LEFT_RADIUS / HIT_RETURN_RADIUS circles, and a text with state + rule above the robot.
"""
import math

from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

NAMESPACE = 'bug2'
CIRCLE_POINTS = 36

GREEN = ColorRGBA(r=0.1, g=0.8, b=0.2, a=1.0)
BLUE = ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0)
ORANGE = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
RED = ColorRGBA(r=1.0, g=0.15, b=0.15, a=1.0)
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def _marker(header, marker_id, marker_type, color, width=0.02):
    marker = Marker()
    marker.header = header
    marker.ns = NAMESPACE
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    marker.color = color
    return marker


def _line(header, marker_id, color, xy_points):
    marker = _marker(header, marker_id, Marker.LINE_STRIP, color)
    marker.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in xy_points]
    return marker


def _circle(header, marker_id, color, center, radius):
    cx, cy = center
    points = [(cx + radius * math.cos(2 * math.pi * i / CIRCLE_POINTS),
               cy + radius * math.sin(2 * math.pi * i / CIRCLE_POINTS))
              for i in range(CIRCLE_POINTS + 1)]
    return _line(header, marker_id, color, points)


def _deleted(header, marker_id):
    marker = Marker()
    marker.header = header
    marker.ns = NAMESPACE
    marker.id = marker_id
    marker.action = Marker.DELETE
    return marker


def bug2_markers(header, robot_xy, start, goal, home, hit_point, state, rule, leg, hand,
                 goal_tolerance, hit_left_radius, hit_return_radius):
    """MarkerArray with the current Bug2 situation. hit_point may be None (no hit yet)."""
    markers = [
        _line(header, 0, BLUE, [start, goal]),                       # m-line of this leg
        _circle(header, 1, GREEN, goal, goal_tolerance),              # target of this leg
        _circle(header, 2, WHITE, home, 0.05),                        # start of the run
    ]
    if hit_point is None or state != 'WALL_FOLLOW':
        markers += [_deleted(header, 3), _deleted(header, 4), _deleted(header, 5)]
    else:
        hit = _marker(header, 3, Marker.SPHERE, RED)
        hit.pose.position.x, hit.pose.position.y = float(hit_point[0]), float(hit_point[1])
        hit.scale.x = hit.scale.y = hit.scale.z = 0.08
        markers += [hit,
                    _circle(header, 4, ORANGE, hit_point, hit_left_radius),
                    _circle(header, 5, RED, hit_point, hit_return_radius)]

    text = _marker(header, 6, Marker.TEXT_VIEW_FACING, WHITE)
    text.pose.position.x, text.pose.position.y = float(robot_xy[0]), float(robot_xy[1])
    text.pose.position.z = 0.4
    text.scale.z = 0.12                                              # text height [m]
    text.text = f'{state} {rule} | leg {leg} | {"right" if hand > 0 else "left"} hand'
    markers.append(text)
    return MarkerArray(markers=markers)


def robot_outline(base_header, radius):
    """Create 3 outline (circle + line to the front) in the robot frame (base_footprint).

    frame_locked: Foxglove moves it with the robot's TF every frame, so it follows the robot
    smoothly. (Nav2's local_costmap/published_footprint only exists while Nav2 runs, and
    stood still in round 1.)
    """
    points = [(radius * math.cos(2 * math.pi * i / CIRCLE_POINTS),
               radius * math.sin(2 * math.pi * i / CIRCLE_POINTS))
              for i in range(CIRCLE_POINTS + 1)]
    marker = _line(base_header, 8, ORANGE, points + [(0.0, 0.0)])   # circle, then to the centre
    marker.frame_locked = True
    return marker


def pose_stamped(header, x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header = header
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose
