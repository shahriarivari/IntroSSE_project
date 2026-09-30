#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import LightringLeds, LedColor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


NS = "/robot_4"

# ---- LiDAR layout (from driveLidar) ----
# 720 readings, 0.5 deg apart:
#   ranges[0] = back, ranges[180] = right, ranges[360] = front, ranges[540] = left
# -> angle of reading i (robot frame) = (i - 360) * 0.5 deg   (0 = front, + = left, - = right)
MIN_RANGE = 0.15
MAX_RANGE = 12.0

# ---- obstacle avoidance ----
ROBOT_RADIUS = 0.17                     # Create 3 is ~34 cm wide
MARGIN       = 0.0                   # extra space on each side [m]
CLEARANCE    = ROBOT_RADIUS + MARGIN    # a path needs this much room left and right
LOOKAHEAD    = 0.5                     # a direction is "free" if clear for this far [m]
STOP_DIST    = 0.25                     # no forward driving if something is this close in front
SLOW_DIST    = 0.5                     # half speed if something is this close in front
FRONT_HALF   = math.radians(25)         # "in front" = +-25 deg
CANDIDATES   = np.radians(np.arange(-180, 180, 5))   # directions to test, every 5 deg
HYSTERESIS   = 0.3                      # prefer keeping the previous direction (less wobble)

FORWARD_SPEED = 0.5      # m/s
TURN_SPEED    = 1.0       # rad/s  (positive = left, negative = right)
HEADING_GAIN  = 2.0       # how hard to steer toward the chosen direction
DRIVE_ANGLE   = 0.3       # only drive forward when the direction is within this [rad]

# ---- goal ----
GOAL_X         = 3     # [m] forward from the start (odom x)
GOAL_Y         = 2        # [m] left from the start (odom y)
GOAL_TOLERANCE = 0.15     # close enough to the goal [m]

# ---- going home ----
CRUMB_DIST    = 0.25      # drop a breadcrumb every 25 cm while driving to the goal
REACHED       = 0.12      # close enough to a breadcrumb [m]
HOME_SPEED    = 0.15      # m/s while going home
FINAL_YAW     = math.pi   # heading at home: pi = facing the dock (undock turns 180 deg)
YAW_TOLERANCE = 0.05      # [rad]

# ---- lights ----
REVERSE = True
ROTATE  = 6
LED_DIRECTIONS = [180, 120, 60, 0, -60, -120]   # direction each LED shows (deg), LED 3 = front
RED_DIST    = 0.30        # obstacle closer than this in an LED's direction = red
YELLOW_DIST = 0.60        # closer than this = yellow

OFF    = (0, 0, 0)
YELLOW = (255, 150, 0)
RED    = (255, 0, 0)
GREEN  = (0, 255, 0)
BLUE   = (0, 80, 255)
WHITE  = (255, 255, 255)


def fix_index(i):
    if REVERSE:
        i = 5 - i
    return (i + ROTATE) % 6


def wrap(angle):
    """Wrap an angle to [-pi, pi] (works for numbers and numpy arrays)."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def clamp(v, limit):
    return max(-limit, min(limit, v))


class LidarNavigator(Node):

    def __init__(self):
        super().__init__("lidar_navigator")

        # goal can be changed at start: --ros-args -p goal_x:=2.0 -p goal_y:=1.0
        self.goal_x = self.declare_parameter("goal_x", GOAL_X).value
        self.goal_y = self.declare_parameter("goal_y", GOAL_Y).value

        # lidar (only valid points, in robot frame)
        self.angles = None       # angle of every index, computed once
        self.scan_a = None       # angles of valid points
        self.scan_r = None       # ranges of valid points

        # position + state
        self.state = "GOTO"             # GOTO -> AT_GOAL -> HOME -> (done)
        self.done = False               # main() exits when this becomes True
        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.path = [(0.0, 0.0)]        # breadcrumbs, starting at home
        self.prev_world = None          # last chosen direction (odom frame), for hysteresis
        self.chosen = 0.0               # last chosen direction (robot frame), for logging

        self.create_subscription(
            LaserScan, f"{NS}/scan", self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, f"{NS}/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            Empty, f"{NS}/go_home", self.go_home_callback, 10)

        self.lightPublisher  = self.create_publisher(LightringLeds, f"{NS}/cmd_lightring", 10)
        self.cmdVelPublisher = self.create_publisher(Twist, f"{NS}/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.update)   # 10x per second
        self.get_logger().info(f"Driving to goal ({self.goal_x:.2f}, {self.goal_y:.2f})")

    # ================= callbacks =================
    def scan_callback(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        n = len(r)
        if self.angles is None or len(self.angles) != n:
            # index n/2 = front (0), index 0 = back (-pi), index 3n/4 = left (+pi/2)
            self.angles = (np.arange(n) - n / 2) * (2 * math.pi / n)
        valid = np.isfinite(r) & (r > MIN_RANGE) & (r < MAX_RANGE)
        self.scan_a = self.angles[valid]
        self.scan_r = r[valid]

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.have_odom = True

        if self.state == "GOTO":
            lx, ly = self.path[-1]
            if math.hypot(self.x - lx, self.y - ly) > CRUMB_DIST:
                self.path.append((self.x, self.y))

    def go_home_callback(self, msg):
        if self.state not in ("GOTO", "AT_GOAL"):
            return
        self.state = "HOME"
        self.prev_world = None
        self.get_logger().info(
            f"Going home from ({self.x:.2f}, {self.y:.2f}) via {len(self.path)} breadcrumbs")

    # ================= main loop =================
    def update(self):
        if not self.have_odom or self.scan_r is None:
            return                                   # wait for odom + lidar

        if self.state == "GOTO":
            self.goto()
        elif self.state == "AT_GOAL":
            self.set_lights([WHITE] * 6)
        elif self.state == "HOME":
            self.go_home()

    # ================= lidar helpers =================
    def sector_min(self, center, half_width):
        """Closest valid reading within center +- half_width (robot frame, radians)."""
        d = np.abs(wrap(self.scan_a - center))
        sel = self.scan_r[d <= half_width]
        return float(sel.min()) if sel.size else math.inf

    def blocked_candidates(self):
        """For every candidate direction: is there an obstacle in a robot-wide corridor?"""
        near = self.scan_r < LOOKAHEAD + CLEARANCE
        a, r = self.scan_a[near], self.scan_r[near]
        if a.size == 0:
            return np.zeros(len(CANDIDATES), dtype=bool)

        rel = a[None, :] - CANDIDATES[:, None]       # angle of each point relative to each direction
        along   = r * np.cos(rel)                    # distance ahead along that direction
        lateral = np.abs(r * np.sin(rel))            # distance sideways from that direction
        in_corridor = (along > 0) & (along < LOOKAHEAD) & (lateral < CLEARANCE)
        return in_corridor.any(axis=1)

    def drive_toward(self, tx, ty, speed):
        """Twist that heads toward (tx, ty) while avoiding obstacles seen by the lidar."""
        twist = Twist()
        target_dir = wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)   # robot frame

        free = CANDIDATES[~self.blocked_candidates()]
        if free.size == 0:
            # boxed in: turn on the spot toward the target side and look again
            twist.angular.z = 0.6 * TURN_SPEED * (1 if target_dir >= 0 else -1)
            self.get_logger().info("No free direction, turning to look for space",
                                   throttle_duration_sec=1.0)
            return twist

        # pick the free direction closest to the target (+ a bit of "keep going the same way")
        cost = np.abs(wrap(free - target_dir))
        if self.prev_world is not None:
            cost += HYSTERESIS * np.abs(wrap(free + self.yaw - self.prev_world))
        choice = float(free[np.argmin(cost)])
        self.prev_world = self.yaw + choice
        self.chosen = choice

        twist.angular.z = clamp(HEADING_GAIN * choice, TURN_SPEED)

        front = self.sector_min(0.0, FRONT_HALF)
        if abs(choice) < DRIVE_ANGLE and front > STOP_DIST:     # turn first, then drive
            twist.linear.x = speed if front > SLOW_DIST else 0.5 * speed
        return twist

    # ================= driving to the goal =================
    def goto(self):
        self.update_lights()

        dist = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        if dist < GOAL_TOLERANCE:
            self.get_logger().info(f"Goal reached ({self.x:.2f}, {self.y:.2f})")
            self.cmdVelPublisher.publish(Twist())
            self.state = "AT_GOAL"
            return

        twist = self.drive_toward(self.goal_x, self.goal_y, FORWARD_SPEED)
        self.cmdVelPublisher.publish(twist)
        self.get_logger().info(
            f"pos=({self.x:.2f}, {self.y:.2f}) to goal={dist:.2f} m "
            f"dir={math.degrees(self.chosen):.0f} deg "
            f"front={self.sector_min(0.0, FRONT_HALF):.2f} m",
            throttle_duration_sec=1.0)

    # ================= going home =================
    def go_home(self):
        self.set_lights([BLUE] * 6)

        if self.path:
            # drive to the most recent breadcrumb, then the one before, ... back to (0, 0)
            tx, ty = self.path[-1]
            if math.hypot(tx - self.x, ty - self.y) < REACHED:
                self.path.pop()
                if not self.path:
                    self.get_logger().info("At (0, 0), turning to final heading")
                return
            twist = self.drive_toward(tx, ty, HOME_SPEED)
        else:
            # all breadcrumbs done: turn to the final heading
            twist = Twist()
            err = float(wrap(FINAL_YAW - self.yaw))
            if abs(err) < YAW_TOLERANCE:
                self.get_logger().info(f"Home reached ({self.x:.2f}, {self.y:.2f})")
                self.cmdVelPublisher.publish(Twist())
                self.done = True
                return
            twist.angular.z = clamp(HEADING_GAIN * err, 0.8)

        self.cmdVelPublisher.publish(twist)

    # ================= lights =================
    def update_lights(self):
        """Each LED shows how close the nearest obstacle is in its direction."""
        colors = []
        for deg in LED_DIRECTIONS:
            d = self.sector_min(math.radians(deg), math.radians(30))
            if d < RED_DIST:
                colors.append(RED)
            elif d < YELLOW_DIST:
                colors.append(YELLOW)
            else:
                colors.append(GREEN)
        self.set_lights(colors)

    def set_lights(self, colors):
        light = LightringLeds()
        light.override_system = True
        for i in range(6):
            r, g, b = colors[i]
            light.leds[fix_index(i)] = LedColor(red=r, green=g, blue=b)
        self.lightPublisher.publish(light)

    def stop(self):
        self.cmdVelPublisher.publish(Twist())            # all zeros = stop
        light = LightringLeds()
        light.override_system = False                    # give lights back to robot
        self.lightPublisher.publish(light)


def main():
    # NO: let Python handle Ctrl+C, so the ROS context is still alive for stop()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LidarNavigator()
    try:
        while rclpy.ok() and not node.done:              # exits by itself when home
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()