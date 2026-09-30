#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from irobot_create_msgs.msg import IrIntensityVector, LightringLeds, LedColor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


NS = "/robot_4"

SENSOR_TO_LED = {
    "ir_intensity_side_left":          1,
    "ir_intensity_left":               2,
    "ir_intensity_front_left":         3,
    "ir_intensity_front_center_left":  3,
    "ir_intensity_front_center_right": 3,
    "ir_intensity_front_right":        4,
    "ir_intensity_right":              5,
}

# Mirrored pairs only: side_left has no right counterpart, so it's not used for steering
LEFT_SENSORS  = ["ir_intensity_left",  "ir_intensity_front_left",  "ir_intensity_front_center_left"]
RIGHT_SENSORS = ["ir_intensity_right", "ir_intensity_front_right", "ir_intensity_front_center_right"]

REVERSE = True
ROTATE  = 6

OFF    = (0, 0, 0)
YELLOW = (255, 150, 0)
RED    = (255, 0, 0)
GREEN  = (0, 255, 0)
BLUE   = (0, 80, 255)

WARN_THRESHOLD   = 150    # above this = yellow
DETECT_THRESHOLD = 500    # above this = red + avoid
TIE_MARGIN       = 100    # left/right closer than this = "straight ahead"
CLEAR_TICKS      = 3      # keep turning 3 x 0.1 s after the path is clear

FORWARD_SPEED = 0.15      # m/s
TURN_SPEED    = 1.0       # rad/s  (positive = left, negative = right)

# ---- going home ----
CRUMB_DIST     = 0.25     # drop a breadcrumb every 25 cm while exploring
REACHED        = 0.08     # close enough to a breadcrumb [m]
HOME_SPEED     = 0.15     # m/s while going home
HEADING_GAIN   = 2.0      # how hard to steer toward the next breadcrumb
DRIVE_ANGLE    = 0.3      # only drive forward when heading error < this [rad]
FINAL_YAW      = math.pi  # heading at home: pi = facing the dock (undock turns 180°)
YAW_TOLERANCE  = 0.05     # [rad]


def fix_index(i):
    if REVERSE:
        i = 5 - i
    return (i + ROTATE) % 6


def value_to_color(value):
    if value < WARN_THRESHOLD:
        return OFF
    elif value < DETECT_THRESHOLD:
        return YELLOW
    else:
        return RED


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(v, limit):
    return max(-limit, min(limit, v))


class IRLightController(Node):

    def __init__(self):
        super().__init__("ir_light_controller")
        self.latestIr = None

        # exploring
        self.turn_dir = 0      # 0 = driving straight, +1 = turning left, -1 = turning right
        self.last_turn = 1     # used to alternate when it's a tie
        self.clear_count = 0

        # position + state
        self.state = "EXPLORE"          # EXPLORE -> HOME -> (done)
        self.done = False               # main() exits when this becomes True
        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.path = [(0.0, 0.0)]        # breadcrumbs, starting at home

        self.create_subscription(
            IrIntensityVector, f"{NS}/ir_intensity", self.ir_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Odometry, f"{NS}/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            Empty, f"{NS}/go_home", self.go_home_callback, 10)

        self.lightPublisher  = self.create_publisher(LightringLeds, f"{NS}/cmd_lightring", 10)
        self.cmdVelPublisher = self.create_publisher(Twist, f"{NS}/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.update)   # 10x per second

    # ================= callbacks =================
    def ir_callback(self, msg):
        self.latestIr = msg

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.have_odom = True

        if self.state == "EXPLORE":
            lx, ly = self.path[-1]
            if math.hypot(self.x - lx, self.y - ly) > CRUMB_DIST:
                self.path.append((self.x, self.y))

    def go_home_callback(self, msg):
        if self.state != "EXPLORE":
            return
        self.state = "HOME"
        self.get_logger().info(
            f"Going home from ({self.x:.2f}, {self.y:.2f}) via {len(self.path)} breadcrumbs")

    # ================= main loop =================
    def update(self):
        if self.state == "EXPLORE":
            self.explore()
        elif self.state == "HOME":
            self.go_home()

    # ================= exploring (your IR logic) =================
    def choose_direction(self, left, right):
        diff = left - right
        if diff > TIE_MARGIN:
            direction = -1                      # obstacle left -> turn right
            reason = "obstacle left"
        elif diff < -TIE_MARGIN:
            direction = 1                       # obstacle right -> turn left
            reason = "obstacle right"
        else:
            direction = -self.last_turn         # straight ahead -> alternate
            reason = "straight ahead"
        self.last_turn = direction
        self.get_logger().info(
            f"L={left} R={right} ({reason}) -> turning {'left' if direction > 0 else 'right'}")
        return direction

    def explore(self):
        if self.latestIr is None:
            return

        values = {r.header.frame_id: r.value for r in self.latestIr.readings}

        # ---- Lights ----
        led_values = [0] * 6
        for name, value in values.items():
            if name in SENSOR_TO_LED:
                i = SENSOR_TO_LED[name]
                led_values[i] = max(led_values[i], value)

        warned = any(v >= WARN_THRESHOLD for v in led_values)
        colors = [value_to_color(v) if warned else GREEN for v in led_values]
        self.set_lights(colors)

        # ---- Driving ----
        left  = max(values.get(n, 0) for n in LEFT_SENSORS)
        right = max(values.get(n, 0) for n in RIGHT_SENSORS)
        detected = max(left, right) >= DETECT_THRESHOLD

        twist = Twist()
        if detected:
            if self.turn_dir == 0:                       # new obstacle: decide once
                self.turn_dir = self.choose_direction(left, right)
            self.clear_count = 0
            twist.angular.z = self.turn_dir * TURN_SPEED
        elif self.turn_dir != 0 and self.clear_count < CLEAR_TICKS:
            self.clear_count += 1                        # turn a little further
            twist.angular.z = self.turn_dir * TURN_SPEED
        else:
            self.turn_dir = 0                            # path clear: drive
            twist.linear.x = FORWARD_SPEED
        self.cmdVelPublisher.publish(twist)

    # ================= going home =================
    def go_home(self):
        self.set_lights([BLUE] * 6)
        if not self.have_odom:
            return

        twist = Twist()

        if self.path:
            # drive to the most recent breadcrumb, then the one before, ... back to (0, 0)
            tx, ty = self.path[-1]
            dx, dy = tx - self.x, ty - self.y
            if math.hypot(dx, dy) < REACHED:
                self.path.pop()
                if not self.path:
                    self.get_logger().info("At (0, 0), turning to final heading")
                return

            err = wrap(math.atan2(dy, dx) - self.yaw)
            twist.angular.z = clamp(HEADING_GAIN * err, TURN_SPEED)
            if abs(err) < DRIVE_ANGLE:                   # turn first, then drive
                twist.linear.x = HOME_SPEED
        else:
            # all breadcrumbs done: turn to the final heading
            err = wrap(FINAL_YAW - self.yaw)
            if abs(err) < YAW_TOLERANCE:
                self.get_logger().info(f"Home reached ({self.x:.2f}, {self.y:.2f})")
                self.cmdVelPublisher.publish(Twist())
                self.done = True
                return
            twist.angular.z = clamp(HEADING_GAIN * err, 0.8)

        self.cmdVelPublisher.publish(twist)

    # ================= helpers =================
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
    node = IRLightController()
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