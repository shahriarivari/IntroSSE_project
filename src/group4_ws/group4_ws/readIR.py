#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from irobot_create_msgs.msg import IrIntensityVector, LightringLeds, LedColor
from geometry_msgs.msg import Twist


SENSOR_TO_LED = {
    "ir_intensity_side_left":          1,
    "ir_intensity_left":               2,
    "ir_intensity_front_left":         3,
    "ir_intensity_front_center_left":  3,
    "ir_intensity_front_center_right": 3,
    "ir_intensity_front_right":        4,
    "ir_intensity_right":              5,
}

REVERSE = True
ROTATE  = 6

OFF    = (0, 0, 0)
YELLOW = (255, 150, 0)
RED    = (255, 0, 0)
GREEN  = (0, 255, 0)

DETECT_THRESHOLD = 50     # above this = "something found"
FORWARD_SPEED    = 0.15   # m/s
TURN_SPEED       = 1.0    # rad/s


def fix_index(i):
    if REVERSE:
        i = 5 - i
    return (i + ROTATE) % 6


def value_to_color(value):
    if value < DETECT_THRESHOLD:
        return OFF
    elif value < 400:
        return YELLOW
    else:
        return RED


class IRLightController(Node):

    def __init__(self):
        super().__init__("ir_light_controller")
        self.latestIr = None

        # Subscriber: IR sensors
        self.irSubscriber = self.create_subscription(
            IrIntensityVector,
            "/robot_4/ir_intensity",
            self.ir_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publisher: light ring
        self.lightPublisher = self.create_publisher(
            LightringLeds, "/robot_4/cmd_lightring", 10)

        # Publisher: driving
        self.cmdVelPublisher = self.create_publisher(
            Twist, "/robot_4/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.update)   # 10x per second

    def ir_callback(self, msg):
        self.latestIr = msg

    def update(self):
        if self.latestIr is None:
            return

        led_values = [0] * 6
        left_total = 0
        right_total = 0
        detected = False

        for reading in self.latestIr.readings:
            name = reading.header.frame_id
            value = reading.value

            if name in SENSOR_TO_LED:
                i = SENSOR_TO_LED[name]
                led_values[i] = max(led_values[i], value)

            if value >= DETECT_THRESHOLD:
                detected = True

            # Which side is the obstacle on?
            if "left" in name:
                left_total += value
            elif "right" in name:
                right_total += value

        # ---- Lights ----
        light = LightringLeds()
        light.override_system = True
        for i in range(6):
            if detected:
                r, g, b = value_to_color(led_values[i])
            else:
                r, g, b = GREEN
            light.leds[fix_index(i)] = LedColor(red=r, green=g, blue=b)
        self.lightPublisher.publish(light)

        # ---- Driving ----
        twist = Twist()
        if detected:
            twist.linear.x = 0.0
            # positive z = turn left, negative z = turn right
            if left_total > right_total:
                twist.angular.z = -TURN_SPEED   # obstacle left -> turn right
            else:
                twist.angular.z = TURN_SPEED    # obstacle right -> turn left
        else:
            twist.linear.x = FORWARD_SPEED
            twist.angular.z = 0.0
        self.cmdVelPublisher.publish(twist)

    def stop(self):
        self.cmdVelPublisher.publish(Twist())          # all zeros = stop
        light = LightringLeds()
        light.override_system = False                  # give lights back to robot
        self.lightPublisher.publish(light)


def main():
    rclpy.init()
    node = IRLightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()