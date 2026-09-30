#!/usr/bin/env python3
import rclpy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

MINIMUM_RANGE=0.15000000596046448
MAXIMUM_RANGE=12.0
# len(msg.ranges) == 720
# Note: values < range_min or > range_max should be discarded

# ranges[0] is the back
# ranges[180] is the right
# ranges[360] is the front
# ranges[540] is the left

'''
    divide ranges up into sectors:
        1. Back-right    [45:135]
        2. Right         [135:225]
        3. Front-right   [225:315]
        4. Front-center  [315:405]
        5. Front-left    [405:495]
        6. Left          [495:585]
        7. Back-left     [585:675]
        8. Back-center   [675:45]
'''


class DriveLidar(Node):
    def __init__(self):
        super().__init__('drive_lidar')
        self.create_subscription(LaserScan, '/robot_4/scan', self.scan_callback, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(Marker, '/robot_4/scan_sectors', 10)

    def scan_callback(self, msg):
        print(msg.ranges[180])
        self.publish_sectors(msg)
        # print(f'minimum range: {msg.range_min}\n maximum range: {msg.range_max}\n length of values: {len(msg.ranges)}\n start angle of scan: {msg.angle_min}\n end angle of scan: {msg.angle_max}')
        pass

    SECTOR_COLORS = [
        (1.0, 0.0, 0.0),   # 1 back-right    red
        (1.0, 0.5, 0.0),   # 2 right         orange
        (1.0, 1.0, 0.0),   # 3 front-right   yellow
        (0.0, 1.0, 0.0),   # 4 front-center  green
        (0.0, 1.0, 1.0),   # 5 front-left    cyan
        (0.0, 0.3, 1.0),   # 6 left          blue
        (0.6, 0.0, 1.0),   # 7 back-left     purple
        (1.0, 0.0, 1.0),   # 8 back-center   pink
    ]

    def publish_sectors(self, msg):
        m = Marker()
        m.header = msg.header              # same frame as the lidar
        m.ns = 'sectors'
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.04                   # point size [m]
        m.scale.y = 0.04

        n = len(msg.ranges)
        w = n // 8                         # 90 for your 720-point scan
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            m.points.append(Point(x=r * math.cos(a), y=r * math.sin(a), z=0.0))

            sector = min(((i - w // 2) % n) // w, 7)   # 0 = back-right ... 7 = back-center
            cr, cg, cb = self.SECTOR_COLORS[sector]
            m.colors.append(ColorRGBA(r=cr, g=cg, b=cb, a=1.0))

        self.marker_pub.publish(m)

def main():
    rclpy.init()
    node = DriveLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()