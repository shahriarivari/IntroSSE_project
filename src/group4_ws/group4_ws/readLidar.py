#!/usr/bin/env python3

from turtle import distance

from numpy import angle

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from statistics import mean, median

class LidarSectors:

    def __init__(self, number_of_sectors):

        self.number_of_sectors = number_of_sectors

    def process_scan(self, msg):

        # --------------------------------------------------
        # Calculate angular properties
        # --------------------------------------------------

        angle_min = msg.angle_min
        angle_max = msg.angle_max

        total_angle = angle_max - angle_min

        sector_width = total_angle / self.number_of_sectors

        # --------------------------------------------------
        # Create empty sectors
        # --------------------------------------------------

        sectors = [
            []
            for _ in range(self.number_of_sectors)
        ]

        # --------------------------------------------------
        # Put each LiDAR measurement into a sector
        # --------------------------------------------------

        for i, distance in enumerate(msg.ranges):

            # Ignore NaN / infinity
            if not math.isfinite(distance):
                continue

            # Ignore values outside valid LiDAR range
            if not (
                msg.range_min
                <= distance
                <= msg.range_max
            ):
                continue

            # Calculate angle of measurement
            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            # Determine sector
            sector_index = int(
                (angle - angle_min)
                / sector_width
            )

            # Safety check
            if sector_index < 0:
                continue

            if sector_index >= self.number_of_sectors:
                sector_index = self.number_of_sectors - 1

            sectors[sector_index].append(distance)

        # --------------------------------------------------
        # Calculate information for each sector
        # --------------------------------------------------

        results = []

        for i, measurements in enumerate(sectors):

            # Angular boundaries
            start_angle = (
                angle_min
                + i * sector_width
            )

            end_angle = (
                start_angle
                + sector_width
            )

            center_angle = (
                start_angle
                + sector_width / 2
            )

            # Convert radians to degrees
            start_angle_deg = math.degrees(
                start_angle
            )

            end_angle_deg = math.degrees(
                end_angle
            )

            center_angle_deg = math.degrees(
                center_angle
            )

            # Calculate statistics
            if measurements:

                minimum = min(measurements)
                average = mean(measurements)
                median_value = median(measurements)
                count = len(measurements)

            else:

                minimum = float("inf")
                average = float("inf")
                median_value = float("inf")
                count = 0

            # Store everything about this sector
            sector_data = {

                "index": i,

                "start_angle": start_angle_deg,

                "end_angle": end_angle_deg,

                "center_angle": center_angle_deg,

                "minimum": minimum,

                "average": average,

                "median": median_value,

                "count": count
            }

            results.append(sector_data)

        return results


class LidarReader(Node):

    def __init__(self):

        super().__init__('lidar_reader')

        # --------------------------------------------------
        # Configuration
        # --------------------------------------------------

        self.number_of_sectors = 8

        self.lidar_sectors = LidarSectors(
            self.number_of_sectors
        )

        # --------------------------------------------------
        # ROS2 subscriber
        # --------------------------------------------------

        self.subscription = self.create_subscription(
            LaserScan,
            '/robot_4/scan',
            self.lidar_callback,
            10
        )

        self.get_logger().info(
            f"LiDAR reader started with "
            f"{self.number_of_sectors} sectors"
        )

    def lidar_callback(self, msg):

        # Process scan
        results = self.lidar_sectors.process_scan(msg)

        # --------------------------------------------------
        # Display sector information
        # --------------------------------------------------

        for sector in results:

            self.get_logger().info(
                f"Sector {sector['index']} | "
                f"Angle: "
                f"{sector['start_angle']:.1f}° → "
                f"{sector['end_angle']:.1f}° | "
                f"Center: "
                f"{sector['center_angle']:.1f}° | "
                f"Min: "
                f"{sector['minimum']:.2f} m | "
                f"Avg: "
                f"{sector['average']:.2f} m | "
                f"Median: "
                f"{sector['median']:.2f} m | "
                f"Points: "
                f"{sector['count']}"
            )





def main():

    rclpy.init()

    node = LidarReader()
    
    try:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__': 
    main()
