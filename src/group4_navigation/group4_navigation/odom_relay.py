"""odom (Create 3, BEST_EFFORT) -> odom_reliable (RELIABLE), for Nav2.

Nav2's controller_server and bt_navigator subscribe to odom with RELIABLE QoS. A RELIABLE
subscriber never receives from a BEST_EFFORT publisher (run 2026-10-13: "New publisher
discovered on topic '/robot_4/odom', offering incompatible QoS ... RELIABILITY_QOS_POLICY").
Started by navigation.launch.py in the robot namespace.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        self.publisher = self.create_publisher(Odometry, 'odom_reliable', 10)
        self.create_subscription(Odometry, 'odom', self.publisher.publish,
                                 qos_profile_sensor_data)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
