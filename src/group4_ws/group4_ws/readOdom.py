import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from nav_msgs.msg import Odometry

class OdomReader(Node):

    def __init__(self):
        super().__init__("odom_reader")
        self.latestOdom = None

        # Subscriber: odometry
        self.odomSubscriber = self.create_subscription(
            Odometry,
            "/robot_4/odom",
            self.odom_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def odom_callback(self, msg):
        self.latestOdom = msg
        print(f'Position x: {self.latestOdom.pose.pose.position.x} \n Position y: {self.latestOdom.pose.pose.position.y} \n Orientation: {self.latestOdom.pose.pose.orientation.z}')

def main():
    rclpy.init()
    node = OdomReader()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()