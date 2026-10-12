import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import StaticTransformBroadcaster

PARENT = 'map'
CHILD = 'goal'


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # goal in the map frame; changeable at runtime
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('goal_yaw', 0.0)   # radians, 0 = facing +x
        self.goal = {n: self.get_parameter(n).value for n in ('goal_x', 'goal_y', 'goal_yaw')}

        self.broadcaster = StaticTransformBroadcaster(self)   # publishes on /tf_static

        # way 1: `ros2 param set` -> this callback
        self.add_on_set_parameters_callback(self.on_params)
        # way 2: a PoseStamped on /set_goal (CLI or Foxglove click)
        self.create_subscription(PoseStamped, 'set_goal', self.on_set_goal, 10)

        self.publish_goal()

    # ---------- inputs ----------
    def on_params(self, params):
        new_goal = dict(self.goal)
        for p in params:
            if p.name in new_goal:
                if p.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(successful=False,
                                               reason=f'{p.name} must be a float, e.g. 3.0')
                new_goal[p.name] = p.value
        self.goal = new_goal
        self.publish_goal()
        return SetParametersResult(successful=True)

    def on_set_goal(self, msg):
        if msg.header.frame_id not in (PARENT, ''):
            self.get_logger().warn(f'set_goal must be in frame "{PARENT}", got "{msg.header.frame_id}"')
            return
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # go through the parameters, so `ros2 param get` always shows the current goal
        self.set_parameters([
            Parameter('goal_x', Parameter.Type.DOUBLE, float(msg.pose.position.x)),
            Parameter('goal_y', Parameter.Type.DOUBLE, float(msg.pose.position.y)),
            Parameter('goal_yaw', Parameter.Type.DOUBLE, float(yaw)),
        ])

    # ---------- output ----------
    def publish_goal(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = PARENT
        t.child_frame_id = CHILD
        t.transform.translation.x = self.goal['goal_x']
        t.transform.translation.y = self.goal['goal_y']
        yaw = self.goal['goal_yaw']
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster.sendTransform(t)   # same child -> replaces the old goal
        self.get_logger().info(
            f'Goal set: x={self.goal["goal_x"]:.2f} y={self.goal["goal_y"]:.2f} '
            f'yaw={self.goal["goal_yaw"]:.2f} (frame {PARENT} -> {CHILD})')


def main():
    rclpy.init()
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()