"""Round 2: send the goal to Nav2 and report the result.

    ros2 launch group4_navigation send_goal.launch.py                 # Bug2's goal
    ros2 launch group4_navigation send_goal.launch.py goal:=3.0,0.0   # or x,y[,yaw_deg]

Goal, first one found:
  1. the 'goal' parameter (x,y or x,y,yaw_deg in the map frame)
  2. bug2/goal (latched PoseStamped), only there while bug2_node still runs
  3. ~/bug2_logs/last_goal.json, written by bug2_node at the start of round 1
Then waits until bt_navigator is active, sends navigate_to_pose, logs the progress and
exits with 0 on success. Ctrl+C cancels the goal (the robot stops).
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose

from group4_navigation import goal_source

TOPIC_WAIT = 3.0        # [s] how long to listen for bug2/goal before falling back to the file
FILE_MAX_AGE = 3600.0   # [s] warn if Bug2's goal file is older than this (an old run?)
FEEDBACK_EVERY = 2.0    # [s] progress log interval
NAVIGATOR = 'bt_navigator'


class GoalSender(Node):
    def __init__(self):
        super().__init__('send_goal')
        self.goal_text = self.declare_parameter('goal', '').value
        self.goal_file = self.declare_parameter('goal_file', goal_source.GOAL_FILE).value
        self.topic_goal = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, 'bug2/goal', self.on_bug2_goal, latched)
        # Relative names: navigate_to_pose = /robot_4/navigate_to_pose (namespace from launch)
        self.navigate = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigator_state = self.create_client(GetState, f'{NAVIGATOR}/get_state')
        self.last_feedback_log = 0.0

    def on_bug2_goal(self, msg):
        orientation = msg.pose.orientation
        self.topic_goal = {'frame': msg.header.frame_id, 'x': msg.pose.position.x,
                           'y': msg.pose.position.y,
                           'yaw': 2 * math.atan2(orientation.z, orientation.w)}

    # ---------- 1. the goal ----------
    def find_goal(self):
        goal = goal_source.parse_goal(self.goal_text)
        if goal is not None:
            self.get_logger().info(f'Goal from the parameter: {goal_source.describe(goal)}')
            return goal

        end = time.monotonic() + TOPIC_WAIT
        while self.topic_goal is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.topic_goal is not None:
            self.get_logger().info(f'Goal from bug2/goal: {goal_source.describe(self.topic_goal)}')
            return self.topic_goal

        goal, age = goal_source.read_goal_file(self.goal_file)
        if goal is None:
            self.get_logger().error(
                f'No goal: no goal:=x,y given, bug2/goal not published and no {self.goal_file}')
            return None
        self.get_logger().info(f'Goal from {self.goal_file} ({age / 60:.0f} min old): '
                               f'{goal_source.describe(goal)}')
        if age > FILE_MAX_AGE:
            self.get_logger().warn('That file is old: is it from THIS round 1?')
        return goal

    # ---------- 2. Nav2 ready? ----------
    def wait_for_nav2(self):
        self.get_logger().info(f'Waiting for {NAVIGATOR} to be active...')
        while rclpy.ok():
            if self.navigator_state.wait_for_service(timeout_sec=1.0):
                future = self.navigator_state.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                result = future.result()
                if result is not None and result.current_state.id == State.PRIMARY_STATE_ACTIVE:
                    break
            self.get_logger().info('... Nav2 not active yet (is navigation.launch.py running?)',
                                   throttle_duration_sec=5.0)
            time.sleep(0.5)
        self.navigate.wait_for_server()
        self.get_logger().info('Nav2 is active')

    # ---------- 3. drive ----------
    def drive(self, goal):
        if goal['frame'] != 'map':
            self.get_logger().warn(f"Goal is in frame '{goal['frame']}', Nav2 plans in 'map' "
                                   '(Bug2 ran with pose_source:=odom?)')
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = goal['frame']
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = goal['x']
        request.pose.pose.position.y = goal['y']
        request.pose.pose.orientation.z = math.sin(goal['yaw'] / 2)
        request.pose.pose.orientation.w = math.cos(goal['yaw'] / 2)

        sent = self.navigate.send_goal_async(request, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Nav2 rejected the goal')
            return False
        self.get_logger().info(f'Goal accepted: driving to {goal_source.describe(goal)}')
        start = time.monotonic()

        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            self.get_logger().warn('Ctrl+C: cancelling the goal')
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=3.0)
            raise

        status = result_future.result().status
        seconds = time.monotonic() - start
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'GOAL REACHED in {seconds:.1f} s')
            return True
        names = {GoalStatus.STATUS_ABORTED: 'ABORTED', GoalStatus.STATUS_CANCELED: 'CANCELED'}
        self.get_logger().error(f'Goal {names.get(status, status)} after {seconds:.1f} s '
                                '(see the Nav2 terminal for the reason)')
        return False

    def on_feedback(self, msg):
        now = time.monotonic()
        if now - self.last_feedback_log < FEEDBACK_EVERY:
            return
        self.last_feedback_log = now
        feedback = msg.feedback
        self.get_logger().info(f'distance remaining {feedback.distance_remaining:.2f} m, '
                               f'recoveries {feedback.number_of_recoveries}')


def main(args=None):
    # No rclpy SIGINT handler: Ctrl+C is then a plain KeyboardInterrupt and ROS is still up,
    # so drive() can still send the cancel to Nav2.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GoalSender()
    success = False
    try:
        goal = node.find_goal()
        if goal is not None:
            node.wait_for_nav2()
            success = node.drive(goal)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
