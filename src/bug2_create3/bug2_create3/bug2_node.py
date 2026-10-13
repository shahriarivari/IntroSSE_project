import csv
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from bug2_create3 import bug2, geometry, go_to_goal, lidar, visual, wall_follow


# Goal RELATIVE TO THE START POSE of the robot (not the odom origin). These are only the
# DEFAULTS; set them per run without editing this file:
#   ros2 launch bug2_create3 bug2.launch.py goal_forward:=4.0 goal_left:=-0.5
GOAL_FORWARD = 3.0                          # [m] ahead of the robot at start
GOAL_LEFT = 0.0                             # [m] to the left of the robot at start (+ = left)

# Where the robot's position comes from:
#   'slam': TF map -> base_footprint (slam_toolbox corrects the odom drift with the lidar;
#           also the frame Nav2 uses, so the logged goal is directly the Nav2 goal)
#   'odom': raw wheel odometry from the odom topic (old behaviour, drifts)
POSE_SOURCE = 'slam'
MAP_FRAME = 'map'
BASE_FRAME = 'base_footprint'
POSE_TIMEOUT = 1.0          # [s] SLAM pose not updated for this long -> stop and wait (SLAM died?)

# Lidar mount: look up the static TF base_footprint -> <scan frame> once before the start and
# measure all distances/angles from the robot centre (what ROBOT_RADIUS, GOAL_CORRIDOR and the
# wall-follow distances mean). False = use the scan as it is (lidar assumed in the centre,
# 0 deg = front: the behaviour before 2026-10-13).
USE_LIDAR_TF = True
LIDAR_TF_TIMEOUT = 5.0      # [s] no lidar TF after this long -> warn and use the scan as it is

# Foxglove: /robot_4/bug2/markers, /bug2/path, /bug2/state, /bug2/rule, /bug2/goal (latched)
PATH_SPACING = 0.10         # [m] one pose in the driven path every this many metres

FRONT_HALF_WIDTH = math.radians(40)         # [rad] front sector during go-to-goal
WF_FRONT_HALF_WIDTH = math.radians(20)      # [rad] front sector during wall-follow
GOAL_CORRIDOR = 0.19                        # [m] half width of the corridor toward the goal
                                            #     (hit + leave check): robot radius 0.17 + 2 cm
ROBOT_RADIUS = 0.17                         # [m] Create 3 (0.34 m wide); GOAL_CORRIDOR adds 2 cm
FRONT = math.radians(0)                     # [rad] sector centres, 0 = front, + = left
SIDE_CENTER = math.radians(-75)             # [rad] right-hand side of the robot ...
SIDE_HALF_WIDTH = math.radians(45)          # [rad] ... from -120 to -30 deg (not far behind!)

PREFERRED_HAND = 1.0        # +1 = right hand. Used at EVERY hit, also on the way back, so
                            # the whole run follows walls the same way (more of the maze seen).

HIT_LEFT_RADIUS = 0.50      # [m] robot counts as "away from the hit point" beyond this ...
HIT_RETURN_RADIUS = 0.20    # [m] ... and as "back at the hit point" within this (odom drift)
ALIGN_LEAD = math.radians(20)  # [rad] stop the 'align' turn this much early: ~0.3 s from scan
                               #       to wheels at TURN_W 1.2 rad/s = ~20 deg overshoot
                               #       (run 13:53: overshoot put the old wall in front -> 'inner'
                               #       turned back onto it)
FAR_SWITCH = 1.0            # [m] wall following leads this much FARTHER from the goal than the
                            #     hit point -> turn around and go the other way (other hand).
                            #     Run 13:53, leg 2: 0.68 m from home it followed the wall away
                            #     and ended 4.5 m away. Other hits that run stayed < 0.3 m.
                            #     Set very large (e.g. 99) to switch this off.
MAX_FOLLOW_LENGTH = 30.0    # [m] give up (UNREACHABLE) after following this far without leaving.
                            #     Safety net for when the follower drifts onto another obstacle
                            #     and so never comes back to the hit point.

LOG_EVERY = 5                               # log every N ticks (5 x 0.1 s = 2x per second)
CSV_DIR = os.path.expanduser('~/bug2_logs')  # one CSV per run, one row per tick
# Goal of leg 1, written at the start of every run. group4_navigation's send_goal reads it
# in round 2 (the bug2/goal topic is gone once this node is stopped).
GOAL_FILE = os.path.join(CSV_DIR, 'last_goal.json')
# x, y, yaw_deg: in the pose frame (map for pose_source 'slam', odom for 'odom').
CSV_COLUMNS = ['t', 'tick', 'state', 'rule', 'x', 'y', 'yaw_deg',
               'goal_dist', 'line_dist', 'hit_goal_dist', 'goal_dir_deg', 'goal_clr',
               'front', 'closest', 'closest_angle_deg', 'v', 'w', 'odom_v', 'odom_w',
               'leg', 'hand', 'hit_dist', 'follow_len', 'other_closest', 'other_angle_deg']
# leg: 1 = to the goal, 2 = back to the start. hand: +1 right, -1 left. hit_dist: distance to the current hit point.
# other_closest/other_angle_deg: closest point on the side we are NOT following, real robot
# frame (left = +). Shows a second obstacle next to us (closest_angle_deg is mirrored for
# the left hand, other_angle_deg is not).


class Bug2Node(Node):
    def __init__(self):
        super().__init__('bug2_node')

        # ROS parameters for the goal (defaults above). dynamic_typing: '-p goal_forward:=4'
        # (an int) is accepted too, we convert to float ourselves.
        any_number = ParameterDescriptor(dynamic_typing=True)
        self.goal_forward = float(
            self.declare_parameter('goal_forward', GOAL_FORWARD, any_number).value)
        self.goal_left = float(
            self.declare_parameter('goal_left', GOAL_LEFT, any_number).value)
        self.pose_source = self.declare_parameter('pose_source', POSE_SOURCE).value
        if self.pose_source not in ('slam', 'odom'):
            raise ValueError(f"pose_source must be 'slam' or 'odom', not '{self.pose_source}'")
        self.map_frame = self.declare_parameter('map_frame', MAP_FRAME).value
        self.base_frame = self.declare_parameter('base_frame', BASE_FRAME).value
        self.pose_frame = self.map_frame if self.pose_source == 'slam' else 'odom'
        self.use_lidar_tf = bool(self.declare_parameter('use_lidar_tf', USE_LIDAR_TF).value)

        # TF for the SLAM pose. tf2 listens on the absolute /tf; the launch file remaps that
        # to the robot's /robot_4/tf (where slam_toolbox and the Create 3 publish).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_pose_stamp = None   # stamp of the latest SLAM pose ...
        self.last_pose_change = None  # ... and when (monotonic) it last changed
        # Relative topic names: the namespace (robot_4) comes from the launch file, or from
        # '--ros-args -r __ns:=/robot_4' with ros2 run. Same convention as SLAM and Nav2.
        self.subscriber_scan = self.create_subscription(
            LaserScan, 'scan', self.listener_scan, qos_profile_sensor_data)
        self.subscriber_odom = self.create_subscription(
            Odometry, 'odom', self.listener_odometry, qos_profile_sensor_data)
        self.cmdVelPublisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # For Foxglove (not used for control). The goal is latched, so a panel opened later,
        # or a round 2 goal sender, still gets it.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_publisher = self.create_publisher(PoseStamped, 'bug2/goal', latched)
        self.marker_publisher = self.create_publisher(MarkerArray, 'bug2/markers', 10)
        self.path_publisher = self.create_publisher(Path, 'bug2/path', 10)
        self.state_publisher = self.create_publisher(String, 'bug2/state', 10)
        self.rule_publisher = self.create_publisher(String, 'bug2/rule', 10)
        self.path = Path()
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

        self.counter = 0
        self.odom_message = None
        self.scan_message = None
        self.lidar_mount = None           # lidar.mount_from_transform(...); None = centre, forward
        self.lidar_mount_ready = not self.use_lidar_tf
        self.lidar_tf_wait_start = None

        # Two legs: 1 = start -> goal, 2 = goal -> back to start.
        # self.start / self.goal are the m-line of the CURRENT leg.
        # All positions are in self.pose_frame (map for 'slam', odom for 'odom').
        self.home = None              # (x, y) where the run started (target of leg 2)
        self.leg = 1
        self.start = None             # (x, y) start of the current m-line
        self.goal = None              # (x, y) target of the current leg

        # Set at every hit.
        self.hit_point = None         # (x, y)
        self.hit_goal_distance = None  # distance to the goal at the hit point [m]
        self.left_hit_point = False   # has the robot been away from the hit point yet?
        self.hand = PREFERRED_HAND    # +1 = right-hand wall following, -1 = left-hand
        self.switched_hand = False    # already tried the other hand for this hit point?
        self.follow_length = 0.0      # distance driven while wall-following since the hit [m]
        self.pass_hit_point = False   # turned around far away: must pass the hit point first
        self.turn_start_yaw = None    # yaw when the 'turnaround' started

        self.previous_side = None     # signed m-line distance of the previous tick (crossings)
        self.previous_xy = None       # position of the previous tick (follow_length)

        self.state = 'GO_TO_GOAL'     # GO_TO_GOAL / WALL_FOLLOW / DONE / UNREACHABLE
        self.wf_rule = 'follow'       # wall follower memory: rule of the previous tick
        self.stop_sent = False        # DONE / UNREACHABLE: final stop sent, cmd_vel left alone
        self.goal_blocked_close = False  # last tick: as close as possible to a goal at a wall

        # Log the settings once, so every log file says what was tested.
        settings = (
            f'START goal=({self.goal_forward:.2f} ahead, {self.goal_left:.2f} left) '
            f'pose_source={self.pose_source} ({self.pose_frame} -> {self.base_frame}) '
            f'use_lidar_tf={self.use_lidar_tf} | '
            f'WALL_DISTANCE={wall_follow.WALL_DISTANCE} WALL_IN_FRONT={wall_follow.WALL_IN_FRONT} '
            f'TURN_RELEASE={wall_follow.TURN_RELEASE} WALL_LOST={wall_follow.WALL_LOST} '
            f'FRONT_CRITICAL={wall_follow.FRONT_CRITICAL} ROUNDING_W={wall_follow.ROUNDING_W} '
            f'ROUNDING_EXIT={wall_follow.ROUNDING_EXIT} '
            f'K_WALL={wall_follow.K_WALL} K_ANGLE={wall_follow.K_ANGLE} '
            f'MAX_V={wall_follow.MAX_V} MAX_W={wall_follow.MAX_W} TURN_W={wall_follow.TURN_W} | '
            f'M_LINE_TOLERANCE={bug2.M_LINE_TOLERANCE} LEAVE_MARGIN={bug2.LEAVE_MARGIN}')
        self.get_logger().info(settings)

        # CSV log: first line = settings (starts with '#'), then a header, then one row per tick.
        os.makedirs(CSV_DIR, exist_ok=True)
        csv_name = os.path.join(CSV_DIR, time.strftime('run_%Y-%m-%d_%H-%M-%S.csv'))
        self.csv_file = open(csv_name, 'w', newline='')
        self.csv_file.write(f'# {settings}\n')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(CSV_COLUMNS)
        self.t0 = None                # time of the first logged tick [s]
        self.get_logger().info(f'CSV log: {csv_name}')

    def listener_scan(self, msg):
        self.scan_message = msg

    def listener_odometry(self, msg):
        self.odom_message = msg

    def current_pose(self):
        """Robot pose (x, y, yaw) in self.pose_frame, or None if there is no (fresh) pose."""
        if self.pose_source == 'odom':
            position = self.odom_message.pose.pose.position
            orientation = self.odom_message.pose.pose.orientation
            return position.x, position.y, geometry.quaternion_to_yaw(orientation.z,
                                                                      orientation.w)

        # 'slam': latest map -> base_footprint (= SLAM's map -> odom + Create 3's odom -> base).
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None

        # TF keeps returning the last known transform when SLAM stops publishing, so check
        # that the pose is still being updated (wall clock on this computer: no robot/laptop
        # clock comparison needed).
        stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
        now = time.monotonic()
        if stamp != self.last_pose_stamp:
            self.last_pose_stamp = stamp
            self.last_pose_change = now
        elif now - self.last_pose_change > POSE_TIMEOUT:
            return None

        translation = tf.transform.translation
        rotation = tf.transform.rotation
        return translation.x, translation.y, geometry.quaternion_to_yaw(rotation.z, rotation.w)

    def resolve_lidar_mount(self, lidar_frame):
        """Before the start, once: where the lidar sits on the robot (static TF)."""
        now = time.monotonic()
        if self.lidar_tf_wait_start is None:
            self.lidar_tf_wait_start = now
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, lidar_frame, Time())
        except TransformException as error:
            if now - self.lidar_tf_wait_start < LIDAR_TF_TIMEOUT:
                if self.counter % 10 == 1:
                    self.get_logger().info(f'Waiting for the lidar TF {self.base_frame} -> '
                                           f'{lidar_frame}...')
                return
            self.get_logger().warn(
                f'No TF {self.base_frame} -> {lidar_frame} after {LIDAR_TF_TIMEOUT} s '
                f'({error}). Using the scan as it is: lidar in the robot centre, 0 deg = front.')
            self.lidar_mount_ready = True
            return

        t, q = tf.transform.translation, tf.transform.rotation
        self.lidar_mount = lidar.mount_from_transform(t.x, t.y, q.x, q.y, q.z, q.w)
        self.lidar_mount_ready = True
        result = ('lidar in the centre, looking forward: scan used as it is'
                  if self.lidar_mount is None else 'scan converted to the robot centre')
        self.get_logger().info(
            f'LIDAR TF {self.base_frame} -> {lidar_frame}: x={t.x:+.3f} y={t.y:+.3f} '
            f'z={t.z:.3f} m, q=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}) -> {result}')

    def publish_visuals(self, x, y, rule):
        """Bug2's state for Foxglove: every tick state/rule, markers + path 2x per second."""
        self.state_publisher.publish(String(data=self.state))
        self.rule_publisher.publish(String(data=rule))
        header = Header(frame_id=self.pose_frame, stamp=self.get_clock().now().to_msg())
        last = self.path.poses[-1].pose.position if self.path.poses else None
        if last is None or math.hypot(x - last.x, y - last.y) >= PATH_SPACING:
            self.path.poses.append(visual.pose_stamped(header, x, y))
        if self.counter % LOG_EVERY != 0:
            return
        self.path.header = header
        self.path_publisher.publish(self.path)
        markers = visual.bug2_markers(
            header, (x, y), self.start, self.goal, self.home, self.hit_point, self.state, rule,
            self.leg, self.hand, go_to_goal.GOAL_TOLERANCE, HIT_LEFT_RADIUS, HIT_RETURN_RADIUS)
        base_header = Header(frame_id=self.base_frame, stamp=header.stamp)
        markers.markers.append(visual.robot_outline(base_header, ROBOT_RADIUS))
        self.marker_publisher.publish(markers)

    def start_return_leg(self, x, y, yaw):
        """Goal reached: new m-line from here back to the start, same Bug2 (right hand)."""
        # Round 2 gets the spot the robot actually reached, not the planned goal: a goal at
        # or in a wall (run 11:19, goal 4 cm from a wall) has no free cell for Nav2 nearby
        # ("Failed to create plan with tolerance"), the reached spot is free by definition.
        self.publish_nav2_goal(x, y, yaw, 'reached')
        self.leg = 2
        self.start = (x, y)
        self.goal = self.home
        # Forget everything about the old m-line and its hit points.
        self.hit_point = None
        self.hit_goal_distance = None
        self.left_hit_point = False
        self.switched_hand = False
        self.hand = PREFERRED_HAND
        self.follow_length = 0.0
        self.pass_hit_point = False
        self.previous_side = None      # otherwise a "crossing" of the new line is faked
        self.goal_blocked_close = False  # was about the old goal
        self.wf_rule = 'follow'
        self.get_logger().info(
            f'GOAL REACHED at ({x:.2f}, {y:.2f}) -> leg 2: back to the start '
            f'({self.home[0]:.2f}, {self.home[1]:.2f})')
        self.set_state('GO_TO_GOAL', 'leg 2 started')

    def set_state(self, new_state, reason):
        """Change state and always log it immediately (not only once per log interval)."""
        self.get_logger().info(f'STATE {self.state} -> {new_state} ({reason})')
        self.state = new_state

    def publish_cmd_vel(self):
        # 1. Nothing received? wait...
        if self.odom_message is None or self.scan_message is None:
            if self.counter % 10 == 1:
                self.get_logger().info('Waiting for odom and scan...')
            self.counter += 1
            return
        if not self.lidar_mount_ready:
            self.resolve_lidar_mount(self.scan_message.header.frame_id)
            if not self.lidar_mount_ready:
                self.counter += 1
                return

        # 2. Calculate all necessary values
        pose = self.current_pose()
        if pose is None:
            # No (fresh) SLAM pose: never drive blind. Before the start just wait; during
            # the run stop the robot until the pose is back.
            if self.start is None:
                if self.counter % 10 == 1:
                    self.get_logger().info(
                        f'Waiting for the SLAM pose ({self.map_frame} -> {self.base_frame}). '
                        'Is slam_toolbox active?')
            elif self.state not in ('DONE', 'UNREACHABLE'):
                self.cmdVelPublisher.publish(Twist())
                if self.counter % 10 == 0:
                    self.get_logger().warn(
                        f'SLAM pose lost or stale (> {POSE_TIMEOUT} s): standing still')
            self.counter += 1
            return
        x, y, yaw = pose

        if self.start is None:
            # First tick: remember the start and put the goal relative to the start pose.
            self.home = (x, y)
            self.start = (x, y)
            fwd, left = self.goal_forward, self.goal_left
            self.goal = (x + fwd * math.cos(yaw) - left * math.sin(yaw),
                         y + fwd * math.sin(yaw) + left * math.cos(yaw))
            self.get_logger().info(
                f'M-LINE ({self.pose_frame}) start=({self.start[0]:.2f}, {self.start[1]:.2f}) '
                f'goal=({self.goal[0]:.2f}, {self.goal[1]:.2f})')
            self.publish_nav2_goal(*self.goal, yaw, 'planned')
        # Target reached? Leg 1: turn the goal into the way back. Leg 2: done.
        # Checked BEFORE the values below, so this tick already uses the new m-line.
        # Also "reached" when, last tick, the target lay against an obstacle and the robot was
        # as close as it can get (bug2.goal_close_enough; run 2026-10-13).
        if (self.state in ('GO_TO_GOAL', 'WALL_FOLLOW')
                and (geometry.distance_to_goal(x, y, *self.goal) < go_to_goal.GOAL_TOLERANCE
                     or self.goal_blocked_close)):
            if self.goal_blocked_close:
                self.get_logger().info(
                    f'Target next to an obstacle: as close as possible '
                    f'({geometry.distance_to_goal(x, y, *self.goal):.2f} m), counts as reached')
            if self.leg == 1:
                self.start_return_leg(x, y, yaw)
            else:
                self.set_state('DONE', f'back at the start ({x:.2f}, {y:.2f})')
        gx, gy = self.goal

        distance = geometry.distance_to_goal(x, y, gx, gy)
        side = geometry.signed_distance_to_line(*self.start, gx, gy, x, y)
        line_distance = abs(side)
        on_line = bug2.on_m_line(side, self.previous_side)
        goal_dir = geometry.wrap_angle(geometry.angle_to_goal(x, y, gx, gy) - yaw)  # robot frame

        # Scan as seen from the robot centre (unchanged if the lidar sits there, see USE_LIDAR_TF).
        scan = self.scan_message
        ranges, sector = lidar.to_base_frame(scan.ranges, scan.angle_min, scan.angle_increment,
                                             scan.range_min, scan.range_max, self.lidar_mount)
        front_half_width = WF_FRONT_HALF_WIDTH if self.state == 'WALL_FOLLOW' else FRONT_HALF_WIDTH
        front = lidar.sector_min(ranges, FRONT, front_half_width, *sector)
        narrow_front = lidar.sector_min(ranges, FRONT, WF_FRONT_HALF_WIDTH, *sector)
        # Free distance in a robot-wide corridor toward the goal. Not a cone: a cone also
        # sees the wall we follow beside us and then blocks the leave (test 2, 2026-10-07).
        goal_clearance = lidar.corridor_clearance(ranges, goal_dir, GOAL_CORRIDOR, *sector,
                                                  body_radius=ROBOT_RADIUS)
        # Wall on the right (hand = +1) or, mirrored, on the left (hand = -1). The wall
        # follower itself only knows "right"; for the left hand we mirror its input/output.
        closest, closest_angle = lidar.sector_closest(ranges, self.hand * SIDE_CENTER,
                                                      SIDE_HALF_WIDTH, *sector)
        if closest_angle is not None:
            closest_angle *= self.hand
        # Only for the log: the closest point on the other side (not used for control).
        other_closest, other_angle = lidar.sector_closest(ranges, -self.hand * SIDE_CENTER,
                                                          SIDE_HALF_WIDTH, *sector)

        # 3. Decide
        v, w = 0.0, 0.0
        rule = '-'                         # which rule was used, for the log

        if self.state == 'GO_TO_GOAL':
            v, w = go_to_goal.compute_command(x, y, yaw, gx, gy)
            # Hit: something between us and the goal, or straight in front while we drive.
            # (While turning in place, v = 0, the front sector sweeps over walls that are
            # not in the way -> ignore 'front' then.)
            # Use the NARROW front (+-20 deg) here: the wide one (+-40 deg) also sees the
            # wall we just left beside us, which caused a false hit right after a leave.
            # If the corridor to the goal is clear all the way to the goal, whatever the
            # narrow front sees is beside/behind the goal (not in the way) -> no hit.
            # (False hit 12 cm from the goal in the 57 s run.)
            # Same for goal_clearance itself: a wall just BEHIND a goal close by is not in
            # the way (run 2026-10-13: goal ~5 cm from a wall was never reached).
            path_to_goal_clear = bug2.goal_reachable(goal_clearance, distance,
                                                     go_to_goal.GOAL_TOLERANCE, GOAL_CORRIDOR)
            blocked_front = v > 0 and narrow_front < wall_follow.WALL_IN_FRONT
            if not path_to_goal_clear and (goal_clearance < wall_follow.WALL_IN_FRONT
                                           or blocked_front):
                v, w, rule = 0.0, 0.0, 'hit'
                self.hit_point = (x, y)
                self.hit_goal_distance = distance
                self.left_hit_point = False
                self.switched_hand = False
                self.pass_hit_point = False
                self.hand = PREFERRED_HAND  # a switch only counts for the hit it was made at
                self.follow_length = 0.0
                # First turn until the obstacle we HIT is on the follow side, so we follow
                # that one and not a neighbour that happens to be closer to the side sector
                # (run 2026-10-07: blocked by B on the left, but followed A on the right).
                self.wf_rule = 'align'
                self.set_state('WALL_FOLLOW',
                               f'front={front:.2f} goal_clearance={goal_clearance:.2f} '
                               f'hit_point=({x:.2f}, {y:.2f}) goal_distance={distance:.2f}')
            else:
                rule = 'drive'

        elif self.state == 'WALL_FOLLOW':
            hit_distance = math.hypot(x - self.hit_point[0], y - self.hit_point[1])
            if self.pass_hit_point:
                # After a turnaround the way back leads past the hit point. That is not
                # "back at the hit point after a full lap", so only start watching for that
                # once we have passed it.
                if hit_distance < HIT_LEFT_RADIUS:
                    self.pass_hit_point = False
            elif hit_distance > HIT_LEFT_RADIUS:
                self.left_hit_point = True
            if self.previous_xy is not None:
                self.follow_length += math.hypot(x - self.previous_xy[0], y - self.previous_xy[1])

            # bug2.on_m_line also counts a crossing between two ticks; pass 0 in that case.
            leave_line_distance = 0.0 if on_line else line_distance
            if bug2.should_leave(leave_line_distance, distance, self.hit_goal_distance):
                rule = 'leave'
                self.set_state('GO_TO_GOAL',
                               f'line_distance={line_distance:.3f} goal_distance={distance:.2f} '
                               f'< hit {self.hit_goal_distance:.2f}, '
                               f'goal_clearance={goal_clearance:.2f}')
            elif (self.left_hit_point and hit_distance < HIT_RETURN_RADIUS
                  and not self.switched_hand):
                # Back at the hit point without a leave point. Often the follower drifted
                # onto a neighbouring obstacle (gaps are only ~50 cm). Before giving up,
                # go around once more with the other hand.
                rule = 'switch'
                self.hand = -self.hand
                self.switched_hand = True
                self.left_hit_point = False
                self.follow_length = 0.0
                self.wf_rule = 'follow'
                self.get_logger().info(
                    f'SWITCH HAND -> {"right" if self.hand > 0 else "left"} at '
                    f'({x:.2f}, {y:.2f}): back at hit point, trying the other way round')
            elif (not self.switched_hand
                  and distance > self.hit_goal_distance + FAR_SWITCH):
                # This way round leads far away from the goal: turn around and follow the
                # same obstacle the other way (other hand). Bug2 still works with either
                # direction; this only picks the shorter one when the first is clearly long.
                rule = 'turnaround'
                self.hand = -self.hand
                self.switched_hand = True
                self.left_hit_point = False
                self.pass_hit_point = True
                self.follow_length = 0.0
                self.wf_rule = 'turnaround'
                self.turn_start_yaw = yaw
                self.get_logger().info(
                    f'TURN AROUND -> {"right" if self.hand > 0 else "left"} hand at '
                    f'({x:.2f}, {y:.2f}): goal distance {distance:.2f} > hit '
                    f'{self.hit_goal_distance:.2f} + {FAR_SWITCH}')
            elif self.left_hit_point and hit_distance < HIT_RETURN_RADIUS:
                rule = 'stuck'
                self.set_state('UNREACHABLE',
                               f'back at hit point ({self.hit_point[0]:.2f}, '
                               f'{self.hit_point[1]:.2f}) without a leave point')
            elif self.follow_length > MAX_FOLLOW_LENGTH:
                rule = 'stuck'
                self.set_state('UNREACHABLE',
                               f'followed {self.follow_length:.1f} m without a leave point')
            elif (self.wf_rule == 'turnaround' and
                  abs(geometry.wrap_angle(yaw - self.turn_start_yaw)) < math.pi - ALIGN_LEAD):
                # Turn ~180 deg in place, away from the wall (left when the wall was on the
                # right), so the same wall ends up on the new follow side.
                v, w, rule = 0.0, -self.hand * wall_follow.TURN_W, 'turnaround'
            elif self.wf_rule == 'align' and self.hand * goal_dir > SIDE_CENTER + ALIGN_LEAD:
                # The obstacle we hit lies in the goal direction. Turn in place (left for
                # the right hand) until the goal direction, and so that obstacle, is at the
                # side-sector centre. Then the follower's closest point is that obstacle.
                v, w, rule = 0.0, self.hand * wall_follow.TURN_W, 'align'
            else:
                if self.wf_rule in ('align', 'turnaround'):
                    self.wf_rule = 'follow'          # aligned / turned: start following
                v, w, rule = wall_follow.compute_wall_follow(
                    front, closest, closest_angle, self.wf_rule)
                w *= self.hand                       # mirror the turn for the left hand
                self.wf_rule = rule

        # DONE / UNREACHABLE: v = w = 0, the robot stands still.

        # Remember this tick for crossing detection and follow length.
        self.previous_side = side
        self.previous_xy = (x, y)
        self.goal_blocked_close = bug2.goal_close_enough(
            distance, goal_clearance, go_to_goal.GOAL_TOLERANCE, GOAL_CORRIDOR)

        # DONE / UNREACHABLE: send one stop, then stay silent on cmd_vel so Nav2 (round 2)
        # can drive without this node overwriting its commands with zeros 10x per second.
        finished = self.state in ('DONE', 'UNREACHABLE')
        if not finished or not self.stop_sent:
            cmd_message = Twist()
            cmd_message.linear.x, cmd_message.angular.z = v, w
            self.cmdVelPublisher.publish(cmd_message)
            self.stop_sent = finished

        # 4. Log: every tick to the CSV ...
        self.write_csv_row(x, y, yaw, distance, line_distance, goal_dir, goal_clearance,
                           front, closest, closest_angle, rule, v, w,
                           other_closest, other_angle)
        self.publish_visuals(x, y, rule)

        # ... and a short line to the terminal every LOG_EVERY ticks
        if self.counter % LOG_EVERY == 0:
            if closest_angle is None:
                side_info = 'closest=  -  @  -  '
            else:
                side_info = f'closest={closest:.2f}@{math.degrees(closest_angle):+4.0f}'
            self.get_logger().info(
                f'{self.counter:4d} {self.state:11s} {rule:6s} | '
                f'x={x:+.2f} y={y:+.2f} yaw={math.degrees(yaw):+4.0f} '
                f'dist={distance:.2f} line={line_distance:.2f} | '
                f'front={front:.2f} goal_clr={goal_clearance:.2f} {side_info} | '
                f'v={v:.2f} w={w:+.2f}')
        self.counter += 1


    def publish_nav2_goal(self, x, y, yaw, source):
        """Goal for round 2 on bug2/goal (latched) and in GOAL_FILE.

        source: 'planned' (at the start, the goal as asked) or 'reached' (where leg 1 ended,
        overwrites the planned one).
        """
        header = Header(frame_id=self.pose_frame, stamp=self.get_clock().now().to_msg())
        self.goal_publisher.publish(visual.pose_stamped(header, x, y, yaw))
        goal = {'frame': self.pose_frame, 'x': round(x, 3), 'y': round(y, 3),
                'yaw': round(yaw, 3), 'source': source,
                'written': time.strftime('%Y-%m-%d %H:%M:%S')}
        with open(GOAL_FILE, 'w') as goal_file:
            json.dump(goal, goal_file)
        self.get_logger().info(
            f'Nav2 goal for round 2 ({source}, frame {self.pose_frame}): x={x:.2f} y={y:.2f} '
            f'-> bug2/goal + {GOAL_FILE}')

    def write_csv_row(self, x, y, yaw, distance, line_distance, goal_dir, goal_clearance,
                      front, closest, closest_angle, rule, v, w,
                      other_closest, other_angle):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        twist = self.odom_message.twist.twist      # what the robot actually does (measured)
        angle = '' if closest_angle is None else f'{math.degrees(closest_angle):.1f}'
        hit = '' if self.hit_goal_distance is None else f'{self.hit_goal_distance:.3f}'
        hit_dist = ('' if self.hit_point is None
                    else f'{math.hypot(x - self.hit_point[0], y - self.hit_point[1]):.3f}')
        other_deg = '' if other_angle is None else f'{math.degrees(other_angle):.1f}'
        self.csv.writerow([
            f'{now - self.t0:.2f}', self.counter, self.state, rule,
            f'{x:.3f}', f'{y:.3f}', f'{math.degrees(yaw):.1f}',
            f'{distance:.3f}', f'{line_distance:.3f}', hit,
            f'{math.degrees(goal_dir):.1f}', f'{goal_clearance:.3f}',
            f'{front:.3f}', f'{closest:.3f}', angle,
            f'{v:.3f}', f'{w:.3f}', f'{twist.linear.x:.3f}', f'{twist.angular.z:.3f}',
            self.leg, int(self.hand), hit_dist, f'{self.follow_length:.2f}',
            f'{other_closest:.3f}', other_deg])
        self.csv_file.flush()                     # keep the data even after a crash / Ctrl+C

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmdVelPublisher.publish(Twist())   # try to stop the robot on Ctrl+C
        except Exception:
            pass                                    # ROS may already be shut down
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()