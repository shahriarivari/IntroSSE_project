import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from bug2_create3 import bug2, geometry, go_to_goal, lidar, wall_follow


# Goal RELATIVE TO THE START POSE of the robot (not the odom origin). These are only the
# DEFAULTS; set them per run without editing this file:
#   ros2 run bug2_create3 <executable> --ros-args -p goal_forward:=4.0 -p goal_left:=-0.5
GOAL_FORWARD = 3.0                          # [m] ahead of the robot at start
GOAL_LEFT = 0.0                             # [m] to the left of the robot at start (+ = left)

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
        self.subscriber_scan = self.create_subscription(
            LaserScan, '/robot_4/scan', self.listener_scan, qos_profile_sensor_data)
        self.subscriber_odom = self.create_subscription(
            Odometry, '/robot_4/odom', self.listener_odometry, qos_profile_sensor_data)
        self.cmdVelPublisher = self.create_publisher(Twist, '/robot_4/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

        self.counter = 0
        self.odom_message = None
        self.scan_message = None

        # Two legs: 1 = start -> goal, 2 = goal -> back to start.
        # self.start / self.goal are the m-line of the CURRENT leg.
        self.home = None              # (x, y) in odom where the run started (target of leg 2)
        self.leg = 1
        self.start = None             # (x, y) in odom, start of the current m-line
        self.goal = None              # (x, y) in odom, target of the current leg

        # Set at every hit.
        self.hit_point = None         # (x, y) in odom
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

        # Log the settings once, so every log file says what was tested.
        settings = (
            f'START goal=({self.goal_forward:.2f} ahead, {self.goal_left:.2f} left) | '
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

    def start_return_leg(self, x, y):
        """Goal reached: new m-line from here back to the start, same Bug2 (right hand)."""
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

        # 2. Calculate all necessary values
        position = self.odom_message.pose.pose.position
        orientation = self.odom_message.pose.pose.orientation
        x, y = position.x, position.y
        yaw = geometry.quaternion_to_yaw(orientation.z, orientation.w)

        if self.start is None:
            # First tick: remember the start and put the goal relative to the start pose.
            self.home = (x, y)
            self.start = (x, y)
            fwd, left = self.goal_forward, self.goal_left
            self.goal = (x + fwd * math.cos(yaw) - left * math.sin(yaw),
                         y + fwd * math.sin(yaw) + left * math.cos(yaw))
            self.get_logger().info(
                f'M-LINE start=({self.start[0]:.2f}, {self.start[1]:.2f}) '
                f'goal=({self.goal[0]:.2f}, {self.goal[1]:.2f})')
        # Target reached? Leg 1: turn the goal into the way back. Leg 2: done.
        # Checked BEFORE the values below, so this tick already uses the new m-line.
        if (self.state in ('GO_TO_GOAL', 'WALL_FOLLOW')
                and geometry.distance_to_goal(x, y, *self.goal) < go_to_goal.GOAL_TOLERANCE):
            if self.leg == 1:
                self.start_return_leg(x, y)
            else:
                self.set_state('DONE', f'back at the start ({x:.2f}, {y:.2f})')
        gx, gy = self.goal

        distance = geometry.distance_to_goal(x, y, gx, gy)
        side = geometry.signed_distance_to_line(*self.start, gx, gy, x, y)
        line_distance = abs(side)
        on_line = bug2.on_m_line(side, self.previous_side)
        goal_dir = geometry.wrap_angle(geometry.angle_to_goal(x, y, gx, gy) - yaw)  # robot frame

        scan = self.scan_message
        sector = (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
        front_half_width = WF_FRONT_HALF_WIDTH if self.state == 'WALL_FOLLOW' else FRONT_HALF_WIDTH
        front = lidar.sector_min(scan.ranges, FRONT, front_half_width, *sector)
        narrow_front = lidar.sector_min(scan.ranges, FRONT, WF_FRONT_HALF_WIDTH, *sector)
        # Free distance in a robot-wide corridor toward the goal. Not a cone: a cone also
        # sees the wall we follow beside us and then blocks the leave (test 2, 2026-10-07).
        goal_clearance = lidar.corridor_clearance(scan.ranges, goal_dir, GOAL_CORRIDOR, *sector,
                                                  body_radius=ROBOT_RADIUS)
        # Wall on the right (hand = +1) or, mirrored, on the left (hand = -1). The wall
        # follower itself only knows "right"; for the left hand we mirror its input/output.
        closest, closest_angle = lidar.sector_closest(scan.ranges, self.hand * SIDE_CENTER,
                                                      SIDE_HALF_WIDTH, *sector)
        if closest_angle is not None:
            closest_angle *= self.hand
        # Only for the log: the closest point on the other side (not used for control).
        other_closest, other_angle = lidar.sector_closest(scan.ranges, -self.hand * SIDE_CENTER,
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
            path_to_goal_clear = goal_clearance > distance
            blocked_front = (v > 0 and narrow_front < wall_follow.WALL_IN_FRONT
                             and not path_to_goal_clear)
            if goal_clearance < wall_follow.WALL_IN_FRONT or blocked_front:
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

        cmd_message = Twist()
        cmd_message.linear.x, cmd_message.angular.z = v, w
        self.cmdVelPublisher.publish(cmd_message)

        # 4. Log: every tick to the CSV ...
        self.write_csv_row(x, y, yaw, distance, line_distance, goal_dir, goal_clearance,
                           front, closest, closest_angle, rule, v, w,
                           other_closest, other_angle)

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