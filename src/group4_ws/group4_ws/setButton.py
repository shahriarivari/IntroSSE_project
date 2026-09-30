import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.msg import InterfaceButtons, AudioNote, HazardDetectionVector, HazardDetection
from irobot_create_msgs.action import AudioNoteSequence, Undock, Dock
from irobot_create_msgs.srv import ResetPose
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty
import os
import signal
import subprocess

NS = '/robot_4'

# (frequency in Hz, duration in nanoseconds)
MELODY = [
    (659, 150000000),
    (587, 150000000),
    (523, 150000000),
    (466, 150000000),
    (440, 150000000),
    (392, 150000000),
    (349, 150000000),
    (330, 400000000),
]


class SetButton(Node):
    def __init__(self):
        super().__init__('set_button')

        # --- subscriptions ---
        self.create_subscription(
            InterfaceButtons, f'{NS}/interface_buttons', self.button_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            HazardDetectionVector, f'{NS}/hazard_detection', self.hazard_callback,
            qos_profile_sensor_data)

        # --- publisher: tells readIR to go home ---
        self.home_pub = self.create_publisher(Empty, f'{NS}/go_home', 10)

        # --- action clients & service client ---
        self.audio_client  = ActionClient(self, AudioNoteSequence, f'{NS}/audio_note_sequence')
        self.undock_client = ActionClient(self, Undock, f'{NS}/undock')
        self.dock_client   = ActionClient(self, Dock, f'{NS}/dock')
        self.reset_client  = self.create_client(ResetPose, f'{NS}/reset_pose')

        self.get_logger().info('Waiting for robot servers...')
        self.audio_client.wait_for_server()
        self.undock_client.wait_for_server()
        self.dock_client.wait_for_server()
        self.reset_client.wait_for_service()
        self.get_logger().info(
            'Ready: button 1 = song | button 2 = start -> go home -> (abort & dock)')

        # --- state ---
        self.was_pressed = False
        self.was_pressed_2 = False
        self.was_bumped = False
        self.playing = False     # song is playing
        self.busy = False        # docking/undocking/resetting in progress
        self.mode = 'idle'       # idle -> exploring -> homing -> idle
        self.process = None      # the running ros2 run, if any

        # checks whether readIR has finished by itself (= home reached)
        self.create_timer(0.5, self.check_program)

    # ================= inputs =================
    def button_callback(self, msg):
        pressed = msg.button_1.is_pressed
        if pressed and not self.was_pressed:
            self.play_song()
        self.was_pressed = pressed

        pressed_2 = msg.button_2.is_pressed
        if pressed_2 and not self.was_pressed_2:
            self.button_2_pressed()
        self.was_pressed_2 = pressed_2

    def hazard_callback(self, msg):
        bumped = any(d.type == HazardDetection.BUMP for d in msg.detections)
        if bumped and not self.was_bumped:
            self.get_logger().info('Bump!')
            self.play_song()
        self.was_bumped = bumped

    # ================= button 2 sequence =================
    def button_2_pressed(self):
        if self.busy:
            self.get_logger().info('Busy, ignoring button')
            return

        if self.mode == 'idle':
            # undock -> reset pose -> start readIR
            self.busy = True
            self.undock()

        elif self.mode == 'exploring':
            # tell readIR to drive back to (0, 0)
            self.home_pub.publish(Empty())
            self.mode = 'homing'
            self.get_logger().info('Going home')

        elif self.mode == 'homing':
            # abort: stop readIR and dock right away
            self.get_logger().info('Aborting, docking')
            self.stop_program()
            self.mode = 'idle'
            self.dock()

    def check_program(self):
        """If readIR exits by itself while going home, it reached (0, 0) -> dock."""
        if self.mode == 'idle' or self.process is None:
            return
        if self.process.poll() is None:
            return                               # still running

        self.process = None
        if self.mode == 'homing':
            self.get_logger().info('Home reached, docking')
            self.mode = 'idle'
            self.dock()
        else:
            self.get_logger().warn('readIR stopped unexpectedly')
            self.mode = 'idle'

    # ================= process =================
    def start_program(self):
        self.process = subprocess.Popen(
            ['ros2', 'run', 'group4_ws', 'readIR'],
            start_new_session=True)
        self.get_logger().info('readIR started')

    def stop_program(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.get_logger().info('readIR stopped')
        self.process = None

    # ================= robot actions =================
    def undock(self):
        self.get_logger().info('Undocking...')
        self.send_action(self.undock_client, Undock.Goal(), 'undock', self.undock_done)

    def undock_done(self, result):
        # reset AFTER undocking, so (0, 0) is the start of the maze, not the dock
        self.get_logger().info('Undock finished, resetting pose')
        self.reset_client.call_async(ResetPose.Request()).add_done_callback(self.reset_done)

    def reset_done(self, future):
        self.get_logger().info('Pose reset to (0, 0)')
        self.start_program()
        self.mode = 'exploring'
        self.busy = False

    def dock(self):
        self.busy = True
        self.get_logger().info('Docking...')
        self.send_action(self.dock_client, Dock.Goal(), 'dock', self.dock_done)

    def dock_done(self, result):
        docked = result.is_docked if result is not None else False
        self.get_logger().info(f'Dock finished, docked: {docked}')
        self.busy = False

    def play_song(self):
        if self.playing:
            return
        self.playing = True

        goal = AudioNoteSequence.Goal()
        goal.iterations = 1
        for freq, ns in MELODY:
            note = AudioNote()
            note.frequency = freq
            note.max_runtime = Duration(sec=0, nanosec=ns)
            goal.note_sequence.notes.append(note)

        self.send_action(self.audio_client, goal, 'song', self.song_done)

    def song_done(self, result):
        self.playing = False

    # ================= helper =================
    def send_action(self, client, goal, name, done_cb):
        """Send an action goal and call done_cb(result) when it finishes."""
        def on_goal_response(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn(f'{name} goal rejected')
                done_cb(None)
                return
            handle.get_result_async().add_done_callback(
                lambda f: done_cb(f.result().result))

        client.send_goal_async(goal).add_done_callback(on_goal_response)


def main():
    rclpy.init()
    node = SetButton()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_program()      # also stop readIR when this node is closed
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()