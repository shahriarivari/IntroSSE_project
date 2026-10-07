import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import InterfaceButtons, AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition

NS = '/robot_4'
SLAM_NODE = f'{NS}/slam_toolbox'     # full name of the SLAM lifecycle node


class SlamButton(Node):
    def __init__(self):
        super().__init__('slam_button')

        # button input (same QoS as setButton, which we know works)
        self.create_subscription(
            InterfaceButtons, f'{NS}/interface_buttons', self.on_buttons,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        # sound feedback, so you know what happened without a laptop
        self.audio_pub = self.create_publisher(AudioNoteVector, f'{NS}/cmd_audio', 10)

        # the two services that `ros2 lifecycle get/set` use
        self.get_state_cli = self.create_client(GetState, f'{SLAM_NODE}/get_state')
        self.change_state_cli = self.create_client(ChangeState, f'{SLAM_NODE}/change_state')

        self.was_pressed = False
        self.busy = False
        self.get_logger().info('Ready: press button 1 to start SLAM')

    # ---------- input ----------
    def on_buttons(self, msg):
        pressed = msg.button_1.is_pressed
        if pressed and not self.was_pressed:        # rising edge only
            self.start_slam()
        self.was_pressed = pressed

    # ---------- lifecycle state machine ----------
    def start_slam(self):
        if self.busy:
            return
        if not self.get_state_cli.service_is_ready():
            self.get_logger().warn(f'{SLAM_NODE} not found - is SLAM running?')
            self.beep([(200, 0.5)])                  # low tone = error
            return
        self.busy = True
        self.ask_state()

    def ask_state(self):
        self.get_state_cli.call_async(GetState.Request()).add_done_callback(self.on_state)

    def on_state(self, future):
        result = future.result()
        if result is None:
            return self.fail('get_state call failed')

        state = result.current_state.id
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info('Configuring SLAM...')
            self.change(Transition.TRANSITION_CONFIGURE)
        elif state == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info('Activating SLAM...')
            self.change(Transition.TRANSITION_ACTIVATE)
        elif state == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info('SLAM is active - mapping')
            self.beep([(880, 0.15), (1320, 0.2)])     # rising beep = success
            self.busy = False
        else:
            self.fail(f'unexpected state: {result.current_state.label}')

    def change(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        self.change_state_cli.call_async(req).add_done_callback(self.on_changed)

    def on_changed(self, future):
        result = future.result()
        if result is None or not result.success:
            return self.fail('transition failed')
        self.ask_state()                              # loop until ACTIVE

    def fail(self, reason):
        self.get_logger().error(reason)
        self.beep([(200, 0.5)])
        self.busy = False

    # ---------- feedback ----------
    def beep(self, notes):
        msg = AudioNoteVector()
        msg.append = False
        for freq, seconds in notes:
            msg.notes.append(AudioNote(
                frequency=freq,
                max_runtime=Duration(sec=0, nanosec=int(seconds * 1e9))))
        self.audio_pub.publish(msg)


def main():
    rclpy.init()
    node = SlamButton()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()