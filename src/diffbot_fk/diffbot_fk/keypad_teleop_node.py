"""Keypad teleop: drive the robot from the keyboard / numpad.

Run it in its own terminal (it needs direct keyboard input):
    ros2 run diffbot_fk keypad_teleop
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from diffbot_fk.kinematics import body_to_wheel_velocity

HELP = """
---------------------------------------------------
 Keypad teleop (differential drive)
---------------------------------------------------
   Letters       Numpad          Direction
   q  w  e       7  8  9       fwd-left   forward  fwd-right
   a  s  d       4  5  6       turn-left  STOP     turn-right
   z  x  c       1  2  3       back-left  backward back-right
   space / s / 5 : stop
   + / -         : change linear speed   (+-10%)
   ] / [         : change angular speed  (+-10%)
   CTRL-C        : quit
---------------------------------------------------
"""

# key -> (linear direction, angular direction)
MOVES = {
    'w': (1, 0), '8': (1, 0),
    'x': (-1, 0), '2': (-1, 0),
    'a': (0, 1), '4': (0, 1),
    'd': (0, -1), '6': (0, -1),
    'q': (1, 1), '7': (1, 1),
    'e': (1, -1), '9': (1, -1),
    'z': (-1, -1), '1': (-1, -1),
    'c': (-1, 1), '3': (-1, 1),
    's': (0, 0), '5': (0, 0), ' ': (0, 0),
}


class KeypadTeleop(Node):

    def __init__(self):
        super().__init__('keypad_teleop')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('max_linear_vel', 0.22)
        self.declare_parameter('max_angular_vel', 2.84)
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('publish_rate', 10.0)

        gp = self.get_parameter
        self.lin = gp('linear_speed').value
        self.ang = gp('angular_speed').value
        self.max_lin = gp('max_linear_vel').value
        self.max_ang = gp('max_angular_vel').value
        self.r = gp('wheel_radius').value
        self.L = gp('wheel_separation').value
        self.dir_lin = 0
        self.dir_ang = 0

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(1.0 / gp('publish_rate').value, self.publish)

    def handle_key(self, key):
        if key in MOVES:
            self.dir_lin, self.dir_ang = MOVES[key]
        elif key == '+':
            self.lin = min(self.lin * 1.1, self.max_lin)
        elif key == '-':
            self.lin *= 0.9
        elif key == ']':
            self.ang = min(self.ang * 1.1, self.max_ang)
        elif key == '[':
            self.ang *= 0.9
        else:
            return
        self.print_state()

    def command(self):
        return self.dir_lin * self.lin, self.dir_ang * self.ang

    def publish(self):
        v, w = self.command()
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def stop(self):
        self.dir_lin = self.dir_ang = 0
        self.publish()

    def print_state(self):
        v, w = self.command()
        w_l, w_r = body_to_wheel_velocity(v, w, self.r, self.L)
        sys.stdout.write(
            f'\rv={v:+.2f} m/s  w={w:+.2f} rad/s  | wheels L={w_l:+6.2f} R={w_r:+6.2f} rad/s  '
            f'| scale v={self.lin:.2f} w={self.ang:.2f}   ')
        sys.stdout.flush()


def read_key(timeout=0.1):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else ''


def main(args=None):
    if not sys.stdin.isatty():
        print('keypad_teleop needs an interactive terminal. Run: ros2 run diffbot_fk keypad_teleop')
        return
    settings = termios.tcgetattr(sys.stdin)
    # No rclpy signal handler, so CTRL-C does not shut down the context before
    # the stop command (v = 0, w = 0) has been sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeypadTeleop()
    print(HELP)
    node.print_state()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key()
            if key == '\x03':
                break
            if key:
                node.handle_key(key.lower())
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
