"""Pattern driver node: drives the robot along a pattern using FK odometry feedback.

Input : /fk/odom  (nav_msgs/Odometry)  pose from the fk_odometry node
        /fk/distance (std_msgs/Float64) total path length, for the summary
Output: /cmd_vel  (geometry_msgs/Twist)

Run (Gazebo + fk_odometry must already be running):
    ros2 run diffbot_fk pattern_driver
    ros2 run diffbot_fk pattern_driver --ros-args -p side_length:=1.5 -p loops:=2
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from diffbot_fk.patterns import SegmentPlanner, square_segments


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PatternDriver(Node):

    def __init__(self):
        super().__init__('pattern_driver')
        self.declare_parameter('pattern', 'square')
        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('loops', 1)
        self.declare_parameter('clockwise', False)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('distance_tolerance', 0.005)
        self.declare_parameter('angle_tolerance', 0.01)
        self.declare_parameter('heading_gain', 2.0)
        self.declare_parameter('start_delay', 1.0)
        self.declare_parameter('control_rate', 20.0)

        gp = self.get_parameter
        pattern = gp('pattern').value
        if pattern != 'square':
            raise ValueError(f"pattern '{pattern}' is not implemented yet (available: square)")

        self.side = gp('side_length').value
        loops = gp('loops').value
        segments = square_segments(self.side, loops, gp('clockwise').value)
        self.planner = SegmentPlanner(
            segments,
            linear_speed=gp('linear_speed').value,
            angular_speed=gp('angular_speed').value,
            distance_tolerance=gp('distance_tolerance').value,
            angle_tolerance=gp('angle_tolerance').value,
            heading_gain=gp('heading_gain').value,
        )
        self.expected_distance = 4 * self.side * loops

        self.pose = None
        self.start_pose = None
        self.start_distance = None
        self.distance = 0.0
        self.start_time = None
        self.finished = False
        self.logged_segments = 0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'fk/odom', self.on_odom, 20)
        self.create_subscription(Float64, 'fk/distance', self.on_distance, 10)
        self.create_timer(1.0 / gp('control_rate').value, self.control_step)
        self.start_delay = gp('start_delay').value
        self.first_odom_time = None

        self.get_logger().info(
            f'Square pattern: side={self.side} m, loops={loops}, '
            f"direction={'CW' if gp('clockwise').value else 'CCW'}, "
            f'{len(segments)} segments. Waiting for /fk/odom ...')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        if self.first_odom_time is None:
            self.first_odom_time = self.get_clock().now()

    def on_distance(self, msg):
        self.distance = msg.data

    def publish(self, v, omega):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self.cmd_pub.publish(msg)

    def control_step(self):
        if self.finished or self.pose is None:
            return
        elapsed = (self.get_clock().now() - self.first_odom_time).nanoseconds * 1e-9
        if elapsed < self.start_delay:
            return
        if self.start_pose is None:
            self.start_pose = self.pose
            self.start_distance = self.distance
            self.start_time = self.get_clock().now()
            x, y, th = self.pose
            self.get_logger().info(
                f'Start pose: x={x:+.3f} m  y={y:+.3f} m  theta={math.degrees(th):+.1f} deg')

        v, omega = self.planner.update(*self.pose)
        self.publish(v, omega)
        self.log_finished_segments()

        if self.planner.done:
            self.finish()

    def log_finished_segments(self):
        while self.logged_segments < len(self.planner.log):
            i, kind, start, end = self.planner.log[self.logged_segments]
            if kind == 'line':
                result = f'distance={math.hypot(end[0] - start[0], end[1] - start[1]):.3f} m'
            else:
                turned = math.degrees(math.atan2(math.sin(end[2] - start[2]),
                                                 math.cos(end[2] - start[2])))
                result = f'turned={turned:+.1f} deg'
            self.get_logger().info(
                f'Segment {i + 1}/{len(self.planner.segments)} ({kind}) done: {result} | '
                f'pose x={end[0]:+.3f} y={end[1]:+.3f} theta={math.degrees(end[2]):+.1f} deg')
            self.logged_segments += 1

    def finish(self):
        self.finished = True
        self.publish(0.0, 0.0)
        x0, y0, th0 = self.start_pose
        x, y, th = self.pose
        duration = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        travelled = self.distance - self.start_distance
        heading_err = math.degrees(math.atan2(math.sin(th - th0), math.cos(th - th0)))
        self.get_logger().info(
            f'Pattern finished in {duration:.1f} s | FK path length={travelled:.3f} m '
            f'(expected {self.expected_distance:.3f} m) | '
            f'closing error={math.hypot(x - x0, y - y0) * 100:.2f} cm, '
            f'heading error={heading_err:+.2f} deg')


def main(args=None):
    # No rclpy signal handler, so CTRL-C still lets us send the stop command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PatternDriver()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, stopping the robot')
    finally:
        # Repeat the stop command a few times so it is not lost.
        for _ in range(5):
            node.publish(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
