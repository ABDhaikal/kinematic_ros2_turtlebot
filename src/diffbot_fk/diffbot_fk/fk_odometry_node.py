"""Forward kinematics odometry node.

to run odometry node:
    source /opt/ros/humble/setup.bash
    source ~/diffbot_ws/install/setup.bash
    ros2 launch diffbot_fk gazebo.launch.py


Input : /joint_states (wheel angular positions from Gazebo / encoders)
Output: /fk/odom      (nav_msgs/Odometry)  pose computed by FK
        /fk/distance  (std_msgs/Float64)   total path length [m]
        /fk/path      (nav_msgs/Path)      trajectory trail for RViz
        TF odom -> base_footprint
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

from diffbot_fk.kinematics import DiffDriveOdometry


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FkOdometryNode(Node):

    def __init__(self):
        super().__init__('fk_odometry')
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('left_wheel_joint', 'wheel_left_joint')
        self.declare_parameter('right_wheel_joint', 'wheel_right_joint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('integration_method', 'exact')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('path_min_step', 0.01)
        self.declare_parameter('log_period', 2.0)

        gp = self.get_parameter
        self.left_joint = gp('left_wheel_joint').value
        self.right_joint = gp('right_wheel_joint').value
        self.odom_frame = gp('odom_frame').value
        self.base_frame = gp('base_frame').value
        self.publish_tf = gp('publish_tf').value
        self.path_min_step = gp('path_min_step').value

        self.odom = DiffDriveOdometry(
            wheel_radius=gp('wheel_radius').value,
            wheel_separation=gp('wheel_separation').value,
            method=gp('integration_method').value,
        )
        self.last_stamp = None

        self.odom_pub = self.create_publisher(Odometry, 'fk/odom', 10)
        self.dist_pub = self.create_publisher(Float64, 'fk/distance', 10)
        self.path_pub = self.create_publisher(Path, 'fk/path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.path = Path()
        self.path.header.frame_id = self.odom_frame

        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 50)
        self.create_service(Empty, 'fk/reset', self.on_reset)
        self.create_timer(gp('log_period').value, self.log_status)

        self.get_logger().info(
            f'FK odometry: r={self.odom.wheel_radius} m, L={self.odom.wheel_separation} m, '
            f'method={self.odom.method}, joints=({self.left_joint}, {self.right_joint})')

    def on_reset(self, request, response):
        self.odom.reset()
        self.path.poses.clear()
        self.get_logger().info('FK odometry reset to (0, 0, 0)')
        return response

    def on_joint_states(self, msg):
        try:
            i_l = msg.name.index(self.left_joint)
            i_r = msg.name.index(self.right_joint)
        except ValueError:
            return  # some other joint_states message (e.g. without the wheels)
        if len(msg.position) <= max(i_l, i_r):
            return

        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        if self.last_stamp is not None and stamp < self.last_stamp:
            # Simulation time went backwards (Gazebo was reset): restart odometry.
            self.odom.reset()
            self.path.poses.clear()
        dt = 0.0 if self.last_stamp is None else (stamp - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = stamp

        self.odom.update(msg.position[i_l], msg.position[i_r], dt)
        self.publish(stamp.to_msg())

    def publish(self, stamp):
        o = self.odom
        q = yaw_to_quaternion(o.theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = o.x
        odom_msg.pose.pose.position.y = o.y
        odom_msg.pose.pose.orientation = q
        odom_msg.twist.twist.linear.x = o.v
        odom_msg.twist.twist.angular.z = o.omega
        for i, c in zip((0, 7, 35), (1e-3, 1e-3, 1e-2)):
            odom_msg.pose.covariance[i] = c
            odom_msg.twist.covariance[i] = c
        self.odom_pub.publish(odom_msg)

        self.dist_pub.publish(Float64(data=o.distance))

        if self.publish_tf:
            t = TransformStamped()
            t.header = odom_msg.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = o.x
            t.transform.translation.y = o.y
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

        last = self.path.poses[-1].pose.position if self.path.poses else None
        if last is None or math.hypot(o.x - last.x, o.y - last.y) >= self.path_min_step:
            ps = PoseStamped()
            ps.header = odom_msg.header
            ps.pose = odom_msg.pose.pose
            self.path.poses.append(ps)
            self.path.header.stamp = stamp
            self.path_pub.publish(self.path)

    def log_status(self):
        o = self.odom
        self.get_logger().info(
            f'x={o.x:+.3f} m  y={o.y:+.3f} m  theta={math.degrees(o.theta):+7.2f} deg | '
            f'v={o.v:+.3f} m/s  w={o.omega:+.3f} rad/s | '
            f'distance={o.distance:.3f} m  displacement={o.displacement:.3f} m  '
            f'rotation={math.degrees(o.rotation):.1f} deg')


def main(args=None):
    rclpy.init(args=args)
    node = FkOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
