from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class CmdVelToMavrosNode(Node):
    """Bridge /cmd_vel commands to MAVROS velocity setpoints for ArduPilot."""

    def __init__(self):
        super().__init__('cmd_vel_to_mavros')

        self.declare_parameter('input_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('debug_cmd_vel_topic', '/teleop/cmd_vel_limited')
        self.declare_parameter(
            'output_cmd_vel_stamped_topic',
            '/mavros/setpoint_velocity/cmd_vel',
        )
        self.declare_parameter(
            'output_cmd_vel_unstamped_topic',
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
        )
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('command_timeout', 0.35)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('enable_reverse', True)
        self.declare_parameter('publish_stamped', True)
        self.declare_parameter('publish_unstamped', True)
        self.declare_parameter('body_frame_id', 'base_link')
        self.declare_parameter('mavros_state_topic', '/mavros/state')

        self.input_cmd_vel_topic = self.get_parameter(
            'input_cmd_vel_topic'
        ).get_parameter_value().string_value
        self.debug_cmd_vel_topic = self.get_parameter(
            'debug_cmd_vel_topic'
        ).get_parameter_value().string_value
        self.output_cmd_vel_stamped_topic = self.get_parameter(
            'output_cmd_vel_stamped_topic'
        ).get_parameter_value().string_value
        self.output_cmd_vel_unstamped_topic = self.get_parameter(
            'output_cmd_vel_unstamped_topic'
        ).get_parameter_value().string_value
        self.publish_rate = self.get_parameter(
            'publish_rate'
        ).get_parameter_value().double_value
        self.command_timeout = self.get_parameter(
            'command_timeout'
        ).get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter(
            'max_linear_speed'
        ).get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter(
            'max_angular_speed'
        ).get_parameter_value().double_value
        self.enable_reverse = self.get_parameter(
            'enable_reverse'
        ).get_parameter_value().bool_value
        self.publish_stamped = self.get_parameter(
            'publish_stamped'
        ).get_parameter_value().bool_value
        self.publish_unstamped = self.get_parameter(
            'publish_unstamped'
        ).get_parameter_value().bool_value
        self.body_frame_id = self.get_parameter(
            'body_frame_id'
        ).get_parameter_value().string_value
        self.mavros_state_topic = self.get_parameter(
            'mavros_state_topic'
        ).get_parameter_value().string_value

        qos = QoSProfile(depth=10)

        self.debug_cmd_vel_pub = self.create_publisher(
            Twist,
            self.debug_cmd_vel_topic,
            qos,
        )
        self.cmd_vel_stamped_pub = self.create_publisher(
            TwistStamped,
            self.output_cmd_vel_stamped_topic,
            qos,
        )
        self.cmd_vel_unstamped_pub = self.create_publisher(
            Twist,
            self.output_cmd_vel_unstamped_topic,
            qos,
        )

        self.create_subscription(
            Twist,
            self.input_cmd_vel_topic,
            self.cmd_vel_callback,
            qos,
        )
        self.create_subscription(
            State,
            self.mavros_state_topic,
            self.mavros_state_callback,
            qos,
        )

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.has_received_command = False
        self.timeout_state = True
        self.last_mavros_connected = None

        self.create_timer(1.0 / max(self.publish_rate, 1.0), self.publish_timer_callback)

        self.get_logger().info(
            'cmd_vel_to_mavros ready: '
            f'input={self.input_cmd_vel_topic}, '
            f'output={self.output_cmd_vel_stamped_topic}, '
            'ArduPilot should be in GUIDED mode before teleoperation.'
        )

    def clamp(self, value, limit_min, limit_max):
        return max(limit_min, min(value, limit_max))

    def clamp_cmd(self, msg: Twist):
        cmd = Twist()

        linear_min = -self.max_linear_speed if self.enable_reverse else 0.0
        cmd.linear.x = self.clamp(msg.linear.x, linear_min, self.max_linear_speed)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = self.clamp(
            msg.angular.z,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        return cmd

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd = self.clamp_cmd(msg)
        self.last_cmd_time = self.get_clock().now()
        self.has_received_command = True

        if self.timeout_state:
            self.get_logger().info('Teleop command stream detected, forwarding to MAVROS.')
        self.timeout_state = False

    def mavros_state_callback(self, msg: State):
        if self.last_mavros_connected is None or self.last_mavros_connected != msg.connected:
            if msg.connected:
                self.get_logger().info('MAVROS connected to FCU.')
            else:
                self.get_logger().warn(
                    'MAVROS not connected to FCU: velocity setpoints will be ignored.'
                )
        self.last_mavros_connected = msg.connected

    def publish_timer_callback(self):
        if not self.has_received_command:
            cmd = Twist()
        else:
            age_seconds = (
                self.get_clock().now() - self.last_cmd_time
            ).nanoseconds / 1e9
            if age_seconds > self.command_timeout:
                cmd = Twist()
                if not self.timeout_state:
                    self.get_logger().warn(
                        'Teleop command timeout reached, sending zero velocity.'
                    )
                self.timeout_state = True
            else:
                cmd = self.last_cmd
                self.timeout_state = False

        self.debug_cmd_vel_pub.publish(cmd)

        if self.publish_unstamped:
            self.cmd_vel_unstamped_pub.publish(cmd)

        if self.publish_stamped:
            stamped = TwistStamped()
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.header.frame_id = self.body_frame_id
            stamped.twist = cmd
            self.cmd_vel_stamped_pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMavrosNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
