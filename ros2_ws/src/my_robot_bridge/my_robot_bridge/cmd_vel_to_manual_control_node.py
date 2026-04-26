from geometry_msgs.msg import Twist
from mavros_msgs.msg import ManualControl
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class CmdVelToManualControlNode(Node):
    """Bridge /cmd_vel commands to MAVROS MANUAL_CONTROL axes."""

    INVALID_AXIS_VALUE = 32767.0
    SIGNED_AXES = {'x', 'y', 'r'}
    UNSIGNED_AXES = {'z'}
    VALID_AXES = SIGNED_AXES | UNSIGNED_AXES

    def __init__(self):
        super().__init__('cmd_vel_to_manual_control')

        self.declare_parameter('input_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_manual_control_topic', '/mavros/mavros/send')
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('throttle_axis', 'x')
        self.declare_parameter('steering_axis', 'y')
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('throttle_gain', 1.0)
        self.declare_parameter('timeout_sec', 0.35)

        self.input_cmd_vel_topic = self.get_parameter(
            'input_cmd_vel_topic'
        ).get_parameter_value().string_value
        self.output_manual_control_topic = self.get_parameter(
            'output_manual_control_topic'
        ).get_parameter_value().string_value
        self.publish_rate = self.get_parameter(
            'publish_rate'
        ).get_parameter_value().double_value
        self.throttle_axis = self.get_parameter(
            'throttle_axis'
        ).get_parameter_value().string_value
        self.steering_axis = self.get_parameter(
            'steering_axis'
        ).get_parameter_value().string_value
        self.max_linear_speed = self.get_parameter(
            'max_linear_speed'
        ).get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter(
            'max_angular_speed'
        ).get_parameter_value().double_value
        self.steering_gain = self.get_parameter(
            'steering_gain'
        ).get_parameter_value().double_value
        self.throttle_gain = self.get_parameter(
            'throttle_gain'
        ).get_parameter_value().double_value
        self.timeout_sec = self.get_parameter(
            'timeout_sec'
        ).get_parameter_value().double_value

        self.validate_parameters()

        qos = QoSProfile(depth=10)
        self.manual_control_pub = self.create_publisher(
            ManualControl,
            self.output_manual_control_topic,
            qos,
        )
        self.create_subscription(
            Twist,
            self.input_cmd_vel_topic,
            self.cmd_vel_callback,
            qos,
        )

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.has_received_command = False
        self.timeout_state = True

        self.create_timer(
            1.0 / max(self.publish_rate, 1.0),
            self.publish_timer_callback,
        )

        if self.throttle_axis in self.UNSIGNED_AXES:
            self.get_logger().warn(
                'throttle_axis=z is unsigned in MANUAL_CONTROL; reverse /cmd_vel '
                'will be clamped to zero.'
            )

        self.get_logger().info(
            'cmd_vel_to_manual_control ready: '
            f'input={self.input_cmd_vel_topic}, '
            f'output={self.output_manual_control_topic}, '
            f'throttle_axis={self.throttle_axis}, '
            f'steering_axis={self.steering_axis}'
        )

    def validate_parameters(self):
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0.0')
        if self.timeout_sec <= 0.0:
            raise ValueError('timeout_sec must be > 0.0')
        if self.max_linear_speed <= 0.0:
            raise ValueError('max_linear_speed must be > 0.0')
        if self.max_angular_speed <= 0.0:
            raise ValueError('max_angular_speed must be > 0.0')
        if self.throttle_axis not in self.VALID_AXES:
            raise ValueError(
                f'throttle_axis must be one of {sorted(self.VALID_AXES)}'
            )
        if self.steering_axis not in self.SIGNED_AXES:
            raise ValueError(
                f'steering_axis must be one of {sorted(self.SIGNED_AXES)}'
            )
        if self.throttle_axis == self.steering_axis:
            raise ValueError('throttle_axis and steering_axis must differ')

    def clamp(self, value, limit_min, limit_max):
        return max(limit_min, min(value, limit_max))

    def normalize_signed(self, value, limit, gain):
        normalized = self.clamp(value / limit, -1.0, 1.0)
        return self.clamp(normalized * gain, -1.0, 1.0)

    def normalize_unsigned(self, value, limit, gain):
        normalized = self.clamp(value / limit, 0.0, 1.0)
        return self.clamp(normalized * gain, 0.0, 1.0)

    def axis_value_from_cmd(self, axis_name, cmd: Twist):
        if axis_name == self.throttle_axis:
            if axis_name in self.UNSIGNED_AXES:
                normalized = self.normalize_unsigned(
                    cmd.linear.x,
                    self.max_linear_speed,
                    self.throttle_gain,
                )
                return float(round(normalized * 1000.0))

            normalized = self.normalize_signed(
                cmd.linear.x,
                self.max_linear_speed,
                self.throttle_gain,
            )
            return float(round(normalized * 1000.0))

        if axis_name == self.steering_axis:
            normalized = self.normalize_signed(
                cmd.angular.z,
                self.max_angular_speed,
                self.steering_gain,
            )
            return float(round(normalized * 1000.0))

        return self.INVALID_AXIS_VALUE

    def build_manual_control_message(self, cmd: Twist):
        msg = ManualControl()
        msg.x = self.axis_value_from_cmd('x', cmd)
        msg.y = self.axis_value_from_cmd('y', cmd)
        msg.z = self.axis_value_from_cmd('z', cmd)
        msg.r = self.axis_value_from_cmd('r', cmd)
        msg.buttons = 0
        msg.buttons2 = 0
        msg.enabled_extensions = 0
        msg.s = 0.0
        msg.t = 0.0
        msg.aux1 = 0.0
        msg.aux2 = 0.0
        msg.aux3 = 0.0
        msg.aux4 = 0.0
        msg.aux5 = 0.0
        msg.aux6 = 0.0
        return msg

    def neutral_message(self):
        return self.build_manual_control_message(Twist())

    def cmd_vel_callback(self, msg: Twist):
        cmd = Twist()
        cmd.linear.x = self.clamp(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        cmd.angular.z = self.clamp(
            msg.angular.z,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        self.last_cmd = cmd
        self.last_cmd_time = self.get_clock().now()
        self.has_received_command = True

        if self.timeout_state:
            self.get_logger().info(
                'Teleop command stream detected, forwarding as MANUAL_CONTROL.'
            )
        self.timeout_state = False

    def publish_timer_callback(self):
        if not self.has_received_command:
            msg = self.neutral_message()
        else:
            age_seconds = (
                self.get_clock().now() - self.last_cmd_time
            ).nanoseconds / 1e9
            if age_seconds > self.timeout_sec:
                msg = self.neutral_message()
                if not self.timeout_state:
                    self.get_logger().warn(
                        'Teleop command timeout reached, republishing neutral MANUAL_CONTROL.'
                    )
                self.timeout_state = True
            else:
                msg = self.build_manual_control_message(self.last_cmd)
                self.timeout_state = False

        self.manual_control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToManualControlNode()

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
