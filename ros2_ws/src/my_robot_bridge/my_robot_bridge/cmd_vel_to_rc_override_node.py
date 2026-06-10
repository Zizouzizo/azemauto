from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class CmdVelToRcOverrideNode(Node):
    """Bridge /cmd_vel commands to MAVROS RC override steering/throttle PWM."""

    RC_CHANNEL_COUNT = 18
    TIMEOUT_BEHAVIOR_NEUTRAL = 'neutral'
    TIMEOUT_BEHAVIOR_RELEASE = 'release'

    def __init__(self):
        super().__init__('cmd_vel_to_rc_override')

        self.declare_parameter('input_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_rc_override_topic', '/mavros/rc/override')
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('steering_channel', 1)
        self.declare_parameter('throttle_channel', 3)
        self.declare_parameter('neutral_pwm', 1500)
        self.declare_parameter('min_pwm', 1100)
        self.declare_parameter('max_pwm', 1900)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('throttle_gain', 1.0)
        self.declare_parameter('timeout_sec', 0.35)
        self.declare_parameter('timeout_behavior', self.TIMEOUT_BEHAVIOR_RELEASE)

        self.input_cmd_vel_topic = self.get_parameter(
            'input_cmd_vel_topic'
        ).get_parameter_value().string_value
        self.output_rc_override_topic = self.get_parameter(
            'output_rc_override_topic'
        ).get_parameter_value().string_value
        self.publish_rate = self.get_parameter(
            'publish_rate'
        ).get_parameter_value().double_value
        self.steering_channel = self.get_parameter(
            'steering_channel'
        ).get_parameter_value().integer_value
        self.throttle_channel = self.get_parameter(
            'throttle_channel'
        ).get_parameter_value().integer_value
        self.neutral_pwm = self.get_parameter(
            'neutral_pwm'
        ).get_parameter_value().integer_value
        self.min_pwm = self.get_parameter(
            'min_pwm'
        ).get_parameter_value().integer_value
        self.max_pwm = self.get_parameter(
            'max_pwm'
        ).get_parameter_value().integer_value
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
        self.timeout_behavior = self.get_parameter(
            'timeout_behavior'
        ).get_parameter_value().string_value

        self.validate_parameters()

        self.steering_index = self.channel_to_index(self.steering_channel)
        self.throttle_index = self.channel_to_index(self.throttle_channel)

        qos = QoSProfile(depth=10)
        self.rc_override_pub = self.create_publisher(
            OverrideRCIn,
            self.output_rc_override_topic,
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
            1.0 / self.publish_rate,
            self.publish_timer_callback,
        )

        self.get_logger().info(
            'cmd_vel_to_rc_override ready: '
            f'input={self.input_cmd_vel_topic}, '
            f'output={self.output_rc_override_topic}, '
            f'steering_ch={self.steering_channel}, '
            f'throttle_ch={self.throttle_channel}, '
            f'pwm=[{self.min_pwm},{self.neutral_pwm},{self.max_pwm}], '
            f'timeout_behavior={self.timeout_behavior}'
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
        if self.min_pwm >= self.neutral_pwm or self.neutral_pwm >= self.max_pwm:
            raise ValueError('Expected min_pwm < neutral_pwm < max_pwm')
        if not 1 <= self.steering_channel <= self.RC_CHANNEL_COUNT:
            raise ValueError(
                f'steering_channel must be within [1, {self.RC_CHANNEL_COUNT}]'
            )
        if not 1 <= self.throttle_channel <= self.RC_CHANNEL_COUNT:
            raise ValueError(
                f'throttle_channel must be within [1, {self.RC_CHANNEL_COUNT}]'
            )
        if self.steering_channel == self.throttle_channel:
            raise ValueError('steering_channel and throttle_channel must differ')
        if self.timeout_behavior not in (
            self.TIMEOUT_BEHAVIOR_NEUTRAL,
            self.TIMEOUT_BEHAVIOR_RELEASE,
        ):
            raise ValueError(
                'timeout_behavior must be "neutral" or "release"'
            )

    def channel_to_index(self, channel_number: int) -> int:
        return channel_number - 1

    def clamp(self, value, limit_min, limit_max):
        return max(limit_min, min(value, limit_max))

    def normalize(self, value, limit, gain):
        normalized = self.clamp(value / limit, -1.0, 1.0)
        return self.clamp(normalized * gain, -1.0, 1.0)

    def normalized_to_pwm(self, normalized):
        if normalized >= 0.0:
            pwm = self.neutral_pwm + normalized * (self.max_pwm - self.neutral_pwm)
        else:
            pwm = self.neutral_pwm + normalized * (self.neutral_pwm - self.min_pwm)

        return int(round(self.clamp(pwm, self.min_pwm, self.max_pwm)))

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
                'Teleop command stream detected, forwarding as RC override.'
            )
        self.timeout_state = False

    def build_override_message(self, steering_pwm, throttle_pwm):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * self.RC_CHANNEL_COUNT
        msg.channels[self.steering_index] = steering_pwm
        msg.channels[self.throttle_index] = throttle_pwm
        return msg

    def neutral_message(self):
        return self.build_override_message(
            self.neutral_pwm,
            self.neutral_pwm,
        )

    def release_message(self):
        return self.build_override_message(
            OverrideRCIn.CHAN_RELEASE,
            OverrideRCIn.CHAN_RELEASE,
        )

    def timeout_message(self):
        if self.timeout_behavior == self.TIMEOUT_BEHAVIOR_NEUTRAL:
            return self.neutral_message()

        return self.release_message()

    def command_message(self):
        throttle_normalized = self.normalize(
            self.last_cmd.linear.x,
            self.max_linear_speed,
            self.throttle_gain,
        )
        steering_normalized = self.normalize(
            self.last_cmd.angular.z,
            self.max_angular_speed,
            self.steering_gain,
        )

        throttle_pwm = self.normalized_to_pwm(throttle_normalized)
        steering_pwm = self.normalized_to_pwm(steering_normalized)

        return self.build_override_message(
            steering_pwm,
            throttle_pwm,
        )

    def publish_timer_callback(self):
        if not self.has_received_command:
            msg = self.timeout_message()
        else:
            age_seconds = (
                self.get_clock().now() - self.last_cmd_time
            ).nanoseconds / 1e9
            if age_seconds > self.timeout_sec:
                msg = self.timeout_message()
                if not self.timeout_state:
                    self.get_logger().warn(
                        'Teleop command timeout reached, publishing '
                        f'{self.timeout_behavior} RC override.'
                    )
                self.timeout_state = True
            else:
                msg = self.command_message()
                self.timeout_state = False

        self.rc_override_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRcOverrideNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
