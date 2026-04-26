import math
from collections import deque

from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class AutoArmDisarmNode(Node):
    """
    Supervise MAVROS arm/disarm transitions with strict sensor gating.

    The node only auto-arms when navigation is truly active and the critical
    topics are fresh. It auto-disarms on inactivity or on safety failures.
    """

    def __init__(self):
        super().__init__('auto_arm_disarm')

        self.declare_parameter('auto_arm_enabled', False)
        self.declare_parameter('auto_disarm_enabled', False)
        self.declare_parameter('guided_mode_name', 'GUIDED')
        self.declare_parameter('disarm_timeout', 1.0)
        self.declare_parameter('required_topics_timeout', 1.0)
        self.declare_parameter('required_state_timeout', 1.0)
        self.declare_parameter('min_scan_rate', 5.0)
        self.declare_parameter('monitor_rate', 5.0)
        self.declare_parameter('guided_settle_time', 0.5)
        self.declare_parameter('arm_request_cooldown', 2.0)
        self.declare_parameter('disarm_request_cooldown', 1.0)
        self.declare_parameter('nonzero_linear_epsilon', 0.03)
        self.declare_parameter('nonzero_angular_epsilon', 0.05)
        self.declare_parameter('require_nav_cmd_for_arm', True)
        self.declare_parameter('require_amcl_for_disarm', True)
        self.declare_parameter('disarm_on_mode_change', True)
        self.declare_parameter('disarm_on_sensor_loss', True)
        self.declare_parameter('collision_stop_confirm_time', 0.5)

        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom/raw')
        self.declare_parameter(
            'collision_polygon_topic',
            '/collision_monitor/polygon_stop',
        )
        self.declare_parameter('set_mode_service', '/mavros/set_mode')
        self.declare_parameter('arming_service', '/mavros/cmd/arming')

        self.auto_arm_enabled = self.get_parameter(
            'auto_arm_enabled'
        ).get_parameter_value().bool_value
        self.auto_disarm_enabled = self.get_parameter(
            'auto_disarm_enabled'
        ).get_parameter_value().bool_value
        self.guided_mode_name = self.get_parameter(
            'guided_mode_name'
        ).get_parameter_value().string_value
        self.disarm_timeout = self.get_parameter(
            'disarm_timeout'
        ).get_parameter_value().double_value
        self.required_topics_timeout = self.get_parameter(
            'required_topics_timeout'
        ).get_parameter_value().double_value
        self.required_state_timeout = self.get_parameter(
            'required_state_timeout'
        ).get_parameter_value().double_value
        self.min_scan_rate = self.get_parameter(
            'min_scan_rate'
        ).get_parameter_value().double_value
        self.monitor_rate = self.get_parameter(
            'monitor_rate'
        ).get_parameter_value().double_value
        self.guided_settle_time = self.get_parameter(
            'guided_settle_time'
        ).get_parameter_value().double_value
        self.arm_request_cooldown = self.get_parameter(
            'arm_request_cooldown'
        ).get_parameter_value().double_value
        self.disarm_request_cooldown = self.get_parameter(
            'disarm_request_cooldown'
        ).get_parameter_value().double_value
        self.nonzero_linear_epsilon = self.get_parameter(
            'nonzero_linear_epsilon'
        ).get_parameter_value().double_value
        self.nonzero_angular_epsilon = self.get_parameter(
            'nonzero_angular_epsilon'
        ).get_parameter_value().double_value
        self.require_nav_cmd_for_arm = self.get_parameter(
            'require_nav_cmd_for_arm'
        ).get_parameter_value().bool_value
        self.require_amcl_for_disarm = self.get_parameter(
            'require_amcl_for_disarm'
        ).get_parameter_value().bool_value
        self.disarm_on_mode_change = self.get_parameter(
            'disarm_on_mode_change'
        ).get_parameter_value().bool_value
        self.disarm_on_sensor_loss = self.get_parameter(
            'disarm_on_sensor_loss'
        ).get_parameter_value().bool_value
        self.collision_stop_confirm_time = self.get_parameter(
            'collision_stop_confirm_time'
        ).get_parameter_value().double_value

        self.state_topic = self.get_parameter(
            'state_topic'
        ).get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).get_parameter_value().string_value
        self.cmd_vel_nav_topic = self.get_parameter(
            'cmd_vel_nav_topic'
        ).get_parameter_value().string_value
        self.amcl_pose_topic = self.get_parameter(
            'amcl_pose_topic'
        ).get_parameter_value().string_value
        self.scan_topic = self.get_parameter(
            'scan_topic'
        ).get_parameter_value().string_value
        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).get_parameter_value().string_value
        self.collision_polygon_topic = self.get_parameter(
            'collision_polygon_topic'
        ).get_parameter_value().string_value
        self.set_mode_service_name = self.get_parameter(
            'set_mode_service'
        ).get_parameter_value().string_value
        self.arming_service_name = self.get_parameter(
            'arming_service'
        ).get_parameter_value().string_value

        default_qos = QoSProfile(depth=10)

        self.create_subscription(
            State,
            self.state_topic,
            self.state_callback,
            default_qos,
        )
        self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            default_qos,
        )
        self.create_subscription(
            Twist,
            self.cmd_vel_nav_topic,
            self.cmd_vel_nav_callback,
            default_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.amcl_pose_callback,
            default_qos,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            default_qos,
        )
        self.create_subscription(
            PolygonStamped,
            self.collision_polygon_topic,
            self.collision_polygon_callback,
            default_qos,
        )

        self.set_mode_client = self.create_client(
            SetMode,
            self.set_mode_service_name,
        )
        self.arming_client = self.create_client(
            CommandBool,
            self.arming_service_name,
        )

        self.state = None
        self.last_state_time = None
        self.last_state_mode = None
        self.last_state_armed = None

        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = None
        self.last_cmd_vel_nonzero_time = None

        self.last_cmd_vel_nav = Twist()
        self.last_cmd_vel_nav_time = None
        self.last_cmd_vel_nav_nonzero_time = None

        self.last_amcl_pose_time = None
        self.last_scan_time = None
        self.last_odom_time = None

        self.scan_timestamps = deque(maxlen=25)
        self.collision_monitor_seen = False
        self.collision_stop_candidate_since = None
        self.last_arm_transition_time = None

        self.mode_future = None
        self.arming_future = None
        self.last_mode_request_time = None
        self.last_arm_request_time = None
        self.last_disarm_request_time = None

        self.last_status_note = None
        self.last_disarm_reason = None

        self.create_timer(
            1.0 / max(self.monitor_rate, 1.0),
            self.monitor_timer_callback,
        )

        self.get_logger().info(
            'auto_arm_disarm ready: '
            f'auto_arm={self.auto_arm_enabled}, '
            f'auto_disarm={self.auto_disarm_enabled}, '
            f'mode={self.guided_mode_name}, '
            f'disarm_timeout={self.disarm_timeout:.2f}s'
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def age(self, timestamp):
        if timestamp is None:
            return math.inf
        return self.now_seconds() - timestamp

    def is_recent(self, timestamp, timeout):
        return self.age(timestamp) <= timeout

    def log_status_once(self, message):
        if self.last_status_note == message:
            return
        self.last_status_note = message
        self.get_logger().info(message)

    def twist_is_nonzero(self, msg: Twist):
        return (
            abs(msg.linear.x) > self.nonzero_linear_epsilon
            or abs(msg.angular.z) > self.nonzero_angular_epsilon
        )

    def scan_rate_hz(self):
        if len(self.scan_timestamps) < 2:
            return 0.0

        elapsed = self.scan_timestamps[-1] - self.scan_timestamps[0]
        if elapsed <= 0.0:
            return 0.0

        return (len(self.scan_timestamps) - 1) / elapsed

    def state_callback(self, msg: State):
        now = self.now_seconds()
        self.state = msg
        self.last_state_time = now

        if self.last_state_mode != msg.mode:
            self.get_logger().info(f'MAVROS mode update: {msg.mode}')
            self.last_state_mode = msg.mode

        if self.last_state_armed != msg.armed:
            self.last_arm_transition_time = now
            if msg.armed:
                self.get_logger().info('Vehicle state update: ARMED')
            else:
                self.get_logger().info('Vehicle state update: DISARMED')
            self.last_state_armed = msg.armed

    def cmd_vel_callback(self, msg: Twist):
        now = self.now_seconds()
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = now
        if self.twist_is_nonzero(msg):
            self.last_cmd_vel_nonzero_time = now

    def cmd_vel_nav_callback(self, msg: Twist):
        now = self.now_seconds()
        self.last_cmd_vel_nav = msg
        self.last_cmd_vel_nav_time = now
        if self.twist_is_nonzero(msg):
            self.last_cmd_vel_nav_nonzero_time = now

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if math.isfinite(x) and math.isfinite(y):
            self.last_amcl_pose_time = self.now_seconds()

    def scan_callback(self, _msg: LaserScan):
        now = self.now_seconds()
        self.last_scan_time = now
        self.scan_timestamps.append(now)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if math.isfinite(x) and math.isfinite(y):
            self.last_odom_time = self.now_seconds()

    def collision_polygon_callback(self, _msg: PolygonStamped):
        if not self.collision_monitor_seen:
            self.get_logger().info(
                'Collision monitor polygon detected, safety supervisor linked.'
            )
        self.collision_monitor_seen = True

    def services_ready(self):
        ready = True

        if not self.set_mode_client.service_is_ready():
            ready = False
            self.log_status_once(
                f'Waiting for MAVROS service {self.set_mode_service_name}.'
            )

        if not self.arming_client.service_is_ready():
            ready = False
            self.log_status_once(
                f'Waiting for MAVROS service {self.arming_service_name}.'
            )

        return ready

    def current_cmd_vel_active(self):
        return self.is_recent(
            self.last_cmd_vel_time,
            self.required_topics_timeout,
        ) and self.twist_is_nonzero(self.last_cmd_vel)

    def current_nav_cmd_active(self):
        return self.is_recent(
            self.last_cmd_vel_nav_time,
            self.required_topics_timeout,
        ) and self.twist_is_nonzero(self.last_cmd_vel_nav)

    def readiness_issues(self, require_motion):
        issues = []

        if self.state is None or not self.is_recent(
            self.last_state_time,
            self.required_state_timeout,
        ):
            issues.append('mavros_state_missing')
            return issues

        if not self.state.connected:
            issues.append('mavros_not_connected')

        if not self.is_recent(self.last_scan_time, self.required_topics_timeout):
            issues.append('scan_missing')
        elif self.scan_rate_hz() < self.min_scan_rate:
            issues.append(
                f'scan_rate_low({self.scan_rate_hz():.1f}<{self.min_scan_rate:.1f})'
            )

        if not self.is_recent(self.last_odom_time, self.required_topics_timeout):
            issues.append('odom_missing')

        if not self.is_recent(self.last_amcl_pose_time, self.required_topics_timeout):
            issues.append('amcl_pose_missing')

        if require_motion:
            if self.require_nav_cmd_for_arm and not self.current_nav_cmd_active():
                issues.append('cmd_vel_nav_inactive')
            if not self.current_cmd_vel_active():
                issues.append('cmd_vel_inactive')

        return issues

    def should_infer_collision_stop(self):
        if not self.collision_monitor_seen:
            self.collision_stop_candidate_since = None
            return False

        if not self.current_nav_cmd_active():
            self.collision_stop_candidate_since = None
            return False

        if not self.is_recent(self.last_cmd_vel_time, self.required_topics_timeout):
            self.collision_stop_candidate_since = None
            return False

        if self.twist_is_nonzero(self.last_cmd_vel):
            self.collision_stop_candidate_since = None
            return False

        now = self.now_seconds()
        if self.collision_stop_candidate_since is None:
            self.collision_stop_candidate_since = now
            return False

        return (
            now - self.collision_stop_candidate_since
        ) >= self.collision_stop_confirm_time

    def inactivity_timeout_reached(self):
        if self.last_arm_transition_time is None:
            return False

        latest_motion_time = max(
            self.last_arm_transition_time,
            self.last_cmd_vel_nonzero_time or self.last_arm_transition_time,
            self.last_cmd_vel_nav_nonzero_time or self.last_arm_transition_time,
        )
        return self.age(latest_motion_time) > self.disarm_timeout

    def disarm_reason(self):
        if self.state is None:
            return None

        if not self.is_recent(self.last_state_time, self.required_state_timeout):
            return 'MAVROS state timed out'

        if not self.state.connected:
            return 'MAVROS disconnected from the FCU'

        if self.disarm_on_mode_change and self.state.mode != self.guided_mode_name:
            return f'vehicle left {self.guided_mode_name} mode'

        if self.disarm_on_sensor_loss:
            issues = self.readiness_issues(require_motion=False)
            filtered_issues = []
            for issue in issues:
                if issue == 'amcl_pose_missing' and not self.require_amcl_for_disarm:
                    continue
                filtered_issues.append(issue)
            if filtered_issues:
                return 'critical topic lost: ' + ', '.join(filtered_issues)

        # Collision monitor does not expose a direct "stop event" topic, so we
        # infer it by comparing the navigation intent with the secured /cmd_vel.
        if self.should_infer_collision_stop():
            return 'collision stop inferred from /cmd_vel_nav -> /cmd_vel clamp'

        if self.inactivity_timeout_reached():
            return (
                'inactivity timeout reached: '
                f'no motion command for {self.disarm_timeout:.1f}s'
            )

        return None

    def request_guided_mode(self, reason):
        if self.mode_future is not None:
            return

        if self.last_mode_request_time is not None and self.age(
            self.last_mode_request_time
        ) < self.arm_request_cooldown:
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = self.guided_mode_name

        self.last_mode_request_time = self.now_seconds()
        self.get_logger().warn(
            f'Requesting mode {self.guided_mode_name}: {reason}'
        )
        self.mode_future = self.set_mode_client.call_async(request)
        self.mode_future.add_done_callback(self.mode_response_callback)

    def mode_response_callback(self, future):
        self.mode_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'SetMode call failed: {exc}')
            return

        mode_sent = getattr(response, 'mode_sent', False)
        success = getattr(response, 'success', False)

        if mode_sent or success:
            self.get_logger().info(
                f'MAVROS accepted the request to switch to {self.guided_mode_name}.'
            )
        else:
            self.get_logger().error(
                f'MAVROS rejected the request to switch to {self.guided_mode_name}.'
            )

    def request_arming(self, arm, reason):
        if self.arming_future is not None:
            return

        cooldown = self.arm_request_cooldown if arm else self.disarm_request_cooldown
        last_request = self.last_arm_request_time if arm else self.last_disarm_request_time
        if last_request is not None and self.age(last_request) < cooldown:
            return

        request = CommandBool.Request()
        request.value = arm

        if arm:
            self.last_arm_request_time = self.now_seconds()
            action = 'ARM'
        else:
            self.last_disarm_request_time = self.now_seconds()
            action = 'DISARM'
            self.last_disarm_reason = reason

        self.get_logger().warn(f'Requesting {action}: {reason}')
        self.arming_future = self.arming_client.call_async(request)
        self.arming_future.add_done_callback(
            lambda future: self.arming_response_callback(future, arm)
        )

    def arming_response_callback(self, future, arm):
        self.arming_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Arming service call failed: {exc}')
            return

        success = getattr(response, 'success', False)
        result = getattr(response, 'result', None)
        action = 'ARM' if arm else 'DISARM'

        if success:
            self.get_logger().info(
                f'{action} request acknowledged by MAVROS.'
                + (f' result={result}' if result is not None else '')
            )
        else:
            self.get_logger().error(
                f'{action} request rejected by MAVROS.'
                + (f' result={result}' if result is not None else '')
            )

    def monitor_timer_callback(self):
        if not self.services_ready():
            return

        if self.state is None:
            self.log_status_once('Waiting for /mavros/state before supervising arm/disarm.')
            return

        if self.auto_disarm_enabled and self.state.armed:
            reason = self.disarm_reason()
            if reason is not None:
                self.request_arming(False, reason)
                return

        if not self.auto_arm_enabled:
            return

        if self.state.armed:
            return

        issues = self.readiness_issues(require_motion=True)
        if issues:
            self.log_status_once(
                'Auto-arm blocked until ready: ' + ', '.join(issues)
            )
            return

        if self.state.mode != self.guided_mode_name:
            self.request_guided_mode(
                'system ready for navigation but vehicle is not yet in GUIDED'
            )
            return

        if self.last_mode_request_time is not None and self.age(
            self.last_mode_request_time
        ) < self.guided_settle_time:
            return

        self.request_arming(
            True,
            'navigation active and all critical topics are valid',
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutoArmDisarmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
