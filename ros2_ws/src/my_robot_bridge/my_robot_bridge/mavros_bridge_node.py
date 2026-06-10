import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


EARTH_RADIUS_METERS = 6378137.0


class MavrosBridgeNode(Node):
    def __init__(self):
        super().__init__('mavros_bridge')

        self.declare_parameter('source_imu_topic', '/mavros/imu/data')
        self.declare_parameter('source_gps_topic', '/mavros/global_position/global')
        self.declare_parameter('source_odom_topic', '/mavros/local_position/odom')

        self.declare_parameter('imu_topic', '/sensors/imu/data')
        self.declare_parameter('gps_topic', '/sensors/gps/fix')
        self.declare_parameter('odom_topic', '/odom/raw')
        self.declare_parameter('imu_marker_topic', '/visualization/imu/orientation')
        self.declare_parameter('gps_marker_topic', '/visualization/gps/current_fix')
        self.declare_parameter('gps_path_topic', '/visualization/gps/path')

        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('gps_frame_id', 'gps_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_odom_tf', True)

        self.source_imu_topic = self.get_parameter(
            'source_imu_topic'
        ).get_parameter_value().string_value
        self.source_gps_topic = self.get_parameter(
            'source_gps_topic'
        ).get_parameter_value().string_value
        self.source_odom_topic = self.get_parameter(
            'source_odom_topic'
        ).get_parameter_value().string_value

        self.imu_topic = self.get_parameter(
            'imu_topic'
        ).get_parameter_value().string_value
        self.gps_topic = self.get_parameter(
            'gps_topic'
        ).get_parameter_value().string_value
        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).get_parameter_value().string_value
        self.imu_marker_topic = self.get_parameter(
            'imu_marker_topic'
        ).get_parameter_value().string_value
        self.gps_marker_topic = self.get_parameter(
            'gps_marker_topic'
        ).get_parameter_value().string_value
        self.gps_path_topic = self.get_parameter(
            'gps_path_topic'
        ).get_parameter_value().string_value

        self.base_frame_id = self.get_parameter(
            'base_frame_id'
        ).get_parameter_value().string_value
        self.imu_frame_id = self.get_parameter(
            'imu_frame_id'
        ).get_parameter_value().string_value
        self.gps_frame_id = self.get_parameter(
            'gps_frame_id'
        ).get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter(
            'odom_frame_id'
        ).get_parameter_value().string_value
        self.publish_odom_tf = self.get_parameter(
            'publish_odom_tf'
        ).get_parameter_value().bool_value

        default_qos = QoSProfile(depth=10)

        self.imu_pub = self.create_publisher(Imu, self.imu_topic, qos_profile_sensor_data)
        self.gps_pub = self.create_publisher(
            NavSatFix,
            self.gps_topic,
            qos_profile_sensor_data,
        )
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, default_qos)
        self.imu_marker_pub = self.create_publisher(
            Marker,
            self.imu_marker_topic,
            default_qos,
        )
        self.gps_marker_pub = self.create_publisher(
            Marker,
            self.gps_marker_topic,
            default_qos,
        )
        self.gps_path_pub = self.create_publisher(Path, self.gps_path_topic, default_qos)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish initial TF at startup so RViz displays the robot model
        # immediately, even before the first odom message arrives.
        self._publish_initial_tf()

        self.last_odom = None
        self.last_odom_position = None
        self.gps_origin = None
        self.gps_path = Path()
        self.gps_path.header.frame_id = self.odom_frame_id

        self.create_subscription(
            Imu,
            self.source_imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            self.source_gps_topic,
            self.gps_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.source_odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'my_robot_bridge started with clean topics: '
            f'imu={self.imu_topic}, gps={self.gps_topic}, odom={self.odom_topic}'
        )

    def _publish_initial_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def imu_callback(self, msg: Imu):
        clean_msg = copy.deepcopy(msg)
        clean_msg.header.frame_id = self.imu_frame_id
        self.imu_pub.publish(clean_msg)

        marker = Marker()
        marker.header.frame_id = self.odom_frame_id
        marker.header.stamp = msg.header.stamp
        marker.ns = 'imu'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.45
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.10
        marker.color.g = 0.85
        marker.color.b = 0.25
        marker.color.a = 1.0

        if self.last_odom_position is not None:
            marker.pose.position.x = self.last_odom_position[0]
            marker.pose.position.y = self.last_odom_position[1]
            marker.pose.position.z = self.last_odom_position[2] + 0.20

        marker.pose.orientation = copy.deepcopy(msg.orientation)
        self.imu_marker_pub.publish(marker)

    def gps_callback(self, msg: NavSatFix):
        clean_msg = copy.deepcopy(msg)
        clean_msg.header.frame_id = self.gps_frame_id
        self.gps_pub.publish(clean_msg)

        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if self.gps_origin is None:
            if msg.status.status < 0:
                self.get_logger().warn(
                    'GPS fix not yet available (status=%d), skipping origin lock.'
                    % msg.status.status
                )
                return
            self.gps_origin = (
                msg.latitude,
                msg.longitude,
                msg.altitude,
            )
            self.get_logger().info(
                'GPS visualization origin locked at '
                f'lat={msg.latitude:.7f}, lon={msg.longitude:.7f}, alt={msg.altitude:.2f}'
            )

        x, y, z = self.project_gps_to_local(
            msg.latitude,
            msg.longitude,
            msg.altitude,
        )

        marker = Marker()
        marker.header.frame_id = self.odom_frame_id
        marker.header.stamp = msg.header.stamp
        marker.ns = 'gps'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 0.95
        marker.color.g = 0.55
        marker.color.b = 0.10
        marker.color.a = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        self.gps_marker_pub.publish(marker)

        pose = PoseStamped()
        pose.header.frame_id = self.odom_frame_id
        pose.header.stamp = msg.header.stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        self.gps_path.header.stamp = msg.header.stamp
        self.gps_path.poses.append(pose)
        if len(self.gps_path.poses) > 500:
            self.gps_path.poses = self.gps_path.poses[-500:]
        self.gps_path_pub.publish(self.gps_path)

    def odom_callback(self, msg: Odometry):
        clean_msg = copy.deepcopy(msg)
        clean_msg.header.frame_id = self.odom_frame_id
        clean_msg.child_frame_id = self.base_frame_id
        self.last_odom = clean_msg
        self.last_odom_position = (
            clean_msg.pose.pose.position.x,
            clean_msg.pose.pose.position.y,
            clean_msg.pose.pose.position.z,
        )
        self.odom_pub.publish(clean_msg)

        if not self.publish_odom_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = clean_msg.header.stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = clean_msg.pose.pose.position.x
        transform.transform.translation.y = clean_msg.pose.pose.position.y
        transform.transform.translation.z = clean_msg.pose.pose.position.z
        transform.transform.rotation = clean_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def project_gps_to_local(self, latitude: float, longitude: float, altitude: float):
        origin_latitude, origin_longitude, origin_altitude = self.gps_origin

        delta_lat = math.radians(latitude - origin_latitude)
        delta_lon = math.radians(longitude - origin_longitude)
        mean_lat = math.radians((latitude + origin_latitude) / 2.0)

        x = EARTH_RADIUS_METERS * delta_lon * math.cos(mean_lat)
        y = EARTH_RADIUS_METERS * delta_lat
        z = altitude - origin_altitude

        return x, y, z


def main(args=None):
    rclpy.init(args=args)
    node = MavrosBridgeNode()

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
