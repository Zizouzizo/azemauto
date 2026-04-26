import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanTimestampBridgeNode(Node):
    def __init__(self):
        super().__init__('scan_timestamp_bridge')

        self.declare_parameter('input_scan_topic', '/scan/raw')
        self.declare_parameter('output_scan_topic', '/scan')
        self.declare_parameter('output_frame_id', '')

        self.input_scan_topic = self.get_parameter(
            'input_scan_topic'
        ).get_parameter_value().string_value
        self.output_scan_topic = self.get_parameter(
            'output_scan_topic'
        ).get_parameter_value().string_value
        self.output_frame_id = self.get_parameter(
            'output_frame_id'
        ).get_parameter_value().string_value

        self.scan_pub = self.create_publisher(
            LaserScan,
            self.output_scan_topic,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            self.input_scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'scan_timestamp_bridge ready: '
            f'input={self.input_scan_topic}, output={self.output_scan_topic}'
        )

    def scan_callback(self, msg: LaserScan):
        stamped_scan = copy.deepcopy(msg)
        stamped_scan.header.stamp = self.get_clock().now().to_msg()
        if self.output_frame_id:
            stamped_scan.header.frame_id = self.output_frame_id
        self.scan_pub.publish(stamped_scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
