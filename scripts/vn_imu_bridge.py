#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from vectornav_msgs.msg import CommonGroup
from sensor_msgs.msg import Imu


class VnImuBridge(Node):
    def __init__(self):
        super().__init__('vn_imu_bridge')
        self.declare_parameter('input_topic', '/vectornav/raw/common')
        self.declare_parameter('output_topic', '/imu0')

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(
            CommonGroup, in_topic, self.callback,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.pub = self.create_publisher(Imu, out_topic, 10)
        self._count = 0
        self.get_logger().info(f'Bridging {in_topic} -> {out_topic}')

    def callback(self, msg: CommonGroup):
        self._count += 1

        if self._count % 100 == 1:
            self.get_logger().info(
                f'[#{self._count}] accel=({msg.accel.x:.3f}, {msg.accel.y:.3f}, {msg.accel.z:.3f}) '
                f'gyro=({msg.angularrate.x:.3f}, {msg.angularrate.y:.3f}, {msg.angularrate.z:.3f})'
            )

        out = Imu()
        out.header = msg.header
        out.angular_velocity = msg.angularrate
        out.linear_acceleration = msg.accel
        # orientation not provided by VN-100T raw IMU output
        out.orientation_covariance[0] = -1.0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = VnImuBridge()
    node.get_logger().info('Bridge node started, waiting for messages...')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
