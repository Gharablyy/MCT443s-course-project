#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class ManualFusionNode(Node):
    def __init__(self):
        super().__init__('manual_fusion_node')
        
        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # Publisher
        self.filtered_pub = self.create_publisher(Odometry, '/filtered_odom', 10)

        # Storage for the latest IMU orientation
        self.latest_imu_orientation = None

    def imu_callback(self, imu_msg):
        # Store the high-precision orientation from the IMU
        self.latest_imu_orientation = imu_msg.orientation

    def odom_callback(self, odom_msg):
        # If we haven't received IMU data yet, don't publish
        if self.latest_imu_orientation is None:
            return

        # Create the new filtered message
        filtered_msg = Odometry()
        filtered_msg.header = odom_msg.header
        filtered_msg.header.frame_id = "odom"
        filtered_msg.child_frame_id = "base_footprint"

        # 1. Take Position and Velocity from Encoders
        filtered_msg.pose.pose.position = odom_msg.pose.pose.position
        filtered_msg.twist.twist = odom_msg.twist.twist

        # 2. OVERWRITE Orientation with IMU Data
        filtered_msg.pose.pose.orientation = self.latest_imu_orientation

        # Publish the combined data
        self.filtered_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ManualFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()