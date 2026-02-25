#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class OmniLaneSwitcher(Node):
    def __init__(self):
        super().__init__('omni_lane_switcher')
        
        # 1. Pub/Sub Setup
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/filtered_odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 2. Parameters
        self.start_x = 0.0
        self.target_x = 10.0
        self.lanes = [0.0, 0.4]  
        self.current_lane_idx = 0 
        
        self.obstacle_threshold = 1.0  
        self.dist_tolerance = 0.05
        self.y_precision = 0.03        
        self.max_speed = 0.2        
        self.strafe_speed = 0.15       
        self.kp_yaw = 2.0              
        self.kp_y = 1.2                

        # Vector Math Setup
        self.target_vec_x = self.target_x - self.start_x
        self.target_mag_sq = self.target_vec_x**2
        
        # 3. State & Timing Variables
        self.state = "FORWARD"
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.obstacle_in_front = False
        self.last_switch_time = self.get_clock().now()
        self.switch_cooldown = 3.0  # seconds

        self.get_logger().info("========================================")
        self.get_logger().info("OMNI MISSION STARTING")
        self.get_logger().info(f"Target Distance: {self.target_x}m")
        self.get_logger().info(f"Lanes Configured: {self.lanes}")
        self.get_logger().info("========================================")

    def get_yaw(self, q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def scan_callback(self, msg):
        middle_idx = len(msg.ranges) // 2
        window = 3 
        front_sector = msg.ranges[middle_idx - window : middle_idx + window]
        valid_hits = [r for r in front_sector if msg.range_min < r < msg.range_max]
        self.obstacle_in_front = bool(valid_hits and min(valid_hits) < self.obstacle_threshold)

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.curr_yaw = self.get_yaw(msg.pose.pose.orientation)
        
        vel = Twist()
        now = self.get_clock().now()
        time_since_last_switch = (now - self.last_switch_time).nanoseconds / 1e9

        # Orientation Lock (Force Yaw to 0)
        yaw_error = 0.0 - self.curr_yaw
        vel.angular.z = self.kp_yaw * yaw_error

        # Calculate Progress
        dist_to_target = self.target_x - self.curr_x
        curr_vec_x = self.curr_x - self.start_x
        dot_product = curr_vec_x * self.target_vec_x

        if self.state == "FORWARD":
            # 1. Obstacle Detection Trigger
            if self.obstacle_in_front and time_since_last_switch > self.switch_cooldown:
                target_lane = 1 - self.current_lane_idx
                self.get_logger().warn(f"!!! OBSTACLE AT {self.curr_x:.2f}m !!! Switching to Lane {target_lane}")
                self.current_lane_idx = target_lane
                self.last_switch_time = now 
                self.state = "STRAFING"
                return

            # 2. Forward Movement Logic
            if dist_to_target > self.dist_tolerance and dot_product < self.target_mag_sq:
                vel.linear.x = min(self.max_speed, dist_to_target + 0.05)
                target_y = self.lanes[self.current_lane_idx]
                vel.linear.y = self.kp_y * (target_y - self.curr_y)
                
                # Progress Logging
                self.get_logger().info(
                    f"State: DRIVE | Remaining: {dist_to_target:.2f}m | Lane: {self.current_lane_idx} | Y: {self.curr_y:.2f}",
                    throttle_duration_sec=0.5
                )
            else:
                self.state = "STOP"

        elif self.state == "STRAFING":
            target_y = self.lanes[self.current_lane_idx]
            y_error = target_y - self.curr_y

            if abs(y_error) < self.y_precision:
                self.get_logger().info(f"SUCCESS: Lane Change Complete. Resuming Forward at X={self.curr_x:.2f}m")
                self.state = "FORWARD"
            else:
                vel.linear.x = 0.0 
                vel.linear.y = self.strafe_speed if y_error > 0 else -self.strafe_speed
                self.get_logger().info(f"State: STRAFING | Target Y: {target_y:.1f} | Current Y: {self.curr_y:.2f}", throttle_duration_sec=0.5)

        elif self.state == "STOP":
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.get_logger().info("****************************************")
            self.get_logger().info("GOAL REACHED - MISSION FINISHED")
            self.get_logger().info(f"Final Position: X={self.curr_x:.3f}, Y={self.curr_y:.3f}")
            self.get_logger().info("****************************************")

        self.cmd_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = OmniLaneSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()