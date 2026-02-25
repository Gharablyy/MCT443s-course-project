#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SiriusFinalController(Node):
    def __init__(self):
        super().__init__('sirius_final_controller')
        
        # 1. Pub/Sub Setup
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/filtered_odom', self.odom_callback, 10)

        # 2. User Inputs
        print("--- Sirius Robot Mission Control ---")
        self.target_dist = float(input("Enter target distance (meters): "))
        self.target_deg = float(input("Enter target heading (degrees): "))
        self.target_rad = math.radians(self.target_deg)

        # 3. Controller Parameters
        self.kp_yaw = 2.0   
        self.max_speed = 0.3  
        self.angle_tolerance = math.radians(0.1) 
        self.dist_tolerance = 0.1 # Changed to 0.01 for 1cm (0.1 was 10cm)
        
        # 4. State Variables
        self.start_pose = None
        self.target_pose = None
        self.state = "INIT"
        self.is_active = True

    def get_yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        if not self.is_active:
            return

        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        curr_yaw = self.get_yaw_from_quat(msg.pose.pose.orientation)

        if self.state == "INIT":
            self.start_pose = (curr_x, curr_y)
            target_x = self.start_pose[0] + (self.target_dist * math.cos(self.target_rad))
            target_y = self.start_pose[1] + (self.target_dist * math.sin(self.target_rad))
            self.target_pose = (target_x, target_y)
            
            self.get_logger().info(f"Target Calculated -> X: {target_x:.3f}, Y: {target_y:.3f}")
            self.state = "ROTATE"
            return

        vel = Twist()
        yaw_error = math.atan2(math.sin(self.target_rad - curr_yaw), math.cos(self.target_rad - curr_yaw))

        if self.state == "ROTATE":
            if abs(yaw_error) > self.angle_tolerance:
                rot_speed = self.kp_yaw * yaw_error
                vel.angular.z = max(0.05, min(0.5, rot_speed)) if rot_speed > 0 else min(-0.05, max(-0.5, rot_speed))
            else:
                vel.angular.z = 0.0
                self.state = "DRIVE"
                self.get_logger().info("Heading Aligned. Moving...")

        elif self.state == "DRIVE":
            # 1. Calculate distance remaining
            dist_to_target = math.sqrt((self.target_pose[0] - curr_x)**2 + (self.target_pose[1] - curr_y)**2)
            
            # 2. Check if we have physically passed the target (Dot Product)
            # This prevents the robot from driving forever if it overshoots by 1mm
            vec_to_target = (self.target_pose[0] - self.start_pose[0], self.target_pose[1] - self.start_pose[1])
            vec_to_curr = (curr_x - self.start_pose[0], curr_y - self.start_pose[1])
            dot_product = vec_to_curr[0] * vec_to_target[0] + vec_to_curr[1] * vec_to_target[1]
            target_mag_sq = vec_to_target[0]**2 + vec_to_target[1]**2

            # CONDITION: Stop if within tolerance OR if we have crossed the target line
            if dist_to_target > self.dist_tolerance and dot_product < target_mag_sq:
                vel.linear.x = min(self.max_speed, dist_to_target + 0.02)
                vel.angular.z = self.kp_yaw * yaw_error # Stay on the line
                self.get_logger().info(f"Remaining: {dist_to_target:.3f}m", throttle_duration_sec=0.5)
            else:
                # FORCE STOP
                self.state = "STOP"

        elif self.state == "STOP":
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.pub.publish(vel)
            self.get_logger().info("MISSION COMPLETE.")
            self.is_active = False
            rclpy.shutdown()

        self.pub.publish(vel)

def main():
    rclpy.init()
    node = SiriusFinalController()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()