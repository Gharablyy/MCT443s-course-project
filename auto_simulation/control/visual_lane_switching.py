#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class OmniLaneSwitcher(Node):
    def __init__(self):
        super().__init__('omni_lane_switcher')
        
        # 1. Pub/Sub Setup
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/filtered_odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/oak_d_lite_camera/image_raw', self.image_callback, 10)

        # 2. Parameters
        self.target_x = 10.0
        self.lanes = [0.0, 0.4]  
        self.current_lane_idx = 0 
        self.obstacle_threshold = 1.0  
        self.dist_tolerance = 0.05
        self.y_precision = 0.03        
        self.max_speed = 0.2        
        self.strafe_speed = 0.2       
        self.kp_cam = 0.4              # Further lowered for stability
        self.angle_threshold_deg = 0.05

        # 3. State & Vision Variables
        self.bridge = CvBridge()
        self.state = "FORWARD"
        self.lane_error_normalized = 0.0 
        self.lane_detected = False
        self.obstacle_in_front = False
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.last_switch_time = self.get_clock().now()
        
        # --- NEW SETTINGS ---
        self.switch_cooldown = 7.0     # Increased cooldown to 3 seconds
        self.vision_width_ratio = 0.4  # Only look at the center 40% of the lane width

        self.get_logger().info("Omni Switcher v6: Narrow Vision & 3s Cooldown Lock active.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = cv_image.shape
            img_center_x = w / 2

            # --- NARROW HORIZONTAL ROI ---
            # Instead of full width, we only look at a center slice (e.g., center 40%)
            # This prevents seeing the white lines of the next lane.
            x_start = int(w * (0.5 - self.vision_width_ratio / 2))
            x_end = int(w * (0.5 + self.vision_width_ratio / 2))
            
            roi_top = int(h * 0.7)
            roi = cv_image[roi_top:h, x_start:x_end]
            roi_w = x_end - x_start
            roi_center_x = roi_w / 2
            
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = []
            for cnt in contours:
                if cv2.contourArea(cnt) > 80:
                    M = cv2.moments(cnt)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        valid_contours.append((cx, cnt))

            if valid_contours:
                valid_contours.sort(key=lambda x: abs(x[0] - roi_center_x))
                best_cx_roi = valid_contours[0][0]
                # Map error back to normalized scale relative to ROI center
                self.lane_error_normalized = (best_cx_roi - roi_center_x) / (roi_w / 2.0)
                self.lane_detected = True
                cv2.circle(roi, (int(best_cx_roi), 20), 8, (0, 255, 0), -1)
            else:
                self.lane_error_normalized = 0.0
                self.lane_detected = False

            # Draw vertical bounds for the narrowed vision
            cv2.rectangle(cv_image, (x_start, roi_top), (x_end, h), (255, 255, 0), 2)
            cv2.imshow("Detection Mask (Narrowed)", binary)
            cv2.imshow("Full View (Yellow Box = Vision Zone)", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Vision Error: {e}")

    def scan_callback(self, msg):
        mid = len(msg.ranges) // 2
        front = msg.ranges[mid-3 : mid+3]
        valid = [r for r in front if msg.range_min < r < msg.range_max]
        self.obstacle_in_front = bool(valid and min(valid) < self.obstacle_threshold)

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        
        vel = Twist()
        now = self.get_clock().now()
        dt = (now - self.last_switch_time).nanoseconds / 1e9

        # --- CONDITION: YAW LOCK ---
        # 1. Yaw error is NOT corrected if state is STRAFING.
        # 2. Yaw error is NOT corrected if we are in COOLDOWN period after a switch.
        in_cooldown = dt < self.switch_cooldown
        
        current_error_deg = abs(self.lane_error_normalized * 15.0) # ROI is narrower, so multiplier is smaller
        
        if self.state == "FORWARD" and not in_cooldown and self.lane_detected and current_error_deg > self.angle_threshold_deg:
            vel.angular.z = -self.kp_cam * self.lane_error_normalized
            v_status = f"FOLLOWING ({current_error_deg:.1f} deg)"
        elif in_cooldown:
            v_status = f"LOCK: COOLDOWN ({self.switch_cooldown - dt:.1f}s rem)"
        elif self.state == "STRAFING":
            v_status = "LOCK: SWITCHING"
        else:
            v_status = "ALIGNED" if self.lane_detected else "LINE LOST"

        if self.state == "FORWARD":
            dist_to_goal = self.target_x - self.curr_x

            if self.obstacle_in_front and not in_cooldown:
                self.get_logger().warn(f"Obstacle @ {self.curr_x:.2f}m! Initiating Switch...")
                self.current_lane_idx = 1 - self.current_lane_idx
                self.last_switch_time = now
                self.state = "STRAFING"
                return

            if dist_to_goal > self.dist_tolerance:
                vel.linear.x = min(self.max_speed, dist_to_goal + 0.05)
                target_y = self.lanes[self.current_lane_idx]
                vel.linear.y = 1.0 * (target_y - self.curr_y)
                
                self.get_logger().info(f"DRIVE | X: {self.curr_x:.2f} | Vision: {v_status}", throttle_duration_sec=0.5)
            else:
                self.state = "STOP"

        elif self.state == "STRAFING":
            target_y = self.lanes[self.current_lane_idx]
            y_err = target_y - self.curr_y
            if abs(y_err) < self.y_precision:
                self.get_logger().info(f"Lane Switch Ready. Stabilization Cooldown Active.")
                self.state = "FORWARD"
            else:
                vel.linear.x = 0.0 
                vel.linear.y = self.strafe_speed if y_err > 0 else -self.strafe_speed
                vel.angular.z = 0.0 # Force zero yaw while strafing

        elif self.state == "STOP":
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.get_logger().info("MISSION FINISHED", throttle_duration_sec=5.0)

        self.cmd_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = OmniLaneSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()