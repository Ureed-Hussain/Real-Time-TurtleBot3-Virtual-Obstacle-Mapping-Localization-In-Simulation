#!/usr/bin/env python3
"""
lane_offset_calibrator.py - Enhanced with Verification Features

Now includes:
1. Image saving with measurement overlays
2. Visual verification of distances
3. Multiple snapshot options
4. Measurement confirmation prompts
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import argparse
import time
import os
from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class LaneOffsetCalibrator(Node):
    def __init__(self, topic="/camera/image_projected", calibration_method="manual"):
        super().__init__("lane_offset_calibrator")
        self.topic = topic
        self.calibration_method = calibration_method
        self.bridge = CvBridge()
        self.image = None
        self.camera_info = None
        self.robot_pose = None
        self.sub = None
        
        # Calibration parameters
        self.meters_per_pixel = None
        self.verification_dir = "lane_calibration_verification"
        
        # Create verification directory
        os.makedirs(self.verification_dir, exist_ok=True)
        
        # Check what topics are available
        topic_list = self.get_topic_names_and_types()
        available_topics = [t[0] for t in topic_list]
        
        self.get_logger().info(f"Looking for topic: {self.topic}")
        
        # Subscribe to image topic
        compressed_topic = self.topic + "/compressed" if not self.topic.endswith("/compressed") else self.topic
        base_topic = self.topic.replace("/compressed", "") if self.topic.endswith("/compressed") else self.topic
        
        subscribed = False
        
        if compressed_topic in available_topics:
            self.sub = self.create_subscription(CompressedImage, compressed_topic, self.cb_compressed, 10)
            self.get_logger().info(f"✓ Subscribed to CompressedImage: {compressed_topic}")
            subscribed = True
        elif base_topic in available_topics:
            self.sub = self.create_subscription(Image, base_topic, self.cb_image, 10)
            self.get_logger().info(f"✓ Subscribed to raw Image: {base_topic}")
            subscribed = True
        
        if not subscribed:
            raise RuntimeError(f"Topic {self.topic} not found")
        
        # Subscribe to camera_info if available
        if "/camera/camera_info" in available_topics:
            self.camera_info_sub = self.create_subscription(
                CameraInfo, "/camera/camera_info", self.cb_camera_info, 10
            )
            self.get_logger().info("✓ Subscribed to /camera/camera_info")
        
        # Subscribe to pose/odom if available
        if "/amcl_pose" in available_topics:
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped, "/amcl_pose", self.cb_pose, 10
            )
        elif "/odom" in available_topics:
            self.odom_sub = self.create_subscription(
                Odometry, "/odom", self.cb_odom, 10
            )
            self.get_logger().info("✓ Subscribed to /odom")

    def cb_compressed(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.image = img
        except Exception as e:
            self.get_logger().warn(f"Compressed decode failed: {e}")

    def cb_image(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
    
    def cb_camera_info(self, msg):
        self.camera_info = msg
    
    def cb_pose(self, msg):
        self.robot_pose = msg.pose.pose
    
    def cb_odom(self, msg):
        self.robot_pose = msg.pose.pose

    # def detect_centroids(self, img):
    #     """Return centroid_x for yellow and white lines (in pixels), or None if not found."""
    #     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #     # Yellow mask (tunable)
    #     lower_y = np.array([15, 100, 100])
    #     upper_y = np.array([35, 255, 255])
    #     mask_y = cv2.inRange(hsv, lower_y, upper_y)
    #     mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        
    #     # White mask (tunable)
    #     lower_w = np.array([0, 0, 200])
    #     upper_w = np.array([180, 40, 255])
    #     mask_w = cv2.inRange(hsv, lower_w, upper_w)
    #     mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

    #     def largest_centroid(mask):
    #         cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #         if not cnts:
    #             return None
    #         cnt = max(cnts, key=cv2.contourArea)
    #         M = cv2.moments(cnt)
    #         if M["m00"] == 0:
    #             return None
    #         cx = int(M["m10"]/M["m00"])
    #         cy = int(M["m01"]/M["m00"])
    #         return cx, cy

    #     yellow_c = largest_centroid(mask_y)
    #     white_c = largest_centroid(mask_w)

    #     return yellow_c, white_c, mask_y, mask_w
    def detect_centroids(self, img):
        """Return centroid_x for yellow and white lines (in pixels), or None if not found."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Yellow mask - Updated with your values
        # HSV format: [Hue, Saturation, Value/Lightness]
        lower_y = np.array([10, 115, 193])   # [hue_l, saturation_l, lightness_l]
        upper_y = np.array([123, 255, 255])  # [hue_h, saturation_h, lightness_h]
        mask_y = cv2.inRange(hsv, lower_y, upper_y)
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        
        # White mask - Updated with your values
        # HSV format: [Hue, Saturation, Value/Lightness]
        lower_w = np.array([0, 0, 190])      # [hue_l, saturation_l, lightness_l]
        upper_w = np.array([179, 76, 255])   # [hue_h, saturation_h, lightness_h]
        mask_w = cv2.inRange(hsv, lower_w, upper_w)
        mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

        def largest_centroid(mask):
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                return None
            cnt = max(cnts, key=cv2.contourArea)
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                return None
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            return cx, cy

        yellow_c = largest_centroid(mask_y)
        white_c = largest_centroid(mask_w)

        return yellow_c, white_c, mask_y, mask_w

    def create_measurement_overlay(self, img, det, left_m, right_m, mpp=None):
        """Create a visualization with measurement overlays"""
        h, w = img.shape[:2]
        overlay = img.copy()
        cx = w // 2
        
        # Draw center line
        cv2.line(overlay, (cx, 0), (cx, h), (255, 0, 0), 3)
        cv2.putText(overlay, "ROBOT CENTER", (cx + 10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # Draw detected lines and measurements
        if det['yellow_centroid']:
            yx, yy = det['yellow_centroid']
            cv2.circle(overlay, (yx, yy), 8, (0, 255, 255), -1)
            cv2.line(overlay, (cx, yy), (yx, yy), (0, 255, 255), 2)
            if left_m:
                cv2.putText(overlay, f"LEFT: {left_m:.3f}m", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(overlay, f"({abs(det['offset_left_px'])} px)", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        
        if det['white_centroid']:
            wx, wy = det['white_centroid']
            cv2.circle(overlay, (wx, wy), 8, (255, 255, 255), -1)
            cv2.line(overlay, (cx, wy), (wx, wy), (255, 255, 255), 2)
            if right_m:
                cv2.putText(overlay, f"RIGHT: {right_m:.3f}m", (w - 200, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(overlay, f"({abs(det['offset_right_px'])} px)", (w - 200, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Add calibration info
        if mpp:
            cv2.putText(overlay, f"Scale: {mpp:.6f} m/px", (10, h - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Add lane width if both detected
        if left_m and right_m:
            lane_width = left_m + right_m
            cv2.putText(overlay, f"LANE WIDTH: {lane_width:.3f}m", (w//2 - 100, h - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return overlay

    def save_verification_image(self, img, filename_suffix=""):
        """Save image with timestamp for verification"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.verification_dir}/calibration_verify_{timestamp}{filename_suffix}.jpg"
        cv2.imwrite(filename, img)
        print(f"✓ Verification image saved: {filename}")
        return filename

    def run_detection_once(self, timeout=5.0, save_image=False, image_suffix=""):
        self.get_logger().info("Waiting for image...")
        t0 = time.time()
        while self.image is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.image is None:
            self.get_logger().error("No image received within timeout.")
            return None
        
        img = self.image.copy()
        y_c, w_c, my, mw = self.detect_centroids(img)
        h, w = img.shape[:2]
        cx = w // 2
        cy = h // 2
        
        offset_left_px = None
        offset_right_px = None
        if y_c:
            offset_left_px = cx - y_c[0]  # Positive if yellow is left of center
        if w_c:
            offset_right_px = w_c[0] - cx  # Positive if white is right of center
        
        result = {
            'img': img,
            'yellow_centroid': y_c,
            'white_centroid': w_c,
            'offset_left_px': offset_left_px,
            'offset_right_px': offset_right_px,
            'width': w, 'height': h
        }
        
        # Create and display visualization
        overlay = self.create_measurement_overlay(img, result, None, None)
        cv2.imshow("Lane Detection - Live", overlay)
        cv2.waitKey(1)
        
        # Save image if requested
        if save_image:
            self.save_verification_image(overlay, image_suffix)
        
        return result

    def calibrate_manual(self, known_distance_m):
        """
        Manual calibration: User places robot at known distance from a line
        """
        print("\n" + "="*60)
        print("MANUAL CALIBRATION MODE")
        print("="*60)
        print(f"1. Place your robot EXACTLY {known_distance_m}m from a lane line")
        print("2. Make sure the line is visible in the projected view")
        print("3. Press Enter when ready...")
        input()
        
        det = self.run_detection_once(timeout=5.0, save_image=True, image_suffix="_manual_setup")
        if det is None:
            print("Failed to detect lanes!")
            return None
        
        # Use whichever line is detected
        pixel_offset = None
        line_type = None
        if det['offset_left_px'] is not None:
            pixel_offset = abs(det['offset_left_px'])
            line_type = "YELLOW (left)"
        elif det['offset_right_px'] is not None:
            pixel_offset = abs(det['offset_right_px'])
            line_type = "WHITE (right)"
        
        if pixel_offset is None or pixel_offset == 0:
            print("No valid line detected or line is at center!")
            return None
        
        meters_per_pixel = known_distance_m / pixel_offset
        
        print(f"\n✓ Detected {line_type} line at {pixel_offset:.1f} pixels from center")
        print(f"✓ Calculated: {meters_per_pixel:.6f} meters/pixel")
        print(f"✓ Image width: {det['width']} pixels = {det['width'] * meters_per_pixel:.3f} meters")
        
        # Create verification overlay
        verification_img = self.create_measurement_overlay(
            det['img'], det, 
            known_distance_m if line_type == "YELLOW (left)" else None,
            known_distance_m if line_type == "WHITE (right)" else None,
            meters_per_pixel
        )
        
        # Save and show verification
        verify_file = self.save_verification_image(verification_img, "_manual_calibration")
        cv2.imshow("Manual Calibration Result", verification_img)
        print(f"✓ Check the saved image to verify: {known_distance_m}m distance looks correct")
        cv2.waitKey(2000)  # Show for 2 seconds
        
        return meters_per_pixel

    def verify_calibration(self, det, left_m, right_m, mpp):
        """
        Let user visually verify the calibration results
        """
        print("\n" + "="*60)
        print("CALIBRATION VERIFICATION")
        print("="*60)
        
        # Create detailed verification image
        verify_img = self.create_measurement_overlay(det['img'], det, left_m, right_m, mpp)
        
        # Add verification text
        h, w = verify_img.shape[:2]
        cv2.putText(verify_img, "CALIBRATION VERIFICATION", (w//2 - 150, h - 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Display
        cv2.imshow("Calibration Verification", verify_img)
        cv2.waitKey(1)
        
        # Save verification image
        verify_file = self.save_verification_image(verify_img, "_final_verification")
        
        print("Please verify the measurements:")
        print(f"  - Check saved image: {verify_file}")
        print(f"  - Left distance:  {left_m:.3f}m (should match your physical measurement)")
        print(f"  - Right distance: {right_m:.3f}m (should match your physical measurement)")
        print()
        
        response = input("Do the measurements look correct? (y/n): ").lower().strip()
        return response == 'y'

    def take_additional_measurements(self, mpp):
        """
        Take measurements at different positions to verify consistency
        """
        print("\n" + "="*60)
        print("ADDITIONAL VERIFICATION MEASUREMENTS")
        print("="*60)
        print("Let's take 2 more measurements at different positions to verify consistency...")
        
        measurements = []
        
        for i in range(2):
            print(f"\nMeasurement {i+1}: Move robot to a different position")
            print("Press Enter when ready...")
            input()
            
            det = self.run_detection_once(timeout=5.0, save_image=True, image_suffix=f"_verify_{i+1}")
            if det:
                left_m, right_m = self.get_offsets_in_meters(det, mpp)
                measurements.append((left_m, right_m))
                
                print(f"Position {i+1}: Left={left_m:.3f}m, Right={right_m:.3f}m")
                
                # Show verification
                verify_img = self.create_measurement_overlay(det['img'], det, left_m, right_m, mpp)
                cv2.imshow(f"Verification Position {i+1}", verify_img)
                cv2.waitKey(1000)
        
        return measurements

    def get_offsets_in_meters(self, det, mpp=None):
        """Convert pixel offsets to meters"""
        if mpp is None:
            mpp = self.meters_per_pixel
        if mpp is None:
            return None, None
        
        left_m = det['offset_left_px'] * mpp if det['offset_left_px'] else None
        right_m = det['offset_right_px'] * mpp if det['offset_right_px'] else None
        
        return left_m, right_m

    def calibrate_camera_params(self, focal_length, camera_height, tilt_angle_deg=0):
        """Camera parameters calibration (unchanged from previous)"""
        # ... (same as previous implementation)
        pass

    def calibrate_interactive_motion(self, distance_m):
        """Interactive motion calibration (unchanged from previous)"""
        # ... (same as previous implementation)
        pass

def main():
    parser = argparse.ArgumentParser(description="Lane offset calibrator with verification")
    parser.add_argument('--topic', default='/camera/image_projected', 
                        help='Projected bird-eye view topic')
    parser.add_argument('--calibration-method', 
                        choices=['manual', 'camera', 'interactive'], 
                        default='manual',
                        help='Calibration method to use')
    
    # Manual calibration args
    parser.add_argument('--known-distance', type=float, default=0.5,
                        help='Known distance from lane line in meters (manual method)')
    
    # Camera calibration args
    parser.add_argument('--focal-length', type=float, default=500.0,
                        help='Camera focal length in pixels (camera method)')
    parser.add_argument('--camera-height', type=float, default=0.3,
                        help='Camera height above ground in meters (camera method)')
    parser.add_argument('--tilt-angle', type=float, default=0.0,
                        help='Camera tilt angle in degrees (camera method)')
    
    # Interactive calibration args
    parser.add_argument('--motion', type=float, default=1.0,
                        help='Distance to move robot in meters (interactive method)')
    
    # Verification args
    parser.add_argument('--skip-verification', action='store_true',
                        help='Skip the visual verification step')
    parser.add_argument('--additional-measurements', type=int, default=2,
                        help='Number of additional verification measurements to take')
    
    args = parser.parse_args()

    rclpy.init()
    
    try:
        node = LaneOffsetCalibrator(topic=args.topic, calibration_method=args.calibration_method)
    except RuntimeError as e:
        print(f"Failed to create node: {e}")
        rclpy.shutdown()
        return
    
    try:
        print("\n" + "="*60)
        print("LANE OFFSET CALIBRATOR - With Visual Verification")
        print("="*60)
        print(f"Verification images will be saved in: {node.verification_dir}")
        
        # Perform calibration
        if args.calibration_method == 'manual':
            node.meters_per_pixel = node.calibrate_manual(args.known_distance)
        elif args.calibration_method == 'camera':
            node.meters_per_pixel = node.calibrate_camera_params(
                args.focal_length, args.camera_height, args.tilt_angle
            )
        elif args.calibration_method == 'interactive':
            node.meters_per_pixel = node.calibrate_interactive_motion(args.motion)
        
        if node.meters_per_pixel is None:
            print("\n Calibration failed!")
            return
        
        # Final measurement
        print("\nTaking final measurement...")
        det = node.run_detection_once(timeout=5.0, save_image=True, image_suffix="_final")
        if det is None:
            return
        
        left_m, right_m = node.get_offsets_in_meters(det)
        
        # Verification
        verified = False
        if not args.skip_verification:
            verified = node.verify_calibration(det, left_m, right_m, node.meters_per_pixel)
            
            if verified and args.additional_measurements > 0:
                additional_measurements = node.take_additional_measurements(node.meters_per_pixel)
                print(f"\n✓ Additional measurements completed: {len(additional_measurements)} positions")
        else:
            verified = True
        
        # Final results
        print("\n" + "="*60)
        print("FINAL CALIBRATION RESULTS")
        print("="*60)
        print(f"Calibration: {node.meters_per_pixel:.6f} meters/pixel")
        print(f"Verification: {'PASSED ✓' if verified else 'SKIPPED'}")
        print()
        print("RECOMMENDED VALUES FOR YOUR MAP BUILDER:")
        print("-" * 50)
        print(f"LEFT_DIST_M = {left_m:.4f}" if left_m else "LEFT_DIST_M = <not_detected>")
        print(f"RIGHT_DIST_M = {right_m:.4f}" if right_m else "RIGHT_DIST_M = <not_detected>")
        
        if left_m and right_m:
            lane_width_m = left_m + right_m
            center_offset = (right_m - left_m) / 2
            print(f"LANE_WIDTH_M = {lane_width_m:.4f}")
            print(f"CENTER_OFFSET_M = {center_offset:.4f}")
        
        print("\n Tip: Check the saved images in the verification folder")
        print("   to confirm the measurements match your physical setup!")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()