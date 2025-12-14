#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import numpy as np
import cv2
import yaml
import threading
import os
import math
import time

ORIGIN_FILE = "origin.yaml"


class ArucoReturnDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        # Parameters
        self.declare_parameter("calibration", True)
        self.calibration = self.get_parameter("calibration").get_parameter_value().bool_value

        # Camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None

        # Saved origin (robot pose + ArUco marker pose)
        self.saved_origin = None
        self.current_pose = None
        self.current_odom = None

        # Guidance
        self.last_guidance_time = 0.0
        self.guidance_interval = 0.5  # seconds
        self.pos_tol = 0.03
        self.z_tol = 0.05
        self.angle_tol_deg = 5.0
        self.warn_odom_missing = False

        # Camera to base_link transform
        self.camera_to_base_x = 0.0
        self.camera_to_base_y = 0.0
        self.camera_to_base_z = -0.1

        # Subscribers
        self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/aruco_pose", 10)
        self.distance_pub = self.create_publisher(Float32, "/aruco_distance", 10)
        self.origin_error_pub = self.create_publisher(PoseStamped, "/aruco_origin_error", 10)
        # NEW: Publisher for visualization image
        self.image_pub = self.create_publisher(CompressedImage, "/aruco_detection/compressed", 10)

        # ArUco setup - UPDATED FOR OPENCV 4.7+
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.marker_size = 0.10  # meters

        # Load origin if exists
        self.load_origin()

        # ENTER thread for calibration mode
        if self.calibration:
            threading.Thread(target=self.wait_for_enter, daemon=True).start()
            self.get_logger().info("Calibration mode ON: press ENTER to save origin when ArUco is seen.")
        else:
            self.get_logger().info("Calibration mode OFF: guiding robot to saved origin when ArUco is visible.")

        self.get_logger().info(f"Aruco detector initialized. calibration={self.calibration}")
        self.get_logger().info("Publishing visualization on /aruco_detection/compressed")

    # -------------------- ENTER handling --------------------
    def wait_for_enter(self):
        while rclpy.ok():
            try:
                input()
            except EOFError:
                break
            if self.current_odom is not None and self.current_pose is not None:
                self.save_origin(self.current_odom, self.current_pose)
                self.get_logger().info(f"Origin saved to {ORIGIN_FILE}")
                self.load_origin()
            else:
                self.get_logger().warn("No odometry or ArUco detected yet; cannot save origin.")

    # -------------------- Save/load --------------------
    def save_origin(self, odom: Odometry, aruco_pose: PoseStamped):
        """Save both robot odometry pose and ArUco marker pose"""
        origin = {
            # Robot odometry pose
            "robot_x": float(odom.pose.pose.position.x),
            "robot_y": float(odom.pose.pose.position.y),
            "robot_z": float(odom.pose.pose.position.z),
            "robot_qx": float(odom.pose.pose.orientation.x),
            "robot_qy": float(odom.pose.pose.orientation.y),
            "robot_qz": float(odom.pose.pose.orientation.z),
            "robot_qw": float(odom.pose.pose.orientation.w),
            # ArUco marker pose relative to camera
            "marker_x": float(aruco_pose.pose.position.x),
            "marker_y": float(aruco_pose.pose.position.y),
            "marker_z": float(aruco_pose.pose.position.z),
            "marker_qx": float(aruco_pose.pose.orientation.x),
            "marker_qy": float(aruco_pose.pose.orientation.y),
            "marker_qz": float(aruco_pose.pose.orientation.z),
            "marker_qw": float(aruco_pose.pose.orientation.w),
            "saved_time": time.time(),
        }
        with open(ORIGIN_FILE, "w") as f:
            yaml.dump(origin, f)

    def load_origin(self):
        if os.path.exists(ORIGIN_FILE):
            with open(ORIGIN_FILE, "r") as f:
                self.saved_origin = yaml.safe_load(f)
            self.get_logger().info(f"Loaded origin from {ORIGIN_FILE}")
        else:
            self.saved_origin = None
            self.get_logger().warn(f"{ORIGIN_FILE} not found (first run).")

    # -------------------- Camera info --------------------
    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera calibration received.")

    # -------------------- Odometry --------------------
    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

    # -------------------- Rotation helpers --------------------
    def rotation_vector_to_quaternion(self, rvec):
        R, _ = cv2.Rodrigues(rvec)
        q = np.empty(4)
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return [float(q[0]), float(q[1]), float(q[2]), float(q[3])]

    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        ], dtype=float)

    # -------------------- Compute errors using both odometry and ArUco --------------------
    def compute_errors(self, robot_pose: Odometry, aruco_pose: PoseStamped):
        """
        Compute error using both robot odometry and ArUco marker feedback.
        Returns both errors for comprehensive guidance.
        """
        if self.saved_origin is None:
            return None, None

        # Validate saved origin
        required_robot_fields = ["robot_x", "robot_y", "robot_z", "robot_qx", "robot_qy", "robot_qz", "robot_qw"]
        required_marker_fields = ["marker_x", "marker_y", "marker_z", "marker_qx", "marker_qy", "marker_qz", "marker_qw"]
        
        if not all(field in self.saved_origin for field in required_robot_fields + required_marker_fields):
            self.get_logger().error(
                f"Saved origin missing required fields. Delete origin.yaml and re-calibrate."
            )
            return None, None

        # --- Odometry-based error ---
        odom_err = PoseStamped()
        odom_err.header.stamp = self.get_clock().now().to_msg()
        odom_err.header.frame_id = "odom"

        odom_err.pose.position.x = robot_pose.pose.pose.position.x - float(self.saved_origin["robot_x"])
        odom_err.pose.position.y = robot_pose.pose.pose.position.y - float(self.saved_origin["robot_y"])
        odom_err.pose.position.z = robot_pose.pose.pose.position.z - float(self.saved_origin["robot_z"])

        q1_odom = np.array([
            robot_pose.pose.pose.orientation.x,
            robot_pose.pose.pose.orientation.y,
            robot_pose.pose.pose.orientation.z,
            robot_pose.pose.pose.orientation.w
        ], dtype=float)
        q2_odom = np.array([
            self.saved_origin["robot_qx"],
            self.saved_origin["robot_qy"],
            self.saved_origin["robot_qz"],
            self.saved_origin["robot_qw"]
        ], dtype=float)
        q2_odom_inv = np.array([-q2_odom[0], -q2_odom[1], -q2_odom[2], q2_odom[3]], dtype=float)
        q_odom_err = self.quaternion_multiply(q1_odom, q2_odom_inv)
        odom_err.pose.orientation.x = float(q_odom_err[0])
        odom_err.pose.orientation.y = float(q_odom_err[1])
        odom_err.pose.orientation.z = float(q_odom_err[2])
        odom_err.pose.orientation.w = float(q_odom_err[3])

        # --- ArUco-based error ---
        marker_err = PoseStamped()
        marker_err.header.stamp = self.get_clock().now().to_msg()
        marker_err.header.frame_id = "camera_link"

        marker_err.pose.position.x = aruco_pose.pose.position.x - float(self.saved_origin["marker_x"])
        marker_err.pose.position.y = aruco_pose.pose.position.y - float(self.saved_origin["marker_y"])
        marker_err.pose.position.z = aruco_pose.pose.position.z - float(self.saved_origin["marker_z"])

        q1_marker = np.array([
            aruco_pose.pose.orientation.x,
            aruco_pose.pose.orientation.y,
            aruco_pose.pose.orientation.z,
            aruco_pose.pose.orientation.w
        ], dtype=float)
        q2_marker = np.array([
            self.saved_origin["marker_qx"],
            self.saved_origin["marker_qy"],
            self.saved_origin["marker_qz"],
            self.saved_origin["marker_qw"]
        ], dtype=float)
        q2_marker_inv = np.array([-q2_marker[0], -q2_marker[1], -q2_marker[2], q2_marker[3]], dtype=float)
        q_marker_err = self.quaternion_multiply(q1_marker, q2_marker_inv)
        marker_err.pose.orientation.x = float(q_marker_err[0])
        marker_err.pose.orientation.y = float(q_marker_err[1])
        marker_err.pose.orientation.z = float(q_marker_err[2])
        marker_err.pose.orientation.w = float(q_marker_err[3])

        return odom_err, marker_err

    # -------------------- Guidance from both sources --------------------
    def guidance_from_errors(self, odom_err: PoseStamped, marker_err: PoseStamped):
        """Generate guidance using both odometry and ArUco feedback"""
        # Use odometry for position, ArUco for fine-tuning approach to marker
        forward_error = odom_err.pose.position.x
        lateral_error = odom_err.pose.position.y
        vertical_error = odom_err.pose.position.z

        # ArUco gives direct distance to marker
        marker_distance = np.sqrt(marker_err.pose.position.x**2 + marker_err.pose.position.y**2 + marker_err.pose.position.z**2)

        now = time.time()
        if now - self.last_guidance_time < self.guidance_interval:
            return None
        self.last_guidance_time = now

        messages = []
        
        # Forward/Backward guidance
        if abs(forward_error) > self.pos_tol:
            if forward_error > 0: messages.append(f"Move FORWARD by ~{abs(forward_error):.3f} m")
            else: messages.append(f"Move BACKWARD by ~{abs(forward_error):.3f} m")
        else:
            messages.append("Forward/Backward aligned")

        # Left/Right guidance
        if abs(lateral_error) > self.pos_tol:
            if lateral_error > 0: messages.append(f"Move RIGHT by ~{abs(lateral_error):.3f} m")
            else: messages.append(f"Move LEFT by ~{abs(lateral_error):.3f} m")
        else:
            messages.append("Left/Right aligned")

        # Height guidance
        if abs(vertical_error) > self.z_tol:
            messages.append(f"Height error: {abs(vertical_error):.3f} m")
        else:
            messages.append("Height aligned")

        # ArUco marker approach guidance
        messages.append(f"Distance to marker: {marker_distance:.3f} m")

        # Rotation guidance using ArUco
        qw = marker_err.pose.orientation.w
        angle_rad = 2.0 * math.acos(min(1.0, max(-1.0, abs(qw))))
        angle_deg = math.degrees(angle_rad)
        if angle_deg > self.angle_tol_deg:
            messages.append(f"Rotate marker view by ~{angle_deg:.1f} deg")
        else:
            messages.append("Marker orientation aligned")

        success = (abs(forward_error) <= self.pos_tol) and (abs(lateral_error) <= self.pos_tol) and (abs(vertical_error) <= self.z_tol) and (angle_deg <= self.angle_tol_deg)
        return messages, success, forward_error, lateral_error, vertical_error, marker_distance, angle_deg

    # -------------------- Image callback --------------------
    def image_callback(self, msg: CompressedImage):
        if self.camera_matrix is None:
            return

        # decode image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # detect marker - UPDATED FOR OPENCV 4.7+
        corners, ids, rejected = self.aruco_detector.detectMarkers(frame)
        
        # Draw detection results on frame for visualization
        if ids is not None and len(ids) > 0:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Estimate pose and draw axis
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            
            for i in range(len(ids)):
                # Draw 3D axis on each marker
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, 
                                rvecs[i], tvecs[i], self.marker_size * 0.5)
                
                # Add distance text
                distance = np.linalg.norm(tvecs[i])
                text = f"ID:{ids[i][0]} Dist:{distance:.2f}m"
                # Get corner position for text placement
                corner = corners[i][0][0]
                cv2.putText(frame, text, (int(corner[0]), int(corner[1]) - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Publish visualization image
        _, encoded_img = cv2.imencode('.jpg', frame)
        viz_msg = CompressedImage()
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.header.frame_id = "camera_link"
        viz_msg.format = "jpeg"
        viz_msg.data = encoded_img.tobytes()
        self.image_pub.publish(viz_msg)
        
        # Continue with original processing if marker detected
        if ids is None or len(ids) == 0:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]
        quat = self.rotation_vector_to_quaternion(rvec)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_link"
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.pose_pub.publish(pose)
        self.current_pose = pose
        self.distance_pub.publish(Float32(data=float(np.linalg.norm(tvec))))

        if self.saved_origin is not None and not self.calibration:
            if self.current_odom is None:
                if not self.warn_odom_missing:
                    self.get_logger().warn("Waiting for /odom messages to compute guidance.")
                    self.warn_odom_missing = True
                return

            # Compute errors using both odometry and ArUco
            odom_err, marker_err = self.compute_errors(self.current_odom, pose)
            if odom_err is not None and marker_err is not None:
                self.origin_error_pub.publish(odom_err)
                guidance = self.guidance_from_errors(odom_err, marker_err)
                if guidance is not None:
                    messages, success, forward_err, lateral_err, vertical_err, marker_dist, ang = guidance
                    for m in messages:
                        self.get_logger().info(m)
                    self.get_logger().info(f"Odometry: forward={forward_err:.3f}m, lateral={lateral_err:.3f}m, vertical={vertical_err:.3f}m | Marker: dist={marker_dist:.3f}m, orient={ang:.2f}deg")
                    if success:
                        self.get_logger().warn("✓ Aligned with saved origin! You are at the exact position.")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoReturnDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()