

#!/usr/bin/env python3
"""
virtual_map_builder.py

Realtime virtual corridor builder for ROS2 package.
Publishes /virtual_map (nav_msgs/OccupancyGrid) aligned to the SLAM map.
Listens to /mission_state to auto-save when mission completes.

Usage as executable:
  ros2 run <package_name> virtual_map_builder
"""

import sys
import os
import yaml
import math
import numpy as np
from PIL import Image
import cv2
from ament_index_python.packages import get_package_share_directory

# Check Python and ROS environment FIRST
print("="*60)
print(f"Python: {sys.executable}")
print(f"Version: {sys.version.split()[0]}")

# Try importing ROS2 packages
print("\nImporting ROS2 packages...")
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
    print("✓ rclpy")
except Exception as e:
    print(f"✗ rclpy: {e}")
    sys.exit(1)

try:
    from nav_msgs.msg import OccupancyGrid, MapMetaData
    from std_msgs.msg import Header, String
    from std_srvs.srv import Trigger
    import geometry_msgs.msg
    print("✓ nav_msgs, std_msgs, std_srvs, geometry_msgs")
except Exception as e:
    print(f"✗ message types: {e}")
    sys.exit(1)

try:
    import tf2_ros
    print("✓ tf2_ros")
except Exception as e:
    print(f"✗ tf2_ros: {e}")
    print("\nERROR: tf2_ros not found!")
    print("Make sure you've sourced your ROS2 workspace:")
    print("  source /opt/ros/humble/setup.bash")
    print("  source ~/turtlebot3_ws/install/setup.bash")
    sys.exit(1)

try:
    from tf_transformations import euler_from_quaternion
    print("✓ tf_transformations")
except Exception as e:
    print(f"⚠ tf_transformations: {e} (will use manual calculation)")
    euler_from_quaternion = None

print("="*60)

# Configuration - edit these values as needed
MAP_YAML_PATH = '/home/masters/turtlebot3_ws/src/slam_lane_tracking_ros2/map/check2.yaml'
LEFT_DIST = 0.017      # meters from robot center to left boundary
RIGHT_DIST = 0.018     # meters to right boundary
DILATE_PIXELS = 3     # widen corridor in pixels (helps Nav2)
PUBLISH_HZ = 5.0      # publish rate
OUTPUT_PGM = os.path.expanduser('~/virtual_map.pgm')
OUTPUT_YAML = os.path.expanduser('~/virtual_map.yaml')

class VirtualMapBuilder(Node):
    def __init__(self):
        super().__init__('virtual_map_builder')
        
        # Declare and get parameters
        self.declare_parameter('map_yaml', MAP_YAML_PATH)
        self.declare_parameter('left_dist', LEFT_DIST)
        self.declare_parameter('right_dist', RIGHT_DIST)
        self.declare_parameter('dilate_pixels', DILATE_PIXELS)
        self.declare_parameter('publish_hz', PUBLISH_HZ)
        self.declare_parameter('output_pgm', OUTPUT_PGM)
        self.declare_parameter('output_yaml', OUTPUT_YAML)
        
        map_yaml_path = self.get_parameter('map_yaml').value
        self.left_dist = self.get_parameter('left_dist').value
        self.right_dist = self.get_parameter('right_dist').value
        self.dilate_pixels = self.get_parameter('dilate_pixels').value
        publish_hz = self.get_parameter('publish_hz').value
        self.output_pgm = self.get_parameter('output_pgm').value
        self.output_yaml = self.get_parameter('output_yaml').value
        
        # Mission control
        self.stop_flag = False
        self.create_subscription(String, '/mission_state', self.mission_cb, 10)
        
        # Service to stop and save map
        self.stop_srv = self.create_service(Trigger, 'stop_map_builder', self.stop_callback)

        # Load map yaml
        map_yaml_path = os.path.expanduser(map_yaml_path)
        if not os.path.exists(map_yaml_path):
            self.get_logger().error(f"Map YAML not found: {map_yaml_path}")
            raise SystemExit(1)

        with open(map_yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        yaml_dir = os.path.dirname(map_yaml_path)
        image_name = meta.get('image')
        map_img_path = image_name if os.path.isabs(image_name) else os.path.join(yaml_dir, image_name)
        if not os.path.exists(map_img_path):
            self.get_logger().error(f"Map image not found: {map_img_path}")
            raise SystemExit(1)

        # Map metadata
        self.resolution = float(meta['resolution'])
        self.origin_x = float(meta['origin'][0])
        self.origin_y = float(meta['origin'][1])

        # Load image
        img = Image.open(map_img_path).convert('L')
        self.map_pixels = np.array(img)
        self.height, self.width = self.map_pixels.shape

        self.get_logger().info(f"Loaded map {map_img_path}")
        self.get_logger().info(f"  Size: {self.width}x{self.height}")
        self.get_logger().info(f"  Resolution: {self.resolution} m/pixel")
        self.get_logger().info(f"  Origin: ({self.origin_x}, {self.origin_y})")
        self.get_logger().info(f"  Corridor width: L={self.left_dist}m, R={self.right_dist}m")

        # Store trajectory points
        self.left_pts = []
        self.right_pts = []

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher with map-compatible QoS (Transient Local for late joiners like map_saver)
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.pub = self.create_publisher(OccupancyGrid, 'virtual_map', map_qos)

        # Timer
        period = 1.0 / float(publish_hz)
        self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info("VirtualMapBuilder ready!")
        self.get_logger().info("Listening to /mission_state for auto-save...")
        self.get_logger().info("Service available: ros2 service call /stop_map_builder std_srvs/srv/Trigger")
        self.get_logger().info("")
        self.get_logger().info("Publishing /virtual_map topic (nav_msgs/OccupancyGrid)")
        self.get_logger().info("  Format: 0=free, 100=occupied (Nav2 compatible)")
        self.get_logger().info("")
        self.get_logger().info("To save map using nav2_map_server:")
        self.get_logger().info("  ros2 run nav2_map_server map_saver_cli -f ~/my_map -t /virtual_map")
        self.get_logger().info("")
    
    def mission_cb(self, msg):
        """Callback for mission state - auto-save when mission completes"""
        if msg.data == "MISSION_COMPLETED" and not self.stop_flag:
            self.get_logger().info("Mission completed → Saving map and stopping.")
            self.save_final_map(self.output_pgm, self.output_yaml)
            self.stop_flag = True
    
    def stop_callback(self, request, response):
        """Service callback to stop map building and save"""
        self.get_logger().info("Received STOP request → saving map and stopping updates.")
        self.stop_flag = True
        self.save_final_map(self.output_pgm, self.output_yaml)
        response.success = True
        response.message = "Map builder stopped and map saved successfully."
        return response

    def world_to_pixel(self, x_world, y_world):
        """Convert world coordinates to pixel coordinates"""
        # World coords: origin at map origin, Y points up (ROS standard)
        # Pixel coords: origin at top-left, Y points down (image standard)
        px = int((x_world - self.origin_x) / self.resolution)
        # Note: we flip Y here because image origin is top-left
        py = int((y_world - self.origin_y) / self.resolution)
        py = self.height - 1 - py  # flip Y axis
        return px, py

    def pixel_valid(self, px, py):
        """Check if pixel is within map bounds"""
        return 0 <= px < self.width and 0 <= py < self.height

    def timer_cb(self):
        """Main callback - get robot pose and build corridor"""
        # Stop processing if mission is complete
        if self.stop_flag:
            return
            
        # Lookup transform map -> base_link
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, 
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=5.0)
            return

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        q = trans.transform.rotation
        
        # Compute yaw
        if euler_from_quaternion is not None:
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        else:
            # Manual quaternion to yaw conversion
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)

        # Compute left & right boundary points (perpendicular to heading)
        lx = tx + self.left_dist * math.cos(yaw + math.pi/2.0)
        ly = ty + self.left_dist * math.sin(yaw + math.pi/2.0)
        rx = tx + self.right_dist * math.cos(yaw - math.pi/2.0)
        ry = ty + self.right_dist * math.sin(yaw - math.pi/2.0)

        # Convert to pixel coordinates
        lpx, lpy = self.world_to_pixel(lx, ly)
        rpx, rpy = self.world_to_pixel(rx, ry)

        # Append if valid
        if self.pixel_valid(lpx, lpy):
            self.left_pts.append((lpx, lpy))
        if self.pixel_valid(rpx, rpy):
            self.right_pts.append((rpx, rpy))

        # Need at least 3 points to form a polygon
        if len(self.left_pts) < 3 or len(self.right_pts) < 3:
            grid_msg = self.build_occupancy_grid(full_occupied=True)
            self.pub.publish(grid_msg)
            self.get_logger().info(
                f"Collecting points... L={len(self.left_pts)}, R={len(self.right_pts)}", 
                throttle_duration_sec=2.0
            )
            return

        # Build polygon: left polyline + reversed right polyline
        poly_pts = np.array(self.left_pts + self.right_pts[::-1], dtype=np.int32)

        # Create canvas: inside corridor=255 (white/free), outside=0 (black/occupied)
        canvas = np.full((self.height, self.width), 0, dtype=np.uint8)  # default: black (occupied)
        cv2.fillPoly(canvas, [poly_pts], 255)  # inside corridor = white (free)

        # Optionally dilate corridor (expand free space)
        if self.dilate_pixels > 0:
            kernel = np.ones((self.dilate_pixels, self.dilate_pixels), dtype=np.uint8)
            canvas = cv2.dilate(canvas, kernel, iterations=1)

        # Publish
        grid_msg = self.build_occupancy_grid_from_canvas(canvas)
        self.pub.publish(grid_msg)

    def build_occupancy_grid_from_canvas(self, canvas):
        """Convert canvas to OccupancyGrid message"""
        # Canvas is already in image coordinates (top-left origin)
        # OccupancyGrid expects bottom-left origin, so flip vertically
        flipped = np.flipud(canvas)
        
        # Map: 255 (white) -> free(0), 0 (black) -> occupied(100)
        occ = np.where(flipped == 255, 0, 100).astype(np.int8)
        data = occ.flatten().tolist()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        info = MapMetaData()
        info.map_load_time = header.stamp
        info.resolution = float(self.resolution)
        info.width = int(self.width)
        info.height = int(self.height)
        
        pose = geometry_msgs.msg.Pose()
        pose.position.x = float(self.origin_x)
        pose.position.y = float(self.origin_y)
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        info.origin = pose

        grid = OccupancyGrid()
        grid.header = header
        grid.info = info
        grid.data = data
        return grid

    def build_occupancy_grid(self, full_occupied=False):
        """Build empty or fully occupied grid"""
        if full_occupied:
            occ = np.full((self.height, self.width), 100, dtype=np.int8)
        else:
            occ = np.full((self.height, self.width), -1, dtype=np.int8)
        data = occ.flatten().tolist()
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        
        info = MapMetaData()
        info.map_load_time = header.stamp
        info.resolution = float(self.resolution)
        info.width = int(self.width)
        info.height = int(self.height)
        
        pose = geometry_msgs.msg.Pose()
        pose.position.x = float(self.origin_x)
        pose.position.y = float(self.origin_y)
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        info.origin = pose
        
        grid = OccupancyGrid()
        grid.header = header
        grid.info = info
        grid.data = data
        return grid

    def save_final_map(self, out_pgm, out_yaml):
        """Save accumulated corridor map to disk"""
        if len(self.left_pts) < 3 or len(self.right_pts) < 3:
            self.get_logger().warn("Not enough points to save final map.")
            return
            
        poly_pts = np.array(self.left_pts + self.right_pts[::-1], dtype=np.int32)
        canvas = np.full((self.height, self.width), 0, dtype=np.uint8)  # black (occupied)
        cv2.fillPoly(canvas, [poly_pts], 255)  # white (free)
        
        if self.dilate_pixels > 0:
            kernel = np.ones((self.dilate_pixels, self.dilate_pixels), dtype=np.uint8)
            canvas = cv2.dilate(canvas, kernel, iterations=1)

        # Convert and save: 255 (white) -> 254 (free in PGM), 0 (black) -> 0 (occupied in PGM)
        # For PGM format: lower values = obstacles, higher values = free space
        flipped = np.flipud(canvas)
        # Map to PGM scale: 0 = occupied (black), 254 = free (white)
        pgm_data = np.where(flipped == 255, 254, 0).astype(np.uint8)
        Image.fromarray(pgm_data).save(out_pgm)
        
        # Write YAML
        yaml_data = {
            'image': os.path.basename(out_pgm),
            'mode': 'trinary',
            'resolution': float(self.resolution),
            'origin': [float(self.origin_x), float(self.origin_y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        with open(out_yaml, 'w') as f:
            yaml.dump(yaml_data, f)
            
        self.get_logger().info(f"Saved: {out_pgm} and {out_yaml}")
        self.get_logger().info(f"Total points: Left={len(self.left_pts)}, Right={len(self.right_pts)}")
        self.get_logger().info("Map colors: WHITE=free space (where robot can drive)")
        self.get_logger().info("            BLACK=occupied (obstacles/boundaries)")

def main():
    rclpy.init()
    node = VirtualMapBuilder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (Ctrl+C)")
        # Save map on manual shutdown
        if not node.stop_flag:
            node.save_final_map(node.output_pgm, node.output_yaml)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()