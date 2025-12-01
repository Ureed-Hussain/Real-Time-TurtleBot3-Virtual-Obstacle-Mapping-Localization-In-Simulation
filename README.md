# Real-Time-TurtleBot3-Virtual-Obstacle-Mapping-Localization

This repository, Real-Time-TurtleBot3-Virtual-Obstacle-Mapping-Localization, is the continuation of my previous project, TurtleBot3 Virtual Obstacle Mapping & Localization in Simulation. The earlier work focused on designing, testing, and validating the complete autonomous mapping pipeline entirely within a simulated environment. Before starting this project, it is strongly recommended to review the simulation repo for a full understanding of the mapping logic and system architecture:
https://github.com/Ureed-Hussain/TurtleBot3_Virtual_Obstacle_Mapping_Localization
In this new repository, the same virtual obstacle mapping and localization framework is extended and implemented in real time on the actual TurtleBot3 robot, using live sensor data and onboard processing to achieve real-world autonomous navigation.



# Part 1: Robot follow the lane

## Step 1: Camera Calibration

This is very important step, For this I recommended to follow this tutorial " https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/ "

### 1. Prepare the Checkerboard pattern
Use a black and white Checkerboard, usually 7×6 or 8×6 in size, print it out and attach it to a solid surface, and measure the squares of your checkerboard accurately.

### 2. Install the ROS 2 Camera Calibration Package
[Remote PC]
~~~
sudo apt update
sudo apt install ros-${ROS_DISTRO}-camera-calibration
source /opt/ros/${ROS_DISTRO}/setup.bash
~~~

### 3. Run the Camera node
First connect with turtlebot by their IP using "ssh", and then Run the camera node based on the camera package you installed.
* For camera-ros
[TurtleBot3 SBC]
~~~
ros2 run camera_ros camera_node --ros-args -p format:='RGB888'
~~~

After runing this command verify you got camera topics.
[Remote PC]
~~~
ros2 topic list
~~~
Topic lists:
~~~
/camera/camera_info
/camera/image_raw
/camera/image_raw/compressed
/camera/image_raw/theora
~~~

### 4. Run calibration node
Specify the size of the checkerboard and the size of the squares as execution arguments. The size of the checkerboard is the number of intersections.
[Remote PC]
~~~
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.023 \
  image:=/camera/image_raw camera:=/camera
~~~

### 5. Proceed with the calibration
When a checkerboard is detected, each intersection is connected. Modify the position of the checkerboard until the green bar on the right is filled to activate the button.

![WhatsApp Image 2025-11-30 at 3 36 30 PM](https://github.com/user-attachments/assets/11dddb26-ede2-4d6c-b682-e25a56a4db2c)

### 6. Apply calibration
Use the results to modify the format of the calibration yaml file you created when you installed the camera package.
[Result]
~~~
**** Calibrating ****
mono pinhole calibration...
D = [-0.24956645163546864, 0.04200151320596171, -0.0015198011708923138, -0.001999724710681652, 0.0]
K = [380.7469672813422, 0.0, 406.9705341838116, 0.0, 378.37382728825185, 281.3751983751762, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [261.0140380859375, 0.0, 405.063803910336, 0.0, 0.0, 296.8274841308594, 265.35472820934956, 0.0, 0.0, 0.0, 1.0, 0.0]
None
# oST version 5.0 parameters


[image]

width
800

height
600

[narrow_stereo]

camera matrix
380.746967 0.000000 406.970534
0.000000 378.373827 281.375198
0.000000 0.000000 1.000000

distortion
-0.249566 0.042002 -0.001520 -0.002000 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
261.014038 0.000000 405.063804 0.000000
0.000000 296.827484 265.354728 0.000000
0.000000 0.000000 1.000000 0.000000
~~~

### 7. Save Yaml file in TurtleBot3 SBC

* Create the calibration directory (if it doesn’t exist)

  [TurtleBot3 SBC]
  ~~~
  mkdir -p /home/ubuntu/.ros/camera_info/
  sudo nano /home/ubuntu/.ros/camera_info/<camera_name>.yaml
  ~~~

* Copy the yaml file

 Make sure to update the image_width, image_height values and camera_name to match your actual camera settings. Ensure the camera_name in the .yaml file matches the actual camera name.
~~~
image_width: 320   # Update to your camera's actual resolution
image_height: 240   # Update to your camera's actual resolution
camera_name: imx219__base_soc_i2c0mux_i2c_1_imx219_10_320x240   # Replace with the actual camera name
frame_id: camera
camera_matrix:
  rows: 3
  cols: 3
  data: [161.0352, 0, 99.6340, 0, 160.4337, 77.6267, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1639958, -0.2718400, 0.0010558, -0.0016656, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [164.6242, 0, 99.2051, 0, 0, 164.5522, 77.7529, 0, 0, 0, 1, 0]
~~~

## Step 2: Camera Intrnsic calibration
Intrinsic calibration focuses on correcting lens distortion and determining the camera’s internal properties, such as focal length and optical center. In real robots, this process is essential. 
For intrinsic calibration, the TurtleBot3 camera is first activated and its raw image stream is published on the /camera/image_raw topic. The intrinsic launch file then processes this stream using the image_proc pipeline: the DebayerNode converts the raw Bayer-pattern sensor data into a proper RGB image, and the RectifyNode prepares the image for calibration by correcting distortions using the camera model. The image_transport republisher converts compressed images to raw format so the calibration tool can receive clean, uncompressed frames. Together, these nodes supply the camera calibration tool with undistorted, correctly formatted images, enabling accurate computation of the camera's intrinsic parameters.

[TurtleBot3 SBC]
~~~
ros2 run camera_ros camera_node --ros-args -p format:='RGB888'
~~~

[Remote PC]

Terminal 1:

~~~
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
~~~

check you got these topic:
~~~
/camera/image_rect
/camera/image_rect/compressed
/camera/image_rect/compressedDepth
/camera/image_rect/theora
/camera/image_rect_color/compressed
~~~

Terminal 2:

~~~
rqt
~~~

and you clearly see you got rectify image:

##################################################### RQT IMAGE #######################################################


## Step 3: Extrinsic calibration
Extrinsic calibration aligns the camera’s perspective with the robot’s coordinate system, ensuring that objects detected in the camera’s view correspond to their actual positions in the robot’s environment. 
i updated little bit image_projection.py in turtlebot3_autorace_camera package, because red trapezoidal shape never comes on ground so i added X-offsets and Y-offsets.

Once the intrinsic_camera_calibration is running, launch the extrinsic calibration process:
~~~
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py calibration_mode:=True
~~~
This will activate the nodes responsible for camera-to-ground projection and compensation.

### Visualization and Parameter Adjustment

1. Execute rqt on Remote PC.
   ~~~
   rqt
   ~~~
2. Navigate to Plugins > Visualization > Image view. Create two image view windows.
3. Select the /camera/image_extrinsic_calib topic in one window and /camera/image_projected in the other.

   The first topic shows an image with a red trapezoidal shape and the latter shows the ground projected view (Bird’s eye view).
   

   ############### add normal image and projected image ######################
   /camera/image_extrinsic_calib (Left) and /camera/image_projected (Right)
4. Navigate to Plugins > Configuration > Dynamic Reconfigure.
5. Adjust the parameters in /camera/image_projection and /camera/image_compensation to tune the camera’s calibration.

    Change the /camera/image_projection value to adjust the /camera/image_extrinsic_calib topic.
    Intrinsic camera calibration modifies the perspective of the image in the red trapezoid.

    Adjust /camera/image_compensation to fine-tune the /camera/image_projected bird’s-eye view.

<img width="1155" height="704" alt="image" src="https://github.com/user-attachments/assets/f353715a-637a-4da1-9abf-29b9b678d63c" />

### Saving Calibration Data
Once the best projection settings are found, the calibration data must be saved to ensure that the parameters persist across sessions. One way to save the extrinsic calibration data is by manually editing the YAML configuration files.  

1. Navigate to the directory where the calibration files are stored:
   ~~~
   cd ~/turtlebot3_ws/src/turtlebot3_autorace/turtlebot3_autorace_camera/calibration/extrinsic_calibration/
   ~~~
2. Open the relevant YAML file (e.g., projection.yaml) in a text editor:
   ~~~
   gedit projection.yaml
   ~~~
3. Modify the projection parameters to match the values obtained from dynamic reconfiguration.

This method ensures that the extrinsic calibration parameters are correctly saved for future runs.

<img width="881" height="287" alt="image" src="https://github.com/user-attachments/assets/d81beb00-d911-47bf-beee-23909ec6e1b4" />

### Check Calibration Result
After completing the calibration process, follow the instructions below on the Remote PC to verify the calibration results.

1. Stop the current extrinsic calibration process.

   If the extrinsic calibration was launched in calibration_mode:=True, stop the process by closing the terminal or pressing Ctrl + C

2. Launch the extrinsic calibration node without calibration mode.

   This ensures that the system applies the saved calibration parameters for verification. 
   ~~~
   ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
   ~~~

3. Execute rqt and navigate Plugins > Visualization > Image view.

   ~~~
   rqt
   ~~~
4. With successful calibration settings, the bird-eye view image should appear like below when the /camera/image_projected topic is selected.

   ####################projected image#################################

## Step 4: Lane Detection
Lane detection allows the TurtleBot3 to recognize lane markings and follow them autonomously. The system processes camera images from a real TurtleBot3 , applies color filtering, and identifies lane boundaries.

This section explains how to launch the lane detection system, visualize the detected lane markings, and calibrate the parameters to ensure accurate tracking.

### Launching Lane Detection in Real-Turtlebot3

To begin, start the Turtlebot3 camera.
[TurtleBot3 SBC]
~~~
ros2 run camera_ros camera_node --ros-args -p format:='RGB888'
~~~

Next, run the camera calibration processes, which ensure that the detected lanes are accurately mapped to the robot’s perspective:

~~~
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
~~~

These steps activate intrinsic and extrinsic calibration to correct any distortions in the camera feed.

Finally, launch the lane detection node in calibration mode to begin detecting lanes:

~~~
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py calibration_mode:=True
~~~

### Visualizing Lane Detection Output
To inspect the detected lanes, open rqt on Remote PC
~~~
rqt
~~~

Then navigate to Plugins > Visualization > Image View and open three image viewers to display different lane detection results:

* /detect/image_lane/compressed, | /detect/image_yellow_lane_marker/compressed : a yellow range color filtered image. | /detect/image_white_lane_marker/compressed : a white range color filtered image.
######image#################

Navigate to Plugins > Configuration > Dynamic Reconfigure > Select the lane Detect and adjust yellow and white line.

####################image###########################


### Calibrating Lane Detection Parameters

For optimal accuracy, tuning detection parameters is necessary. Adjusting these parameters ensures the robot properly identifies lanes under different lighting and environmental conditions.

1. Open the lane.yaml file located in turtlebot3_autorace_detect/param/lane/ and write your modified values to this file. This will ensure the camera uses the modified parameters for future launches. 
~~~
cd ~/turtlebot3_ws/src/turtlebot3_autorace/turtlebot3_autorace_detect/param/lane
gedit lane.yaml
~~~

### Running Lane Tracking
Once calibration is complete, restart the lane detection node without the calibration option:
~~~
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py
~~~

### Precaution
Make Sure Yellow line on Left side and White Line on right otherwise this code will be crash.

## Step 5: Control Node
Final step, to run the robot that follow the lane.

[TurtleBot3 SBC]
~~~
ros2 run camera_ros camera_node --ros-args -p format:='RGB888'
~~~

[Remote PC]

Terminal 1:
~~~
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
~~~

Terminal 2:
~~~
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
~~~

Terminal 3:
~~~
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py
~~~

Terminal 4:
~~~
ros2 launch turtlebot3_autorace_mission control_lane.launch.py
~~~

# Part 2: Mapping

## Step 1: Get Offsets:
This step is very important, First measure the closed line distance manually in meters.

################# image to measure by scale ########################

### Mathematical calculation to calculate the Lane Offsets:
#### 1. Pixel Offset of Each Lane Line
The horizontal offset of a detected lane line from the robot’s camera center is computed from the centroid position of the segmented line.
If the image width is W, the center is Cx=W/2, and the detected lane line is at pixel coordinate x:
* Left (yellow) line offset

$$
Δxleft​=Cx​−xyellow​
$$

* Right (white) line offset

$$
Δxright​=xwhite​−Cx
$$

This yields positive offsets for both sides.

#### 2. Conversion From Pixels to Meters
During manual calibration, the robot is placed at a known physical distance D from the lane line.
Given the measured pixel offset Δx, the scale factor (meters per pixel) is:

$$
s=meters per pixel= D / Δx
$$

Once the scale s is known, all pixel offsets are converted to meters:

$$
dleft​=s⋅Δxleft​,   dright​=s⋅Δxright​
$$

#### 3. Lane Width Estimation

The two lane boundaries are assumed to be parallel, so the lane width is simply the sum of the left and right distances:

$$
Wlane = dleft + dright
$$

This represents the physical distance between the yellow and white lane lines.

#### 4. Robot Lateral Offset From Lane Center

If the robot is not perfectly centered inside the lane, its lateral shift relative to the lane’s geometric center is:

$$
center offset = (dright​−dleft)/2​​
$$

A positive value means the robot is shifted left; a negative value means it is shifted right.


### How to run the code:

First clone the "________________" package. 

First run the gazebo world:
[TurtleBot3 SBC]
~~~
ros2 run camera_ros camera_node --ros-args -p format:='RGB888'
~~~

[Remote PC]

Then run Intrinsic calibration and Extrinsic calibration launch file and make sure you got the projected image topic as we seen in Extrinsic calibration.
~~~
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
~~~

Then run you just know the aprrox. distance (in meters) of any closed line to the robot, and then run
~~~
ros2 run mapping_lane_tracking lane_offset_fusion --calibration-method manual --known-distance 0.13
~~~

Then Press enter and your calibration will start

<img width="1062" height="696" alt="image" src="https://github.com/user-attachments/assets/1b63ea19-5898-4fe5-811f-ae1ffefb0824" />

If you think your measurment is not correct, enter "y" and move the robot little bit and again press enter:

<img width="969" height="649" alt="image" src="https://github.com/user-attachments/assets/637aa682-c338-4e77-8df8-898c8b459424" />

After this you will get left and right distance of the lane you can see this in image:

<img width="1001" height="665" alt="image" src="https://github.com/user-attachments/assets/3aa4918b-4bc5-4967-a5fe-9d27a4f6ebb1" />


## Step 2: Run SLAM Node:
We are runing slam node because we need the map coordinate, so later we can convert this map into our virtual map by using the offsets of lane.

[TurtleBot3 SBC]

~~~
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
~~~

[Remote PC]
~~~
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
~~~

Automatically Rviz is open and you will see this:

<img width="1058" height="672" alt="image" src="https://github.com/user-attachments/assets/42841413-e0f1-497a-bfef-67a1b7b363c3" />

After runing this just save the rough map, because later we need the Resoulation of map to convert the robot pose coordinates into the map

[Remote PC]
~~~
ros2 run nav2_map_server map_saver_cli -f ~/map
~~~


## Step 3: Run Virtual Map Builder:

### Mathematics and Logic Behind

#### 1. Transforming Robot Pose into Map Coordinates


The robot’s position (x,y,θ) is obtained from the TF transform map → base_link.
To compute yaw from quaternion q=(x,y,z,w), the standard equation is used:

$$
θ = atan2(2(wz+xy),1−2(y2+z2))
$$

This gives the robot’s heading angle in the 2D map.

#### 2. Computing Left and Right Corridor Boundary Points

The corridor boundaries are offset from the robot by fixed distances 
L (left) and R (right), perpendicular to the heading.


A perpendicular direction to heading θ is θ ± π/2.
Thus, the world-coordinate positions of the left and right boundary points are:

$$
\begin{aligned}
x_L &= x + L \cos\left(\theta + \frac{\pi}{2}\right) \\
y_L &= y + L \sin\left(\theta + \frac{\pi}{2}\right) \\
x_R &= x + R \cos\left(\theta - \frac{\pi}{2}\right) \\
y_R &= y + R \sin\left(\theta - \frac{\pi}{2}\right)
\end{aligned}
$$

These points trace the corridor walls as the robot moves.

#### 3. Converting World Coordinates to Map Pixel Coordinates

The occupancy grid map uses pixel coordinates with the origin at the top-left, while ROS2 uses a metric coordinate frame (origin at map origin).
Using map resolution r (meters per pixel) and origin (x0,y0):

$$
\begin{aligned}
p_x &= \frac{x - x_0}{r}, \\
p_y &= \mathrm{height} - 1 - \frac{y - y_0}{r}
\end{aligned}
$$

The Y-axis is flipped because image coordinates increase downward.

#### 4. Forming the Virtual Corridor Polygon

The collected left boundary points and the reversed right boundary points form a closed polygon:

$$
\begin{aligned}
P=[L1​,L2​,…,Ln​,Rn​,Rn−1​,…,R1​]
\end{aligned}
$$

This polygon represents the drivable corridor around the robot.

#### 5. Filling Corridor Interior (Image Space)

OpenCV’s polygon fill marks the interior region as free (white) and the exterior as occupied (black):

$$
\text{canvas}(x, y) = 
\begin{cases}
255 \text{ (white)}, & \text{if } (x, y) \in P \\
0 \text{ (black)}, & \text{otherwise}
\end{cases}
$$

An optional dilation enlarges the corridor by a structuring kernel (navigation safety margin).

#### 6. Converting Image Map to ROS OccupancyGrid

ROS2 uses values:

* 0 = free

* 100 = occupied

Thus the conversion is:

**Image to Occupancy Grid Mapping:**

$$
\begin{aligned}
\text{canvas}(x, y) &= 
\begin{cases}
255, & (x, y) \in P \\
0, & (x, y) \notin P
\end{cases} \\
\text{occ}(x, y) &= 
\begin{cases}
0, & \text{canvas}(x, y) = 255 \\
100, & \text{canvas}(x, y) = 0
\end{cases}
\end{aligned}
$$

The image is vertically flipped before publishing, aligning image coordinates with ROS map coordinates.

#### 7. Saving the Final Virtual Map

PGM format uses:

* 254 = free,

* 0 = occupied.

Thus:

**Complete Image to PGM Pipeline:**

$$
\begin{aligned}
\text{canvas}(x, y) &= 
\begin{cases}
255, & (x, y) \in P \\
0, & (x, y) \notin P
\end{cases} \\
\text{occ}(x, y) &= 
\begin{cases}
0, & \text{canvas}(x, y) = 255 \\
100, & \text{canvas}(x, y) = 0
\end{cases} \\
\text{pgm}(x, y) &= 
\begin{cases}
254, & \text{canvas}(x, y) = 255 \\
0, & \text{canvas}(x, y) = 0
\end{cases}
\end{aligned}
$$

The YAML file stores resolution, origin, and thresholds for Nav2 map loading.

### How to Run the code:

Run the Following commands on each Terminal:
[TurtleBot3 SBC]

~~~
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
~~~
[TurtleBot3 SBC]

~~~
ros2 run camera_ros camera_node --ros-args -p format:='RGB888'
~~~

[Remote PC]

Terminal 1:
~~~
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
~~~

Terminal 2:
~~~
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
~~~

Terminal 3:
~~~
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py
~~~

Terminal 4:
~~~
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
~~~

Change the Left and Right offset that we have calculate in get lane offset In the virtual_map_builder script.

Then Run

Terminal 5:
~~~
ros2 run mapping_lane_tracking virtual_map_builder
~~~

In Rviz Windown Change the Map topic to /virtual_map:

<img width="1118" height="765" alt="image" src="https://github.com/user-attachments/assets/ee76b199-27fe-424d-a115-94b52a10e31a" />

Terminal 6:
~~~
ros2 launch turtlebot3_autorace_mission control_lane.launch.py
~~~

[Screencast from 11-29-2025 11:47:22 PM.webm](https://github.com/user-attachments/assets/b70b8810-495f-49f5-8cd4-aa0e795dfe30)

When robot explore the full map then run the command to save the map:

Terminal 7:
~~~
ros2 run nav2_map_server map_saver_cli -f ~/my_virtual_map -t /virtual_map
~~~

<img width="613" height="510" alt="image" src="https://github.com/user-attachments/assets/b6e0530a-0549-4053-b5c1-9b2083350b55" />

After saved the map you got two files one is my_virtual_map.pgm file and other is my_virtual_map.yaml 

# Part 3: Navigation

In this Virtual map Normal parameters of Turtelbot3_navigation2 is not work very well so for this we should update the burger_cam.yaml file:

which is located:
 
 ~~~
 ~/ros2_ws/src/turtlebot3/turtlebot3_navigation2/param/humble
 ~~~

reduce the robot size:
~~~
local_costmap:
robot_radius: 0.05
inflation_radius: 0.5
global_costmap:
robot_radius: 0.05
inflation_radius: 0.1
~~~

Then run:

~~~
cd ~/turtlebot3_ws && colcon build --symlink-install
~~~

After Colcon build Open gazebo and load your map.

Terminal 1:

~~~
ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py
~~~


Terminal 2:

~~~
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/my_virtual_map.yaml
~~~

First Estimate the pose:

<img width="940" height="817" alt="image" src="https://github.com/user-attachments/assets/da35dde4-d559-4c82-b238-60c225c5fdd6" />

Then send the goal using Nav2 Goal:


[Screencast from 11-30-2025 12:56:47 AM.webm](https://github.com/user-attachments/assets/2769e0e0-2bd6-4c64-85d7-7640775a8cc2)

## Future Work
In future work, a key improvement is to increase the reliability of navigation, as the robot occasionally drifts outside the intended driving corridor. A promising solution is to replace the standard global costmap with a lane-corridor mask generated directly from the lane-detection system. By converting the detected lane boundaries into an occupancy mask—where all pixels outside the lane are marked as obstacles (100) and all pixels inside the lane are marked as free space (0)—Nav2 will be constrained to plan only within the lane. This approach effectively transforms the TurtleBot3 into a lane-aware autonomous robot, similar to the behavior of real self-driving cars, and can significantly improve path-planning accuracy and stability.





