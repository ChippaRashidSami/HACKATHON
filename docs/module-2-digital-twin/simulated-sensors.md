---
sidebar_position: 3
title: "Simulated Sensors: LiDAR, Depth Cameras, and IMUs"
---

# Simulated Sensors: LiDAR, Depth Cameras, and IMUs

## Learning Objectives

After completing this chapter, you will be able to:
- Understand how to simulate LiDAR, depth cameras, and IMUs in Gazebo
- Configure sensor properties to match real sensor specifications
- Publish sensor data to ROS 2 topics from simulation
- Visualize simulated sensor data in RViz
- Understand the differences between simulated and real sensor data

## Introduction

Sensors are critical components of any robotic system, providing the robot with information about its environment and its own state. In humanoid robots, various sensors enable perception, navigation, and interaction. Simulating these sensors accurately in Gazebo is essential for developing and testing perception and navigation algorithms before deployment on real hardware.

This chapter covers the simulation of three key sensor types: LiDAR for 3D environment mapping, depth cameras for scene understanding, and IMUs for orientation and acceleration sensing.

## Sensor Simulation in Gazebo

Gazebo provides realistic sensor simulation through plugins that generate data similar to real sensors. The key principles for accurate sensor simulation are:

1. **Physical Accuracy**: Simulated sensors should have the same characteristics as their real counterparts
2. **Noise Models**: Real sensors have noise and inaccuracies that should be simulated
3. **ROS Integration**: Sensor data should be published to appropriate ROS 2 topics
4. **Performance**: Simulation should run in real-time for interactive development

### Sensor Plugin Architecture

Gazebo uses plugins to simulate sensors. The common architecture includes:

- **Sensor Model**: Defines the type and parameters of the sensor
- **Physics Engine**: Computes what the sensor should detect
- **Noise Model**: Adds realistic noise and imperfections
- **ROS Interface**: Publishes data to ROS topics

## LiDAR Simulation

LiDAR (Light Detection and Ranging) sensors are crucial for environment mapping and navigation. In Gazebo, LiDAR simulation is achieved through ray tracing.

### 2D LiDAR (Laser Scanner)

A typical 2D LiDAR configuration in a URDF/XACRO file:

```xml
<!-- Gazebo plugin for 2D LiDAR -->
<gazebo reference="laser_link">
  <sensor type="ray" name="laser_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -π radians -->
          <max_angle>3.14159</max_angle>   <!-- π radians -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/laser</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>laser_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### 3D LiDAR (GPU Ray)

For more complex 3D LiDAR like Velodyne sensors:

```xml
<gazebo reference="velodyne_link">
  <sensor type="gpu_ray" name="velodyne-VLP-16">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159265359</min_angle>
          <max_angle>3.14159265359</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.26179938779</min_angle>  <!-- -15 degrees -->
          <max_angle>0.26179938779</max_angle>   <!-- 15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_velodyne_gpu_laser.so">
      <ros>
        <namespace>/velodyne</namespace>
        <remapping>~/out:=pointcloud</remapping>
      </ros>
      <frame_name>velodyne_link</frame_name>
      <min_range>0.1</min_range>
      <max_range>100.0</max_range>
      <gaussian_noise>0.008</gaussian_noise>
    </plugin>
  </sensor>
</gazebo>
```

### Noise Models for LiDAR

Real LiDAR sensors have noise that should be simulated:

```xml
<sensor type="ray" name="laser_sensor">
  <!-- ... other configuration ... -->
  <ray>
    <!-- ... other ray config ... -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
    </noise>
  </ray>
</sensor>
```

## Depth Camera Simulation

Depth cameras provide both color images and depth information, which is crucial for scene understanding and 3D reconstruction.

### Depth Camera Configuration

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="camera_sensor">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/camera</namespace>
        <remapping>rgb/image_raw:=image_color</remapping>
        <remapping>depth/image_raw:=image_depth</remapping>
        <remapping>depth/camera_info:=camera_info</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <point_cloud_cutoff>0.1</point_cloud_cutoff>
      <point_cloud_cutoff_max>3.0</point_cloud_cutoff_max>
    </plugin>
  </sensor>
</gazebo>
```

### Point Cloud Generation

Depth cameras often generate point clouds for 3D processing:

```xml
<plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
  <!-- ... other config ... -->
  <point_cloud>true</point_cloud>
  <point_cloud_topic>depth/points</point_cloud_topic>
  <point_cloud_cutoff>0.1</point_cloud_cutoff>
  <point_cloud_cutoff_max>3.0</point_cloud_cutoff_max>
</plugin>
```

## IMU Simulation

IMUs provide critical information about the robot's orientation and acceleration, which is essential for balance and navigation in humanoid robots.

### IMU Configuration

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.02</stddev>
            <bias_mean>0.00001</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.02</stddev>
            <bias_mean>0.00001</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.02</stddev>
            <bias_mean>0.00001</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <body_name>base_link</body_name>
      <update_rate>100</update_rate>
      <gaussian_noise>0.001</gaussian_noise>
    </plugin>
  </sensor>
</gazebo>
```

## Complete Sensor Setup Example

Here's a complete example of adding multiple sensors to a humanoid robot:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="sensor_humanoid">
  
  <!-- Include Gazebo plugins -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.15" length="0.5"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.15" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- IMU link -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Camera link -->
  <link name="camera_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- Laser link (for 2D LiDAR) -->
  <link name="laser_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
    </visual>
  </link>

  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.15 0 -0.15" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo sensor definitions -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.02</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.02</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.02</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/imu</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
        <frame_name>imu_link</frame_name>
        <body_name>base_link</body_name>
        <update_rate>100</update_rate>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor type="depth" name="camera_sensor">
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <ros>
          <namespace>/camera</namespace>
          <remapping>rgb/image_raw:=image_color</remapping>
          <remapping>depth/image_raw:=image_depth</remapping>
          <remapping>depth/camera_info:=camera_info</remapping>
        </ros>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="laser_link">
    <sensor type="ray" name="laser_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-2.35619</min_angle>  <!-- -135 degrees -->
            <max_angle>2.35619</max_angle>   <!-- 135 degrees -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/laser</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Working with Sensor Data in ROS 2

Once sensors are configured in Gazebo, you can access their data in ROS 2:

### Python Example for Processing LiDAR Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LiDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/laser/scan',  # Adjust topic name based on your configuration
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Timer to periodically process data
        self.timer = self.create_timer(0.1, self.process_data)

    def lidar_callback(self, msg):
        """Process incoming LiDAR data"""
        # Store the latest scan
        self.last_scan = msg
        
        # Filter out invalid ranges
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        
        if valid_ranges:
            # Calculate some statistics
            avg_distance = sum(valid_ranges) / len(valid_ranges)
            min_distance = min(valid_ranges)
            
            self.get_logger().info(f'Avg distance: {avg_distance:.2f}m, Min: {min_distance:.2f}m')

    def process_data(self):
        """Periodic processing of sensor data"""
        if hasattr(self, 'last_scan'):
            # Perform processing specific to your application
            self.detect_obstacles(self.last_scan)

    def detect_obstacles(self, scan_msg):
        """Detect obstacles in LiDAR data"""
        # Simple obstacle detection: check for objects within 1m
        obstacle_ranges = [r for r in scan_msg.ranges 
                          if scan_msg.range_min <= r <= 1.0 and not np.isnan(r)]
        
        if len(obstacle_ranges) > 10:  # More than 10 points indicate significant obstacle
            self.get_logger().warn('Obstacle detected within 1m!')

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LiDARProcessor()
    
    try:
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Example for Processing Camera Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.bridge = CvBridge()
        
        # Subscribe to camera image
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_color',  # Adjust topic name based on your configuration
            self.image_callback,
            10)
        
        # Subscribe to camera info for calibration
        self.info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10)

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Example: detect faces using OpenCV
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            faces = face_cascade.detectMultiScale(gray, 1.1, 4)
            
            # Draw rectangles around faces
            for (x, y, w, h) in faces:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            
            # Display the image
            cv2.imshow('Camera Feed with Face Detection', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def info_callback(self, msg):
        """Process camera calibration info"""
        # Store camera matrix and distortion coefficients
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)
        
        # These can be used for image undistortion or 3D reconstruction
        self.get_logger().info('Camera calibration info received')

def main(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    
    try:
        rclpy.spin(camera_processor)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        camera_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Example for Processing IMU Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',  # Adjust topic name based on your configuration
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Store previous orientation for velocity calculation
        self.prev_orientation = None
        self.orientation_history = []

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        # Extract orientation (quaternion)
        orientation = msg.orientation
        # Convert quaternion to Euler angles
        euler = self.quaternion_to_euler(orientation)
        
        # Extract angular velocity
        angular_vel = msg.angular_velocity
        
        # Extract linear acceleration
        linear_acc = msg.linear_acceleration
        
        # Log the data
        self.get_logger().info(
            f'Roll: {math.degrees(euler.x):.2f}°, '
            f'Pitch: {math.degrees(euler.y):.2f}°, '
            f'Yaw: {math.degrees(euler.z):.2f}°'
        )
        
        # Store for history analysis
        self.orientation_history.append(euler)
        if len(self.orientation_history) > 100:
            self.orientation_history.pop(0)

    def quaternion_to_euler(self, quaternion):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        """
        import math
        
        # Extract quaternion components
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # Convert to Euler angles (in radians)
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Create and return Vector3 message
        euler_angles = Vector3()
        euler_angles.x = roll
        euler_angles.y = pitch
        euler_angles.z = yaw
        
        return euler_angles

def main(args=None):
    rclpy.init(args=args)
    imu_processor = IMUProcessor()
    
    try:
        rclpy.spin(imu_processor)
    except KeyboardInterrupt:
        pass
    finally:
        imu_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Visualizing Sensor Data in RViz

RViz provides powerful tools for visualizing sensor data:

### Configuring RViz for Sensor Visualization

1. **LaserScan Display**: Add a LaserScan display and set the topic to your LiDAR scan topic (e.g., `/laser/scan`)
2. **Image Display**: Add an Image display and set the topic to your camera image topic (e.g., `/camera/image_color`)
3. **RobotModel Display**: Visualize the robot model with the sensor positions
4. **TF Display**: Show coordinate frames for sensor orientation
5. **PointCloud Display**: Visualize 3D point cloud data from depth cameras

### RViz Configuration Example

```yaml
# Example RViz configuration snippet
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /LaserScan1
        - /Image1
        - /RobotModel1
      Splitter Ratio: 0.5
    Tree Height: 787
Visualization Manager:
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: LaserScan
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /laser/scan
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Min Value: 0
      Name: Image
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /camera/image_color
    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: RobotModel
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
```

## Sensor Fusion Concepts

For humanoid robots, sensor fusion combines data from multiple sensors to get a more accurate understanding of the environment:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Subscribe to multiple sensors
        self.lidar_sub = self.create_subscription(LaserScan, '/laser/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_color', self.camera_callback, 10)
        
        # Publisher for fused state
        self.odom_pub = self.create_publisher(Odometry, '/fused_odom', 10)
        
        # Initialize state
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.velocity = np.array([0.0, 0.0, 0.0])
        
        # Timer for publishing fused state
        self.timer = self.create_timer(0.05, self.publish_fused_state)  # 20 Hz

    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection and mapping"""
        # Simple obstacle detection
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        if valid_ranges:
            min_range = min(valid_ranges)
            if min_range < 0.5:  # Obstacle within 0.5m
                self.get_logger().warn('Close obstacle detected!')

    def imu_callback(self, msg):
        """Update orientation from IMU"""
        # Update orientation using IMU quaternion
        self.orientation = np.array([
            msg.orientation.x,
            msg.orientation.y, 
            msg.orientation.z,
            msg.orientation.w
        ])
        
        # Integrate angular velocity for orientation
        angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Simple integration (in practice, use proper integration methods)
        dt = 0.01  # Assuming 100Hz IMU
        rotation_vec = angular_vel * dt
        rotation_quat = self.rotation_vector_to_quaternion(rotation_vec)
        self.orientation = self.quat_multiply(self.orientation, rotation_quat)

    def camera_callback(self, msg):
        """Process camera data for visual features"""
        # In a real implementation, you'd extract visual features here
        # For now, just acknowledge the data
        pass

    def rotation_vector_to_quaternion(self, rotation_vec):
        """Convert rotation vector to quaternion"""
        angle = np.linalg.norm(rotation_vec)
        if angle == 0:
            return np.array([0, 0, 0, 1])
        
        axis = rotation_vec / angle
        half_angle = angle / 2
        sin_half = np.sin(half_angle)
        
        return np.array([
            axis[0] * sin_half,
            axis[1] * sin_half, 
            axis[2] * sin_half,
            np.cos(half_angle)
        ])

    def quat_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return np.array([w, x, y, z])

    def publish_fused_state(self):
        """Publish combined state estimate"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Fill position
        msg.pose.pose.position.x = float(self.position[0])
        msg.pose.pose.position.y = float(self.position[1])
        msg.pose.pose.position.z = float(self.position[2])
        
        # Fill orientation
        msg.pose.pose.orientation.x = float(self.orientation[0])
        msg.pose.pose.orientation.y = float(self.orientation[1])
        msg.pose.pose.orientation.z = float(self.orientation[2])
        msg.pose.pose.orientation.w = float(self.orientation[3])
        
        # Fill velocity
        msg.twist.twist.linear.x = float(self.velocity[0])
        msg.twist.twist.linear.y = float(self.velocity[1])
        msg.twist.twist.linear.z = float(self.velocity[2])
        
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion = SensorFusionNode()
    
    try:
        rclpy.spin(sensor_fusion)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Diagram: Sensor Integration Architecture

```
                Sensor Integration Architecture
                            
    [LiDAR] → [Gazebo Plugin] → [ROS 2 Topic: /laser/scan] → [Perception Node]
       ↑           ↑                     ↑                         ↑
    [Real]    [Simulation]          [Communication]           [Processing]

    [Camera] → [Gazebo Plugin] → [ROS 2 Topic: /camera/image] → [Vision Node]
       ↑           ↑                     ↑                        ↑
    [Real]    [Simulation]          [Communication]          [Processing]

    [IMU] → [Gazebo Plugin] → [ROS 2 Topic: /imu/data] → [State Estimation]
       ↑          ↑                     ↑                      ↑
    [Real]   [Simulation]          [Communication]         [Processing]

                           ↓
                    [Sensor Fusion]
                           ↓
                    [Combined State]
                           ↓
                   [Robot Control]
```

## Performance Considerations

Simulating multiple sensors in Gazebo can be computationally intensive:

### Optimization Strategies

1. **Reduce Update Rates**: Lower sensor update rates where precision allows
2. **Simplify Models**: Use simpler meshes for collision detection
3. **Limit Range**: Set appropriate sensor ranges to avoid unnecessary computation
4. **Selective Visualization**: Disable sensor visualization when not needed
5. **Efficient Point Clouds**: Use appropriate point cloud densities

```xml
<!-- Example: Optimized sensor configuration -->
<sensor type="ray" name="laser_sensor">
  <update_rate>5</update_rate>  <!-- Lower rate for less critical sensors -->
  <!-- ... other config ... -->
</sensor>
```

## Common Issues and Troubleshooting

### Sensor Data Issues
- **No Data**: Check topic names and ensure Gazebo plugins are loaded
- **Delayed Data**: Verify update rates and network configurations
- **Inaccurate Data**: Check noise parameters and physics settings

### Performance Issues
- **Slow Simulation**: Reduce sensor complexity or update rates
- **High CPU Usage**: Optimize collision meshes and physics parameters

### RViz Visualization Issues
- **Missing Data**: Check topic connections in RViz
- **Wrong Frame**: Verify TF tree and frame names

## Summary

Simulated sensors in Gazebo provide realistic data for developing and testing humanoid robot perception systems. Key aspects include:

- Configuring sensors with realistic parameters and noise models
- Connecting sensors to ROS 2 topics for processing
- Integrating multiple sensors through sensor fusion
- Visualizing data in RViz for debugging and validation
- Optimizing performance for real-time simulation

Proper sensor simulation is crucial for developing robust perception and navigation systems that can transfer effectively from simulation to real robot hardware.

## Exercises

### Exercise 1: LiDAR Configuration
Configure a 2D LiDAR in your humanoid robot model with realistic parameters and visualize the data in RViz.

### Exercise 2: Sensor Fusion
Create a simple sensor fusion node that combines IMU and odometry data to improve pose estimation.

### Exercise 3: Camera Processing
Implement a computer vision algorithm that processes camera data from simulation to detect specific objects or features.

## Next Steps

This completes Module 2 on the Digital Twin (Gazebo & Unity). In the next module, we'll explore how to enhance these simulation capabilities with NVIDIA Isaac for AI-powered perception and navigation in humanoid robots.