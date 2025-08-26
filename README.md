---

# 🌱 Vine Robot for Disaster Response (ROS2 Workspace)

This repository contains the ROS2 workspace for a **vine robot integrated with a tracked tank base**, designed for **disaster response and search operations**.
The robot combines **locomotion, SLAM, teleoperation, vine deployment, and air compressor control** into a modular ROS2 system.

---

## 📂 Workspace Structure

```
ros2_ws/
│── src/
│   ├── vine_control/              # Controls vine extension and steering
│   ├── disaster_bot_bringup/      # Launch files for starting the robot system
│   ├── motor_control/             # Locomotion control for tracked wheels
│   ├── air_compressor_control/    # Handles compressor and airflow system
│   ├── teleop_control/            # Remote control interface
│   ├── disaster_bot_description/  # URDF/Xacro and robot model description
│   └── slam_control/              # SLAM integration with LiDAR
│── install/
│── build/
│── log/
```

---

## ⚙️ Requirements

* **ROS2 Humble / Iron** (tested on Humble)
* **Python 3.10+**
* `colcon` build system
* `rviz2`, `gazebo` (for simulation and visualization)
* Hardware:

  * Raspberry Pi (main compute)
  * Lidar sensor
  * DC motors + motor driver
  * Air compressor & solenoid valves
  * Servo motors for vine steering

---

## 🚀 Build Instructions

Clone the repository and build:

```bash
# Clone repo
git clone https://github.com/<your-username>/<your-repo>.git
cd ros2_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --symlink-install

# Source overlay
source install/setup.bash
```

---

## ▶️ Running the System

### 1. Bringup the robot

```bash
ros2 launch disaster_bot_bringup bringup.launch.py
```

### 2. Teleoperation

```bash
ros2 run teleop_control teleop_node
```

### 3. Motor control (manual run)

```bash
ros2 run motor_control motor_node
```

### 4. Vine mechanism control

```bash
ros2 run vine_control vine_node
```

### 5. Air compressor control

```bash
ros2 run air_compressor_control compressor_node
```

### 6. SLAM (LiDAR-based navigation)

```bash
ros2 launch slam_control slam.launch.py
```

### 7. Visualization

```bash
rviz2 -d src/disaster_bot_description/rviz/disaster_bot.rviz
```

---

## 📦 Package Details

### **1. `disaster_bot_description`**

* Contains the **URDF/Xacro** models of the robot.
* Defines tracked base, vine mechanism, and LiDAR mount.
* Used in RViz/Gazebo simulations.

### **2. `disaster_bot_bringup`**

* Central launch package.
* Starts motor control, teleop, vine, compressor, and SLAM together.
* Ensures correct namespace and parameter configs.

### **3. `motor_control`**

* Controls the DC motors driving the tracked wheels.
* Uses velocity commands (`geometry_msgs/Twist`).
* Supports joystick/keyboard input from teleop.

### **4. `teleop_control`**

* Remote control package.
* Allows manual robot control via keyboard or joystick.
* Publishes velocity commands to `cmd_vel`.

### **5. `vine_control`**

* Manages **vine extension and steering**.
* Interfaces with servos controlling vine spool and direction.
* Supports incremental extension (forward/backward) and left/right steering.

### **6. `air_compressor_control`**

* Interfaces with **air compressor + solenoid valves**.
* Controls airflow into vine for extension.
* Ensures safe operation with start/stop commands.

### **7. `slam_control`**

* Integrates **LiDAR-based SLAM** for mapping and navigation.
* Launch file runs mapping node + RViz configuration.
* Outputs occupancy grid maps of disaster zones.

---

## 📊 Future Improvements

* Add **autonomous navigation** with Nav2.
* Integrate onboard **camera feeds** for operator feedback.
* Enhance simulation in Gazebo with physics-based vine model.

---

## 👤 Author

Developed by **Rish** for disaster response robotics research.

---
