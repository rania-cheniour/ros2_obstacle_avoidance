# 🤖 ROS 2 Obstacle Avoidance Simulator

A simple, fast, and reliable ROS 2 (Humble) package that simulates a robot performing **basic navigation with reactive obstacle avoidance** using simulated LIDAR data.

[![Demo Video]([https://img.youtube.com/vi/YOUR_VIDEO_ID/0.jpg)](https://drive.google.com/file/d/1jM2jWAsLHMVZnFVbEk-NEk04mSWwwSrh/view?usp=sharing)]([https://youtu.be/YOUR_VIDEO_ID](https://drive.google.com/file/d/1jM2jWAsLHMVZnFVbEk-NEk04mSWwwSrh/view?usp=sharing))  
▶️ **Watch the 2-minute demo**: [[ROS 2 Obstacle Avoidance](https://drive.google.com/file/d/1jM2jWAsLHMVZnFVbEk-NEk04mSWwwSrh/view?usp=sharing)]

---

## ✅ Features

- Moves forward at **0.7 m/s** in clear space  
- **Turns right** when an obstacle is detected within **1.0 meters**  
- **Filters out false detections** (self-collisions, noise, invalid LIDAR readings)  
- **Logs clear status messages** (`MOVING`, `TURNING`) — extra feature  
- Works out-of-the-box with **TurtleBot3 Gazebo simulation**

---

## 🛠️ Setup Instructions

> Tested on **Ubuntu 22.04 + ROS 2 Humble**

### 1. Create & Build Workspace
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/ros2_obstacle_ws/src
cd ~/ros2_obstacle_ws/src

# Clone this repository
git clone https://github.com/rania-cheniour/ros2_obstacle_avoidance.git

# Install dependencies
cd ..
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select ros2_obstacle_avoidance

# Source the workspace
source install/setup.bash

### 2. Run obstacle avoidance node
# Source your workspace
source ~/ros2_obstacle_ws/install/setup.bash

# Launch the avoider
ros2 launch ros2_obstacle_avoidance avoidance.launch.py
