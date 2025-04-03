# Map Waypoint Follower

## Overview
`map_waypoint_follower` is a ROS 2 package that enables a robot to follow waypoints **beyond the local map** using the **Nav2 Stack**. Since the global map might not cover the entire operational area, this package dynamically generates intermediate goal points within the currently mapped region. The robot iteratively expands the map and moves toward the final destination while ensuring smooth navigation.

## Features
✅ Uses the **Nav2 Stack** for autonomous navigation
✅ Dynamically generates intermediate waypoints to expand navigation range
✅ Supports **SLAM-based mapping** to progressively build the map
✅ Works with **various robots** supporting ROS 2 and Nav2
✅ Compatible with **GPS or other localization techniques**

## Installation
### Prerequisites
Ensure you have the following installed:
- ROS 2 (Humble or later recommended)
- Navigation2 (`nav2_bringup`)
- SLAM Toolbox (if using mapping)
- Python3

### Dependencies
Install the required dependencies:
```bash
pip install setuptools==58.2.0 opencv-contrib-python==4.6.0.66 pyqt5 pyqt5-tools subprocesses
```

### Clone and Build
```bash
cd ~/ros2_ws/src  # Navigate to your ROS 2 workspace
git clone https://github.com/ihtroop/map_waypoint_follower.git
cd ~/ros2_ws
colcon build --packages-select map_waypoint_follower
source install/setup.bash
```

## Usage
### Launching the Waypoint Follower
```bash
ros2 launch map_waypoint_follower waypoint_follower.launch.py
```
This will start the navigation system and allow the robot to follow dynamically generated waypoints.

### Setting a Goal Outside the Current Map
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{...}'
```
- The system will generate **intermediate goals** and incrementally expand the mapped region until the robot reaches the final destination.

## Autonomous Traversal
### Building and Running
1. Navigate to the **controls package**:
   ```bash
   cd ~/ros2_ws/src/map_waypoint_follower/controls
   ```
2. Run the Python GUI script:
   ```bash
   python3 1_gui_1.py
   ```
3. Build the package using **colcon**:
   ```bash
   colcon build --packages-select gui
   ```
4. Start the simulation environment:
   ```bash
   ros2 launch gazebo_ros gazebo.launch.py
   ros2 launch nav2_bringup bringup_launch.py
   ros2 launch slam_toolbox online_async_launch.py
   ros2 launch rviz2 rviz2.launch.py
   ```
5. Run **user_ip** to input x, y, yaw values for Nav2 Stack:
   ```bash
   ros2 run map_waypoint_follower user_ip
   ```
6. Run **interm_goal** to compute intermediate goals based on the current map:
   ```bash
   ros2 run map_waypoint_follower interm_goal
   ```
7. Run **final_goal** to interface the above nodes with Nav2 Stack:
   ```bash
   ros2 run map_waypoint_follower final_goal
   ```

## Configuration
You can modify the behavior by tuning the parameters in:
```bash
config/waypoint_follower.yaml
```
Key parameters include:
- **`waypoint_spacing`**: Distance between intermediate waypoints
- **`max_exploration_range`**: Maximum distance the robot can navigate outside the known map
- **`relocalization_strategy`**: Whether to use GPS, AMCL, or other localization methods

## Future Improvements
🚀 **Optimize path planning** for smoother waypoint transitions  
🚀 **Improve dynamic mapping** for better obstacle handling  
🚀 **Integrate advanced localization techniques** (e.g., sensor fusion with IMU & GPS)


