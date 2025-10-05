# robot_chase

Rick chases Morty in Gazebo using TF transforms and a simple proportional controller for distance and yaw.

**Prerequisites:**  
Requires the [barista_robot_description](https://github.com/legalaspro/barista_robot_description) package for the two robots (/rick and /morty).  
Launch them with:  
```bash
ros2 launch barista_robot_description barista_two_robots.launch.py
```

## Installation
```bash
# Clone into workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/legalaspro/barista_robot_description
git clone https://github.com/legalaspro/robot_chase

# Install deps & build
cd ..
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select barista_robot_description robot_chase
source install/setup.bash
```

## Usage
1. **Launch robots** (Terminal 1):  
   ```bash:disable-run
   ros2 launch barista_robot_description barista_two_robots.launch.py
   ```

2. **Drive Morty** (Terminal 2):  
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel
   ```

3. **Start chaser** (Terminal 3):  
   ```bash
   ros2 run robot_chase robot_chase
   ```  

Rick will publish to `/rick/cmd_vel` and follow Morty.

## What It Does
- Queries TF: `rick/base_link` → `morty/base_link`.  
- Computes: `error_distance = hypot(dx, dy)`, `error_yaw = atan2(dy, dx)` (normalized to [-π, π]).  
- Commands: `linear.x = 0.5 * error_distance`, `angular.z = 1.0 * error_yaw`.  
- Stops gently when < 0.36m (robot diameter + margin).

## Verification
- Echo commands: `ros2 topic echo /rick/cmd_vel`  
- Check TF: `ros2 run tf2_ros tf2_echo rick/base_link morty/base_link`  
- List topics: `ros2 topic list | grep -E '/(rick|morty)/'`