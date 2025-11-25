
## 🚀 Quick Start — Current State (Jazzy)

## Dependnecies
* ros jassy
```bash
cd robotic-arm-new
sudo apt install python3-rosdep
sudo rosdep update
sudo rosdep init
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-jazzy-joint-trajectory-controller
```

**1) Build once**
```bash
cd ~/Downloads/robotics_final
source /opt/ros/jazzy/setup.bash
colcon build
```

**2) Terminal layout (each terminal must source Jazzy + the workspace)**

- **Terminal 1 – Gazebo + robot description**
  ```bash
  cd ~/Downloads/robotics_final
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch panda_description gazebo.launch.py
  ```

- **Terminal 2 – ros2_control controllers**
  ```bash
  cd ~/Downloads/robotics_final
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch panda_controller controller.launch.py
  ```

- **Terminal 3 – MoveIt planning**
  ```bash
  cd ~/Downloads/robotics_final
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch panda_moveit moveit.launch.py
  ```

If you only want planning without controllers/Gazebo, you can run MoveIt with `fake_execution:=true` instead of Terminal 1+2.

---
