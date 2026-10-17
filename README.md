# `maze_world`

This ROS 2 Jazzy package starts a `ros_gz_sim` maze with a TurtleBot3 Burger,
provides a `slam_toolbox` mapping workflow, and provides a Nav2 launch file for
goal navigation. It also contains a small online PyTorch DQN example for local
scan-only obstacle avoidance.

## 1. Install dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-description \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard python3-numpy
python3 -m pip install --user torch
```

Source ROS and build the package:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select maze_world --symlink-install
source install/setup.bash
```

## 2. Start the maze

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch maze_world maze.launch.py
```

The launch file starts Gazebo, spawns the Burger at `(0.0, 0.0, 0.1)`, starts
`robot_state_publisher`, and bridges `/scan`, `/cmd_vel`, `/odom`, `/tf`, and
the simulation clock. The bridge intentionally uses `geometry_msgs/msg/Twist`
for `/cmd_vel`, which matches the DQN node.

## 3. Build a map

Leave the maze terminal running. In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml
```

In a third terminal, teleoperate the robot and drive every corridor:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

When the map is complete, save it into the package's `maps` directory:

```bash
mkdir -p ~/ros2_ws/src/maze_world/maps
ros2 run nav2_map_server map_saver_cli -f \
  ~/ros2_ws/src/maze_world/maps/map
cd ~/ros2_ws
colcon build --packages-select maze_world --symlink-install
source install/setup.bash
```

This creates `maps/map.yaml` and `maps/map.pgm`. Keep the map saver and SLAM
process running until the command reports that both files were written.

## 4. Navigate with Nav2 and RViz2

Stop the SLAM node (the Gazebo terminal can remain running), then launch Nav2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch maze_world nav2.launch.py
```

Start RViz2 in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Set the fixed frame to `map` if needed. Use **2D Pose Estimate** once to set
the robot pose, then use **2D Goal Pose** to send goals. Nav2 publishes
`Twist` because `config/nav2_params.yaml` disables stamped velocity commands.

To use a map at a different path:

```bash
ros2 launch maze_world nav2.launch.py map:=/absolute/path/to/map.yaml
```

## 5. Run the DQN example

With the maze launch running, run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run maze_world dqn_nav_node.py
```

The node discretizes `/scan` into 24 values, selects from five velocity
actions, stores transitions, and trains a small online DQN with a target
network. It publishes `geometry_msgs/msg/Twist` on `/cmd_vel`.

This is a local scan-only learner: without a goal topic or pose input it cannot
solve arbitrary point-to-point navigation. Use it for the assignment's
obstacle-avoidance demonstration, and use Nav2 for map-based goal planning.
