# IE4060 — safer Nav2 tuning + optional wider maze (ROS 2 Jazzy)

This is a **patch** for your already installed `IE4060-TurtleBot3-Autonomous-Navigation` project. It **does not replace** your original assignment world, maps, DQN code or configuration. It adds `scripts/optimize_navigation.py`, which creates new config and (optionally) 0.75 m-grid world/map files.

## 1. Apply the patch ONCE

Download `IE4060_Nav2_Optimization_Patch.zip` in your **Ubuntu VM**. Open a terminal:

```bash
cd ~/ie4060_ws/src/IE4060-TurtleBot3-Autonomous-Navigation
unzip -o ~/Downloads/IE4060_Nav2_Optimization_Patch.zip -d .
python3 scripts/optimize_navigation.py
cd ~/ie4060_ws
colcon build --symlink-install --packages-select ie4060_maze
```

If `optimize_navigation.py` says `nav2_params.yaml is incomplete`, run the earlier installation/config-generation step first:

```bash
cd ~/ie4060_ws/src/IE4060-TurtleBot3-Autonomous-Navigation
python3 scripts/prepare_configs.py --initial-pose auto
python3 scripts/optimize_navigation.py
cd ~/ie4060_ws
colcon build --symlink-install --packages-select ie4060_maze
```

The patch generates:

- `config/nav2_params_optimized.yaml`: 0.5 m **assignment** maze, 0.16 m inflation, 0.15 m/s forward speed, 1.0 rad/s max rotation.
- `config/nav2_params_roomy.yaml`: optional 0.75 m maze, 0.20 m inflation, 0.18 m/s forward speed, 1.1 rad/s max rotation.
- `worlds/maze_roomy.world`: scaled maze walls with **same layout**, wider physical passages, robot *not* scaled.
- `maps/maze_roomy.yaml` and `maps/maze_roomy.pgm`: geometry-derived reference map matching the widened world.

Both configurations use the installed ROS 2 Jazzy Burger config as a base, preserve the robot collision radius of 0.11 m, use NavFn with A*, and reduce DWB controller frequency to 10 Hz (lower CPU load in VMware).

## 2. Option A (recommended for ASSIGNMENT): Original 0.5 m maze, optimized navigation

Close old simulation/Nav2/SLAM/RViz processes before starting. Do not launch SLAM Toolbox and AMCL navigation at the same time.

**Terminal 1 — Gazebo**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ie4060_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch ie4060_maze simulation.launch.py
```

**Terminal 2 — Nav2 and RViz2**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ie4060_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch ie4060_maze navigation.launch.py \
  params_file:=$HOME/ie4060_ws/src/IE4060-TurtleBot3-Autonomous-Navigation/config/nav2_params_optimized.yaml
```

This keeps the **original 0.5m x 0.5m grid cell geometry**, which matches Figure 1 in your assignment.

## 3. Option B (OPTIONAL EXPERIMENT): Wider 0.75 m corridors

**WARNING:** This changes the maze dimensions to 4.5m x 4.5m. It preserves the wall layout but no longer matches the 0.5 m dimensions on the assignment figure. Keep Option A for the formally specified environment; label Option B as a scaled experiment in the report.

**Terminal 1 — Gazebo (WIDENED WORLD)**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ie4060_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch ie4060_maze simulation.launch.py \
  world:=$HOME/ie4060_ws/src/IE4060-TurtleBot3-Autonomous-Navigation/worlds/maze_roomy.world \
  x_pose:=-1.875 y_pose:=1.875
```

**Terminal 2 — Nav2 (MATCHING WIDENED REFERENCE MAP + INITIAL POSE)**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ie4060_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch ie4060_maze navigation.launch.py \
  map:=$HOME/ie4060_ws/src/IE4060-TurtleBot3-Autonomous-Navigation/maps/maze_roomy.yaml \
  params_file:=$HOME/ie4060_ws/src/IE4060-TurtleBot3-Autonomous-Navigation/config/nav2_params_roomy.yaml
```

For either mode, test the following in **Terminal 3**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ie4060_ws/install/setup.bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /amcl
```

All should be `active [3]`. If needed, set **2D Pose Estimate** in RViz2 to match the robot's actual position and heading. The initial pose in the config is correct *only* for the exact matching spawn position and map.

**Verify that parameters are in effect**:

```bash
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /planner_server GridBased.use_astar
```

For Option A you should see `0.16`, `0.16`, `0.15`, and `True`. For Option B: `0.20`, `0.20`, `0.18`, `True`.

## 4. RViz2 — make the free space easy to see

In RViz2, maximize the window, scroll to zoom in on the map, and optionally hide the **Global Costmap** and **Local Costmap** *displays*, and the LaserScan display, while leaving **Map** enabled. This hides pink/cyan cost overlays; it **does not disable collision avoidance**. Nav2 continues using its costmaps internally. For a goal, click inside a white/free corridor (ideally near its center), not on dark walls or in unknown space. Start with a close goal before longer multi-turn routes.

## 5. Diagnosing an aborted route

If Nav2 shows `aborted` (as it did after 21 recoveries in your screenshot), check in Gazebo and RViz2 whether the robot's pose and laser scans overlap the map's walls. A wrong initial pose, inconsistent world+map, or physical collision can cause aborts regardless of speed or inflation settings. Do not attempt to "fix" a true collision by decreasing the robot radius below its real size.

Check the navigation launch terminal for `No valid trajectories`, `Failed to make progress`, `No path`, `Robot is out of bounds`, `Collision`, or TF errors. **The patch is a starting tune, not a guarantee of successful navigation.** If speed causes oscillation, reduce `FollowPath.max_vel_x` back to 0.12 and retest. If the planner cannot find a safe path, investigate goal location/map validity first.

## 6. Compare efficiency fairly

For actual Gazebo results you can log Nav2 outcomes, simulated time, and traveled odometry distance:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ie4060_ws/install/setup.bash
ros2 run ie4060_maze goal_metrics --ros-args -p use_sim_time:=true
```

This writes to `~/ie4060_ws/results/nav2_runs.csv`. Test the same goals and comparable starting poses in each variant, record collisions separately, and include success/abort outcomes. Note: the optional maze is 1.5× larger, so shorter travel time is not guaranteed despite the higher speed.

The included `python3 -m dqn.evaluate` remains an **offline grid-based test**, not a live Gazebo TurtleBot3 controller. The new widened experiment should not be compared as though its DQN grid distances represent Gazebo trajectories.
