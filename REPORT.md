# Autonomous Navigation of TurtleBot3 Burger in Gazebo

This document is a report draft for IE4060 Assignment 02. Replace the
bracketed result fields with measurements from the final demonstration before
submitting the report. Do not invent travel times or path lengths.

## 1. Objective

The objective is to navigate a TurtleBot3 Burger through a six-by-six maze in
Gazebo using ROS 2 Jazzy. The robot maps the environment with LiDAR-based
SLAM, localizes against the saved occupancy grid, and navigates to user
selected RViz2 goal poses while avoiding obstacles.

## 2. Maze design

The world in `worlds/my_maze.world` follows Figure 1 of the assignment. It
contains six-by-six 0.5 m cells, a ground plane, a directional light, and
static wall segments with 0.5 m pitch. The walls are 0.08 m thick and 0.4 m
high. The robot is spawned at `(0.0, 0.0, 0.1)`. The internal wall segments
leave connected corridors and the spawn location clear.

The world is started by `launch/maze.launch.py`. This launch file starts
Gazebo, spawns the Burger model, publishes the robot description, and bridges
the Gazebo topics required by ROS 2:

- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/odom` (`nav_msgs/msg/Odometry`)
- `/tf`
- `/joint_states`
- `/clock`

## 3. Mapping and localization

The robot is manually driven through the maze while
`slam_toolbox/online_async_launch.py` builds the occupancy grid. The map is
saved using `nav2_map_server map_saver_cli`, producing `maps/map.yaml` and
`maps/map.pgm`. The saved map is then loaded by Nav2's AMCL localization
component.

Record the final mapping details here:

- Mapping duration: **[      ]**
- Map resolution: **[      ]**
- Mapping issues and corrections: **[      ]**

## 4. Navigation and planning

`launch/nav2.launch.py` includes the Jazzy `nav2_bringup` launch and loads
`config/nav2_params.yaml`. The parameter file is based on the Jazzy Nav2
configuration and uses:

- AMCL for localization.
- A static map layer and LiDAR obstacle layer.
- NavFn planning with the configured planner.
- DWB local control.
- Inflation and collision checking for safe wall clearance.
- Unstamped `geometry_msgs/msg/Twist` output for the TurtleBot3 Burger bridge.

The user sets the robot pose with RViz2 **2D Pose Estimate** and selects a
destination with **2D Goal Pose**. Nav2 then computes and follows a collision
free route.

## 5. Deep Q-learning component

`maze_world/dqn_nav_node.py` is an online DQN demonstration. It compresses the
LiDAR scan into 24 normalized state values and selects one of five discrete
actions: forward, forward-left, forward-right, rotate-left, or rotate-right.
Transitions are stored in replay memory and used to train a policy network
and target network.

The DQN node is a local obstacle-avoidance controller. It is not run at the
same time as Nav2 because both would publish to `/cmd_vel`. Nav2 is the
controller used for the clicked-goal demonstration.

## 6. Results

Measure at least three goal runs after the map and initial pose are stable.
Record the same start and goal coordinates when comparing methods.

| Run | Start | Goal | Travel time (s) | Path length (m) | Collisions | Result |
|---|---|---|---:|---:|---:|---|
| 1 | [ ] | [ ] | [ ] | [ ] | 0 | [success/failure] |
| 2 | [ ] | [ ] | [ ] | [ ] | 0 | [success/failure] |
| 3 | [ ] | [ ] | [ ] | [ ] | 0 | [success/failure] |

Discuss whether the robot reached three different clicked goals, how closely
the route followed the corridor center, and whether replanning occurred.

## 7. Challenges and improvements

Observed challenges may include scan noise, an inaccurate initial AMCL pose,
goals placed too close to inflated obstacles, and velocity-command conflicts
when multiple controllers are running. The current implementation addresses
these with simulation time, an explicit scan bridge, obstacle inflation, and a
clear separation between Nav2 and DQN execution.

Possible improvements include training the DQN in repeated randomized maze
episodes, adding goal direction and odometry to its state, using a learned
local policy as a Nav2 controller plugin, tuning inflation and velocity
limits, and adding automated travel-time/path-length logging.

## 8. Demonstration checklist

- [ ] Build the package from a clean terminal.
- [ ] Launch the maze and verify `/scan`, `/odom`, and `/tf`.
- [ ] Run SLAM and save `map.yaml` and `map.pgm`.
- [ ] Launch Nav2 with the saved map.
- [ ] Set the initial pose in RViz2.
- [ ] Navigate to at least three different clicked goal poses.
- [ ] Record time, path length, and success for each run.
- [ ] Add screenshots of Gazebo, the SLAM map, and three successful goals.
