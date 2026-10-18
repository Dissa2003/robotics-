#!/usr/bin/env bash
# Run in a sourced ROS terminal while Gazebo and Nav2 are running.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/ie4060_ws/install/setup.bash"
printf '\n== Mapping + localization ==\n'
ros2 topic info /scan || true
ros2 topic info /map || true
ros2 topic info /initialpose || true
printf '\n== Nav2 lifecycle ==\n'
for n in amcl map_server controller_server planner_server bt_navigator behavior_server velocity_smoother; do
  printf '%s: ' "$n"
  timeout 4s ros2 lifecycle get "/$n" || true
done
printf '\n== TF map->base_link (may print one initial warning) ==\n'
timeout 6s ros2 run tf2_ros tf2_echo map base_link || true
printf '\n== Navigation action ==\n'
ros2 action list -t | grep navigate_to_pose || true
printf '\n== Velocity topic ==\n'
ros2 topic info /cmd_vel -v || true
