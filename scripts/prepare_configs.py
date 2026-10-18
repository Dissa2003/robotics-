#!/usr/bin/env python3
"""Copy distro-compatible TurtleBot3/Nav2 + slam_toolbox defaults, then tune our maze.
Run AFTER apt installation, BEFORE colcon build; no Internet used by this script.
"""
from pathlib import Path
import argparse
import os
import yaml

PROJECT = Path(__file__).resolve().parents[1]


def require(path):
    if not path.is_file():
        raise SystemExit(f'Missing file: {path}\nInstall the corresponding ros-jazzy-* package first.')
    return path


def set_value(dict_obj, key, value):
    if not isinstance(dict_obj, dict):
        raise ValueError(f'Expected mapping when setting {key}')
    dict_obj[key] = value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--initial-pose', choices=['auto', 'rviz'], default='auto',
                        help='auto uses reference-world spawn (-1.25,+1.25,0); rviz requires 2D Pose Estimate')
    args = parser.parse_args()
    prefix = Path(os.environ.get('ROS_PREFIX', '/opt/ros/jazzy'))
    stock = require(prefix / 'share/turtlebot3_navigation2/param/burger.yaml')
    with stock.open() as f:
        nav = yaml.safe_load(f)

    def node(name):
        return nav[name]['ros__parameters']

    amcl = node('amcl')
    amcl['set_initial_pose'] = args.initial_pose == 'auto'
    if args.initial_pose == 'auto':
        # Valid only with supplied geometry-derived reference map and default spawn.
        amcl['initial_pose'] = {'x': -1.25, 'y': 1.25, 'z': 0.0, 'yaw': 0.0}
    amcl['tf_broadcast'] = True
    amcl['transform_tolerance'] = 1.0
    amcl['update_min_d'] = 0.05
    amcl['update_min_a'] = 0.1
    amcl['scan_topic'] = '/scan'

    for key in ('local_costmap', 'global_costmap'):
        cm = nav[key][key]['ros__parameters']
        cm['robot_radius'] = 0.11   # Burger approximate collision envelope
        cm['inflation_layer']['inflation_radius'] = 0.22
        cm['inflation_layer']['cost_scaling_factor'] = 8.0
        cm['transform_tolerance'] = 1.0
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    local['width'], local['height'] = 2, 2

    planner = node('planner_server')
    planner['GridBased']['use_astar'] = True
    planner['GridBased']['allow_unknown'] = False
    planner['GridBased']['tolerance'] = 0.10
    planner['expected_planner_frequency'] = 2.0

    ctl = node('controller_server')
    ctl['FollowPath']['max_vel_x'] = 0.12
    ctl['FollowPath']['max_speed_xy'] = 0.12
    ctl['FollowPath']['max_vel_theta'] = 0.8
    ctl['FollowPath']['trans_stopped_velocity'] = 0.05
    ctl['goal_checker']['xy_goal_tolerance'] = 0.12
    ctl['progress_checker']['required_movement_radius'] = 0.05
    ctl['progress_checker']['movement_time_allowance'] = 20.0
    ctl['enable_stamped_cmd_vel'] = True
    for name in ('behavior_server', 'velocity_smoother', 'collision_monitor'):
        if name in nav:
            node(name)['enable_stamped_cmd_vel'] = True
    if 'velocity_smoother' in nav:
        smoother = node('velocity_smoother')
        smoother['max_velocity'] = [0.15, 0.0, 0.9]
        smoother['min_velocity'] = [-0.15, 0.0, -0.9]

    dest = PROJECT / 'config/nav2_params.yaml'
    with dest.open('w') as f:
        f.write('# Generated using installed TurtleBot3 Jazzy burger.yaml. Edit and retest if needed.\n')
        yaml.safe_dump(nav, f, sort_keys=False)
    print('Generated:', dest)

    slam_default = require(prefix / 'share/slam_toolbox/config/mapper_params_online_async.yaml')
    with slam_default.open() as f:
        slam = yaml.safe_load(f)
    slam_params = slam['slam_toolbox']['ros__parameters']
    slam_params['map_frame'] = 'map'
    slam_params['odom_frame'] = 'odom'
    slam_params['base_frame'] = 'base_footprint'
    slam_params['scan_topic'] = '/scan'
    slam_params['minimum_travel_distance'] = 0.10
    slam_params['minimum_travel_heading'] = 0.15
    with (PROJECT / 'config/slam_params.yaml').open('w') as f:
        yaml.safe_dump(slam, f, sort_keys=False)
    print('Generated:', PROJECT / 'config/slam_params.yaml')
    print('Using initial pose:', args.initial_pose)


if __name__ == '__main__':
    main()
