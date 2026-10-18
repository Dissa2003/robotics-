#!/usr/bin/env python3
"""IE4060 Nav2 tuning and optional roomier geometry-accurate Gazebo maze.

Usage, from project directory after scripts/install_project.sh has succeeded:
  python3 scripts/optimize_navigation.py

Does NOT overwrite the original worlds/maze.world, maps/maze.* or config/nav2_params.yaml.
The widened world is an experiment (0.75m instead of the assignment's 0.5m cells).
"""
import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError as exc:
    raise SystemExit('Missing PyYAML. Run: sudo apt install python3-yaml') from exc

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'config'
WORLD = ROOT / 'worlds'
MAPS = ROOT / 'maps'
FACTOR = 1.5                  # 0.5m grid cells become 0.75m
RESOLUTION = 0.02
ORIGIN = -2.40
WIDTH = HEIGHT = 240


def fmt(values):
    return ' '.join(f'{v:.4f}' for v in values)


def node(config, name):
    return config[name]['ros__parameters']


def write_nav(config, output, *, wide):
    # Keep the distro-matched ROS/Nav2 parameter keys and plugin list.
    cfg = copy.deepcopy(config)
    linear = 0.18 if wide else 0.15
    angular = 1.10 if wide else 1.00
    inflation = 0.20 if wide else 0.16

    amcl = node(cfg, 'amcl')
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {
        'x': -1.875 if wide else -1.25,
        'y': 1.875 if wide else 1.25,
        'z': 0.0,
        'yaw': 0.0,
    }
    amcl['tf_broadcast'] = True
    amcl['scan_topic'] = '/scan'

    for name in ('local_costmap', 'global_costmap'):
        cm = cfg[name][name]['ros__parameters']
        # Keep an honest Burger collision radius; do NOT shrink to fake clearance.
        cm['robot_radius'] = 0.11
        if 'inflation_layer' not in cm or not isinstance(cm['inflation_layer'], dict):
            raise SystemExit(f'Missing inflation_layer in {name} of Nav2 YAML')
        cm['inflation_layer']['inflation_radius'] = inflation
        cm['inflation_layer']['cost_scaling_factor'] = 8.0
    # Do not enlarge rolling costmap unnecessarily on a VMware VM.

    planner = node(cfg, 'planner_server')
    if 'GridBased' not in planner:
        raise SystemExit('Expected planner_server GridBased plugin; inspect original YAML')
    planner['GridBased']['use_astar'] = True
    planner['GridBased']['allow_unknown'] = False
    planner['GridBased']['tolerance'] = 0.12

    ctl = node(cfg, 'controller_server')
    if 'FollowPath' not in ctl:
        raise SystemExit('Expected DWB FollowPath controller; inspect original YAML')
    ctl['controller_frequency'] = 10.0  # lighter than 20Hz on VMware
    fp = ctl['FollowPath']
    fp['max_vel_x'] = linear
    fp['max_speed_xy'] = linear
    fp['max_vel_theta'] = angular
    fp['sim_time'] = 1.2
    fp['vx_samples'] = 16
    fp['vtheta_samples'] = 28
    # Narrow turns need a low forward projection for alignment critics.
    fp['PathAlign.forward_point_distance'] = 0.10
    fp['GoalAlign.forward_point_distance'] = 0.10
    if 'goal_checker' in ctl:
        ctl['goal_checker']['xy_goal_tolerance'] = 0.12
        ctl['goal_checker']['yaw_goal_tolerance'] = 0.22
    if 'progress_checker' in ctl:
        ctl['progress_checker']['required_movement_radius'] = 0.05
        ctl['progress_checker']['movement_time_allowance'] = 20.0
    ctl['enable_stamped_cmd_vel'] = True
    for name in ('behavior_server', 'velocity_smoother', 'collision_monitor'):
        if name in cfg:
            node(cfg, name)['enable_stamped_cmd_vel'] = True
    if 'velocity_smoother' in cfg:
        sm = node(cfg, 'velocity_smoother')
        # Smoother upper limits must not clip the chosen controller speed.
        sm['max_velocity'] = [linear + 0.02, 0.0, angular + 0.10]
        sm['min_velocity'] = [-linear - 0.02, 0.0, -angular - 0.10]

    with output.open('w') as f:
        f.write('# IE4060 tuned from generated Jazzy TurtleBot3 parameters.\n')
        f.write('# {} version; keep original nav2_params.yaml as backup.\n'.format(
            'Optional 0.75m wider maze' if wide else 'Assignment 0.50m grid'))
        yaml.safe_dump(cfg, f, sort_keys=False)


def create_roomy_world():
    src = WORLD / 'maze.world'
    if not src.exists():
        raise SystemExit(f'Missing base world {src}')
    root = ET.parse(src).getroot()
    walls = []
    for model in root.findall('./world/model'):
        name = model.get('name', '')
        if name != 'floor' and not name.startswith('wall_'):
            continue
        pose_node = model.find('pose')
        if pose_node is None or not pose_node.text:
            raise SystemExit(f'No pose on {name}')
        pos = [float(s) for s in pose_node.text.split()]
        pos[0] *= FACTOR
        pos[1] *= FACTOR
        pose_node.text = fmt(pos)
        for sz in model.findall('./link/*/geometry/box/size'):
            size = [float(x) for x in sz.text.split()]
            if name == 'floor':
                size[0] *= FACTOR
                size[1] *= FACTOR
            elif name.startswith('wall_h_'):
                size[0] *= FACTOR    # length only, NOT wall thickness
            elif name.startswith('wall_v_'):
                size[1] *= FACTOR    # length only, NOT wall thickness
            else:
                raise SystemExit(f'Unrecognized wall orientation: {name}')
            sz.text = fmt(size)
        if name.startswith('wall_'):
            dims = [float(s) for s in model.findtext('./link/collision/geometry/box/size').split()]
            walls.append((pos[0], pos[1], dims[0], dims[1]))
    if len(walls) != 19:
        raise SystemExit(f'Expected 19 wall segments, found {len(walls)}')
    output = WORLD / 'maze_roomy.world'
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)
    return output, walls


def make_reference_map(walls):
    pixels = bytearray()
    for row in range(HEIGHT):
        y = ORIGIN + (HEIGHT - row - 0.5) * RESOLUTION
        for col in range(WIDTH):
            x = ORIGIN + (col + 0.5) * RESOLUTION
            inside = -2.25 < x < 2.25 and -2.25 < y < 2.25
            blocked = any(abs(x - wx) <= sx / 2 + RESOLUTION / 4 and
                          abs(y - wy) <= sy / 2 + RESOLUTION / 4
                          for wx, wy, sx, sy in walls)
            pixels.append(0 if blocked else (254 if inside else 127))
    (MAPS / 'maze_roomy.pgm').write_bytes(
        f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode() + pixels)
    metadata = {
        'image': 'maze_roomy.pgm', 'mode': 'trinary',
        'resolution': RESOLUTION, 'origin': [ORIGIN, ORIGIN, 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25,
    }
    with (MAPS / 'maze_roomy.yaml').open('w') as f:
        f.write('# Reference occupancy map generated from optional widened Gazebo walls.\n')
        f.write('# NOT a LiDAR SLAM measurement.\n')
        yaml.safe_dump(metadata, f, sort_keys=False)


def main():
    source = CONFIG / 'nav2_params.yaml'
    if not source.exists():
        raise SystemExit(f'Missing {source}. Run bash scripts/install_project.sh first.')
    with source.open() as f:
        config = yaml.safe_load(f)
    needed = {'amcl', 'planner_server', 'controller_server', 'global_costmap', 'local_costmap'}
    if not isinstance(config, dict) or not needed.issubset(config):
        raise SystemExit('nav2_params.yaml is incomplete; run install_project.sh (after fixing set -u) first.')
    write_nav(config, CONFIG / 'nav2_params_optimized.yaml', wide=False)
    world, walls = create_roomy_world()
    make_reference_map(walls)
    write_nav(config, CONFIG / 'nav2_params_roomy.yaml', wide=True)
    print('Created files (no existing project data overwritten):')
    for p in [CONFIG / 'nav2_params_optimized.yaml', world,
              MAPS / 'maze_roomy.pgm', MAPS / 'maze_roomy.yaml',
              CONFIG / 'nav2_params_roomy.yaml']:
        print(' -', p.relative_to(ROOT))
    print('Next: cd ~/ie4060_ws && colcon build --symlink-install --packages-select ie4060_maze')


if __name__ == '__main__':
    main()
