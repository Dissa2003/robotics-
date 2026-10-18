#!/usr/bin/env python3
"""Generate a geometrically exact reference occupancy map from worlds/maze.world.
This is NOT a LiDAR SLAM map. It is for simulator bringup and debugging.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dqn.maze_env import read_walls

RES = 0.02
SIZE = 160
ORIGIN = -1.60


def build():
    walls = read_walls(ROOT / 'worlds/maze.world')
    pixels = bytearray()
    # PGM image rows run top to bottom; occupancy origin is bottom left
    for row in range(SIZE):
        y = ORIGIN + (SIZE - row - 0.5) * RES
        for col in range(SIZE):
            x = ORIGIN + (col + 0.5) * RES
            inside = -1.5 < x < 1.5 and -1.5 < y < 1.5
            occupied = any(abs(x - wx) <= sx / 2 + RES / 4 and
                           abs(y - wy) <= sy / 2 + RES / 4
                           for wx, wy, sx, sy in walls)
            pixels.append(0 if occupied else (254 if inside else 127))
    dest = ROOT / 'maps'
    dest.mkdir(exist_ok=True)
    (dest / 'maze.pgm').write_bytes(f'P5\n{SIZE} {SIZE}\n255\n'.encode() + pixels)
    (dest / 'maze.yaml').write_text('''# Generated from Gazebo collision geometry; NOT a SLAM measurement.\nimage: maze.pgm\nmode: trinary\nresolution: 0.02\norigin: [-1.60, -1.60, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n''')
    print('Generated maps/maze.pgm and maps/maze.yaml from collision geometry')


if __name__ == '__main__':
    build()
